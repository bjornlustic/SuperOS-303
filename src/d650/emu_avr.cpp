// emu_avr.cpp — AT90USB1286 integration: runs the emulated TB-303 on the SuperOS
// board with MIDI (notes + clock), DIN sync, and a SuperOS-style config menu.
//
// Clock model (from the ROM's 0x3C ISR): /INT is a fixed ~555 Hz sampler; the
// tempo/sync clock is edge-detected on the CLOCK status input (PA3). So the
// selected source (internal / MIDI clock / DIN sync) drives H.in[CLOCK]; /INT
// is pulsed at ~555 Hz regardless, scheduled in the emulated-cycle domain (one
// edge per 204.5 machine cycles, see the batch loop). See emu_sync.* and
// PORTING_NOTES.md.

#ifdef SUPEROS_COMBINED
// Combined build: main.cpp owns the Timer3 vector and g_flash; entry points
// are renamed and dispatched from combined.cpp, and all d650 persistence
// (uPD444 store + settings) moves to the internal EEPROM.
#define DRIVERS_NO_ISR
// The loop glue (input scan, LED publish, /INT scheduling) shares the
// real-time budget with the interpreter; keep it at -O3 under the global -Os.
#pragma GCC optimize("O3")
#endif

#include <Arduino.h>
#include <util/delay.h>   // _delay_ms: busy-wait, safe after cli() (0x4A)
#include "src/pins.h"
#include "src/drivers.h"
#include "src/sequence.h"        // MAX_STEPS/NUM_SLOTS/... used by flash_persist.h
#include "src/flash_persist.h"

#ifdef SUPEROS_COMBINED
#include <avr/eeprom.h>
#include "src/combined.h"
#define setup emu_setup
#define loop  emu_loop
#endif

extern "C" {
#include "emu/ucom4.h"
#include "emu/d650_host.h"
#include "emu/tb303_rom.h"
#include "emu/emu_settings.h"
#include "emu/emu_sync.h"
#include "emu/emu_notes.h"
}

static const uint8_t FB_EMU_SETTINGS = 120;   // flash block id (distinct from SuperOS)

#ifndef SUPEROS_COMBINED
FlashEeprom g_flash;   // flash_persist.h declares it extern; SuperOS defines it in main.cpp
#endif

// ---------------------------------------------------------------------------
#ifdef SUPEROS_COMBINED
static d650_host &H = *(d650_host *)g_fw_arena;   // overlays SuperOS's Engine
#else
static d650_host   H;
#endif
static PinState    g_inputs[INPUT_COUNT];
static EmuSettings g_set;
static EmuSync     g_sync;
static EmuNotesOut g_out;
static EmuLive     g_live;

static const uint32_t CYC_PER_US_Q16 = 7447;   // 113.636 kHz machine cycles, 16.16
static uint32_t g_last_us = 0, g_budget_q16 = 0;
static bool     g_menu = false;

// /INT delivery in the EMULATED-CYCLE domain. The board's /INT source is a
// fixed ~555 Hz oscillator: one rising edge every 204.5 machine cycles of the
// 113.636 kHz CPU clock (1.8 ms). The old scheme consumed emu_sync's
// wall-clock int_pending FLAG once per loop() pass; at the measured ~420
// passes/s (2.4 ms/pass > 1.8 ms period) the ROM's ISR ran at ~420 Hz, not
// 555 (g_sync.int_edges counts scheduled periods, not delivered pulses, so
// the telemetry still read 555/s and masked the deficit). The ISR is the
// ROM's input/tap sampler, so key sampling and every tick-counted timing ran
// ~25% slow. Scheduling by emulated cycle count (Q8 fraction keeps the
// long-term rate exact) delivers every edge inside the batch loop at the
// exact cycle, independent of host pass rate.
static const uint32_t INT_PERIOD_CYC_Q8 =
    (uint32_t)((uint64_t)EMU_INT_PERIOD_US * CYC_PER_US_Q16 / 256u);  // 204.5 cyc in Q8
static int32_t  s_int_due_q8 = 0;      // countdown (Q8 cycles) to the next /INT edge
static uint32_t g_int_edges_cyc = 0;   // /INT edges actually delivered to the CPU

// Always-on perf counters (1 Hz tick in loop). The diag build prints them on
// USB serial; the production build reports them in the SysEx status reply, so
// real-time health (cyc/s ~= 113636, caps stays 0) is verifiable over USB MIDI.
static uint16_t g_caps = 0;                       // backlog-drop events since boot
static uint32_t g_cyc_mark = 0, g_cyc_per_s = 0;
static uint32_t g_stat_ms = 0;
// Stall telemetry: max gap between loop() entries and max single-pass duration
// since the last 0x40 status request (reset on each reply). Catches any host
// stall (flash program, USB blocking, ISR storm) with ~us resolution while a
// user reproduces a lag report over the SysEx poll.
static uint32_t g_max_gap_us = 0, g_max_pass_us = 0;
// Loop-shape counters (1 Hz): passes/s and wall time inside the emulation
// batch per second. Together with cyc/s these solve the pass-budget equation
// (per-pass overhead vs interpreter cost) on live production hardware.
static uint32_t g_pass_cnt = 0, g_emu_us_acc = 0;
static uint16_t g_pass_per_s = 0, g_emu_ms_per_s = 0;
// Section-profile fields in the 0x41 reply (bytes 30-39). The live timers
// were removed once the overhead hunt concluded (each pass paid ~30 us of
// micros() calls); the reply bytes stay for layout stability and read 0.
// Re-add micros() around loop sections and accumulate here when profiling.
static uint32_t g_sec_us[4];
static uint16_t g_sec_ms_per_s[4];
static uint16_t g_scan_ms_per_s = 0;
static uint16_t g_flash_writes = 0;               // pattern-block programs since boot
static uint16_t g_gate_edges = 0;                 // sequencer gate rising edges
static uint8_t  g_gate_prev  = 0;

// pattern persistence: the full 3x uPD444 store is D650_EXT_BYTES (1536) packed
// bytes, split into 192-byte flash blocks (ids 100..107).
static const uint8_t PATT_BLK_BASE = 100, PATT_BLK_LEN = 192;
static const uint8_t PATT_BLK_N = D650_EXT_BYTES / PATT_BLK_LEN;   // 8

// Deferred incremental pattern save (see loop). flash_write_page halts the
// CPU ~9 ms/page, and when the append bank fills, FlashEeprom's GC reprograms
// EVERY live record page in one write() call: 100 ms .. ~1 s of CPU halt.
// While halted the emulated 303 is frozen and the 1 kHz tempo-knob CLOCK
// sampler misses 24 ppqn edges outright, so a save landing mid-play made the
// sequence stand still and stumble back in (worst right after flipping the
// WRITE dial to PLAY: the editing gate released a whole pent-up flush).
// Policy now: flash is touched only when a stall is invisible — in the menu
// (303 frozen) or when nothing has played and no key has moved for a while —
// one changed block at a time, spaced SAVE_SPACING_MS apart, with the GC paid
// preemptively at idle so it can never land inside a later save. A mid-play
// safety net trickles long-unsaved edits out one block per second, only right
// after a gate falling edge (farthest from the next note) and never into a GC.
static bool     s_save_pending = false;
static uint8_t  s_save_cursor  = 0;
static uint32_t s_ram_quiet_ms = 0;
static uint32_t s_ui_ms        = 0;    // last panel key/dial edge
static uint32_t s_gate_edge_ms = 0;    // last sequencer gate rising edge
static uint32_t s_gate_fall_ms = 0;    // last sequencer gate falling edge
static uint32_t s_flush_hold_ms = 0;   // no flash program before this time
#ifndef SUPEROS_COMBINED
static bool     s_gc_dead      = false; // GC failed (over-full arena): stop trying
#endif
static const uint16_t SAVE_QUIET_MS       = 1500;  // RAM quiet before any save
static const uint16_t SAVE_ACT_MS         = 1000;  // recent gate/key edge = active
#ifdef SUPEROS_COMBINED
// EEPROM byte programs never halt the CPU, so no spacing is needed: the
// sweeper self-paces on eeprom_is_ready() (~3.3 ms/byte, ~300 B/s drain).
static const uint16_t SAVE_SPACING_MS     = 0;
static const uint16_t SAVE_TRICKLE_GAP_MS = 0;
#else
static const uint16_t SAVE_SPACING_MS     = 150;   // between idle block programs
static const uint16_t SAVE_TRICKLE_GAP_MS = 1000;  // spacing of mid-play programs
#endif
static const uint32_t SAVE_TRICKLE_MS     = 10000; // mid-play safety flush after
static const uint8_t  SAVE_GC_HEADROOM    = 12;    // idle GC below this many free pages

// External transport -> RUN line as a LEVEL, exactly like the real 303's
// DIN-sync RUN wire (it feeds the same PA0 input as the RUN/STOP button and
// stays high while the master runs). A 30 ms press pulse was tried first and
// the ROM stopped sequencing the moment it fell, so the line must be held.
// The panel button still passes through whenever the external line is low.
// s_run_force: SysEx 0x49 test override (0/1 = force level, 2 = auto).
static uint8_t s_run_force = 2;

#ifdef SUPEROS_USB_MIDI
// SysEx key injection (test-only): 0x4B <idx> <0|1> holds/releases an input,
// 0x4C <idx> <count> <press_ms> <gap_ms> runs a tap train. Applied by OR-ing
// into H.in after the panel scan, so latency tests exercise the exact same
// 2 ms frame quantization as physical keys. Used to reproduce and verify
// fast-tap registration on hardware without hands on the panel.
static uint8_t  s_inj_mask[(D650_IN_COUNT + 7) / 8];
static uint8_t  s_inj_any = 0;   // any hold bit set (skips the mask loop when idle)
static uint8_t  s_inj_idx = 0xFF, s_inj_count = 0, s_inj_press = 0, s_inj_gap = 0;
static uint32_t s_inj_t0 = 0;
#endif

static void patt_load_();
static bool patt_save_step();
static uint8_t run_line_level();

// ---------------------------------------------------------------------------
// MIDI OUT (DIN, USART1 @ 31250; mirrored to usbMIDI under SUPEROS_USB_MIDI,
// same dual-output scheme as stock SuperOS midi.cpp).
// ---------------------------------------------------------------------------
static inline void midi_tx(uint8_t b) { Serial1.write(b); }
static uint8_t midi_out_ch() { return g_set.midi_channel ? (uint8_t)(g_set.midi_channel - 1) : 0; }
static void send_note_on (void *, uint8_t n, uint8_t v){
  midi_tx(0x90|midi_out_ch()); midi_tx(n); midi_tx(v);
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendNoteOn(n, v, (uint8_t)(midi_out_ch() + 1));
  usbMIDI.send_now();
#endif
}
static void send_note_off(void *, uint8_t n){
  midi_tx(0x80|midi_out_ch()); midi_tx(n); midi_tx(0);
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendNoteOff(n, 0, (uint8_t)(midi_out_ch() + 1));
  usbMIDI.send_now();
#endif
}

// ---------------------------------------------------------------------------
// CV hook: emulated ports -> SuperOS DAC + MIDI-note-out translation.
// ---------------------------------------------------------------------------
#ifdef EMU_USB_DIAG
static uint32_t dg_cv_commits = 0;
uint32_t dg_emu_us = 0, dg_steps = 0, dg_loops = 0;   // profiling
// bisect switches, toggled over USB serial: L = LED refresh ISR, D = DAC
// output, E = emulation stepping. For isolating input crosstalk sources.
static bool dg_led_off = false, dg_dac_off = false, dg_emu_off = false;
// W = interactive matrix mapping: continuously scan the raw matrix and report
// stable state changes per (select config, line) as the user works the panel.
bool dg_watch = false;
// Watch modes: W = classic 5-config scan, X = 16 PH combos, Z = full 256-value
// PORTF sweep (PH low nibble + PG high nibble) to resolve the board's true
// select logic. State packed 2 bits/line so the full table fits in SRAM.
uint8_t dg_watch_mode = 0;   // 0=W, 1=X, 2=Z
static uint16_t dg_last[256];
static void dg_watch_scan() {
  uint16_t ncfg = (dg_watch_mode == 2) ? 256 : (dg_watch_mode == 1) ? 16 : 5;
  for (uint16_t cfg = 0; cfg < ncfg; cfg++) {
    if      (dg_watch_mode == 2) PORTF = (uint8_t)cfg;
    else if (dg_watch_mode == 1) PORTF = (uint8_t)cfg;      // PG low
    else PORTF = (cfg == 0) ? 0x0F : (uint8_t)(0x0F & ~(1 << (cfg - 1)));
    delayMicroseconds(15);
    uint8_t cnt[8] = {0};
    for (uint8_t n = 0; n < 64; n++) {           // ~25 ms window per config:
      uint8_t v = PINB;                          // spans the tempo-clock period,
      for (uint8_t b = 0; b < 8; b++) if (v & (1 << b)) cnt[b]++;
      delayMicroseconds(390);                    // so a toggling line reads "mid"
    }
    for (uint8_t b = 0; b < 8; b++) {
      uint8_t st = (cnt[b] < 16) ? 0 : (cnt[b] > 48) ? 1 : 2;
      uint8_t sh = (uint8_t)(b << 1);
      uint8_t old = (dg_last[cfg] >> sh) & 3;
      if (st != old) {
        dg_last[cfg] = (uint16_t)((dg_last[cfg] & ~(3u << sh)) | ((uint16_t)st << sh));
        Serial.print(F("# F="));
        if (PORTF < 16) Serial.print('0');
        Serial.print(PORTF, HEX);
        Serial.print(b < 4 ? F(" PA") : F(" PB"));
        Serial.print(b & 3);
        Serial.print(F(" -> "));
        Serial.println(st == 2 ? F("clk") : (st ? F("1") : F("0")));
      }
    }
  }
  PORTF = 0x0F;
  Serial.println(F("# sweep done"));
}
#endif

static void hook_cv(void *, uint8_t pitch, uint8_t gate, uint8_t accent, uint8_t slide) {
#ifdef EMU_USB_DIAG
  if (dg_dac_off) { dg_cv_commits++; return; }
#endif
  DAC::SetPitch(pitch); DAC::SetGate(gate); DAC::SetAccent(accent); DAC::SetSlide(slide);
  DAC::Send();
  if (gate && !g_gate_prev) { g_gate_edges++; s_gate_edge_ms = millis(); }
  else if (!gate && g_gate_prev) s_gate_fall_ms = millis();
  g_gate_prev = gate;
#ifdef EMU_USB_DIAG
  dg_cv_commits++;
#endif
  // Sequencer notes -> MIDI OUT whenever the ROM produces gates. Not gated on
  // g_sync.run: with an external source selected but its transport stopped,
  // the ROM can play from the panel RUN/STOP on the fallback internal clock,
  // and those notes must go out too.
  if (g_set.midi_out)
    emu_notes_out_update(&g_out, pitch, gate, accent, slide);
}
// Persistent LED image. The emulated ROM writes LEDs event-driven (a row at a
// time from its 555 Hz ISR), but Leds::Swap() clears the front buffer every
// frame — publishing raw events would show mostly-dark frames. Latch state
// here and republish the whole image once per frame.
//
// Mode LEDs (16..19) get a duty-cycle filter: they share PORTC with the
// uPD444 data bus, so pattern-RAM traffic in the write modes sprays data
// nibbles into the latch and a raw latch showed each transient for a whole
// 4 ms frame (visible TIME/PITCH LED flicker while editing). Integrate ON
// time in EMULATED machine cycles and threshold per frame, which is what the
// physical LED + eye do on the real board.
//
// Threshold is 1/16 of the frame, NOT majority: the ROM signals pattern
// clear (CLEAR + pattern key held) by pulsing PORTC=0xF at ~21% duty while
// it spends the rest of the loop writing zeros to the RAM; on the real
// board all four LEDs glow dimly for as long as the combo is held. Measured
// per 454-cycle frame (desktop probe): that indication never drops below
// 10%, while data-transient noise on OFF LEDs during pitch entry peaks at
// 2% and parked-ON LEDs never drop below 73%, so 1/16 (6.25%) separates all
// three cases with margin. A majority filter ate the clear indication.
// Instantaneous ROM LED image: [0] = LEDs 0-7, [1] = 8-15, [2] bits 0-3 =
// mode LEDs 16-19 (same layout as Leds::back[]). Byte ops via a mask LUT:
// the old single uint32_t image needed a 32-bit variable shift per hook_led
// call (a loop on the shifter-less AVR), and the ROM's scan loop hits
// hook_led four times per PORTG/PORTH write.
static uint8_t  g_led_img[3];
static const uint8_t BIT8[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint8_t  g_led_pub[3] = {0xff, 0xff, 0xff};  // last published back[] image
static uint16_t g_mode_on[4];                 // ON cycles this frame, per mode LED
static uint32_t g_mode_acc_cyc, g_mode_frame_cyc;
static uint8_t  g_mode_bits = 0;              // current instantaneous PORTC state
static void mode_led_account() {
  const uint32_t c = H.cpu.cyc;
  const uint16_t d = (uint16_t)(c - g_mode_acc_cyc);
  if (d) {
    for (uint8_t i = 0; i < 4; i++)
      if (g_mode_bits & (1 << i)) g_mode_on[i] = (uint16_t)(g_mode_on[i] + d);
    g_mode_acc_cyc = c;
  }
}
static void hook_led(void *, uint8_t idx, uint8_t on) {
  if (idx >= 16 && idx < 20) {                // mode LED: integrate, don't latch
    const uint8_t bit = (uint8_t)(1 << (idx - 16));
    const uint8_t cur = (g_mode_bits & bit) ? 1 : 0;
    if (cur == (on ? 1 : 0)) return;          // unchanged: free (common case)
    mode_led_account();                       // account elapsed at the old state
    g_mode_bits ^= bit;
    return;
  }
  if (idx < 16) {
    const uint8_t b = BIT8[idx & 7];
    if (on) g_led_img[idx >> 3] |=  b;
    else    g_led_img[idx >> 3] &= (uint8_t)~b;
  }
}
static void leds_publish() {
  mode_led_account();
  const uint32_t total = H.cpu.cyc - g_mode_frame_cyc;
  if (total) {                                // menu freeze: keep previous image
    for (uint8_t i = 0; i < 4; i++) {
      if ((uint32_t)g_mode_on[i] * 16 > total) g_led_img[2] |=  BIT8[i];
      else                                     g_led_img[2] &= (uint8_t)~BIT8[i];
      g_mode_on[i] = 0;
    }
    g_mode_frame_cyc = H.cpu.cyc;
  }
  // Publish straight into the ISR's back buffer. g_led_img[] already has the
  // back[] layout; the old per-LED Leds::Set loop (20 x 32-bit variable
  // shifts, no barrel shifter) + Swap cost ~300 us per 2 ms frame. Skip
  // entirely when the image is unchanged (the ISR keeps displaying back[];
  // only the menu path still uses Set/Swap).
  const uint8_t b0 = g_led_img[0];
  const uint8_t b1 = g_led_img[1];
  const uint8_t b2 = (uint8_t)(g_led_img[2] & 0x0f);
  if (b0 != g_led_pub[0] || b1 != g_led_pub[1] || b2 != g_led_pub[2]) {
    g_led_pub[0] = b0; g_led_pub[1] = b1; g_led_pub[2] = b2;
    const uint8_t s = SREG; cli();
    Leds::back[0] = b0; Leds::back[1] = b1; Leds::back[2] = b2;
    SREG = s;
  }
}
static const d650_drivers DRV = { hook_cv, hook_led, nullptr };

// ---------------------------------------------------------------------------
// MIDI IN parser (DIN). Realtime clock/transport + Note On/Off (live play + thru).
// ---------------------------------------------------------------------------
static uint8_t s_status = 0, s_d1 = 0; static bool s_have_d1 = false;
static bool on_our_channel(uint8_t st) {
  if (g_set.midi_channel == 0) return true;                 // omni
  return (st & 0x0f) == (g_set.midi_channel - 1);
}
static void midi_handle_channel(uint8_t st, uint8_t d1, uint8_t d2) {
  const uint8_t type = st & 0xf0;
  if (g_set.midi_thru) { midi_tx(st); midi_tx(d1); if (type!=0xC0 && type!=0xD0) midi_tx(d2); }
  if (!on_our_channel(st)) return;
  if (type == 0x90 && d2 > 0)      emu_live_note_on(&g_live, d1, d2);   // live play
  else if (type == 0x80 || (type == 0x90 && d2 == 0)) emu_live_note_off(&g_live, d1);
}
static void midi_in_poll() {
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (b >= 0xF8) {                          // realtime (may interleave)
      // Transport only when MIDI is the selected source: a stray Start in DIN
      // mode would force the RUN line and wait for pulses that never come.
      if (b == 0xF8) emu_sync_ext_pulse(&g_sync, EMU_CLK_MIDI);
      else if (b == 0xFA && g_set.clock_source == EMU_CLK_MIDI) { emu_sync_transport(&g_sync, 1); }
      else if (b == 0xFC && g_set.clock_source == EMU_CLK_MIDI) { emu_sync_transport(&g_sync, 0); emu_notes_out_all_off(&g_out); }
      continue;
    }
    // Status. System common (0xF0-0xF7, incl. SysEx) clears running status so
    // its data bytes below are ignored instead of misparsed as channel messages.
    if (b & 0x80) { s_status = (b < 0xF0) ? b : 0; s_have_d1 = false; continue; }
    if (!s_status) continue;                                      // data inside SysEx etc.
    if (!s_have_d1) { s_d1 = b; s_have_d1 = true; }               // data1
    else {                                                        // data2 -> complete
      midi_handle_channel(s_status, s_d1, b); s_have_d1 = false;
    }
  }
}

// ---------------------------------------------------------------------------
// CLOCK status-line sampler (PA3) + DIN run/stop (PA0). The board's own tempo
// oscillator (panel TEMPO knob) drives PA3 as a 24 ppqn square wave; the DIN
// jack's switching contacts substitute the DIN run and clock on the same RUN
// (PA0) / CLOCK (PA3) status lines when a cable is plugged. So one sampler
// serves both: in INTERNAL mode PA3 carries the knob oscillator (tempo follows
// the knob, exactly like the real 303 / SuperOS-303), and in DIN mode it
// carries the sync-in clock. The 4 ms matrix frame is too coarse for 24 ppqn
// edge detection above ~120 BPM, so sample the lines directly at ~1 kHz:
// select high (same setup PollInputs uses), short settle, read, edge-detect.
// MIDI mode does not touch PA3 (the ROM's clock arrives as 0xF8 pulses).
// ---------------------------------------------------------------------------
static uint8_t s_sync_prev = 0, s_run_prev = 0;
static void clock_in_poll() {
  const uint8_t src = g_set.clock_source;
  if (src != EMU_CLK_DIN && src != EMU_CLK_INTERNAL) return;
  // The LED refresh ISR samples the status lines at ~2 kHz during its own
  // all-selects-high settle window (Leds::last_status_pinb) — same electrical
  // state the old sampler recreated here with a PORTF write + 15 us busy-wait
  // per ms. Edge-detect on the snapshot; costs a few cycles per pass.
  const uint8_t v = Leds::last_status_pinb;
  const uint8_t c = (uint8_t)((v >> 3) & 1);   // CLOCK status line (PA3)
  const uint8_t r = (uint8_t)(v & 1);          // RUN status line (PA0)
  if (c && !s_sync_prev) {                                // rising = one 24 ppqn pulse
    g_sync.clk_last_pulse_us = micros();
    if (src == EMU_CLK_INTERNAL) g_sync.clk_knob = 1;     // tempo knob is alive
    emu_sync_ext_pulse(&g_sync, src);
  }
  s_sync_prev = c;
  // DIN run/stop is a level on PA0; INTERNAL has no external transport (the
  // panel RUN/STOP button drives the ROM's own play latch through the matrix).
  if (src == EMU_CLK_DIN && r != s_run_prev) {
    emu_sync_transport(&g_sync, r);
    if (!r) emu_notes_out_all_off(&g_out);
  }
  s_run_prev = r;
}

// ---------------------------------------------------------------------------
// Settings persistence (flash block; internal EEPROM in the combined build)
// ---------------------------------------------------------------------------
#ifdef SUPEROS_COMBINED
static void settings_save() {
  uint8_t b[EMU_SETTINGS_LEN]; emu_settings_serialize(&g_set, b);
  eeprom_update_block(b, EE_EMU_SETTINGS, EMU_SETTINGS_LEN);
  eeprom_update_byte(EE_EMU_MAGIC, EE_EMU_MAGIC_VAL);
}
static void settings_load() {
  if (eeprom_read_byte(EE_EMU_MAGIC) == EE_EMU_MAGIC_VAL) {
    uint8_t b[EMU_SETTINGS_LEN];
    eeprom_read_block(b, EE_EMU_SETTINGS, EMU_SETTINGS_LEN);
    emu_settings_deserialize(&g_set, b);
  } else {
    emu_settings_defaults(&g_set);
    settings_save();   // stamps the magic; pattern area stays 0xFF (virgin)
  }
}
#else
static void settings_load() {
  uint8_t b[EMU_SETTINGS_LEN];
  if (g_flash.read(FB_EMU_SETTINGS, b, EMU_SETTINGS_LEN) == EMU_SETTINGS_LEN)
    emu_settings_deserialize(&g_set, b);
  else emu_settings_defaults(&g_set);
}
static void settings_save() {
  uint8_t b[EMU_SETTINGS_LEN]; emu_settings_serialize(&g_set, b);
  g_flash.write(FB_EMU_SETTINGS, b, EMU_SETTINGS_LEN);
}
#endif
static void settings_apply() {
  Leds::brightness = g_set.led_brightness;
  emu_sync_set_source(&g_sync, g_set.clock_source);
  emu_sync_set_bpm(&g_sync, g_set.internal_bpm);
  emu_notes_out_init(&g_out, g_set.pitch_base);
  g_out.note_on = send_note_on; g_out.note_off = send_note_off; g_out.u = nullptr;
  emu_live_init(&g_live, g_set.pitch_base);
}

// ---------------------------------------------------------------------------
// MIDI clock out: when we are the tempo leader (internal source) and MIDI out
// is enabled, every internal 24 ppqn rising edge goes out as 0xF8 on DIN and
// USB so downstream gear / a DAW can follow the 303's tempo. External sources
// are not echoed back (the sender already has the clock). No Start/Stop: the
// ROM's play latch is internal to the emulated CPU, so the clock free-runs
// (like a TR-606's DIN clock).
// ---------------------------------------------------------------------------
static uint32_t s_clk_out_mark = 0;
static void clock_out_service() {
  if (g_set.clock_source != EMU_CLK_INTERNAL || !g_set.midi_out) {
    s_clk_out_mark = g_sync.clk_edges;
    return;
  }
  while (s_clk_out_mark != g_sync.clk_edges) {
    ++s_clk_out_mark;
    midi_tx(0xF8);
#ifdef SUPEROS_USB_MIDI
    usbMIDI.sendRealTime(0xF8);
#endif
  }
}

// ---------------------------------------------------------------------------
// USB MIDI (SUPEROS_USB_MIDI): RX mirror of the DIN handling, plus a small
// SysEx status/config protocol. Framing is SuperOS's (F0 7D <cmd> ... F7);
// commands live at 0x40+ so they never collide with the web editor's 0x10-0x2F
// range, and stock SuperOS ignores them. This is what makes the production
// build (which has no USB serial) testable and configurable over USB-C:
//   0x40              -> reply 0x41: clock source, channel, BPM, run, gate,
//                        pitch, accent/slide, clk_edges(14b), cyc/s(21b),
//                        caps(14b); all 7-bit fields, LSB first
//   0x42 <src>        -> set clock source 0=INTERNAL 1=MIDI 2=DIN (saved)
//   0x43 <lo7> <hi7>  -> set internal BPM 20..300 (saved)
//   0x44 <ch>         -> set MIDI channel 0=omni, 1..16 (saved)
//   0x46 <blk>        -> read pattern RAM block: reply 0x47 <blk> <220 packed>
//   0x47 <blk> <220b> -> write pattern RAM block; reply 0x48 <blk> <0=ok 1=err>
//   0x4A              -> reboot into the SysEx bootloader (no reply: the
//                        device detaches and re-enumerates as
//                        "SuperOS-303 Bootloader"; send the update .syx,
//                        its trailing 0x02 command reboots back to the app)
// Each config command answers with the 0x41 status reply as the ack.
// Blocks are the flash-persist granularity: 8 x 192 bytes of the packed
// uPD444 store (H.ext, 2 nibbles/byte = the full 3072-nibble pattern memory),
// 7-bit packed for SysEx (192 raw -> 220 wire bytes). This is the raw-RAM
// foundation for web editing and pattern backup: the host page reads/writes
// native 303 pattern memory directly, no format translation in firmware.
// ---------------------------------------------------------------------------
#ifdef SUPEROS_USB_MIDI
static bool chan_ok(uint8_t ch /*1..16*/) {
  return g_set.midi_channel == 0 || ch == g_set.midi_channel;
}
static void usb_send_status() {
  uint8_t r[46];
  r[0]  = 0x7D; r[1] = 0x41;
  r[2]  = g_set.clock_source;
  r[3]  = g_set.midi_channel;
  r[4]  = (uint8_t)(g_set.internal_bpm & 0x7F);
  r[5]  = (uint8_t)((g_set.internal_bpm >> 7) & 0x7F);
  r[6]  = (uint8_t)(g_sync.run & 1);
  r[7]  = (uint8_t)(H.gate & 1);
  r[8]  = (uint8_t)(H.pitch & 0x7F);
  r[9]  = (uint8_t)((H.accent & 1) | ((H.slide & 1) << 1));
  r[10] = (uint8_t)(g_sync.clk_edges & 0x7F);
  r[11] = (uint8_t)((g_sync.clk_edges >> 7) & 0x7F);
  r[12] = (uint8_t)(g_cyc_per_s & 0x7F);
  r[13] = (uint8_t)((g_cyc_per_s >> 7) & 0x7F);
  r[14] = (uint8_t)((g_cyc_per_s >> 14) & 0x7F);
  r[15] = (uint8_t)(g_caps & 0x7F);
  r[16] = (uint8_t)((g_caps >> 7) & 0x7F);
  r[17] = H.in[D650_IN_RUN] & 1;            // current RUN line level at the ROM
  r[18] = (uint8_t)(g_gate_edges & 0x7F);
  r[19] = (uint8_t)((g_gate_edges >> 7) & 0x7F);
  // stall telemetry since the previous status request, 128 us units (14 bit
  // spans 2.1 s), then reset so each poll reports its own interval
  uint32_t q = g_max_gap_us >> 7;  if (q > 16383) q = 16383;
  r[20] = (uint8_t)(q & 0x7F); r[21] = (uint8_t)((q >> 7) & 0x7F);
  q = g_max_pass_us >> 7;          if (q > 16383) q = 16383;
  r[22] = (uint8_t)(q & 0x7F); r[23] = (uint8_t)((q >> 7) & 0x7F);
  r[24] = (uint8_t)(g_flash_writes & 0x7F);
  r[25] = (uint8_t)((g_flash_writes >> 7) & 0x7F);
  r[26] = (uint8_t)(g_pass_per_s & 0x7F);
  r[27] = (uint8_t)((g_pass_per_s >> 7) & 0x7F);
  r[28] = (uint8_t)(g_emu_ms_per_s & 0x7F);
  r[29] = (uint8_t)((g_emu_ms_per_s >> 7) & 0x7F);
  for (uint8_t i = 0; i < 4; i++) {
    r[30 + 2*i] = (uint8_t)(g_sec_ms_per_s[i] & 0x7F);
    r[31 + 2*i] = (uint8_t)((g_sec_ms_per_s[i] >> 7) & 0x7F);
  }
  r[38] = (uint8_t)(g_scan_ms_per_s & 0x7F);
  r[39] = (uint8_t)((g_scan_ms_per_s >> 7) & 0x7F);
  // live H.in bitmap (40 inputs, 7 per byte): shows the ROM's current input
  // lines incl. the physical mode dial — needed to interpret injection tests
  for (uint8_t i = 0; i < 6; i++) r[40 + i] = 0;
  for (uint8_t i = 0; i < D650_IN_COUNT; i++)
    if (H.in[i]) r[40 + i / 7] |= (uint8_t)(1 << (i % 7));
  g_max_gap_us = g_max_pass_us = 0;
  usbMIDI.sendSysEx(sizeof(r), r, false);   // core wraps F0 .. F7
}
// 7-bit pack/unpack (SuperOS midi.cpp scheme): each group of up to 7 raw bytes
// goes out as 1 MSB-bitmap byte + 7 low-7-bit bytes. 192 raw -> 220 wire bytes.
static const uint16_t PATT_WIRE_LEN = 220;
static uint16_t pack7(const uint8_t *src, uint16_t len, uint8_t *out) {
  uint16_t o = 0;
  for (uint16_t i = 0; i < len; i += 7) {
    uint8_t msb = 0;
    const uint8_t n = (len - i >= 7) ? 7 : (uint8_t)(len - i);
    for (uint8_t b = 0; b < n; ++b)
      if (src[i + b] & 0x80) msb |= (uint8_t)(1u << b);
    out[o++] = msb;
    for (uint8_t b = 0; b < n; ++b) out[o++] = src[i + b] & 0x7F;
  }
  return o;
}
static void unpack7(const uint8_t *src, uint16_t wire_len, uint8_t *out) {
  uint16_t i = 0, o = 0;
  while (i < wire_len) {
    const uint8_t msb = src[i++];
    for (uint8_t b = 0; b < 7 && i < wire_len; ++b)
      out[o++] = (uint8_t)(src[i++] | ((msb >> b) & 1 ? 0x80 : 0));
  }
}
static void usb_send_ram_block(uint8_t blk) {
  uint8_t r[3 + PATT_WIRE_LEN];
  r[0] = 0x7D; r[1] = 0x47; r[2] = blk;
  const uint16_t wl = pack7(&H.ext[(uint16_t)blk * PATT_BLK_LEN], PATT_BLK_LEN, r + 3);
  usbMIDI.sendSysEx((uint16_t)(3 + wl), r, false);
}
static void usb_ram_ack(uint8_t blk, uint8_t status) {
  const uint8_t r[4] = { 0x7D, 0x48, blk, status };
  usbMIDI.sendSysEx(4, r, false);
}
// 0x4A: reboot into the SysEx bootloader without the power-on button combo.
// Parks BOOT_MAGIC in GPIOR0 (survives the jmp; any real reset clears it) so
// the bootloader stays resident instead of bouncing back to the app, and
// detaches USB first so the host sees a clean disconnect before the
// bootloader's own device attaches.
static void enter_bootloader() {
  // Flush any pending pattern edits first: the jump never returns and the
  // packed uPD444 store in SRAM is lost. Bounded: <= 8 blocks, changed only.
  while (s_save_pending) patt_save_step();
  cli();
  UDCON |= (1 << DETACH);
  _delay_ms(30);                 // host must register the drop first
  USBCON = 0;                    // leave the controller clean for the
  PLLCSR = 0;                    // bootloader's own init
  GPIOR0 = 0xB7;                 // BOOT_MAGIC, keep in sync with bootload.c
  asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
}
static void usb_sysex_msg(const uint8_t *data, unsigned int sz) {
  if (sz < 4 || data[0] != 0xF0 || data[sz - 1] != 0xF7) return;
  const uint8_t *p = data + 1;
  const unsigned n = sz - 2;
  if (p[0] != 0x7D || n < 2) return;
  switch (p[1]) {
  case 0x40: usb_send_status(); break;
  case 0x42:
    if (n >= 3 && p[2] < EMU_CLK_COUNT) {
      g_set.clock_source = p[2];
      settings_apply(); settings_save(); usb_send_status();
    }
    break;
  case 0x43:
    if (n >= 4) {
      uint16_t bpm = (uint16_t)(p[2] | ((uint16_t)p[3] << 7));
      if (bpm >= 20 && bpm <= 300) {
        g_set.internal_bpm = bpm;
        settings_apply(); settings_save(); usb_send_status();
      }
    }
    break;
  case 0x44:
    if (n >= 3 && p[2] <= 16) {
      g_set.midi_channel = p[2];
      settings_apply(); settings_save(); usb_send_status();
    }
    break;
  case 0x46:
    if (n >= 3 && p[2] < PATT_BLK_N) usb_send_ram_block(p[2]);
    break;
  case 0x49:   // RUN line test override: 0/1 force level, 2 = auto (default)
    if (n >= 3 && p[2] <= 2) { s_run_force = p[2]; usb_send_status(); }
    break;
  case 0x4B:   // key hold override: <idx> <0|1> (test only)
    if (n >= 4 && p[2] < D650_IN_COUNT) {
      if (p[3]) s_inj_mask[p[2] >> 3] |= (uint8_t)(1 << (p[2] & 7));
      else      s_inj_mask[p[2] >> 3] &= (uint8_t)~(1 << (p[2] & 7));
      s_inj_any = 0;
      for (uint8_t i = 0; i < sizeof(s_inj_mask); i++) s_inj_any |= s_inj_mask[i];
      usb_send_status();
    }
    break;
  case 0x4C:   // key tap train: <idx> <count> <press_ms> <gap_ms> (test only)
    if (n >= 6 && p[2] < D650_IN_COUNT) {
      s_inj_idx = p[2]; s_inj_count = p[3];
      s_inj_press = p[4]; s_inj_gap = p[5];
      s_inj_t0 = millis();
      usb_send_status();
    }
    break;
  case 0x4A:
    enter_bootloader();          // does not return
    break;
  case 0x47:
    if (n >= 3 + PATT_WIRE_LEN && p[2] < PATT_BLK_N) {
      const uint8_t blk = p[2];
      uint8_t *dst = &H.ext[(uint16_t)blk * PATT_BLK_LEN];
      unpack7(p + 3, PATT_WIRE_LEN, dst);
#ifdef SUPEROS_COMBINED
      // Blocking update of just this block (changed bytes only). Web editing
      // happens stopped; worst case (all 192 bytes differ) is ~630 ms.
      eeprom_update_block(dst, EE_EMU_PATT + (uint16_t)blk * PATT_BLK_LEN, PATT_BLK_LEN);
      eeprom_update_byte(EE_EMU_MAGIC, EE_EMU_MAGIC_VAL);
#else
      g_flash.write((uint8_t)(PATT_BLK_BASE + blk), dst, PATT_BLK_LEN);  // persist just this block
#endif
      usb_ram_ack(blk, 0);
    } else if (n >= 3) {
      usb_ram_ack(p[2], 1);
    }
    break;
  default: break;   // SuperOS web-editor pattern commands (0x10-0x2F): need a
                    // SuperOS<->native-303 pattern format translation, not done
  }
}
// USB SysEx reassembly (same reason as SuperOS midi.cpp): the AVR usb_midi
// core's complete-message buffer is only 60 bytes, but the 0x47 RAM write is
// F0 + 3 header + 220 packed + F7 = 225 bytes. Oversized messages are dropped.
static uint8_t  s_usx_buf[3 + 3 + PATT_WIRE_LEN + 2];
static uint16_t s_usx_len  = 0;
static bool     s_usx_drop = false;
static void usb_sysex_partial(const uint8_t *data, uint16_t length, bool complete) {
  if (s_usx_len == 0) s_usx_drop = false;
  if (!s_usx_drop) {
    if (s_usx_len + length <= sizeof(s_usx_buf)) {
      for (uint16_t i = 0; i < length; ++i) s_usx_buf[s_usx_len++] = data[i];
    } else {
      s_usx_drop = true;
    }
  }
  if (complete) {
    if (!s_usx_drop) usb_sysex_msg(s_usx_buf, s_usx_len);
    s_usx_len = 0;
    s_usx_drop = false;
  }
}
static void usb_midi_poll() {
  // Bounded drain (same rationale as SuperOS midi.cpp): read() returns false
  // for SysEx-continuation packets too, so keep going past a false return.
  // But bail out as soon as the RX endpoint is empty: this poll runs every
  // pass, and 48 no-op read() calls are measurable against the real-time
  // margin. The probe is the same RWAL/RXOUTI check read() performs first
  // (endpoint 4 = MIDI_RX_ENDPOINT in the AVR usb_midi core).
  for (uint8_t n = 0; n < 48; ++n) {
    {
      const uint8_t s = SREG; cli();
      UENUM = 4;
      const uint8_t c = UEINTX;
      SREG = s;
      if (!(c & ((1 << RWAL) | (1 << RXOUTI)))) break;
    }
    if (!usbMIDI.read()) continue;
    const uint8_t t = usbMIDI.getType();
    if (t == usbMIDI.NoteOn) {
      if (!chan_ok(usbMIDI.getChannel())) continue;
      const uint8_t v = usbMIDI.getData2();
      if (v) emu_live_note_on(&g_live, usbMIDI.getData1(), v);
      else   emu_live_note_off(&g_live, usbMIDI.getData1());
    } else if (t == usbMIDI.NoteOff) {
      if (chan_ok(usbMIDI.getChannel())) emu_live_note_off(&g_live, usbMIDI.getData1());
    } else if (t == usbMIDI.Clock) {
      emu_sync_ext_pulse(&g_sync, EMU_CLK_MIDI);
    } else if (t == usbMIDI.Start || t == usbMIDI.Continue) {
      if (g_set.clock_source == EMU_CLK_MIDI) emu_sync_transport(&g_sync, 1);
    } else if (t == usbMIDI.Stop) {
      if (g_set.clock_source == EMU_CLK_MIDI) {
        emu_sync_transport(&g_sync, 0);
        emu_notes_out_all_off(&g_out);
      }
    }
  }
}
#endif

// ---------------------------------------------------------------------------
// Config menu (mirrors SuperOS): FUNCTION + CLEAR to enter. Number keys 1-8
// (+ D# = high bank) set MIDI channel 1-16. TIME cycles clock source
// (INTERNAL->MIDI->DIN); TIME LED lit when external (like SuperOS DIN indicator).
// A# + UP/DOWN sets brightness. CLEAR exits.
// ---------------------------------------------------------------------------
static void menu_show_channel() {
  uint8_t dc = g_set.midi_channel ? g_set.midi_channel : 1;
  static bool s_high = false;
  if (g_inputs[DSHARP_KEY].rising()) s_high = !s_high;
  Leds::Set(DSHARP_KEY_LED, s_high);
  if (s_high) { if (dc >= 9 && dc <= 16) Leds::Set(OutputIndex((dc-9)&7), true); }
  else        { if (dc >= 1 && dc <= 8)  Leds::Set(OutputIndex((dc-1)&7), true); }
  for (uint8_t i = 0; i < 8; ++i)
    if (g_inputs[i].rising()) { g_set.midi_channel = (uint8_t)(i + 1 + (s_high ? 8 : 0)); settings_save(); break; }
}
static void process_menu() {
  Leds::Set(FUNCTION_MODE_LED, true);
  menu_show_channel();

  // clock source: TIME cycles INTERNAL -> MIDI -> DIN. TIME LED lit = external;
  // PITCH LED lit additionally = DIN (the DIN signals ride the board's own
  // RUN/CLOCK status lines via the sync jack's switching contacts).
  if (g_inputs[TIME_KEY].rising()) {
    g_set.clock_source = (uint8_t)((g_set.clock_source + 1) % EMU_CLK_COUNT);
    settings_apply(); settings_save();
  }
  Leds::Set(TIME_MODE_LED,  g_set.clock_source != EMU_CLK_INTERNAL);  // external lit
  Leds::Set(PITCH_MODE_LED, g_set.clock_source == EMU_CLK_DIN);

  // MIDI thru on ACCENT
  Leds::Set(ACCENT_KEY_LED, g_set.midi_thru != 0);
  if (g_inputs[ACCENT_KEY].rising()) { g_set.midi_thru = !g_set.midi_thru; settings_save(); }

  // brightness: A# + UP/DOWN
  if (g_inputs[ASHARP_KEY].held()) {
    Leds::Set(ASHARP_KEY_LED, true);
    if (g_inputs[UP_KEY].rising()   && g_set.led_brightness < 8) { g_set.led_brightness++; Leds::brightness=g_set.led_brightness; settings_save(); }
    if (g_inputs[DOWN_KEY].rising() && g_set.led_brightness > 1) { g_set.led_brightness--; Leds::brightness=g_set.led_brightness; settings_save(); }
  }

#ifdef SUPEROS_COMBINED
  // G# = boot the SuperOS firmware. Dim G# marks the option; flush the
  // uPD444 store to EEPROM first (bounded: changed bytes only), then reboot.
  Leds::SetDim(GSHARP_KEY_LED, true);
  if (g_inputs[GSHARP_KEY].rising()) {
    while (s_save_pending) patt_save_step();
    combined_switch_firmware(FW_SUPEROS);   // does not return
  }
#endif

  // exit on CLEAR or FUNCTION (entry used FUNCTION *held* + CLEAR rising, so
  // a fresh FUNCTION press here is unambiguous)
  if (g_inputs[CLEAR_KEY].rising() || g_inputs[FUNCTION_KEY].rising()) g_menu = false;
}

// ---------------------------------------------------------------------------
// Input refresh: poll matrix; enter/exit menu; feed emulator when not in menu.
// ---------------------------------------------------------------------------
// Direct-port matrix scan, replacing the generic PollInputs. That routine
// spends ~460 us per frame in array-indexed digitalRead/digitalWrite (the
// slow Arduino path: ~5 us each x 40 pins); at the 2 ms input frame that was
// ~23% of the CPU and pushed the loop past real time (measured via the
// section profile in the 0x41 reply). All eight matrix lines live on PINB
// (bits 0-3 = PA status, 4-7 = PB buttons; confirmed by the diag matrix
// probes) and the selects are PORTF bits 0-3, so each row is one settle +
// one PINB read: 5 x 15 us settles + cheap shifts ~= 105 us total.
// The PA (status) nibble needs a LONGER settle than the PB (button) nibble:
// those lines are actively driven high in other select states (the tempo osc
// on PA3, held mode keys during the LED ISR's row phases, PITCH/FUNCTION in
// the all-high phase) and then decay through the switch board's 15 K
// pulldowns when the select changes. Read at +15 us the residual charge
// still reads "pressed": holding PITCH ghosted C# (PA2@PH0) into pattern-
// play transpose, and holding FUNCTION ghosted TAP (PA1, all-high) firing
// random notes. The stock PollInputs never saw this because its interleaved
// slow digitalReads landed the status pins at +17..+35 us after the select.
// So: buttons from one PINB read at +15 us (driven hard via the row buffer,
// valid early), status nibble from a second read at +29 us.
#define PA_EXTRA_SETTLE_US 14
static void fast_poll_inputs(PinState *in) {
  PORTF = 0x0f;                                  // all selects high
  digitalWriteFast(PG0_PIN, LOW);                // clear LED drive residue
  digitalWriteFast(PG1_PIN, LOW);                // (same as PollInputs)
  digitalWriteFast(PG2_PIN, LOW);
  digitalWriteFast(PG3_PIN, LOW);
  delayMicroseconds(SWITCH_DELAY + PA_EXTRA_SETTLE_US);
  uint8_t v = PINB;
  for (uint8_t j = 0; j < 4; ++j)                // status pins: RUN/TAP/-/CLOCK
    in[EXTRA_PIN_OFFSET + j].push((v >> j) & 1);
  for (uint8_t i = 0; i < 4; ++i) {
    PORTF = (uint8_t)(0x0f & ~(1 << i));         // row select low
    delayMicroseconds(SWITCH_DELAY);
    const uint8_t vb = PINB;                     // buttons: valid at +15 us
    delayMicroseconds(PA_EXTRA_SETTLE_US);
    const uint8_t va = PINB;                     // status: residual decayed
    for (uint8_t j = 0; j < 4; ++j) {
      in[ 0 + i*4 + j].push((vb >> (4 + j)) & 1); // PBx = button block
      in[16 + i*4 + j].push((va >> j) & 1);       // PAx = status block
    }
  }
  PORTF = 0x0f;
}

static void refresh_inputs() {
  Leds::PauseRefresh();
  fast_poll_inputs(g_inputs);
  // Re-light the matrix NOW and restart the ISR period. The scan blanks the
  // matrix (all selects high, columns low) and used to leave it dark until
  // the next free-running Timer3 tick, 0..1024 us away at a phase drifting
  // 48 us per 2 ms frame (2000 us scan vs 1024 us tick): the dark tail swept
  // 8%..58% of every frame at ~23 Hz: the visible matrix-LED flicker (the
  // direct-drive mode LEDs were immune). One manual refresh plus a timer
  // phase reset pins the scan and the row multiplex to a fixed phase: the
  // blanking becomes a constant ~150 us per 2 ms frame (uniform ~7% duty
  // cost, invisible), zero beat.
  TCNT3 = 0;
  TIFR3 = (1 << OCF3A);          // drop any tick that came due during the scan
#ifdef EMU_USB_DIAG
  if (!dg_led_off) { Leds::SendISR(); Leds::ResumeRefresh(); }
#else
  Leds::SendISR();
  Leds::ResumeRefresh();
#endif

  // menu entry: FUNCTION held + CLEAR rising (TAP not held), like SuperOS
  if (!g_menu && g_inputs[FUNCTION_KEY].held() && g_inputs[CLEAR_KEY].rising() && !g_inputs[TAP_NEXT].held()) {
    g_menu = true;
    // The frozen 303 keeps the last-fed inputs latched (FUNCTION held, etc.)
    // and used to see that stale state for one frame after menu exit before
    // the next scan re-fed live keys. Resume from a clean slate instead.
    for (uint8_t i = 0; i < D650_IN_COUNT; i++) H.in[i] = 0;
    return;
  }

  if (g_menu) { process_menu(); return; }             // emulator frozen while in menu

  // held(), not read(): read() is the raw last sample, so a contact-bounce low
  // landing on the 4 ms sample point showed the ROM a release+repress mid-tap
  // (inconsistent recognition lag / double entries in step write). held() is
  // high if ANY of the last 3 samples was high: press recognition still starts
  // on the first sample, bounce lows are bridged, release stretches ~8 ms
  // (harmless for keys). CLOCK is overwritten from g_sync every pass and RUN
  // is OR-ed with the external level below, so neither is affected.
  // One pass: feed H.in from the debounced samples AND detect key/dial edges
  // (any edge on inputs 0..TAP_NEXT = hands on the panel; holds flash programs
  // off per the save policy). CLOCK (35) toggles freely and stays excluded.
  uint8_t edge = 0;
  for (uint8_t i = 0; i < D650_IN_COUNT && i < INPUT_COUNT; i++) {
    const uint8_t st = g_inputs[i].state;
    H.in[i] = (st & STATE_ON) ? 1 : 0;                // held()
    if (i <= TAP_NEXT) {
      const uint8_t on = st & STATE_ON;
      edge |= (on == STATE_RISING) | (on == STATE_FALLING);
    }
  }
  if (edge) s_ui_ms = millis();
  if (run_line_level()) H.in[D650_IN_RUN] = 1;        // DIN-style run line (level)

#ifdef SUPEROS_USB_MIDI
  // test-only SysEx key injection (see s_inj_* above). Any active injection
  // counts as panel activity so the flash-save policy behaves as with real keys.
  if (s_inj_any || s_inj_count) {
    if (s_inj_any) {
      uint8_t m = 1, b = 0;
      for (uint8_t i = 0; i < D650_IN_COUNT; i++) {
        if (s_inj_mask[b] & m) H.in[i] = 1;
        m <<= 1; if (!m) { m = 1; b++; }
      }
    }
    if (s_inj_count && s_inj_idx < D650_IN_COUNT) {
      const uint32_t el = millis() - s_inj_t0;
      const uint16_t period = (uint16_t)s_inj_press + s_inj_gap;
      const uint32_t tap = period ? el / period : s_inj_count;
      if (tap >= s_inj_count) s_inj_count = 0;
      else if (el - tap * period < s_inj_press) H.in[s_inj_idx] = 1;
    }
    s_ui_ms = millis();
  }
#endif
}

// ---------------------------------------------------------------------------
void setup() {
  for (uint8_t i = 0; i < sizeof(OUTPUTS); i++) pinMode(OUTPUTS[i], OUTPUT);
  // Plain INPUT like stock SuperOS: the switch board has its own pulldowns and
  // row drivers. INPUT_PULLUP floats the PA lines high = every key reads held.
  for (uint8_t i = 0; i < sizeof(INPUTS);  i++) pinMode(INPUTS[i],  INPUT);
  DDRC = 0xff;
  Leds::BeginRefresh();
  // Quarter the LED refresh ISR rate (3.9 kHz -> 977 Hz): the ISR busy-waits
  // 15 us per entry; at 3.9 kHz that cost ~10% of the CPU the interpreter
  // needs for real time, at 1.95 kHz still ~4%. Row multiplex is 244 Hz
  // (invisible) and the dim-PWM cycle stays ~61 Hz because SendISR now
  // advances the PWM phase every 2 ticks. Status-line snapshots for the
  // CLOCK/RUN sampler drop to 977 Hz — still >4x oversampled at 300 BPM
  // (24 ppqn half period 4.2 ms).
  OCR3A = 255;
  Serial1.begin(31250);                 // DIN MIDI
#ifdef SUPEROS_USB_MIDI
  // 3-arg (partial) overload: reassembled in usb_sysex_partial so messages
  // bigger than the core's 60-byte buffer (the 0x47 RAM write) work.
  usbMIDI.setHandleSystemExclusive(usb_sysex_partial);
#endif
#ifdef EMU_USB_DIAG
  Serial.begin(9600);                   // USB CDC telemetry (baud ignored)
  TCCR1A = 0; TCCR1B = (1 << CS11) | (1 << CS10);  // Timer1 free-run /64: 4 us/tick (bench clock)
#endif

#ifndef SUPEROS_COMBINED
  flash_persist_begin();   // combined: SuperOS mounts the arena; d650 uses EEPROM
#endif
  settings_load();
  d650_init(&H, tb303_rom, &DRV);
  patt_load_();   // restore pattern RAM (already nibble-packed in the host)
  emu_sync_init(&g_sync, &g_set, micros());
  settings_apply();
  g_last_us = micros();
}

#ifdef SUPEROS_COMBINED
// Internal-EEPROM persistence: patt_save_step() is a non-blocking byte
// sweeper. It scans from a cursor for the next byte that differs from EEPROM
// (bounded per call) and programs it only when the EEPROM engine is free, so
// the CPU never stalls: a byte program runs ~3.3 ms in hardware while the
// interpreter keeps running. The whole flash save policy (quiet gating, GC
// headroom) exists to hide multi-ms CPU halts; EEPROM has none, so the same
// call sites just run with zero spacing.
static uint16_t s_ee_cursor = 0;

static void patt_load_() {
  if (eeprom_read_byte(EE_EMU_MAGIC) != EE_EMU_MAGIC_VAL) return; // virgin: keep zeroed store
  eeprom_read_block(H.ext, EE_EMU_PATT, D650_EXT_BYTES);
}
static bool patt_save_step() {
  if (!eeprom_is_ready()) return false;
  uint8_t scanned = 0;
  while (s_ee_cursor < D650_EXT_BYTES && scanned < 64) {
    const uint8_t want = H.ext[s_ee_cursor];
    if (eeprom_read_byte(EE_EMU_PATT + s_ee_cursor) != want) {
      eeprom_write_byte(EE_EMU_PATT + s_ee_cursor, want);
      s_ee_cursor++;
      g_flash_writes++;      // telemetry: bytes here, pages in the flash build
      return true;
    }
    s_ee_cursor++; scanned++;
  }
  if (s_ee_cursor >= D650_EXT_BYTES) { s_ee_cursor = 0; s_save_pending = false; }
  return false;
}
#else
static void patt_load_() {
  for (uint8_t b = 0; b < PATT_BLK_N; b++)
    g_flash.read((uint8_t)(PATT_BLK_BASE + b), &H.ext[(uint16_t)b * PATT_BLK_LEN], PATT_BLK_LEN);
}
// One step of the deferred save: compare the cursor block against flash and
// write only if it differs, so the worst per-call stall is a single record
// program and unchanged blocks cost no flash wear. Returns true if a page
// was actually programmed (the caller spaces the next call accordingly).
static bool patt_save_step() {
  uint8_t tmp[PATT_BLK_LEN];
  const uint8_t id = (uint8_t)(PATT_BLK_BASE + s_save_cursor);
  const uint8_t *src = &H.ext[(uint16_t)s_save_cursor * PATT_BLK_LEN];
  bool wrote = false;
  if (g_flash.read(id, tmp, PATT_BLK_LEN) != PATT_BLK_LEN || memcmp(tmp, src, PATT_BLK_LEN) != 0) {
    g_flash.write(id, src, PATT_BLK_LEN);
    g_flash_writes++;
    wrote = true;
  }
  if (++s_save_cursor >= PATT_BLK_N) { s_save_cursor = 0; s_save_pending = false; }
  return wrote;
}
#endif

// External RUN line level: high while an external clock source's transport is
// running (or forced by the SysEx 0x49 test override).
static uint8_t run_line_level() {
  if (s_run_force != 2) return s_run_force;
  return (g_set.clock_source != EMU_CLK_INTERNAL && g_sync.run) ? 1 : 0;
}

void loop() {
  const uint32_t now = micros();

  // stall telemetry: gap since the previous pass entered
  static uint32_t s_prev_entry_us = 0;
  if (s_prev_entry_us) {
    const uint32_t gap = now - s_prev_entry_us;
    if (gap > g_max_gap_us) g_max_gap_us = gap;
  }
  s_prev_entry_us = now;

#ifdef EMU_USB_DIAG
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if      (ch == 'L') { dg_led_off = !dg_led_off; if (dg_led_off) Leds::PauseRefresh(); else Leds::ResumeRefresh(); Serial.print(F("# led_isr=")); Serial.println(!dg_led_off); }
    else if (ch == 'D') { dg_dac_off = !dg_dac_off; Serial.print(F("# dac=")); Serial.println(!dg_dac_off); }
    else if (ch == 'E') { dg_emu_off = !dg_emu_off; Serial.print(F("# emu=")); Serial.println(!dg_emu_off); }
    else if (ch == 'W' || ch == 'X' || ch == 'Z') {
      extern bool dg_watch; extern uint8_t dg_watch_mode;
      dg_watch_mode = (ch == 'Z') ? 2 : (ch == 'X') ? 1 : 0;
      dg_watch = !dg_watch;
      if (dg_watch) Leds::PauseRefresh(); else if (!dg_led_off) Leds::ResumeRefresh();
      Serial.print(F("# watch=")); Serial.print(dg_watch);
      Serial.print(F(" mode=")); Serial.println(dg_watch_mode);
    }
    else if (ch == 'R') {
      // Row-select candidate probe: hold PH2 low (matrix enabled), drive one
      // candidate output pin LOW at a time (others high), sample PA/PB.
      // Candidates: unused "memory" pins and mode-LED pins on the CPU board.
      Leds::PauseRefresh();
      struct { volatile uint8_t *port; uint8_t bit; const char *name; } cand[] = {
        { &PORTD, 0, "PD0(PE2)" }, { &PORTD, 1, "PD1(PE3)" },
        { &PORTD, 4, "PD4(ledT)" }, { &PORTD, 5, "PD5(ledA#)" },
        { &PORTD, 6, "PD6(ledP)" }, { &PORTD, 7, "PD7(ledF)" },
        { &PORTC, 6, "PC6(PF2)" }, { &PORTC, 7, "PC7(PF3)" },
        { &PORTE, 6, "PE6(acc)" }, { &PORTE, 7, "PE7(PE1)" },
      };
      uint8_t save_d = PORTD, save_c = PORTC, save_e = PORTE;
      for (int8_t k = -1; k < 10; k++) {
        // all candidates high, then one low
        PORTD |= 0xF3; PORTC |= 0xC0; PORTE |= 0xC0;
        if (k >= 0) *cand[k].port &= (uint8_t)~(1 << cand[k].bit);
        PORTF = 0x0B;                      // PH2 low, PH0/1/3 high, PG low
        delayMicroseconds(30);
        uint16_t hi[8] = {0};
        for (uint16_t n = 0; n < 200; n++) {
          uint8_t v = PINB;
          for (uint8_t b = 0; b < 8; b++) if (v & (1 << b)) hi[b]++;
          delayMicroseconds(100);
        }
        Serial.print(F("# cand "));
        Serial.print(k < 0 ? "none-low" : cand[k].name);
        Serial.print(F(" PA:"));
        for (uint8_t b = 0; b < 4; b++) { Serial.print(hi[b]); Serial.print(b < 3 ? ',' : ' '); }
        Serial.print(F("PB:"));
        for (uint8_t b = 4; b < 8; b++) { Serial.print(hi[b]); Serial.print(b < 7 ? ',' : ' '); }
        Serial.println();
      }
      PORTD = save_d; PORTC = save_c; PORTE = save_e; PORTF = 0x0F;
      if (!dg_led_off) Leds::ResumeRefresh();
      Serial.println(F("# cand done"));
    }
    else if (ch == 'M') {
      // Matrix truth table: for each select config, sample the raw PA/PB pins
      // (AVR PINB: low nibble = PA0-3, high nibble = PB0-3) 1000x and report
      // per-line high counts. PORTF low nibble = PH selects, high = PG (low).
      Leds::PauseRefresh();
      for (int cfg = 0; cfg < 5; cfg++) {
        PORTF = (cfg == 0) ? 0x0F : (0x0F & ~(1 << (cfg - 1)));
        delayMicroseconds(SWITCH_DELAY);
        uint16_t hi[8] = {0};
        for (uint16_t n = 0; n < 1000; n++) {
          uint8_t v = PINB;
          for (uint8_t b = 0; b < 8; b++) if (v & (1 << b)) hi[b]++;
          delayMicroseconds(37);   // ~37 ms window: spans tempo-clock cycles
        }
        Serial.print(cfg == 0 ? F("# allhigh ") : F("# row"));
        if (cfg) { Serial.print(cfg - 1); Serial.print(' '); }
        Serial.print(F("PA:"));
        for (uint8_t b = 0; b < 4; b++) { Serial.print(hi[b]); Serial.print(b < 3 ? ',' : ' '); }
        Serial.print(F("PB:"));
        for (uint8_t b = 4; b < 8; b++) { Serial.print(hi[b]); Serial.print(b < 7 ? ',' : ' '); }
        Serial.println();
      }
      PORTF = 0x0F;
      if (!dg_led_off) Leds::ResumeRefresh();
    }
  }
#endif

#ifdef EMU_USB_DIAG
  if (dg_watch) { dg_watch_scan(); return; }   // mapping mode: raw scans only
#endif

  // 1) I/O: MIDI in, DIN sync; panel matrix + LED frame at 2 ms, matching the
  // ROM's own 555 Hz input sampler so a fresh sample is available for nearly
  // every ISR tick. (Was 4 ms; the extra PollInputs busy-wait costs ~1.9% CPU,
  // paid from the ~9% real-time margin to halve press->LED latency.)
  midi_in_poll();
#ifdef SUPEROS_USB_MIDI
  usb_midi_poll();
#endif
  clock_in_poll();
  static uint32_t next_poll_us = 0;
  bool frame = false;
  if ((int32_t)(now - next_poll_us) >= 0) { next_poll_us = now + 2000; refresh_inputs(); frame = true; }

  // 2) clock router: CLOCK tempo line. (emu_sync still schedules its
  // wall-clock int_pending flag, but the AVR loop no longer consumes it: /INT
  // is delivered in the emulated-cycle domain inside the batch below. Desktop
  // tests keep using int_pending/int_edges from emu_sync unchanged.)
  emu_sync_service(&g_sync, now);
  H.in[D650_IN_CLOCK] = g_sync.clk_level;
  clock_out_service();                   // leader clock -> DIN (+USB) 0xF8

  // External transport -> RUN line level, applied between input frames too so
  // a Start/Stop reaches the ROM's 555 Hz sampler without frame latency.
  const uint32_t ms_now = millis();

  // MIDI transport liveness (MIDI source only; DIN run is a sampled level and
  // self-corrects). A Start whose master then vanishes without a Stop (cable
  // pulled, DAW closed) would hold the RUN line forced high forever with no
  // CLOCK pulses: sequencer frozen and the panel button locked out. Treat
  // 500 ms without a clock edge (12 clocks even at 30 BPM) as a Stop.
  if (g_set.clock_source == EMU_CLK_MIDI) {
    static uint32_t s_live_ms = 0, s_edge_mark = 0;
    static uint8_t  s_live_run_prev = 0;
    if (g_sync.run) {
      if (!s_live_run_prev || g_sync.clk_edges != s_edge_mark) s_live_ms = ms_now;
      if ((ms_now - s_live_ms) > 500) {
        emu_sync_transport(&g_sync, 0);
        emu_notes_out_all_off(&g_out);
      }
    }
    s_edge_mark = g_sync.clk_edges;
    s_live_run_prev = (uint8_t)g_sync.run;
  }

  if (!g_menu && run_line_level()) H.in[D650_IN_RUN] = 1;

  // 3) real-time cycle budget (skip while menu freezes the 303).
  //
  // Batch policy: keep every pass SHORT and drop deep backlog instead of
  // catching up. The old policy ran up to 4096 cycles in one pass to burn
  // down backlog; that batch takes 36-47 ms of wall clock, during which no
  // input frame or LED publish can run, and the pass itself re-creates the
  // next 36 ms of backlog — one overload spike (heavy editing hooks + MIDI
  // TX + the ~9% margin) snowballed into seconds of 40 ms input sampling.
  // Measured on hardware: caps=109, max_pass=47 ms, and fast taps (15-25 ms
  // press) vanishing between input frames. Catching up buys nothing
  // musically anyway: the ROM's sequencing follows the CLOCK edges (tempo
  // knob / MIDI clock), not the emulated cycle count, and the ROM's own main
  // loop is mostly idle scanning. So: run at most EMU_MAX_BATCH cycles per
  // pass (I/O cadence stays ~2 ms), carry at most EMU_MAX_BACKLOG, drop the
  // rest (g_caps counts drop events — nonzero means overload/stall happened).
  const uint32_t EMU_MAX_BATCH   = 384;    // ~3.4 ms emulated per pass
  const uint32_t EMU_MAX_BACKLOG = 1024;   // ~9 ms; beyond this, drop
  uint32_t dt = now - g_last_us; g_last_us = now;
  if (dt > 50000) dt = 50000;
  g_budget_q16 += dt * CYC_PER_US_Q16;
  uint32_t owed = g_budget_q16 >> 16;
  if (owed > EMU_MAX_BACKLOG) {
    g_caps++;
    g_budget_q16 -= (owed - EMU_MAX_BACKLOG) << 16;
    owed = EMU_MAX_BACKLOG;
  }
  if (owed > EMU_MAX_BATCH) owed = EMU_MAX_BATCH;
  g_budget_q16 -= owed << 16;              // consume what this pass will run

  if (!g_menu
#ifdef EMU_USB_DIAG
      && !dg_emu_off
#endif
     ) {
    uint32_t spent = 0;
    const uint32_t emu_t0 = micros();
#ifdef EMU_USB_DIAG
    extern uint32_t dg_emu_us, dg_steps, dg_loops;
    uint32_t t0 = micros(); dg_loops++;
#endif
    // Batched execution, chunked at the next /INT edge: the countdown bounds
    // each chunk so the ~555 Hz pulse lands on its exact emulated cycle (the
    // rising edge latches int_f immediately; pulsing high+low between chunks
    // suffices). Multiple edges per pass are delivered when owed spans them.
    while (spent < owed) {
      if (s_int_due_q8 <= 0) {
        d650_clock(&H, 1); d650_clock(&H, 0);
        g_int_edges_cyc++;
        s_int_due_q8 += (int32_t)INT_PERIOD_CYC_Q8;
      }
      uint32_t chunk = owed - spent;
      const uint32_t due = (uint32_t)(s_int_due_q8 + 255) >> 8;  // cycles to the edge
      if (chunk > due) chunk = due;
      const uint32_t ran = d650_run(&H, chunk);
      spent += ran;
      s_int_due_q8 -= (int32_t)(ran << 8);
    }
    // The final d650_run overshoots `owed` by up to one instruction (<= 3
    // cycles). Charge the overshoot to the budget: unaccounted it accumulated
    // to ~0.5% above the real 113.636 kHz machine-cycle rate.
    if (spent > owed) {
      const uint32_t oq = (spent - owed) << 16;
      g_budget_q16 = (g_budget_q16 > oq) ? (g_budget_q16 - oq) : 0;
    }
    g_emu_us_acc += micros() - emu_t0;
#ifdef EMU_USB_DIAG
    dg_steps += spent;   // now counts machine cycles, not instructions
    dg_emu_us += micros() - t0;
#endif
    // Live MIDI play overrides CV while the sequencer transport is stopped.
    // The gate must also be written LOW once when the live note ends:
    // resolve() returns 0 then, and skipping the write left the gate stuck
    // high forever, so no later note could retrigger the envelope (notes
    // just changed pitch and decayed away).
    static uint8_t s_live_gate = 0;
    if (!g_sync.run) {
      uint8_t p,g,a,sl;
      if (emu_live_resolve(&g_live, &p,&g,&a,&sl)) {
        DAC::SetPitch(p); DAC::SetGate(g); DAC::SetAccent(a); DAC::SetSlide(sl); DAC::Send();
        s_live_gate = 1;
      } else if (s_live_gate) {
        DAC::SetGate(0); DAC::SetSlide(0); DAC::Send();
        s_live_gate = 0;
      }
    } else if (s_live_gate) {
      // transport took over mid-note: drop the live voice, ROM drives the DAC
      g_live.active = 0;
      s_live_gate = 0;
    }
  }

  // 4) publish LEDs once per input frame (menu draws its own frame directly);
  // persist pattern RAM when changed and quiet
  if (frame) {
    if (g_menu) {                    // menu draws via Set/Swap into back[]:
      Leds::Swap();                  // invalidate the direct-publish cache
      g_led_pub[0] = g_led_pub[1] = g_led_pub[2] = 0xff;
    } else leds_publish();
  }

  // Deferred incremental pattern persistence: note RAM writes (restarting the
  // quiet timer and the scan cursor), then flush per the save policy (see the
  // comment at the s_save_* statics): stalls only when they are invisible.
  if (d650_dirty(&H)) {
    d650_clear_dirty(&H);
    s_save_pending = true;
    s_save_cursor  = 0;
#ifdef SUPEROS_COMBINED
    s_ee_cursor    = 0;
#endif
    s_ram_quiet_ms = ms_now;
  }
  const bool editing    = g_inputs[FUNCTION_KEY].held() || g_inputs[WRITE_MODE].held();
  const bool seq_active = (uint32_t)(ms_now - s_gate_edge_ms) < SAVE_ACT_MS;
  const bool ui_active  = (uint32_t)(ms_now - s_ui_ms)        < SAVE_ACT_MS;
  // A stall is invisible in the menu (303 frozen, RAM cannot change) or when
  // nothing has played, no live MIDI note sounds, and no hands are on the panel.
  const bool idle = g_menu || (!editing && !ui_active && !seq_active
                               && H.gate == 0 && !g_live.active);
  if ((int32_t)(ms_now - s_flush_hold_ms) >= 0) {
    if (s_save_pending) {
      if (idle && (g_menu || ms_now - s_ram_quiet_ms >= SAVE_QUIET_MS)) {
        if (patt_save_step()) s_flush_hold_ms = ms_now + SAVE_SPACING_MS;
      } else if (!g_menu && !editing && seq_active
                 && ms_now - s_ram_quiet_ms >= SAVE_TRICKLE_MS
                 && H.gate == 0 && (uint32_t)(ms_now - s_gate_fall_ms) < 40
#ifndef SUPEROS_COMBINED
                 && g_flash.free_pages() >= 2
#endif
                 ) {
        // Mid-play safety net for long unsaved sessions: one block right
        // after a gate fall (farthest from the next note), never into a GC.
        if (patt_save_step()) s_flush_hold_ms = ms_now + SAVE_TRICKLE_GAP_MS;
      }
    }
#ifndef SUPEROS_COMBINED
    else if (idle && !s_gc_dead && g_flash.free_pages() < SAVE_GC_HEADROOM) {
      // Pay the long GC stall now, while nobody is listening or editing, so
      // it can never land inside a save while the sequencer runs.
      if (!g_flash.maintain(SAVE_GC_HEADROOM)) s_gc_dead = true;
      s_flush_hold_ms = ms_now + 5000;
    }
#endif
  }

  // 1 Hz perf counters (always on; surfaced via SysEx status in production)
  g_pass_cnt++;
  if (ms_now - g_stat_ms >= 1000) {
    g_stat_ms = ms_now;
    g_cyc_per_s = H.cpu.cyc - g_cyc_mark;
    g_cyc_mark  = H.cpu.cyc;
    g_pass_per_s   = (g_pass_cnt > 16383) ? 16383 : (uint16_t)g_pass_cnt;
    g_emu_ms_per_s = (uint16_t)(g_emu_us_acc / 1000);
    g_pass_cnt = 0; g_emu_us_acc = 0;
    for (uint8_t i = 0; i < 4; i++) { g_sec_ms_per_s[i] = (uint16_t)(g_sec_us[i] / 1000); g_sec_us[i] = 0; }
  }

  // stall telemetry: how long this pass took
  const uint32_t pass_us = micros() - now;
  if (pass_us > g_max_pass_us) g_max_pass_us = pass_us;

#ifdef EMU_USB_DIAG
  // 1 Hz telemetry over USB serial. Expected on healthy hardware:
  //   cyc/s ~= 113636 (real-time pacing), int/s ~= 555 (ISR sampler),
  //   clk/s = bpm*24/60 rising edges (48 @ 120 BPM internal).
  static uint32_t dg_ms, dg_int, dg_clk, dg_cyc;
  uint32_t ms = millis();
  if (ms - dg_ms >= 1000) {
    dg_ms = ms;
    uint32_t ints = g_int_edges_cyc - dg_int;   dg_int = g_int_edges_cyc;   // DELIVERED edges
    uint32_t clks = g_sync.clk_edges - dg_clk;  dg_clk = g_sync.clk_edges;
    uint32_t cycs = H.cpu.cyc - dg_cyc;         dg_cyc = H.cpu.cyc;
    Serial.print(F("up=")); Serial.print(ms / 1000);
    Serial.print(F("s cyc/s=")); Serial.print(cycs);
    Serial.print(F(" int/s=")); Serial.print(ints);
    Serial.print(F(" clk/s=")); Serial.print(clks);
    Serial.print(F(" pc=0x")); Serial.print(H.cpu.pc, HEX);
    Serial.print(F(" gate=")); Serial.print(H.gate);
    Serial.print(F(" pitch=")); Serial.print(H.pitch);
    Serial.print(F(" acc=")); Serial.print(H.accent);
    Serial.print(F(" slide=")); Serial.print(H.slide);
    Serial.print(F(" gate_edges=")); Serial.print(g_gate_edges);
    Serial.print(F(" cv=")); Serial.print(dg_cv_commits);
    Serial.print(F(" dirty=")); Serial.print(s_save_pending);
    Serial.print(F(" menu=")); Serial.print(g_menu);
    Serial.print(F(" emu_us=")); Serial.print(dg_emu_us);
    Serial.print(F(" steps=")); Serial.print(dg_steps);
    Serial.print(F(" loops=")); Serial.print(dg_loops);
    Serial.print(F(" caps=")); Serial.print(g_caps);   // passes that hit the backlog cap (cumulative)
    dg_emu_us = dg_steps = dg_loops = 0;
    // pure interpreter cost, interrupts off: AVR cycles per emulated machine
    // cycle (real-time budget is 141; runs the machine ~1.1 ms ahead, diag only)
    if (!g_menu) {
      cli();
      uint16_t t0 = TCNT1;
      uint32_t bc = d650_run(&H, 128);
      uint16_t t1 = TCNT1;
      sei();
      Serial.print(F(" bench_avrcyc_per_mcyc="));
      Serial.print((uint32_t)((uint16_t)(t1 - t0)) * 64u / bc);
    }
    Serial.print(F(" keys="));
    for (uint8_t i = 0; i < D650_IN_COUNT; i++)
      if (H.in[i]) { Serial.print(i); Serial.print(','); }
    Serial.println();
  }
#endif
}
