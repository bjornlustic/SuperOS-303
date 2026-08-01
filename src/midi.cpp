/*
 * SuperOS-303 pattern SysEx (host ↔ DIN MIDI)
 * API: midi_api.h — implementation in this file.
 *
 * Framing: F0 7D <cmd> ... F7   (manufacturer ID 0x7D non-commercial)
 *
 * Pattern commands:
 *   10h  Host→303  Request one pattern: <pat:0..15>
 *   11h  303→Host  Pattern data: <pat> <xor_lo7> <xor_hi1> <packed pattern> <var>
 *   12h  Host→303  Set pattern: same as 11h body; XOR over the raw blob must match.
 *   13h  Host→303  Request all 16 patterns (303 sends sixteen 11h messages, queued).
 *   14h  303→Host  ACK/NAK: <status> 0=ok 1=bad_checksum 2=bad_pattern
 *   15h  303→Host  Step position: <pat:0..15> <step> <group>  (sent at pattern wrap)
 *   19h  Host→303  Step lock: <pat:0..15> <step> <locked:0|1>  (RAM only)
 *
 * Config commands:
 *   20h  Host→303  Request config
 *   21h  303→Host  Config data: <midi_ch:0..16> <flags: bit0=clock_receive>
 *   22h  Host→303  Set config: same as 21h body
 *
 * Slide fix: note_off_cb tracks the most-recent live note; only clears s_live_gate when
 * the Note Off matches that note. This prevents releasing note 1 from killing note 2
 * during a live slide (two-finger legato play).
 */

#include <Arduino.h>
#include <MIDI.h>
#include <string.h>
#include <util/delay.h>
#include "engine.h"
#include "pins.h"
#include "midi_api.h"
#ifdef SUPEROS_COMBINED
#include <avr/eeprom.h>
#include "combined.h"
#endif

struct SuperOsMidiSettings {
  // Running status compresses back-to-back same-channel messages (e.g. the variation-3
  // poly chord: 4 note-ons on one channel -> status byte only on the first), tightening
  // onset spread. Standard MIDI; set false if a receiver mishandles it.
  static const bool UseRunningStatus = true;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  // Largest inbound message is the 0x26 poly-blob set: F0 + 5 header + packed
  // blob + F7 (the library buffers F0..F7 inclusive). Derived so the buffer
  // tracks POLY_BLOB_SIZE (139 bytes at 32 steps).
  static const unsigned SysExMaxSize =
      5 + (POLY_BLOB_SIZE + (POLY_BLOB_SIZE + 6) / 7) + 2;
  static const bool UseSenderActiveSensing = false;
  static const bool UseReceiverActiveSensing = false;
  static const uint16_t SenderActiveSensingPeriodicity = 0;
};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, SuperOsMidiSettings);

// --- Dual MIDI output (DIN + USB) -----------------------------------------------
// Every performance message (notes, clock, transport) is sent to DIN (Serial1) and,
// when built with USB MIDI, mirrored to the USB-MIDI port so a host can record the
// 303 and play it from USB. All MIDI.sendNoteOn/Off/Clock/Start/Stop/Continue call
// sites below route through these. SysEx (the web-editor protocol) stays DIN-only
// for now — USB SysEx is Phase 2.
// --- Per-tick note batching -------------------------------------------------------
// Between midi_tick_begin() and midi_tick_flush() (the sequencer gate-ticks), note
// on/offs are collected instead of sent. The flush then writes USB first -- all of
// the tick's events back-to-back plus send_now(), so they leave in ONE USB packet
// and every voice lands on the same host timestamp -- then DIN as one contiguous
// burst (ons first, offs after). Sending DIN inline per note (the old path) blocked
// 320us/byte between USB writes whenever the UART FIFO was congested, splitting the
// tick's USB events across 1ms frames: that was the audible per-note delay.
// Outside a batch (live play, audition, metronome, stop paths) sends are immediate.
static bool    s_tick_batch = false;
static uint8_t s_on_note[8], s_on_vel[8], s_on_ch[8], s_on_n = 0;
static uint8_t s_off_note[12], s_off_ch[12], s_off_n = 0;

static void out_note_on_now(byte n, byte v, byte ch) {
  MIDI.sendNoteOn(n, v, ch);
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendNoteOn(n, v, ch);
#endif
}
static void out_note_off_now(byte n, byte v, byte ch) {
  MIDI.sendNoteOff(n, v, ch);
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendNoteOff(n, v, ch);
#endif
}
static void queue_off(byte note, byte ch) {
  if (s_off_n < sizeof(s_off_note)) { s_off_note[s_off_n] = note; s_off_ch[s_off_n] = ch; ++s_off_n; }
  else out_note_off_now(note, 0, ch); // overflow safety
}
static inline void out_note_on(byte n, byte v, byte ch) {
  if (s_tick_batch && s_on_n < sizeof(s_on_note)) {
    s_on_note[s_on_n] = n; s_on_vel[s_on_n] = v; s_on_ch[s_on_n] = ch; ++s_on_n;
    return;
  }
  out_note_on_now(n, v, ch);
}
static inline void out_note_off(byte n, byte v, byte ch) {
  if (s_tick_batch) { queue_off(n, ch); return; }
  out_note_off_now(n, v, ch);
}
static inline void out_clock() {
  MIDI.sendClock();
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendRealTime(usbMIDI.Clock);
#endif
}
static inline void out_start() {
  MIDI.sendStart();
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendRealTime(usbMIDI.Start);
#endif
}
static inline void out_stop() {
  MIDI.sendStop();
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
}
static inline void out_continue() {
  MIDI.sendContinue();
#ifdef SUPEROS_USB_MIDI
  usbMIDI.sendRealTime(usbMIDI.Continue);
#endif
}

static Engine *g_eng = nullptr;
static bool g_clk_run = false;
static uint8_t s_in_channel = 0; // 0 = omni
static bool s_midi_clock_rx = true;
static bool s_midi_thru = false;
// Deferred settings persistence: set by the 0x22 handler, flushed by
// midi_flush_pending_saves() so EEPROM writes don't stall RX inside SysEx parsing.
static bool s_settings_dirty = false;

// Per-pattern dirty bitmap for deferred pattern-EEPROM persistence. Any
// SysEx that mutates pattern RAM (0x12, 0x16, 0x18, 0x19, 0x1B) sets the
// corresponding bit. midi_flush_pending_pattern_saves() writes one pattern
// per call when idle so the EEPROM stall stays short.
static uint16_t s_pat_dirty_mask = 0;
static uint32_t s_last_web_edit_ms = 0;
static inline void mark_pat_dirty(uint8_t pat) {
  s_pat_dirty_mask |= uint16_t(1u << (pat & 0x0f));
  s_last_web_edit_ms = millis();
}

static uint8_t out_ch() {
  return s_in_channel == 0 ? 1 : s_in_channel;
}

// Per-variation MIDI output channel. Variation 0 (index sentinel 0) routes
// through out_ch() so omni/in-channel behaviour is preserved; variations 1/2
// use the stored var2/var3 channels.
static uint8_t s_var_channel[NUM_VARIATIONS] = {0, 2, 3};
static uint8_t out_ch_for_var(uint8_t var) {
  return (var >= NUM_VARIATIONS || var == 0 || s_var_channel[var] == 0)
         ? out_ch() : s_var_channel[var];
}
void midi_set_var_channels(uint8_t v2, uint8_t v3) {
  s_var_channel[1] = (v2 >= 1 && v2 <= 16) ? v2 : 2;
  s_var_channel[2] = (v3 >= 1 && v3 <= 16) ? v3 : 3;
}

void midi_apply_settings(uint8_t midi_in_channel_0_omni_16, bool midi_clock_receive, bool midi_thru) {
  s_in_channel = midi_in_channel_0_omni_16 <= 16 ? midi_in_channel_0_omni_16 : 1;
  s_midi_clock_rx = midi_clock_receive;
  s_midi_thru = midi_thru;
  if (s_in_channel == 0)
    MIDI.begin(MIDI_CHANNEL_OMNI);
  else
    MIDI.begin(s_in_channel);
  if (s_midi_thru)
    MIDI.turnThruOn();
  else
    MIDI.turnThruOff();
}

// --- TX queue (non-blocking multi-pattern dump) ---------------------------------
static const uint16_t kTxCap = 512;
#ifdef SUPEROS_COMBINED
// This ring is SuperOS-only (the d650 side has its own midi_tx straight to
// Serial1), so it lives in the arena tail instead of its own BSS -- see
// FW_ARENA_SUPEROS_TAIL in combined.h. Zeroed at boot with the rest of the
// arena, and tx_clear() resets the indices before any use.
static_assert(kTxCap <= FW_ARENA_SUPEROS_TAIL, "TX ring exceeds the arena tail");
static uint8_t *const s_tx = g_fw_arena + sizeof(Engine);
#else
static uint8_t s_tx[kTxCap];
#endif
static uint16_t s_tx_w, s_tx_r;

static void tx_clear() {
  s_tx_w = s_tx_r = 0;
}

static bool tx_push_byte(uint8_t b) {
  uint16_t n = uint16_t(s_tx_w + 1);
  if (n >= kTxCap) n = 0;
  if (n == s_tx_r) return false;
  s_tx[s_tx_w] = b;
  s_tx_w = n;
  return true;
}

static void midi_tx_drain() {
  while (s_tx_r != s_tx_w && Serial1.availableForWrite() > 0) {
    Serial1.write(s_tx[s_tx_r]);
    s_tx_r++;
    if (s_tx_r >= kTxCap) s_tx_r = 0;
  }
}

// Per-transport host presence: set when a SysEx arrives on that transport (the web
// editor sends 0x20 on connect). Telemetry/replies go only to transports where an
// editor has actually talked to us. Without this, standalone playback queued the
// editor broadcasts into the DIN ring anyway -- in Track mode ~210 bytes (0x23 +
// 0x15) per pattern wrap = ~70ms of 31250-baud wire time -- filling the Serial1 TX
// FIFO ahead of note bytes, so notes queued behind SysEx and their sends blocked.
static bool s_din_host_seen = false;
static bool s_usb_host_seen = false;
// No editor has ever talked to us on any transport: tx_push_message would discard
// the message anyway, so heavy builders (pattern pack, 202-byte track state) can
// skip the serialize/pack work entirely during standalone playback.
static inline bool host_seen_any() { return s_din_host_seen || s_usb_host_seen; }

static bool tx_push_message(const uint8_t *inner, uint16_t inner_len) {
#ifdef SUPEROS_USB_MIDI
  // USB before the DIN ring, and never gated on ring space: the web editor (USB
  // host) handshakes far faster than the 31250-baud Serial1 ring drains, so the
  // DIN ring fills after a handful of replies. Gating USB on ring space dropped
  // replies mid-dump (the "stalls at ~6/48" bug). hasTerm=false -> the core wraps
  // F0..F7 around `inner`. Non-blocking / dropped when no USB host is configured.
  if (s_usb_host_seen)
    usbMIDI.sendSysEx(inner_len, inner, false);
#endif
  if (!s_din_host_seen) return true;
  // DIN (Serial1) ring: check space first so we never leave a partial F0..no-F7.
  uint16_t avail = (s_tx_r + kTxCap - s_tx_w - 1) % kTxCap;
  if (avail < inner_len + 2) return false; // ring full -> DIN drops it (USB already sent)
  tx_push_byte(0xF0);
  for (uint16_t i = 0; i < inner_len; ++i) tx_push_byte(inner[i]);
  tx_push_byte(0xF7);
  return true;
}

// --- 7-bit pack / unpack (PATTERN_SIZE raw -> kPackedPatternLen packed) -------
static uint16_t pack_7bit(const uint8_t *src, uint16_t len, uint8_t *out) {
  uint16_t o = 0;
  for (uint16_t i = 0; i < len; i += 7) {
    uint8_t msb = 0;
    const uint8_t n = (len - i >= 7) ? 7 : static_cast<uint8_t>(len - i);
    for (uint8_t b = 0; b < n; ++b)
      if (src[i + b] & 0x80) msb |= static_cast<uint8_t>(1u << b);
    out[o++] = msb;
    for (uint8_t b = 0; b < n; ++b)
      out[o++] = src[i + b] & 0x7F;
  }
  return o;
}

// 7-bit packing inflation: every 7 raw bytes -> 1 MSB byte + 7 data bytes.
// 92 raw -> 13 full chunks (104 packed bytes) + 1 partial (1 + 1 = 2) = 106.
static constexpr uint16_t kPackedPatternLen =
    PATTERN_SIZE + ((PATTERN_SIZE + 6) / 7);

static bool unpack_7bit(const uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t out_len) {
  uint16_t oi = 0, ii = 0;
  while (oi < out_len) {
    if (ii >= in_len) return false;
    const uint8_t msb = in[ii++];
    for (uint8_t b = 0; b < 7 && oi < out_len; ++b) {
      if (ii >= in_len) return false;
      out[oi] = static_cast<uint8_t>(in[ii++] | ((msb & (1u << b)) ? 0x80u : 0u));
      ++oi;
    }
  }
  return true;
}

static uint8_t xor_blob_n(const uint8_t *p, uint16_t len) {
  uint8_t x = 0;
  for (uint16_t i = 0; i < len; ++i) x ^= p[i];
  return x;
}

// 7-bit packed length of a variation-3 poly blob (227 raw -> 260 packed).
static constexpr uint16_t kPackedPolyLen = POLY_BLOB_SIZE + ((POLY_BLOB_SIZE + 6) / 7);

// 0x25: variation-3 poly blob reply (device -> host). Sent in place of the mono
// 0x11 when the requested slot's variation 3 is poly.
// noinline is load-bearing, not a hint: this frames ~492 bytes of buffers (a
// 227-byte raw blob + a 265-byte packed one). Inlined into
// enqueue_pattern_reply it pushed that frame to 793 bytes, so EVERY pattern
// reply paid the poly cost and handle_sysex_body -> enqueue_pattern_reply
// needed 1041 bytes of stack -- more than the RAM left after the arena. That
// overflowed into BSS and hung the SuperOS side the moment the editor loaded
// patterns. Keeping it out of line means only the poly path pays.
static __attribute__((noinline)) void enqueue_poly_reply(uint8_t pat, uint8_t slot) {
  if (!host_seen_any()) return;
  PolyVoice pv;
  if (g_eng->poly_active_ && slot == g_eng->abs_slot(g_eng->get_patsel()))
    pv = g_eng->poly_;                 // resident (possibly edited) copy
  else if (g_eng->poly_edit_dirty_ && g_eng->poly_edit_slot_ == int16_t(slot))
    pv = g_eng->poly_edit_;            // buffered (not yet flushed) non-resident edit
  else
    ReadPolyAt(pv, slot);
  uint8_t raw[POLY_BLOB_SIZE];
  pv.serialize(raw);
  const uint8_t cx = xor_blob_n(raw, POLY_BLOB_SIZE);
  uint8_t inner[5 + kPackedPolyLen];
  inner[0] = 0x7D; inner[1] = 0x25; inner[2] = uint8_t(pat & 0x0F);
  inner[3] = static_cast<uint8_t>(cx & 0x7F);
  inner[4] = static_cast<uint8_t>((cx >> 7) & 1);
  const uint16_t pl = pack_7bit(raw, POLY_BLOB_SIZE, inner + 5);
  tx_push_message(inner, static_cast<uint16_t>(5 + pl));
}

static void enqueue_pattern_reply(uint8_t pat, uint8_t var = 0) {
  if (!g_eng || !host_seen_any()) return;
  if (var == 2 && GlobalSettings.var3_is_poly(g_eng->abs_slot(uint8_t(pat & 0x0F)))) {
    enqueue_poly_reply(uint8_t(pat & 0x0F), g_eng->abs_slot(uint8_t(pat & 0x0F)));
    return;
  }
  uint8_t raw[PATTERN_SIZE];
  g_eng->export_pattern_blob_var(pat, var, raw);
  const uint8_t cx = xor_blob_n(raw, PATTERN_SIZE);
  uint8_t inner[5 + kPackedPatternLen + 1];
  inner[0] = 0x7D;
  inner[1] = 0x11;
  inner[2] = pat & 0x0F;
  inner[3] = static_cast<uint8_t>(cx & 0x7F);
  inner[4] = static_cast<uint8_t>((cx >> 7) & 1);
  const uint16_t pl = pack_7bit(raw, PATTERN_SIZE, inner + 5);
  inner[5 + pl] = static_cast<uint8_t>(var & 0x03); // trailing variation byte
  tx_push_message(inner, static_cast<uint16_t>(5 + pl + 1));
}

static void send_ack(uint8_t status) {
  const uint8_t inner[3] = {0x7D, 0x14, status};
  tx_push_message(inner, 3);
}

// --- Per-step probability table (SysEx 0x2C request / 0x2D reply) ---------------
// Variation 1 only: 3 bytes/step = 192 raw -> packed via pack_7bit.
static constexpr uint16_t kProbRawLen    = 3 * MAX_STEPS;
static constexpr uint16_t kPackedProbLen = kProbRawLen + ((kProbRawLen + 6) / 7);

static void enqueue_prob_reply(uint8_t pat) {
  if (!g_eng || !host_seen_any()) return;
  uint8_t raw[kProbRawLen];
  g_eng->export_prob_table(pat, raw);
  const uint8_t cx = xor_blob_n(raw, kProbRawLen);
  uint8_t inner[5 + kPackedProbLen];
  inner[0] = 0x7D;
  inner[1] = 0x2D;
  inner[2] = pat & 0x0F;
  inner[3] = static_cast<uint8_t>(cx & 0x7F);
  inner[4] = static_cast<uint8_t>((cx >> 7) & 1);
  const uint16_t pl = pack_7bit(raw, kProbRawLen, inner + 5);
  tx_push_message(inner, static_cast<uint16_t>(5 + pl));
}

// Device -> host full probability-table push (0x2D), used after a hardware
// randomize so the web editor's PROB lane resyncs in one message instead of
// 64 per-step 0x2B broadcasts flooding the TX ring. The editor's 0x2D handler
// just stores + repaints, so an unsolicited push is safe (no request tracker).
void midi_send_prob_table(uint8_t pat) { enqueue_prob_reply(pat); }

// --- Chain state RX (from web via SysEx 0x1A) -----------------------------------
static bool    s_rx_chain_pending     = false;
static uint8_t s_rx_chain_active_len  = 0;
static uint8_t s_rx_chain_active_pats[4] = {};
static uint8_t s_rx_chain_queued_len  = 0;
static uint8_t s_rx_chain_queued_pats[4] = {};

void midi_send_chain_state(uint8_t active_len, const uint8_t *active_pats,
                            uint8_t queued_len, const uint8_t *queued_pats) {
  uint8_t inner[12];
  inner[0] = 0x7D; inner[1] = 0x1A;
  inner[2] = active_len & 0x07;
  for (uint8_t i = 0; i < 4; ++i)
    inner[3 + i] = (active_pats && i < active_len) ? (active_pats[i] & 0x0F) : 0;
  inner[7] = queued_len & 0x07;
  for (uint8_t i = 0; i < 4; ++i)
    inner[8 + i] = (queued_pats && i < queued_len) ? (queued_pats[i] & 0x0F) : 0;
  tx_push_message(inner, 12);
}

bool midi_get_received_chain(uint8_t *out_active_len, uint8_t out_active_pats[4],
                              uint8_t *out_queued_len, uint8_t out_queued_pats[4]) {
  if (!s_rx_chain_pending) return false;
  s_rx_chain_pending = false;
  if (s_rx_chain_active_len == 0xff) {
    // Sentinel: config request asked us to re-broadcast current state.
    // Signal main.cpp with active_len=0xff so it just calls emit_chain_state().
    *out_active_len = 0xff;
    *out_queued_len = 0;
    return true;
  }
  *out_active_len = s_rx_chain_active_len;
  for (uint8_t i = 0; i < 4; ++i) out_active_pats[i] = s_rx_chain_active_pats[i];
  *out_queued_len = s_rx_chain_queued_len;
  for (uint8_t i = 0; i < 4; ++i) out_queued_pats[i] = s_rx_chain_queued_pats[i];
  return true;
}

// --- Sequential dump after 0x13 -------------------------------------------------
static bool s_dump_active = false;
static uint8_t s_dump_next = 0;

static void dump_try_advance() {
  if (!s_dump_active || s_tx_r != s_tx_w) return;
  if (s_dump_next >= 16) { s_dump_active = false; return; }
  enqueue_pattern_reply(s_dump_next++);
}

// --- Config reply (0x21) --------------------------------------------------------
// Fixed 6 config bytes, then the firmware version as ASCII (7-bit safe). Older
// hosts read fixed offsets and ignore the trailing bytes.
#ifndef SUPEROS_VERSION
#define SUPEROS_VERSION "?"
#endif
static void send_config_reply() {
  static const char kVer[] = SUPEROS_VERSION;
  const uint8_t fl  = static_cast<uint8_t>((GlobalSettings.midi_clock_receive ? 1 : 0) |
                                            (GlobalSettings.midi_thru          ? 2 : 0));
  const uint8_t dir = g_eng ? static_cast<uint8_t>(g_eng->get_direction()) : 0;
  uint8_t inner[8 + sizeof(kVer) - 1];
  inner[0] = 0x7D; inner[1] = 0x21;
  inner[2] = GlobalSettings.midi_channel;
  inner[3] = fl;  inner[4] = dir;
  inner[5] = GlobalSettings.led_brightness;
  inner[6] = GlobalSettings.var2_channel;
  inner[7] = GlobalSettings.var3_channel;
  for (uint8_t i = 0; i < sizeof(kVer) - 1; ++i)
    inner[8 + i] = static_cast<uint8_t>(kVer[i] & 0x7F);
  tx_push_message(inner, sizeof(inner));
}

// D650C mask-ROM status for the web editor (SysEx 0x36 -> 0x37). The upload
// itself is handled only by the emulator (emu_avr.cpp); this lets the editor
// show what is installed while the SuperOS side is running. Streams the EEPROM
// sum on demand so no 2 KB RAM buffer is needed here.
//   0 = no ROM installed (D650_ROM_IN_RAM, EEPROM empty)
//   1 = valid user upload in EEPROM
//   2 = ROM embedded in flash (plain combined, or D650_ROM_EMBEDDED fallback)
//   0x7F = no emulator (app-only build)
static uint8_t rom_status() {
#ifdef SUPEROS_COMBINED
#ifdef D650_ROM_IN_RAM
  if (eeprom_read_byte(EE_ROM_MAGIC) == EE_ROM_MAGIC_VAL) {
    uint16_t s = 0;
    for (uint16_t i = 0; i < 2048; ++i) s = (uint16_t)(s + eeprom_read_byte(EE_ROM_DATA + i));
    const uint16_t want = (uint16_t)eeprom_read_byte(EE_ROM_SUM)
                        | ((uint16_t)eeprom_read_byte(EE_ROM_SUM + 1) << 8);
    if (s == want) return 1;
  }
#ifdef D650_ROM_EMBEDDED
  return 2;
#else
  return 0;
#endif
#else
  return 2;   // plain combined: ROM is always embedded in program flash
#endif
#else
  return 0x7F;
#endif
}
static void midi_send_rom_status() {
  const uint8_t inner[3] = {0x7D, 0x37, rom_status()};
  tx_push_message(inner, 3);
}

// --- SysEx parse ----------------------------------------------------------------
static void handle_sysex_body(const uint8_t *p, unsigned n) {
  if (!g_eng || n < 2) return;
  if (p[0] != 0x7D) return;
  const uint8_t cmd = p[1];

  switch (cmd) {
  case 0x36: // web editor: D650C mask-ROM status query -> 0x37 reply
    midi_send_rom_status();
    break;
  case 0x34: { // panel diagnostic -> 0x35: debounced held-state of every input
    // (7 inputs per byte, InputIndex order) + the derived DialMode. Lets a
    // host see the mode dial, group dial and modifier keys remotely.
    extern PinState inputs[INPUT_COUNT];
    uint8_t r[3 + (INPUT_COUNT + 6) / 7];
    r[0] = 0x7D; r[1] = 0x35;
    memset(r + 3, 0, sizeof(r) - 3);
    for (uint8_t i = 0; i < INPUT_COUNT; ++i)
      if (inputs[i].held()) r[3 + i / 7] |= uint8_t(1 << (i % 7));
    r[2] = uint8_t(dial_mode_of(read_input_state(inputs)));
    tx_push_message(r, sizeof(r));
    break;
  }
  case 0x10: { // request pattern; optional trailing <var> selects the variation
    if (n < 3) return;
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t var = (n >= 4) ? (p[3] & 0x03) : 0;
    enqueue_pattern_reply(pat, var);
    break;
  }
  case 0x12: { // set pattern. Optional trailing byte = variation (0=var1/default,
               // 1=var2, 2=var3). var1 -> live RAM; var2/3 -> that slot's flash half.
    if (n < 5 + kPackedPatternLen) { send_ack(1); return; }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t cx = static_cast<uint8_t>(p[3] | (p[4] << 7));
    const uint8_t var = (n > 5 + kPackedPatternLen) ? (p[5 + kPackedPatternLen] & 0x03) : 0;
    uint8_t raw[PATTERN_SIZE];
    if (!unpack_7bit(p + 5, kPackedPatternLen, raw, PATTERN_SIZE)) { send_ack(1); return; }
    if (xor_blob_n(raw, PATTERN_SIZE) != cx) { send_ack(1); return; }
    if (var == 0) {
      // Variation 1 (the 303 / CV voice). Never persist inline: a flash write
      // blocks the UART long enough to drop the next SysEx. RAM update only;
      // engine.stale persists at the natural save points (RUN stop, WRITE exit).
      if (!g_eng->import_pattern_blob(pat, raw, /*persist_eeprom=*/false)) { send_ack(2); return; }
      // Apply the blob's per-pattern direction to the live engine if active.
      if (pat == g_eng->get_patsel()) {
        const uint8_t d = g_eng->pattern[pat].get_direction_stored();
        g_eng->SetDirection(static_cast<SequenceDirection>(d));
      }
      g_eng->stale = true;
      mark_pat_dirty(pat);
    } else if (var < NUM_VARIATIONS) {
      // Variations 2/3 are MIDI-only shadow voices. For the active slot, edit the
      // resident shadow in RAM (no flash write -> no sequencer stall); it persists
      // on stop/switch. For a non-active slot (not resident), buffer in RAM and
      // defer the flash write (idle flush / persist_shadows) -- an inline page
      // write halts the CPU long enough to drop the next SysEx, and the editor's
      // live-edit debounce would burn a flash write per edit.
      if (!g_eng->apply_shadow_blob(pat, var, raw)) {
        const uint8_t L = raw[PATTERN_SIZE - 1];
        if (L < 1 || L > MAX_STEPS) { send_ack(2); return; }
        g_eng->shadow_edit_blob(g_eng->abs_slot(pat), var, raw);
      }
      s_last_web_edit_ms = millis(); // arm the idle-flush quiet timer for shadows
    }
    send_ack(0);
    break;
  }
  case 0x13: // request all
    s_dump_next = 0;
    s_dump_active = true;
    dump_try_advance();
    break;
  case 0x16: { // host → 303: set single step (pitch + time); optional trailing <var>
    if (n < 7 || !g_eng) return;
    const uint8_t pat  = p[2] & 0x0F;
    const uint8_t step = p[3] & 0x3F;
    const uint8_t pitchByte = static_cast<uint8_t>((p[4] & 0x7F) | ((p[5] & 0x01) << 7));
    const uint8_t timeNib   = p[6] & 0x0F;
    const uint8_t var = (n >= 8) ? (p[7] & 0x03) : 0; // 0=var1, 1/2=var2/var3 shadow
    // Variations 2/3 are MIDI-only shadow voices. Per-step edits apply only to the
    // RESIDENT (active-slot) shadow in RAM -- no flash, no full-blob stall -- which is
    // what fixes the var2 note/rest lag. The web only sends a per-step var>0 message
    // for the live resident slot; anything else still goes via the full 0x12 blob.
    Sequence *seqp;
    if (var == 0) {
      seqp = &g_eng->pattern[pat];
    } else if (var < NUM_VARIATIONS && (pat & uint8_t(NUM_PATTERNS - 1)) == g_eng->get_patsel()) {
      seqp = &g_eng->shadow_[var - 1];
    } else {
      return; // non-resident shadow per-step not supported (web sends the full blob)
    }
    Sequence &seq = *seqp;
    if (step >= seq.length) return;
    sequence_write_time_with_pitch_sync(seq, step, timeNib);
    // pitchByte == PITCH_EMPTY (0xFF) is the editor's "time-only edit"
    // sentinel: leave the pitch stream alone (auto-extension already ran).
    if (timeNib == 1 && pitchByte != PITCH_EMPTY) {
      const uint8_t slot = seq.pitch_index_for_note(step);
      if (slot < seq.get_pitch_count())
        seq.pitch[slot] = pitchByte;
    }
    if (var == 0) {
      g_eng->stale = true;
      mark_pat_dirty(pat);
    } else {
      g_eng->shadow_notecount_[var - 1] = seq.note_count(); // keep the gate-follow in sync
      g_eng->shadow_stale_ = true;
      g_eng->shadow_dirty_ms_ = millis();
    }
    break;
  }
  case 0x18: { // host → 303: set pattern length
    if (n < 4 || !g_eng) return;
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t len = p[3] & 0x7F;
    if (len < 1 || len > MAX_STEPS) return;
    // Same invariants as the hardware editor: triplet cap, pitch_count rebuild,
    // pitch[] tail clear (a raw length assignment left pitch_count stale).
    g_eng->ApplyLength(g_eng->pattern[pat], len);
    mark_pat_dirty(pat);
    break;
  }
  case 0x19: { // host → 303: set step lock (RAM-only since OS-303 v0.6 layout)
    if (n < 5 || !g_eng) return;
    const uint8_t pat    = p[2] & 0x0F;
    const uint8_t step   = p[3] & 0x3F;
    const bool    locked = (p[4] & 0x01) != 0;
    Sequence &seq = g_eng->pattern[pat];
    if (step >= MAX_STEPS) return;
    const uint8_t mask = uint8_t(1u << (step & 7));
    if (locked)
      seq.step_lock_ram[step >> 3] |= mask;
    else
      seq.step_lock_ram[step >> 3] &= ~mask;
    // step_lock is RAM-only in the OS-303 layout: do not mark stale.
    break;
  }
  case 0x1A: { // set chain state from host
    if (n < 12) return;
    s_rx_chain_active_len = p[2] & 0x07;
    if (s_rx_chain_active_len > 4) s_rx_chain_active_len = 4;
    for (uint8_t i = 0; i < 4; ++i)
      s_rx_chain_active_pats[i] = p[3 + i] & 0x0F;
    s_rx_chain_queued_len = p[7] & 0x07;
    if (s_rx_chain_queued_len > 4) s_rx_chain_queued_len = 4;
    for (uint8_t i = 0; i < 4; ++i)
      s_rx_chain_queued_pats[i] = p[8 + i] & 0x0F;
    s_rx_chain_pending = true;
    break;
  }
  case 0x1D: { // host → 303: queue pattern; immediate when stopped
    if (n < 3 || !g_eng) return;
    g_eng->SetPattern(p[2] & 0x0F, !g_clk_run);
    break;
  }
  case 0x1F: { // host → 303: set the hardware edit-target variation (0..2)
    if (n < 4 || !g_eng) return;
    g_eng->SetEditVar(p[3] & 0x03);
    break;
  }
  case 0x26: { // host -> 303: set variation-3 poly blob (implies poly)
    if (n < 5u + kPackedPolyLen) { send_ack(1); return; }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t cx  = static_cast<uint8_t>(p[3] | (p[4] << 7));
    uint8_t raw[POLY_BLOB_SIZE];
    if (!unpack_7bit(p + 5, kPackedPolyLen, raw, POLY_BLOB_SIZE)) { send_ack(1); return; }
    if (xor_blob_n(raw, POLY_BLOB_SIZE) != cx) { send_ack(1); return; }
    const uint8_t slot = g_eng->abs_slot(pat);
    g_eng->drop_poly_edit(slot);              // this full blob supersedes buffered per-step edits
    const bool was_poly = GlobalSettings.var3_is_poly(slot);
    if (!was_poly) {                          // only flash settings when the flag CHANGES;
      GlobalSettings.set_var3_poly(slot, true); // every chord edit hits an already-poly slot,
      s_settings_dirty = true;                 // so this avoids a settings flash write per edit
    }
    if (g_eng->poly_active_ && slot == g_eng->abs_slot(g_eng->get_patsel())) {
      g_eng->poly_.deserialize(raw);
      g_eng->poly_.ensure_chords_for_notes();
      g_eng->poly_stale_ = true;
    } else {
      // Non-resident: buffer in RAM (same deferred path as per-step 0x27 edits)
      // instead of an inline flash write that stalls RX mid-SysEx-stream.
      g_eng->poly_edit_blob(slot, raw);
      if (!was_poly && !g_clk_run && pat == g_eng->get_patsel()) {
        g_eng->persist_shadows();      // flushes the buffer to flash first
        g_eng->ReloadShadows();        // active slot just became poly -> play it
      }
    }
    s_last_web_edit_ms = millis();
    send_ack(0);
    break;
  }
  case 0x27: { // host -> 303: set ONE variation-3 poly step (efficient single-step edit)
    // Payload: <pat> <step> <n0> <n1> <n2> <n3> <time> <flags>
    // n0..n3 = 6-bit packed voices (POLY_EMPTY = empty); flags bit0=accent bit1=slide.
    // Wire is STEP-indexed; the firmware stores into the chord stream at the step's
    // note-index (set_step), so chords pull from a list like var1/2. Avoids resending
    // the whole 227-byte blob (a ~85ms wire stall) for each note change.
    if (n < 10) { send_ack(1); return; }
    const uint8_t pat  = p[2] & 0x0F;
    const uint8_t step = p[3] & 0x3F;
    if (step >= POLY_STEPS) { send_ack(2); return; }
    const uint8_t slot = g_eng->abs_slot(pat);
    if (!GlobalSettings.var3_is_poly(slot)) {
      GlobalSettings.set_var3_poly(slot, true);
      s_settings_dirty = true;
    }
    const bool resident = g_eng->poly_active_ &&
                          slot == g_eng->abs_slot(g_eng->get_patsel());
    if (resident) {
      g_eng->poly_.set_step(step, p[4] & 0x3F, p[5] & 0x3F, p[6] & 0x3F, p[7] & 0x3F,
                            p[8] & 0x03, p[9] & 0x01, (p[9] >> 1) & 0x01);
      g_eng->poly_stale_      = true;
      g_eng->shadow_dirty_ms_ = millis(); // coalesced flush when stopped + quiet
    } else {
      // Non-resident: buffer the edit in RAM and defer the flash write to the idle
      // save / next reload, so editing a non-playing chord never stalls the voice.
      g_eng->poly_edit_step(slot, step, p[4] & 0x3F, p[5] & 0x3F, p[6] & 0x3F, p[7] & 0x3F,
                            p[8] & 0x03, p[9] & 0x01, (p[9] >> 1) & 0x01);
    }
    s_last_web_edit_ms = millis();
    send_ack(0);
    break;
  }
  case 0x29: { // host -> 303: set variation-3 poly/mono flag
    if (n < 4) return;
    const uint8_t pat  = p[2] & 0x0F;
    const bool    poly = (p[3] != 0);
    const uint8_t slot = g_eng->abs_slot(pat);
    g_eng->drop_poly_edit(slot);       // poly/mono flag change supersedes buffered edits
    GlobalSettings.set_var3_poly(slot, poly);
    s_settings_dirty = true;
    if (!g_clk_run && pat == g_eng->get_patsel()) {
      g_eng->persist_shadows();
      g_eng->ReloadShadows();          // (de)activate poly_ for the active slot
    }
    send_ack(0);
    break;
  }
  case 0x2A: { // host -> 303: set per-pattern scale (mask + enabled); optional <var>
    if (n < 6 || !g_eng) return;
    const uint8_t  pat  = p[2] & 0x0F;
    const uint16_t mask = uint16_t((p[3] & 0x7F) | (uint16_t(p[4] & 0x1F) << 7));
    const bool     en   = (p[5] & 0x01) != 0;
    const uint8_t  var  = (n >= 7) ? (p[6] & 0x03) : 0;
    Sequence *seqp;
    if (var == 0) {
      seqp = &g_eng->pattern[pat];
    } else if (var < NUM_VARIATIONS &&
               (pat & uint8_t(NUM_PATTERNS - 1)) == g_eng->get_patsel()) {
      seqp = &g_eng->shadow_[var - 1];
    } else {
      return; // non-resident shadow scale not supported (web sends the full blob)
    }
    seqp->set_scale_mask(mask);
    seqp->set_scale_enabled(en);
    if (var == 0) {
      g_eng->stale = true;
      mark_pat_dirty(pat);
    } else {
      g_eng->shadow_stale_ = true;
      g_eng->shadow_dirty_ms_ = millis();
    }
    break;
  }
  case 0x2B: { // host -> 303: set one step's three probability bytes (var1 only)
    if (n < 7 || !g_eng) return;
    const uint8_t pat  = p[2] & 0x0F;
    const uint8_t step = p[3] & 0x3F;
    // Optional trailing msbs byte carries the three bit-7s (bit0->b0 ...):
    // high-nibble levels 8-13 set bit 7, which a bare SysEx byte cannot carry.
    const uint8_t msbs = (n >= 8) ? p[7] : 0;
    const uint8_t b0   = uint8_t((p[4] & 0x7F) | ((msbs & 1) << 7)); // accent | slide << 4
    const uint8_t b1   = uint8_t((p[5] & 0x7F) | ((msbs & 2) << 6)); // down | up << 4
    const uint8_t b2   = uint8_t((p[6] & 0x7F) | ((msbs & 4) << 5)); // bit0 up-double
    if (pat == g_eng->get_patsel())
      g_eng->prob_set_resident(step, b0, b1, b2);    // RAM only, flash deferred
    else
      g_eng->prob_edit_set(g_eng->abs_slot(pat), step, b0, b1, b2);
    s_last_web_edit_ms = millis();
    break; // never echoed (0x2A convention)
  }
  case 0x2C: { // host -> 303: request probability table -> 0x2D reply
    if (n < 3 || !g_eng) return;
    enqueue_prob_reply(p[2] & 0x0F);
    break;
  }
  case 0x2D: { // host -> 303: set full var1 probability table (128 raw bytes)
    if (n < 5 + kPackedProbLen || !g_eng) { send_ack(1); return; }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t cx  = static_cast<uint8_t>(p[3] | (p[4] << 7));
    uint8_t raw[kProbRawLen];
    if (!unpack_7bit(p + 5, kPackedProbLen, raw, kProbRawLen)) { send_ack(1); return; }
    if (xor_blob_n(raw, kProbRawLen) != cx) { send_ack(1); return; }
    // Full 8-bit bytes are legal here (high-nibble levels 8-13 set bit 7);
    // the 7-bit packing already carried them intact -- never re-mask.
    if (pat == g_eng->get_patsel()) {
      memcpy(g_eng->prob_var1_, raw, kProbRawLen);
      g_eng->prob_stale_ = true;
      g_eng->shadow_dirty_ms_ = millis();
    } else {
      g_eng->prob_edit_blob(g_eng->abs_slot(pat), raw);
    }
    s_last_web_edit_ms = millis();
    send_ack(0);
    break;
  }
  case 0x4A: { // reboot into the SysEx bootloader (same command as the d650c
    // side): flush pending saves, park BOOT_MAGIC in GPIOR0 so the bootloader
    // stays resident (bootload.c), detach USB first for a clean re-enumerate.
    if (g_eng) {
      g_eng->persist_shadows();   // flush buffered shadow/poly edits (Save skips them when !stale)
      if (g_eng->stale) g_eng->Save();
      if (g_eng->track_stale) g_eng->SaveTrack();
      midi_flush_pending_saves();
      midi_flush_pending_pattern_saves(*g_eng);
    }
    cli();
#ifdef SUPEROS_USB_MIDI
    UDCON |= (1 << DETACH);
    _delay_ms(30);                 // host must register the drop first
    USBCON = 0;
    PLLCSR = 0;
#endif
    GPIOR0 = 0xB7;                 // BOOT_MAGIC, keep in sync with bootload.c
    asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
    break;
  }

#ifdef SUPEROS_COMBINED
  case 0x2F: { // switch firmware to the D650C emulator (reboots; no reply)
    if (g_eng) {
      g_eng->persist_shadows();   // flush buffered shadow/poly edits (Save skips them when !stale)
      if (g_eng->stale) g_eng->Save();
      if (g_eng->track_stale) g_eng->SaveTrack();
      midi_flush_pending_saves();
      midi_flush_pending_pattern_saves(*g_eng);
    }
    combined_switch_firmware(FW_D650);   // does not return
    break;
  }
#endif

  case 0x20: { // request config
    send_config_reply();
    // Also broadcast current group so web editor syncs on connect
    if (g_eng) {
      const uint8_t grp[3] = {0x7D, 0x1C, g_eng->get_group()};
      tx_push_message(grp, 3);
    }
    s_rx_chain_pending = true; // signal main.cpp to re-broadcast chain state
    s_rx_chain_active_len = 0xff; // sentinel: "just re-emit current state"
    break;
  }
  case 0x22: { // set config
    if (n < 4) return;
    const uint8_t ch = p[2];
    const uint8_t fl = p[3];
    if (ch <= 16) GlobalSettings.midi_channel = ch;
    GlobalSettings.midi_clock_receive = (fl & 1) != 0;
    GlobalSettings.midi_thru          = (fl & 2) != 0;
    if (n >= 5 && g_eng) {
      const SequenceDirection d = static_cast<SequenceDirection>(p[4] & 0x07);
      g_eng->SetDirection(d);
      GlobalSettings.sequence_direction = static_cast<uint8_t>(d);
    }
    if (n >= 6) {
      const uint8_t br = p[5];
      if (br >= 1 && br <= 8) GlobalSettings.led_brightness = br;
      // main.cpp loop syncs Leds::brightness from GlobalSettings each tick.
    }
    if (n >= 8) { // per-variation 2/3 MIDI output channels (1..16)
      const uint8_t v2 = p[6], v3 = p[7];
      if (v2 >= 1 && v2 <= 16) GlobalSettings.var2_channel = v2;
      if (v3 >= 1 && v3 <= 16) GlobalSettings.var3_channel = v3;
      midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
    }
    // Defer EEPROM write to idle; inline save blocks UART RX long enough
    // to drop the next SysEx the web sends.
    s_settings_dirty = true;
    midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
    send_ack(0);
    send_config_reply(); // echo back new config
    break;
  }
  default:
    break;
  }
}

static void sysex_cb(byte *data, unsigned sz) {
  if (sz < 4 || data[0] != 0xF0 || data[sz - 1] != 0xF7) return;
  s_din_host_seen = true;
  handle_sysex_body(reinterpret_cast<const uint8_t *>(data + 1),
                    static_cast<unsigned>(sz - 2));
}

#ifdef SUPEROS_USB_MIDI
// USB SysEx reassembly. The Teensy usb_midi core's RX buffer is only 60 bytes
// (USB_MIDI_SYSEX_MAX), but the editor's pattern (0x12) and poly-blob (0x26) writes
// exceed it. So we register the CHUNKED (partial) handler: the core hands us
// the message in <=60-byte pieces (complete=0) plus a final piece (complete=1), and
// we reassemble the whole F0..F7 message here, then dispatch like the DIN path.
// Largest inbound message is the 0x26 poly-blob set: F0 + 5 header + kPackedPolyLen
// + F7. Oversized messages are dropped via usb_sysex_drop below.
static constexpr uint16_t kUsbSysexCap = 5 + kPackedPolyLen + 2;
#ifdef SUPEROS_COMBINED
// Shared with the d650 side's reassembly buffer (see combined.h): one firmware
// runs per boot, so they never both need it.
static_assert(kUsbSysexCap <= FW_USB_SYSEX_SCRATCH, "USB SysEx scratch too small");
static uint8_t *const usb_sysex_buf = g_usb_sysex_scratch;
#else
static uint8_t  usb_sysex_buf[kUsbSysexCap];
#endif
static uint16_t usb_sysex_len  = 0;
static bool     usb_sysex_drop = false;   // overflow guard: skip rest of an oversized msg
static void usb_sysex_partial(const uint8_t *data, uint16_t length, bool complete) {
  if (length && data[0] == 0xF0) {        // first chunk of a new message -> restart
    usb_sysex_len = 0;
    usb_sysex_drop = false;
  }
  if (!usb_sysex_drop) {
    if (usb_sysex_len + length <= kUsbSysexCap) {
      for (uint16_t i = 0; i < length; ++i) usb_sysex_buf[usb_sysex_len++] = data[i];
    } else {
      usb_sysex_drop = true;              // too big for our buffer -> ignore the remainder
    }
  }
  if (complete) {
    if (!usb_sysex_drop && usb_sysex_len >= 4 &&
        usb_sysex_buf[0] == 0xF0 && usb_sysex_buf[usb_sysex_len - 1] == 0xF7) {
      s_usb_host_seen = true;
      handle_sysex_body(usb_sysex_buf + 1, usb_sysex_len - 2);
    }
    usb_sysex_len = 0;
    usb_sysex_drop = false;
  }
}
#endif

// --- Bank select state (for Ableton Program Change mapping) ----------------------
// CC 0 = group (0-3), CC 32 = section (0=Bank A patterns 0-7, 1=Bank B patterns 8-15)
// PC value = pattern within section (0-7)
// pattern_index = section * 8 + pc; group selects which group of 16 to load.
static uint8_t s_bank_group   = 0; // last received CC 0 value
static uint8_t s_bank_section = 0; // last received CC 32 value

// --- Live note stack (legato / slide-back) ---------------------------------------
// Tracks all physically-held notes so releasing the top note slides back to
// whatever is still held underneath (e.g. hold C, slide up to B, release B → slide back to C).
static constexpr uint8_t kNoteStackSize = 8;
static uint8_t s_note_stack[kNoteStackSize];
static uint8_t s_note_stack_vel[kNoteStackSize]; // velocity stored per note
static uint8_t s_note_stack_depth = 0;

static void live_stack_clear() {
  s_note_stack_depth = 0;
}

static void live_stack_remove(uint8_t note) {
  for (uint8_t i = 0; i < s_note_stack_depth; ++i) {
    if (s_note_stack[i] == note) {
      for (uint8_t j = i; j < s_note_stack_depth - 1; ++j) {
        s_note_stack[j]     = s_note_stack[j + 1];
        s_note_stack_vel[j] = s_note_stack_vel[j + 1];
      }
      --s_note_stack_depth;
      return;
    }
  }
}

static void live_stack_push(uint8_t note, uint8_t vel) {
  live_stack_remove(note); // remove if already present (avoid duplicates)
  if (s_note_stack_depth < kNoteStackSize) {
    s_note_stack[s_note_stack_depth]     = note;
    s_note_stack_vel[s_note_stack_depth] = vel;
    ++s_note_stack_depth;
  }
}

// --- Live gate / accent / slide -------------------------------------------------
static bool    s_live_accent = false;
static bool    s_live_gate   = false;
static bool    s_live_slide  = false;
static uint8_t s_live_note   = 0;
// Ticks to force the accent pin LOW after a slid-into accented note so it re-edges
// (low->high) even when the previous note was also accented. A slide keeps the gate
// continuously high, so there is no gate retrigger to fire the accent; without this
// the accent pin stays high across the slide and the accented note does not accent.
static uint8_t s_live_acc_retrig = 0;

bool midi_live_accent() { return s_live_accent && s_live_acc_retrig == 0; }
bool midi_live_gate()   { return s_live_gate; }
bool midi_live_slide()  { return s_live_slide; }
uint8_t midi_live_note() { return s_live_note; }

static void note_on_cb(byte ch, byte pitch, byte vel) {
  if (vel == 0) return;
  if (s_in_channel != 0 && ch != s_in_channel) return;

  if (!g_clk_run) {
    const uint8_t prev_note = s_live_note;
    const bool was_playing  = s_live_gate;

    live_stack_push(static_cast<uint8_t>(pitch), static_cast<uint8_t>(vel));

    if (!s_midi_thru) {
      if (was_playing && prev_note != static_cast<uint8_t>(pitch)) {
        // Legato: slide from previous note — Note On new BEFORE Note Off old.
        out_note_on(pitch, vel, ch);
        out_note_off(prev_note, 0, ch);
        s_live_slide = true;
      } else {
        out_note_on(pitch, vel, ch);
        s_live_slide = false;
      }
    } else {
      // Thru: note already forwarded by library; just update slide state.
      s_live_slide = (was_playing && prev_note != static_cast<uint8_t>(pitch));
    }

    s_live_accent = (vel >= 100);
    s_live_gate   = true;
    s_live_note   = static_cast<uint8_t>(pitch);
    // Slid-into accented note: re-edge the accent pin (gate stays high on a slide,
    // so there is no gate retrigger to fire the accent).
    if (s_live_slide && s_live_accent) s_live_acc_retrig = 2;
  } else if (!s_midi_thru) {
    out_note_on(pitch, vel, ch);
  }

  if (g_eng)
    g_eng->midi_apply_note_on(static_cast<uint8_t>(pitch), static_cast<uint8_t>(vel));
}

static void note_off_cb(byte ch, byte pitch, byte vel) {
  (void)vel;
  if (s_in_channel != 0 && ch != s_in_channel) return;

  if (!g_clk_run) {
    const bool was_top = (static_cast<uint8_t>(pitch) == s_live_note);
    live_stack_remove(static_cast<uint8_t>(pitch));

    if (was_top && s_note_stack_depth > 0) {
      // Slide back: Note On new BEFORE Note Off old for portamento ordering.
      const uint8_t back_note = s_note_stack[s_note_stack_depth - 1];
      const uint8_t back_vel  = s_note_stack_vel[s_note_stack_depth - 1];
      out_note_on(back_note, back_vel, ch);
      if (!s_midi_thru)
        out_note_off(static_cast<uint8_t>(pitch), 0, ch);
      s_live_note   = back_note;
      s_live_slide  = true;
      s_live_gate   = true;
      s_live_accent = (back_vel >= 100); // slide-back adopts the held note's accent
      if (s_live_accent) s_live_acc_retrig = 2; // re-edge accent (slide keeps gate high)
      if (g_eng)
        g_eng->midi_apply_note_on(back_note, back_vel);
    } else {
      // When thru is off, explicitly send Note Off (covers normal release and
      // duplicate Note Off for already-slid notes, harmless to receiving synth).
      if (!s_midi_thru)
        out_note_off(static_cast<uint8_t>(pitch), 0, ch);
      if (was_top) {
        s_live_gate   = false;
        s_live_slide  = false;
        s_live_note   = 0;
        s_live_accent = false; // drop accent when the gate closes so the next
                               // accented note re-asserts a fresh accent edge
      }
    }
  } else if (!s_midi_thru) {
    out_note_off(pitch, 0, ch);
  }
}

// --- Audition note (pitch write / edit step preview) ----------------------------
static uint8_t s_audition_note = 0;
static bool    s_audition_on   = false;
static uint8_t s_audition_ch   = 0;

// Audition note (pitch write/edit/step) plays on the channel of the variation
// being edited, so stepping through variation 2/3 sounds the external synth on
// its own MIDI channel (variation 1 = the 303's main channel).
static uint8_t audition_ch() {
  return out_ch_for_var(g_eng ? g_eng->get_edit_var() : 0);
}

void midi_audition_note_on(uint8_t note, uint8_t vel) {
  const byte ch = static_cast<byte>(audition_ch());
  if (s_audition_on && s_audition_note == note && s_audition_ch == ch) return;
  const bool    was_on    = s_audition_on;
  const uint8_t prev_note = s_audition_note;
  const uint8_t prev_ch   = s_audition_ch;
  // New Note On BEFORE the previous Note Off: overlapping auditions read as
  // legato (slide) on external mono synths, same convention as the sequencer
  // and live-play outputs.
  out_note_on(note, vel, ch);
  s_audition_note = note;
  s_audition_ch   = ch;
  s_audition_on   = true;
  if (was_on) out_note_off(prev_note, 0, prev_ch);
}

void midi_audition_note_off() {
  if (s_audition_on) {
    out_note_off(s_audition_note, 0, s_audition_ch);
    s_audition_on = false;
  }
}

// --- Keyboard-play notes (live keyboard mode, per-key MIDI out) ------------------
// Each held key keeps its own Note On open until that key is released, so the
// MIDI output overlaps exactly like the player's fingers: external mono synths
// slide on the overlap, and a DAW records every note's true length instead of
// notes stopping when the next one starts. Duplicate pitches (two keys mapping
// to the same MIDI note) are refcounted so releasing one key cannot cut a
// still-held twin.
static uint8_t s_kb_out_note[8];
static uint8_t s_kb_out_ch[8];
static uint8_t s_kb_out_n = 0;

void midi_kb_note_on(uint8_t note, uint8_t vel) {
  if (s_kb_out_n >= 8) return;
  const uint8_t ch = static_cast<uint8_t>(audition_ch());
  bool already = false;
  for (uint8_t i = 0; i < s_kb_out_n; ++i)
    if (s_kb_out_note[i] == note && s_kb_out_ch[i] == ch) { already = true; break; }
  s_kb_out_note[s_kb_out_n] = note;
  s_kb_out_ch[s_kb_out_n]   = ch;
  ++s_kb_out_n;
  if (!already) out_note_on(note, vel, ch);
}

void midi_kb_note_off(uint8_t note) {
  for (uint8_t i = 0; i < s_kb_out_n; ++i) {
    if (s_kb_out_note[i] != note) continue;
    const uint8_t ch = s_kb_out_ch[i];
    for (uint8_t j = i; j < s_kb_out_n - 1; ++j) {
      s_kb_out_note[j] = s_kb_out_note[j + 1];
      s_kb_out_ch[j]   = s_kb_out_ch[j + 1];
    }
    --s_kb_out_n;
    bool remaining = false;
    for (uint8_t j = 0; j < s_kb_out_n; ++j)
      if (s_kb_out_note[j] == note && s_kb_out_ch[j] == ch) { remaining = true; break; }
    if (!remaining) out_note_off(note, 0, ch);
    return;
  }
}

void midi_kb_all_notes_off() {
  for (uint8_t i = 0; i < s_kb_out_n; ++i) {
    bool dup = false;
    for (uint8_t j = 0; j < i; ++j)
      if (s_kb_out_note[j] == s_kb_out_note[i] && s_kb_out_ch[j] == s_kb_out_ch[i]) { dup = true; break; }
    if (!dup) out_note_off(s_kb_out_note[i], 0, s_kb_out_ch[i]);
  }
  s_kb_out_n = 0;
}

// --- Audition chord (variation-3 poly edit: sound the whole chord, MIDI only) ----
// Plays every voice of a chord on variation 3's channel so the user hears the full
// chord while navigating poly steps. Same voice->note mapping as poly_gate_tick.
static uint8_t s_aud_chord[POLY_VOICES];
static uint8_t s_aud_chord_n  = 0;
static uint8_t s_aud_chord_ch = 0;

void midi_audition_chord_off() {
  for (uint8_t i = 0; i < s_aud_chord_n; ++i)
    out_note_off(s_aud_chord[i], 0, s_aud_chord_ch);
  s_aud_chord_n = 0;
}

void midi_audition_chord_on(const uint8_t *voices, bool accent, int16_t transpose) {
  midi_audition_chord_off();                              // close any open audition chord
  const byte ch = static_cast<byte>(out_ch_for_var(2));   // variation 3 channel
  const uint8_t vel = accent ? 127 : 80;
  for (uint8_t i = 0; i < POLY_VOICES; ++i) {
    if (voices[i] == POLY_EMPTY) continue;
    int n = 36 + int(unpack_pitch_linear(voices[i])) + int(transpose);
    if (n > 127) n = 127;
    if (n < 0)   n = 0;
    out_note_on(static_cast<byte>(n), vel, ch);
    s_aud_chord[s_aud_chord_n++] = static_cast<uint8_t>(n);
  }
  s_aud_chord_ch = ch;
}

// --- Step position broadcast (SysEx 0x15) ---------------------------------------
// Wrap-only anchor: callers must send this only at pattern wrap (time_pos -> 0)
// or other low-rate events. Per-16th sends produced an audible click because the
// 7-byte burst current-pulse coupled into the analog audio rail. The web editor
// counts incoming 24 PPQN MIDI clock to interpolate steps between anchors.
void midi_send_step_position(uint8_t pat, uint8_t step) {
  const uint8_t grp = g_eng ? g_eng->get_group() : 0;
  const uint8_t inner[5] = {0x7D, 0x15, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F), (uint8_t)(grp & 0x07)};
  tx_push_message(inner, 5);
}

// --- Length update broadcast (SysEx 0x18) ----------------------------------------
void midi_send_length_update(uint8_t pat, uint8_t len, uint8_t var) {
  const uint8_t inner[5] = {0x7D, 0x18, (uint8_t)(pat & 0x0F), (uint8_t)(len & 0x7F),
                            (uint8_t)(var & 0x03)};
  tx_push_message(inner, 5);
}

// --- Direction update broadcast (SysEx 0x17) -------------------------------------
void midi_send_direction_update(uint8_t direction, uint8_t var) {
  const uint8_t inner[4] = {0x7D, 0x17, (uint8_t)(direction & 0x07), (uint8_t)(var & 0x03)};
  tx_push_message(inner, 4);
}

// --- Track state broadcast (SysEx 0x23) -----------------------------------------
// Telemetry for the web editor's Track view. All payload bytes are 7-bit safe.
// Layout: [10 header bytes] [MAX_CHAIN pattern nibbles] [MAX_CHAIN transpose bytes]
//         [MAX_CHAIN last-flag bools]. With MAX_CHAIN=64 inner = 10 + 64*3 = 202 B.
void midi_send_track_state(uint8_t dial_mode, uint8_t track_idx, bool track_active,
                            bool clk_run, Engine &engine) {
  if (!host_seen_any()) return;
  uint8_t inner[10 + MAX_CHAIN * 3];
  inner[0] = 0x7D;
  inner[1] = 0x23;
  inner[2] = dial_mode & 0x03;
  inner[3] = track_idx & 0x07;
  inner[4] = track_active ? 1 : 0;
  inner[5] = clk_run ? 1 : 0;
  inner[6] = engine.get_chain_len() & 0x7F;
  inner[7] = engine.get_chain_pos() & 0x7F;
  inner[8] = engine.get_patsel() & 0x0F;
  inner[9] = engine.get_group() & 0x07;
  for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
    inner[10 + i]                 = engine.p_chain_get(i) & 0x0F;
    inner[10 + MAX_CHAIN + i]     = engine.TrackGetTranspose(i) & 0x7F;
    inner[10 + MAX_CHAIN * 2 + i] = engine.t_chain_last_get(i) ? 1 : 0;
  }
  tx_push_message(inner, sizeof(inner));
}

// --- Group update broadcast (SysEx 0x1C) -----------------------------------------
void midi_send_group_update(uint8_t group) {
  const uint8_t inner[3] = {0x7D, 0x1C, (uint8_t)(group & 0x07)};
  tx_push_message(inner, 3);
}

// Which variation the hardware is currently editing (303<->editor sync, SysEx 0x1F).
void midi_send_edit_variation(uint8_t pat, uint8_t var) {
  const uint8_t inner[4] = {0x7D, 0x1F, (uint8_t)(pat & 0x0F), (uint8_t)(var & 0x03)};
  tx_push_message(inner, 4);
}

// --- Variation-3 poly/mono flag broadcast (SysEx 0x29) --------------------------
void midi_send_poly_flag(uint8_t pat, uint8_t flag) {
  const uint8_t inner[4] = {0x7D, 0x29, (uint8_t)(pat & 0x0F), (uint8_t)(flag ? 1 : 0)};
  tx_push_message(inner, 4);
}

// --- Per-pattern scale broadcast (SysEx 0x2A) -----------------------------------
// Device -> host after a hardware scale edit. Same wire as the host->device edit;
// the device never echoes a received 0x2A, so there is no feedback loop.
void midi_send_scale_update(uint8_t pat, uint16_t mask, bool enabled, uint8_t var) {
  const uint8_t inner[7] = {0x7D, 0x2A, (uint8_t)(pat & 0x0F),
                            (uint8_t)(mask & 0x7F), (uint8_t)((mask >> 7) & 0x1F),
                            (uint8_t)(enabled ? 1 : 0), (uint8_t)(var & 0x03)};
  tx_push_message(inner, 7);
}

// --- Per-step probability broadcast (SysEx 0x2B) --------------------------------
// Device -> host after a hardware probability edit. Same wire as the host->device
// edit; the device never echoes a received 0x2B, so there is no feedback loop.
// b0 = accent|slide levels, b1 = down|up levels, b2 = up-double (variation 1).
void midi_send_prob_step(uint8_t pat, uint8_t step, uint8_t b0, uint8_t b1, uint8_t b2) {
  // Trailing msbs byte: bit0/1/2 = bit 7 of b0/b1/b2 (levels 8-13 need it).
  const uint8_t msbs = (uint8_t)(((b0 >> 7) & 1) | ((b1 >> 6) & 2) | ((b2 >> 5) & 4));
  const uint8_t inner[8] = {0x7D, 0x2B, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F),
                            (uint8_t)(b0 & 0x7F), (uint8_t)(b1 & 0x7F), (uint8_t)(b2 & 0x7F),
                            msbs};
  tx_push_message(inner, 8);
}

// Broadcast ONE variation-3 poly step (device -> host) after a hardware chord edit,
// so the web editor mirrors panel edits live. Same 0x27 wire as the host->device edit;
// the device only sends this on hardware edits and never echoes a received 0x27, so
// there is no feedback loop (mirrors the mono 0x16 model).
void midi_send_poly_step(uint8_t pat, uint8_t step) {
  if (!g_eng) return;
  const PolyVoice &p = g_eng->poly_;
  const uint8_t ci = p.chord_index_for_step(step);
  const uint8_t *v = p.chord(ci);
  const uint8_t flags = uint8_t((p.accent(ci) ? 1 : 0) | (p.slide(ci) ? 2 : 0));
  const uint8_t inner[10] = {0x7D, 0x27, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F),
                             (uint8_t)(v[0] & 0x3F), (uint8_t)(v[1] & 0x3F),
                             (uint8_t)(v[2] & 0x3F), (uint8_t)(v[3] & 0x3F),
                             (uint8_t)(p.time(step) & 0x03), flags};
  tx_push_message(inner, 10);
}

// --- Active pattern broadcast (SysEx 0x1E) ---------------------------------------
// Used while stopped so the web editor follows hardware pat-key presses
// without flagging the pill as "playing" (which 0x15 would do). Includes the
// current group so the web can resync even if its hwGroup state is stale.
void midi_send_active_pattern(uint8_t pat) {
  const uint8_t grp = g_eng ? g_eng->get_group() : 0;
  const uint8_t inner[4] = {0x7D, 0x1E, (uint8_t)(pat & 0x0F), (uint8_t)(grp & 0x07)};
  tx_push_message(inner, 4);
}

// --- Metronome MIDI notes --------------------------------------------------------
static uint8_t s_metro_note_on = 0;

void midi_metronome_tick(bool bar_start) {
  // Metronome clicks, matching the d650c measurement: bar start = E5 (76,
  // DAC 51), other clicks = E6 (88, DAC 63). Note-off comes one step later
  // via midi_metronome_stop().
  const uint8_t note = bar_start ? 76 : 88;
  if (s_metro_note_on) {
    out_note_off(s_metro_note_on, 0, static_cast<byte>(out_ch()));
  }
  out_note_on(note, 80, static_cast<byte>(out_ch()));
  s_metro_note_on = note;
}

void midi_metronome_stop() {
  if (s_metro_note_on) {
    out_note_off(s_metro_note_on, 0, static_cast<byte>(out_ch()));
    s_metro_note_on = 0;
  }
}

// --- Full pattern broadcast (SysEx 0x11) ----------------------------------------
// Used after hardware edits that change the whole pattern (e.g. Clear).
void midi_send_pattern_update(uint8_t pat) {
  enqueue_pattern_reply(pat & 0x0F);
}

// --- Step edit broadcast (SysEx 0x16) -------------------------------------------
// Trailing <var> tags which variation the edit belongs to (0=var1, 1/2=var2/3).
void midi_send_step_update(uint8_t pat, uint8_t step, uint8_t pitch_byte, uint8_t time_nibble,
                           uint8_t var) {
  const uint8_t inner[8] = {
    0x7D, 0x16,
    static_cast<uint8_t>(pat & 0x0F),
    static_cast<uint8_t>(step & 0x3F),
    static_cast<uint8_t>(pitch_byte & 0x7F),       // low 7 bits
    static_cast<uint8_t>((pitch_byte >> 7) & 0x01), // bit 7 (slide/empty flag)
    static_cast<uint8_t>(time_nibble & 0x0F),
    static_cast<uint8_t>(var & 0x03)
  };
  tx_push_message(inner, 8);
}

// --- midi_init ------------------------------------------------------------------
void midi_init(Engine *engine) {
  g_eng = engine;
  // Enable internal pull-up on the MIDI RX pin so an unplugged 3.5mm jack
  // does not leave PD2 floating and self-clocking the UART from EMI picked
  // up off the LED matrix / DAC port writes.
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleSystemExclusive(sysex_cb);
  MIDI.setHandleNoteOn(note_on_cb);
  MIDI.setHandleNoteOff(note_off_cb);
#ifdef SUPEROS_USB_MIDI
  // 3-arg (partial) overload -> reassembled in usb_sysex_partial (handles >60-byte msgs).
  usbMIDI.setHandleSystemExclusive(usb_sysex_partial);
#endif
  tx_clear();
  s_dump_active = false;
}

static uint8_t s_seq_note = 0;
static bool s_seq_note_on = false;
static uint8_t s_seq_note_ch = 0;   // channel the open main note was sent on

// --- Shared clock / transport / channel-message RX (DIN and USB paths) ----------
// When following incoming MIDI clock, echo clock + transport to MIDI OUT so the
// web editor (and any downstream gear) can chase the playhead -- mirroring what
// DIN-sync mode already does (it generates the clock on OUT). Clock is forwarded
// 1:1 so piled-up pulses aren't under-sent.
static void rx_clock(uint8_t &midi_clock_pulses) {
  if (s_midi_clock_rx) { ++midi_clock_pulses; out_clock(); }
}
static void rx_start(Engine &engine, bool &midi_clk) {
  if (s_midi_clock_rx) { midi_clk = true; engine.Reset(); out_start(); }
}
static void rx_continue(bool &midi_clk) {
  if (s_midi_clock_rx) { midi_clk = true; out_continue(); } // resume (no reset)
}
static void rx_stop(Engine &engine, bool &midi_clk) {
  if (s_midi_clock_rx) { midi_clk = false; engine.Reset(); out_stop(); }
}
// CC 0 = group (0-3), CC 32 = section (0=patterns 0-7, 1=patterns 8-15).
static void rx_control_change(uint8_t ch, uint8_t cc, uint8_t val) {
  if (s_in_channel != 0 && ch != s_in_channel) return;
  if (cc == 0)  s_bank_group   = val < NUM_GROUPS ? val : NUM_GROUPS - 1;
  if (cc == 32) s_bank_section = val < 2 ? val : 1;
}
// PC value = pattern within the CC32-selected section (Ableton bank-select mapping).
static void rx_program_change(Engine &engine, uint8_t ch, uint8_t pc) {
  if (s_in_channel != 0 && ch != s_in_channel) return;
  const uint8_t pat = s_bank_section * 8 + (pc < 8 ? pc : 7);
  if (s_bank_group != engine.get_group())
    engine.SetGroup(s_bank_group);
  engine.SetPattern(pat, true);
  engine.get_sequence().Reset();
}

void midi_poll(Engine &engine, bool clk_run, bool &midi_clk, uint8_t &midi_clock_pulses) {
  g_eng = &engine;
  if (clk_run && !g_clk_run) {
    // Transitioning to running: discard any held live notes.
    live_stack_clear();
    s_live_gate  = false;
    s_live_slide = false;
    s_live_note  = 0;
  }
  g_clk_run = clk_run;
  midi_clock_pulses = 0;
  // Count down the slid-accent retrigger so the forced accent-low window expires a
  // couple of polls after the note-on that armed it (set later in this same poll).
  if (s_live_acc_retrig) --s_live_acc_retrig;

  if (!s_midi_clock_rx) midi_clk = false;

  if (!clk_run && s_seq_note_on) {
    out_note_off(s_seq_note, 0, s_seq_note_ch);
    s_seq_note_on = false;
  }
  if (!clk_run) midi_shadows_all_notes_off(engine);

  while (MIDI.read()) {
    switch (MIDI.getType()) {
    case midi::MidiType::Clock:    rx_clock(midi_clock_pulses);    break;
    case midi::MidiType::Start:    rx_start(engine, midi_clk);     break;
    case midi::MidiType::Continue: rx_continue(midi_clk);          break;
    case midi::MidiType::Stop:     rx_stop(engine, midi_clk);      break;
    case midi::MidiType::ControlChange:
      rx_control_change(MIDI.getChannel(), MIDI.getData1(), MIDI.getData2());
      break;
    case midi::MidiType::ProgramChange:
      rx_program_change(engine, MIDI.getChannel(), MIDI.getData1());
      break;
    default:
      break;
    }
  }

#ifdef SUPEROS_USB_MIDI
  // USB MIDI RX — mirror of the DIN handling above. Notes route through the same
  // live-play handlers; clock/transport drive the engine identically and echo to
  // both ports via out_*(). USB SysEx (web editor / updates) is Phase 2: ignored.
  // Bounded drain. usbMIDI.read() returns false for SysEx-continuation packets, not
  // only for "empty", so a plain while(read()) loop dribbles a multi-packet SysEx in
  // at one packet per poll (a ~265B poly write took ~90 polls). The host->device
  // endpoint is 64B x2 banks = 32 packets, so drain a full bank per poll: keep going
  // past a false return so continuation packets flush too. An empty read() is only a
  // few register checks, so the fixed bound is cheap when idle; leftover packets (host
  // refills between polls) drain on the next poll.
  for (uint8_t n = 0; n < 48; ++n) {
    if (!usbMIDI.read()) continue;   // false = empty OR a SysEx-continuation packet consumed
    const uint8_t ut = usbMIDI.getType();
    if (ut == usbMIDI.NoteOn) {
      note_on_cb(usbMIDI.getChannel(), usbMIDI.getData1(), usbMIDI.getData2());
    } else if (ut == usbMIDI.NoteOff) {
      note_off_cb(usbMIDI.getChannel(), usbMIDI.getData1(), usbMIDI.getData2());
    } else if (ut == usbMIDI.Clock) {
      rx_clock(midi_clock_pulses);
    } else if (ut == usbMIDI.Start) {
      rx_start(engine, midi_clk);
    } else if (ut == usbMIDI.Continue) {
      rx_continue(midi_clk);
    } else if (ut == usbMIDI.Stop) {
      rx_stop(engine, midi_clk);
    } else if (ut == usbMIDI.ControlChange) {
      rx_control_change(usbMIDI.getChannel(), usbMIDI.getData1(), usbMIDI.getData2());
    } else if (ut == usbMIDI.ProgramChange) {
      rx_program_change(engine, usbMIDI.getChannel(), usbMIDI.getData1());
    }
  }
#endif

  midi_tx_drain();
  dump_try_advance();
}

void midi_leader_transport(bool clocked, bool clk_run, bool midi_transport_slave,
                           bool run_rising, bool run_falling) {
  if (midi_transport_slave) return;
  if (run_rising)  out_start();
  if (run_falling) out_stop();
  if (clocked && clk_run) out_clock();
}

// Per-tick batch flush (state + collection in out_note_on/off near the top).
// Ons before offs preserves slide legato (new-on precedes old-off) and keeps a
// step's onsets across var1/var2/var3 as one tight cluster (running status then
// compresses same-channel runs). Stop paths (midi_shadows_all_notes_off, transport
// stop) run outside a batch and close notes immediately, so nothing is left
// stranded when the gate-ticks stop running.
void midi_tick_begin() {
  s_tick_batch = true;
}
void midi_tick_flush() {
  s_tick_batch = false;
  if (s_on_n == 0 && s_off_n == 0) return;
#ifdef SUPEROS_USB_MIDI
  // USB first: all events written back-to-back, then committed as one packet so
  // the host timestamps every voice of this tick identically.
  for (uint8_t i = 0; i < s_on_n; ++i)  usbMIDI.sendNoteOn(s_on_note[i], s_on_vel[i], s_on_ch[i]);
  for (uint8_t i = 0; i < s_off_n; ++i) usbMIDI.sendNoteOff(s_off_note[i], 0, s_off_ch[i]);
  usbMIDI.send_now();
#endif
  for (uint8_t i = 0; i < s_on_n; ++i)  MIDI.sendNoteOn(s_on_note[i], s_on_vel[i], s_on_ch[i]);
  for (uint8_t i = 0; i < s_off_n; ++i) MIDI.sendNoteOff(s_off_note[i], 0, s_off_ch[i]);
  s_on_n = 0;
  s_off_n = 0;
}

// Main voice (variation 1) MIDI, driven from the analog gate every clock tick so
// the MIDI note length tracks the 303 hardware gate exactly: a note sounds only
// while get_gate() is high (half-step for plain notes, extended by ties/slides).
// Pitch changing while the gate stays high = a slide, so the new note is sent
// before the old note-off (legato overlap).
void midi_seq_gate_tick(Engine &engine, int16_t transpose) {
  const byte och = static_cast<byte>(out_ch());

  // Output channel moved: close the open note on its old channel first.
  if (s_seq_note_on && och != s_seq_note_ch) {
    queue_off(s_seq_note, s_seq_note_ch);
    s_seq_note_on = false;
  }

  const bool gate = engine.get_gate();
  if (!gate) {
    if (s_seq_note_on) { queue_off(s_seq_note, och); s_seq_note_on = false; }
    return;
  }

  int n = 36 + int(engine.get_pitch_scaled()) + transpose;
  if (n > 127) n = 127;
  if (n < 0)   n = 0;
  const uint8_t vel = engine.get_accent() ? 127 : 80;

  if (!s_seq_note_on) {
    out_note_on(static_cast<byte>(n), vel, och);
    s_seq_note = static_cast<uint8_t>(n);
    s_seq_note_on = true;
    s_seq_note_ch = och;
  } else if (s_seq_note != static_cast<uint8_t>(n)) {
    out_note_on(static_cast<byte>(n), vel, och);   // slide: new on before old off
    queue_off(s_seq_note, och);
    s_seq_note = static_cast<uint8_t>(n);
    s_seq_note_ch = och;
  }
  // else: same note, gate still high -> hold
}

// --- Multitimbral shadow voices (the 2 variations not on CV/gate) -------------
// MIDI-only. Forward-advance each non-empty shadow one 16th, then emit
// notes/ties/rests with per-note slide and accent. No ratchets in v1.
static uint8_t s_shadow_note[NUM_VARIATIONS - 1]    = {0, 0};
static bool    s_shadow_note_on[NUM_VARIATIONS - 1] = {false, false};

// Variation 3 polyphonic voice MIDI. Tracks the set of notes currently on so a
// chord change drops only the notes that left and adds only the new ones (common
// tones are not retriggered -> legato across slides/ties).
static uint8_t s_poly_on[POLY_VOICES];
static uint8_t s_poly_on_count = 0;

static void poly_all_off(byte och) {
  for (uint8_t i = 0; i < s_poly_on_count; ++i)
    out_note_off(s_poly_on[i], 0, och);
  s_poly_on_count = 0;
}

static void poly_gate_tick(Engine &engine, int16_t transpose) {
  const byte och = static_cast<byte>(out_ch_for_var(2)); // variation 3 channel
  if (!engine.poly_active_) { poly_all_off(och); return; }
  const int8_t half = int8_t(engine.step_period() >> 1);
  const int8_t clk  = engine.clk_count;
  const bool gate = !engine.poly_resting_ && (engine.poly_slide_gate_ || clk < half);
  if (!gate) { poly_all_off(och); return; }

  const PolyVoice &p = engine.poly_;
  const uint8_t ci  = engine.poly_chord_pos_;   // chord-stream index now sounding
  const uint8_t *v  = p.chord(ci);
  const uint8_t vel = p.accent(ci) ? 127 : 80;

  uint8_t want[POLY_VOICES];
  uint8_t wn = 0;
  for (uint8_t i = 0; i < POLY_VOICES; ++i) {
    if (v[i] == POLY_EMPTY) continue;
    int n = 36 + int(engine.shadow_[1].scale_quantize_linear(unpack_pitch_linear(v[i]))) + transpose;
    if (n > 127) n = 127;
    if (n < 0)   n = 0;
    want[wn++] = static_cast<uint8_t>(n);
  }
  // Drop notes no longer in the chord, then add the new ones (kept notes hold).
  for (uint8_t i = 0; i < s_poly_on_count;) {
    bool keep = false;
    for (uint8_t j = 0; j < wn; ++j) if (want[j] == s_poly_on[i]) { keep = true; break; }
    if (!keep) { queue_off(s_poly_on[i], och); s_poly_on[i] = s_poly_on[--s_poly_on_count]; }
    else ++i;
  }
  for (uint8_t j = 0; j < wn; ++j) {
    bool on = false;
    for (uint8_t i = 0; i < s_poly_on_count; ++i) if (s_poly_on[i] == want[j]) { on = true; break; }
    if (!on) { out_note_on(want[j], vel, och); s_poly_on[s_poly_on_count++] = want[j]; }
  }
}

// Shadow voices (variations 2/3) MIDI, driven from each shadow's gate state every
// clock tick -- same model as the main voice so var2/3 note length tracks the
// analog gate. Engine::AdvanceShadows() (called at the 16th boundary) computes the
// per-shadow resting/slide_gate; here we sample the gate window with clk_count.
void midi_shadows_gate_tick(Engine &engine, int16_t transpose) {
  const int8_t half = int8_t(engine.step_period() >> 1);
  const int8_t clk  = engine.clk_count;
  // `transpose` is variation 1's effective transpose (global performance +
  // var1 pattern transpose + track step). Strip var1's pattern transpose so each
  // shadow adds its OWN pattern transpose -> per-variation transpose.
  const int base = int(transpose) - int(engine.get_pattern_transpose());
  for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
    const byte och = static_cast<byte>(out_ch_for_var(engine.shadow_var_[i]));
    // Variation 3 (i==1) plays from the poly voice when poly is active -- mute the
    // mono shadow so the two can't sound at once.
    const bool gate = engine.shadow_notecount_[i] && !engine.shadow_resting_[i] &&
                      !(i == 1 && engine.poly_active_) &&
                      (engine.shadow_slide_gate_[i] || clk < half);
    if (!gate) {
      if (s_shadow_note_on[i]) {
        queue_off(s_shadow_note[i], och);
        s_shadow_note_on[i] = false;
      }
      continue;
    }
    Sequence &sq = engine.shadow_[i];
    int n = 36 + int(sq.scale_quantize_linear(sq.get_pitch())) + base + int(int8_t(sq.transpose));
    if (n > 127) n = 127;
    if (n < 0)   n = 0;
    const uint8_t vel = sq.get_accent() ? 127 : 80;
    if (!s_shadow_note_on[i]) {
      out_note_on(static_cast<byte>(n), vel, och);
      s_shadow_note[i] = static_cast<uint8_t>(n);
      s_shadow_note_on[i] = true;
    } else if (s_shadow_note[i] != static_cast<uint8_t>(n)) {
      out_note_on(static_cast<byte>(n), vel, och);   // slide: new on before old off
      queue_off(s_shadow_note[i], och);
      s_shadow_note[i] = static_cast<uint8_t>(n);
    }
    // else: same note, gate still high -> hold
  }
  // Variation 3 polyphonic voice uses variation 3's own pattern transpose.
  poly_gate_tick(engine, static_cast<int16_t>(base + int(int8_t(engine.shadow_[1].transpose))));
}

void midi_shadows_all_notes_off(Engine &engine) {
  for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
    if (s_shadow_note_on[i]) {
      const byte och = static_cast<byte>(out_ch_for_var(engine.shadow_var_[i]));
      out_note_off(s_shadow_note[i], 0, och);
      s_shadow_note_on[i] = false;
    }
  }
  poly_all_off(static_cast<byte>(out_ch_for_var(2)));
}

// Flush pending EEPROM writes accumulated by SysEx handlers.
// Called from the main loop at natural save points (RUN stop, WRITE exit) so
// the 3–100 ms EEPROM stall happens outside the SysEx fast path.
void midi_flush_pending_saves() {
  if (s_settings_dirty) {
    GlobalSettings.save_midi_to_storage();
    s_settings_dirty = false;
  }
}

// Incremental persistence of web-edited patterns. Writes at most ONE pattern
// per call. Only fires when: clock is stopped (EEPROM write halts the CPU and
// would glitch playback audio) AND at least 2s have passed since the last
// SysEx edit (so a burst of edits coalesces into one write per pattern).
void midi_flush_pending_pattern_saves(Engine &engine) {
  if (s_pat_dirty_mask == 0 && !engine.shadow_stale_ && !engine.poly_stale_ &&
      !engine.poly_edit_dirty_ && !engine.shadow_edit_dirty_ &&
      !engine.prob_stale_ && !engine.prob_edit_dirty_) return;
  if (g_clk_run) return;
  const uint32_t now = millis();
  // Buffered non-resident poly edits (web) persist after a 2s quiet period -- the
  // deferred write happens here, while stopped, instead of inline during playback.
  if (engine.poly_edit_dirty_ && (now - engine.poly_edit_ms_) >= 2000) {
    engine.flush_poly_edit();
    return;
  }
  // Buffered non-resident var2/3 mono blobs (web 0x12) use the same quiet-period
  // deferral so a live-edit burst coalesces into one flash write.
  if (engine.shadow_edit_dirty_ && (now - engine.shadow_edit_ms_) >= 2000) {
    engine.flush_shadow_edit();
    return;
  }
  // Buffered non-resident probability edits: same quiet-period deferral.
  if (engine.prob_edit_dirty_ && (now - engine.prob_edit_ms_) >= 2000) {
    engine.flush_prob_edit();
    return;
  }
  // Variation 2/3 (shadow) and probability-table edits -- hardware or web --
  // persist after a 2s quiet period (own timer, refreshed on each edit) so
  // bursts coalesce into one flash write instead of stalling the LED ISR on
  // every keypress. persist_shadows also flushes the resident prob tables.
  if ((engine.shadow_stale_ || engine.poly_stale_ || engine.prob_stale_) &&
      (now - engine.shadow_dirty_ms_) >= 2000) {
    engine.persist_shadows();
    return;
  }
  if (s_pat_dirty_mask == 0) return;
  if (now - s_last_web_edit_ms < 2000) return;
  for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
    const uint16_t bit = uint16_t(1u << i);
    if (s_pat_dirty_mask & bit) {
      engine.persist_pattern(i);
      s_pat_dirty_mask &= ~bit;
      return; // one per tick
    }
  }
}
