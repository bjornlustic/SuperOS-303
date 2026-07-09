// Copyright (c) 2026, Nicholas J. Michalek
//
// main.cpp — setup/loop, MIDI & DIN clock, UI modes, Engine → DAC output
//

#ifndef DEBUG
#define DEBUG 0
#endif

#include <Arduino.h>
#include "pins.h"
#include "drivers.h"
#include "engine.h"
#include "midi_api.h"
#include "flash_store.h"

#ifdef SUPEROS_COMBINED
// Combined build: combined.cpp owns the real setup/loop and dispatches here
// or to the d650c emulator per the EEPROM firmware-select byte.
#include "combined.h"
// avr-gcc ships no <new>; placement new for the g_fw_arena Engine.
inline void *operator new(size_t, void *p) noexcept { return p; }
#define setup superos_setup
#define loop  superos_loop
#endif

FlashEeprom g_flash;
PersistentSettings GlobalSettings;

// =============================================================================
// Globals — timing, debounced inputs, engine, UI timers
// =============================================================================
static uint8_t clk_count = 0;
static uint8_t transpose = 12; // global performance transpose, 0..47 (12 = no transpose)
// Effective transpose = global performance + per-pattern transpose (-24..+24) + track
// step. Signed so per-pattern down-transpose can pull the note below the baseline.
static int16_t total_transpose = 12;
// Clamp a transposed 6-bit CV value to the DAC range so a down-transpose floors at 0
// instead of wrapping to the top of the range.
//
// Factory pitch standard (TB-303 Service Notes p.6, CV adjustment): the
// untransposed low C key must emit 1.000 V at the CV jack = DAC code 23
// (transfer V = (code-11)/12), which is what the original D650C mask ROM
// emits (measured: key C = 23). SuperOS's linear+transpose baseline lands one
// code above that, so every DAC value is shifted down one here. MIDI mapping
// is untouched (key C = note 48 both firmwares, both directions).
static inline uint8_t clamp_cv(int v) { --v; return uint8_t(v < 0 ? 0 : (v > 63 ? 63 : v)); }

#ifdef SUPEROS_COMBINED
static Engine &engine = *(Engine *)g_fw_arena;  // overlays the d650 machine;
                                                // placement-new'ed in setup
#endif

static PinState inputs[INPUT_COUNT];

static uint8_t s_prev_tracknum = 0xff; // 0xff = not yet initialized
static uint8_t s_display_group = 0;    // group shown by dial (may differ from playing group when running)
static uint8_t s_group_debounce_val   = 0xff; // pending new group value
static uint8_t s_group_debounce_count = 0;    // consecutive frames seen
static constexpr uint8_t GROUP_DEBOUNCE_FRAMES = 5;
static bool step_counter = false;
static bool midi_clk = false;
static uint8_t s_time_edit_steps = 0; // counts writes in the current TIME_MODE edit session

/// Stopped-clock CV preview: audition paths set these; unified DAC block applies them.
static bool s_tap_pitch_preview_gate = false;
static uint8_t s_tap_pitch_preview_cv = 0;
static bool s_tap_pitch_preview_accent = false;
// Slide intent captured at audition-arm time. Stored-note audition (TAP /
// BACK / step-select) captures the stored bit; new-note write captures
// SLIDE_KEY.held(). DAC reads this rather than seq.get_slide() so the bit
// at the post-advance pitch_pos cannot bleed into a fresh write's audition.
static bool s_tap_pitch_preview_slide = false;
static uint8_t s_tap_pitch_preview_retrig = 0; // ticks to force gate low for envelope retrigger
static bool s_back_pitch_preview_gate = false;
static uint8_t s_back_pitch_preview_cv = 0;

static elapsedMillis pattern_cleared_flash_timer;
static constexpr uint16_t PATTERN_CLEARED_FLASH_MS = 400;
// Holds the flash LEDs lit until CLEAR is released after a pattern-clear action,
// so the user sees confirmation for as long as they keep CLEAR held.
static bool s_pat_cleared_hold = false;

// Metronome tap-write state (CLEAR+write+clk_run in NORMAL_MODE)
static bool s_metronome_active                  = false;
static bool s_metro_tap_released_since_last_beat = false; // TAP fell during this beat → next press = NOTE, not TIE
static bool s_metro_prev_note                   = false;
static bool s_metro_has_activity                = false;  // any TAP seen this pass; exit at wrap if true
static bool s_metro_gate_pulse                  = false;
static bool s_metro_is_downbeat                 = false;  // downbeat accent flag (every 8 steps)
static uint8_t s_metro_pitch_cv                 = 0;      // final DAC pitch for metronome click
static elapsedMillis s_metro_gate_timer;

// Direction mode (FN + TIME_KEY)
static bool s_dir_mode = false;

// Keyboard play mode (FN + PITCH_KEY toggle while dial is in Pattern Play).
// Pitched keys play the 303 voice live (DAC override) without modifying the
// pattern. Mirrors stopped-clock PITCH_MODE audition behavior but persists
// indefinitely and works whether the sequencer is running or stopped.
static bool s_keyboard_mode = false;
// Press-order stack of pitched-key slots held during keyboard mode. Used to
// drive legato slide-on-overlap (no gate retrigger) and slide-back when the
// top note is released while older notes are still held.
static uint8_t s_kb_stack_key[8];
static uint8_t s_kb_stack_cv[8];
static uint8_t s_kb_stack_note[8]; // MIDI note sent for this key (for its Note Off)
static uint8_t s_kb_stack_depth = 0;
// Octave latch: TIME_KEY toggles, TIME_MODE_LED shows state. Latched (LED on):
// tap DOWN/UP to step the octave register 0..3 and it holds. Off: DOWN/UP must
// be held while pressing keys (stock behavior; octave 3 = DOWN+UP together).
static bool    s_kb_oct_latch = true;
static uint8_t s_kb_oct       = 1;

static void kb_stack_clear() { s_kb_stack_depth = 0; }
static uint8_t kb_stack_note_of(uint8_t key) {
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i)
    if (s_kb_stack_key[i] == key) return s_kb_stack_note[i];
  return 0xFF;
}
static void kb_stack_remove(uint8_t key) {
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i) {
    if (s_kb_stack_key[i] == key) {
      for (uint8_t j = i; j < s_kb_stack_depth - 1; ++j) {
        s_kb_stack_key[j]  = s_kb_stack_key[j + 1];
        s_kb_stack_cv[j]   = s_kb_stack_cv[j + 1];
        s_kb_stack_note[j] = s_kb_stack_note[j + 1];
      }
      --s_kb_stack_depth;
      return;
    }
  }
}
static bool kb_stack_push(uint8_t key, uint8_t cv, uint8_t note) {
  kb_stack_remove(key);
  if (s_kb_stack_depth >= 8) return false;
  s_kb_stack_key[s_kb_stack_depth]  = key;
  s_kb_stack_cv[s_kb_stack_depth]   = cv;
  s_kb_stack_note[s_kb_stack_depth] = note;
  ++s_kb_stack_depth;
  return true;
}

// Track Write: CLEAR ("bar reset") arms "next TAP_NEXT writes the last step".
// The TAP_NEXT after CLEAR writes at the current cursor, marks it as the last
// chain step, and resets the cursor so the next session starts at step 0.
static bool s_track_arm_last = false;


// Step-select mode (FN + PITCH_KEY held): pick one step via black-key bank + white key.
// Chase LED lights only when playhead is within the active bank. -1 = no selection.
// `s_step_sel_edit` = entered the per-step detail editor via ACCENT_KEY rising.
static int     s_step_sel      = -1;
static uint8_t s_step_sel_base = 0; // 0,8,16,24 (+32 in extended half)
static bool    s_step_sel_ext  = false; // A# toggle: reach steps 32..63
static bool    s_step_sel_edit = false;
static bool    s_step_sel_time = false; // true = time sub-mode, false = pitch sub-mode
static bool    s_step_sel_mode = false; // toggled: FN+PITCH enters, FN exits
// Chain slot being viewed in step-select. 0..3 = DOWN/UP/ACCENT/SLIDE. Only
// meaningful when s_chain_active && s_chain_len >= 2. Reset to 0 on each entry.
static uint8_t s_step_sel_chain_view = 0;

// Incremental pattern sync state (drains 2 steps/loop while running)
static uint8_t s_pat_sync_pat = 0;
static uint8_t s_pat_sync_pos = 0;
static uint8_t s_pat_sync_len = 0;

// FN+write length entry state
static bool    s_len_extended     = false;
static uint8_t s_len_black_base   = 0;
static bool    s_len_black_pressed = false;

#ifndef SUPEROS_COMBINED
static Engine engine;
#endif

// Pattern chain: while stopped, hold anchor key + tap adjacent keys to build a chain
// (same bank, consecutive, max 4).  While playing, hold any chain key to loop that pattern.
static uint8_t s_chain_pats[4]    = {0, 0, 0, 0};
static uint8_t s_chain_len        = 0;
static uint8_t s_chain_pos        = 0;
static bool    s_chain_active     = false;
static uint8_t s_chain_anchor_key = 0xff; // key index 0-7; 0xff = not building
static uint8_t s_chain_bank       = 0;    // bank (0=A, 1=B) of the chain being built
static bool     s_chain_hold_loop       = false; // true this frame: loop when target reached
static uint8_t  s_chain_hold_target_pat = 0xff;  // actual pattern to loop (0xff = any/none)
static uint8_t  s_chain_queued[4]    = {0, 0, 0, 0};
static uint8_t  s_chain_queue_len    = 0;     // ≥1 = pattern(s) waiting to activate
static uint8_t  s_chain_hold_key     = 0xff;  // key being tracked for tap/hold
static uint32_t s_chain_hold_ms      = 0;     // millis() when hold key was pressed
static bool     s_chain_hold_crossed = false; // hold threshold crossed
static const uint16_t CHAIN_HOLD_MS  = 300;   // tap vs. hold threshold (ms)

// Broadcast current chain state to web editor (SysEx 0x1A).
static void emit_chain_state() {
  midi_send_chain_state(
    s_chain_active ? s_chain_len : 0, s_chain_pats,
    s_chain_queue_len, s_chain_queued);
}

// Broadcast track-mode state (SysEx 0x23) for the web editor's Track view.
// Call at every event that changes track state: dial transition, chain edit,
// cursor change, RUN edges, chain wrap.
static void emit_track_state(DialMode dial, bool clk_run, uint8_t track_idx) {
  midi_send_track_state(uint8_t(dial), track_idx, engine.track_active, clk_run, engine);
}

// =============================================================================
// Setup menu: hold FUNCTION + press CLEAR (TAP not held) → main menu.
//   C → MIDI channel (C# + white keys 1–8, D# then 9–16; CLEAR → main).
//   D → MIDI clock: TIME_MODE_LED on = internal only; LED off = MIDI clock receive.
//     Press TIME_KEY to toggle; CLEAR → main.
//   E → MIDI thru: SLIDE_KEY_LED on = thru enabled; off = disabled.
//     Press SLIDE_KEY to toggle; CLEAR → main. CLEAR in main exits menu entirely.
//   EEPROM bytes 16–19 + midi_apply_settings() — survives power cycle.
// =============================================================================
static const InputIndex kCfgWhiteKeys[8] = {
    C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY, B_KEY, C_KEY2};

enum class CfgMenu : uint8_t { Off, Midi };
static CfgMenu s_cfg_menu = CfgMenu::Off;
static bool s_cfg_suppress_clear_exit = false;

static uint8_t cfg_display_channel() {
  uint8_t c = GlobalSettings.midi_channel;
  if (c == 0 || c > 16) c = 1;
  return c;
}

static void cfg_save_midi() {
  GlobalSettings.save_midi_to_storage();
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
  midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
}

// Combined MIDI config screen:
//   Pat keys 0-7 → set MIDI channel (1-8, or 9-16 if D# high-bank latched).
//   D# key      → toggle high-bank for channel selection (LED solid while latched).
//   Pat LED     → lit on the slot of the active channel within its bank.
//   TIME mode LED ON  = DIN sync (midi_clock_receive == false).
//   TIME mode LED OFF = MIDI sync (midi_clock_receive == true).
//   TIME_KEY rising → toggle clock source.
//   ACCENT LED ON  = MIDI OUT (midi_thru == false).
//   ACCENT LED OFF = MIDI THRU (midi_thru == true).
//   ACCENT_KEY rising → toggle thru.
//   CLEAR or FN rising → exit menu.

// Preset scale shapes as 12-bit class masks rooted at C (bit0). Order matches the
// web editor's SCALES list exactly so FN-cycling lands on the same scale.
static const uint16_t SCALE_PRESETS[] PROGMEM = {
  0xAB5, // Major
  0x5AD, // Minor
  0x6AD, // Dorian
  0x6B5, // Mixolydian
  0xAD5, // Lydian
  0x5AB, // Phrygian
  0x56B, // Locrian
  0x555, // Whole Tone
  0x6DB, // Half-Whole Dim.
  0xB6D, // Whole-Half Dim.
  0x4E9, // Minor Blues
  0x4A9, // Minor Pentatonic
  0x295, // Major Pentatonic
  0x9AD, // Harmonic Minor
  0x9B5, // Harmonic Major
  0x6CD, // Dorian #4
  0x5B3, // Phrygian Dominant
  0xAAD, // Melodic Minor
  0xB55, // Lydian Augmented
  0x6D5, // Lydian Dominant
  0x55B, // Super Locrian
  0x57B, // 8-Tone Spanish
  0x9B3, // Bhairav
  0x9CD, // Hungarian Minor
  0x18D, // Hirajoshi
  0x4A3, // In-Sen
  0x463, // Iwato
  0x28D, // Kumoi
  0x18B, // Pelog Selisir
  0x1A3, // Pelog Tembung
  0xDDD, // Messiaen 3
  0x9E7, // Messiaen 4
  0x8E3, // Messiaen 5
  0xD75, // Messiaen 6
  0xBEF, // Messiaen 7
};
static constexpr uint8_t NUM_SCALE_PRESETS = 35;
static bool    s_scale_mode       = false; // per-pattern scale editor active
static bool    s_scale_fn_entry   = false; // FN still held from the entry gesture
static uint8_t s_scale_cycle_root = 0xFF;  // last FN+note root this FN-hold (0xFF = none)
static uint8_t s_scale_cycle_idx  = 0;     // preset index within the cycle

// Per-pattern scale editor. Entered by FN + ACCENT in Pattern Write; edits the
// active edit pattern's scale (stored in its reserved metadata, saved with the
// pattern). Non-destructive quantization applies live to playback.
//   Without FN: pitch keys C..B toggle individual classes; High C (pitch_leds[12])
//     toggles the scale on/off.
//   With FN held: pressing a note loads a preset scale rooted at that note --
//     first press = Major, pressing the SAME note again advances through the
//     preset list (Minor, Dorian, ...). A different note resets to Major at that
//     root. Releasing FN resets the cycle, so the next FN+note is Major.
//   FN tap (press and release without a preset note) exits the editor. The
//   FN release that ends the entry gesture (FN + ACCENT) is ignored.
void ProcessScaleMode() {
  Leds::Set(FUNCTION_MODE_LED, true);
  Sequence &s = engine.get_edit_sequence();

  if (inputs[FUNCTION_KEY].falling()) {
    if (s_scale_fn_entry)
      s_scale_fn_entry = false;         // entry gesture's release: stay
    else if (s_scale_cycle_root == 0xFF)
      s_scale_mode = false;             // FN tap with no preset: exit
    s_scale_cycle_root = 0xFF;
    s_scale_cycle_idx = 0;
  }

  for (uint8_t i = 0; i < 12; ++i)
    Leds::Set(pitch_leds[i], s.scale_allows(i));
  Leds::Set(pitch_leds[12], s.scale_enabled());

  bool changed = false;
  if (inputs[FUNCTION_KEY].held()) {
    for (uint8_t i = 0; i < 12; ++i) {
      if (!inputs[pitched_keys[i]].rising()) continue;
      if (i == s_scale_cycle_root)
        s_scale_cycle_idx = uint8_t((s_scale_cycle_idx + 1) % NUM_SCALE_PRESETS);
      else { s_scale_cycle_root = i; s_scale_cycle_idx = 0; }
      const uint16_t shape = pgm_read_word(&SCALE_PRESETS[s_scale_cycle_idx]);
      s.set_scale_mask(uint16_t(((shape << i) | (shape >> (12 - i))) & 0x0FFF));
      s.set_scale_enabled(true);
      changed = true;
      break;
    }
  } else {
    for (uint8_t i = 0; i < 12; ++i) {
      if (inputs[pitched_keys[i]].rising()) { s.toggle_scale_class(i); changed = true; }
    }
    if (inputs[pitched_keys[12]].rising()) {
      s.set_scale_enabled(!s.scale_enabled());
      changed = true;
    }
  }

  // Edits apply live from RAM; the pattern (with its scale) is saved lazily via
  // engine.stale, so there is no per-toggle flash write to stall playback. Each
  // change is broadcast to the web editor (SysEx 0x2A).
  if (changed) {
    engine.stale = true;
    midi_send_scale_update(engine.get_patsel(), s.scale_mask(), s.scale_enabled(),
                           engine.get_edit_var());
  }
}

static void process_config_menu() {
  if (s_cfg_menu == CfgMenu::Off) return;
  Leds::Set(FUNCTION_MODE_LED, true);

  static bool s_high_bank = false;
  if (inputs[DSHARP_KEY].rising()) s_high_bank = !s_high_bank;
  Leds::Set(DSHARP_KEY_LED, s_high_bank);

  // Channel-edit target: hold PITCH_KEY -> variation 2 channel, SLIDE_KEY ->
  // variation 3 channel, neither -> the main (variation 1 / device) channel.
  uint8_t *chan_target = &GlobalSettings.midi_channel;
  uint8_t  dc = cfg_display_channel();
  if (inputs[PITCH_KEY].held()) {
    chan_target = &GlobalSettings.var2_channel;
    dc = GlobalSettings.var2_channel;
    Leds::Set(PITCH_MODE_LED, true);
  } else if (inputs[SLIDE_KEY].held()) {
    chan_target = &GlobalSettings.var3_channel;
    dc = GlobalSettings.var3_channel;
    Leds::Set(SLIDE_KEY_LED, true);
  }
  if (s_high_bank) {
    if (dc >= 9 && dc <= 16) Leds::Set(OutputIndex((dc - 9) & 0x7), true);
  } else {
    if (dc >= 1 && dc <= 8)  Leds::Set(OutputIndex((dc - 1) & 0x7), true);
  }
  for (uint8_t i = 0; i < 8; ++i) {
    if (inputs[i].rising()) {
      *chan_target = uint8_t(i + 1 + (s_high_bank ? 8 : 0));
      cfg_save_midi();
      break;
    }
  }

  Leds::Set(TIME_MODE_LED, !GlobalSettings.midi_clock_receive);
  if (inputs[TIME_KEY].rising()) {
    GlobalSettings.midi_clock_receive = !GlobalSettings.midi_clock_receive;
    cfg_save_midi();
  }

  Leds::Set(ACCENT_KEY_LED, !GlobalSettings.midi_thru);
  if (inputs[ACCENT_KEY].rising()) {
    GlobalSettings.midi_thru = !GlobalSettings.midi_thru;
    cfg_save_midi();
  }

  // A# held + UP/DOWN: LED brightness 1..8
  if (inputs[ASHARP_KEY].held()) {
    Leds::Set(ASHARP_KEY_LED, true);
    if (inputs[UP_KEY].rising() && GlobalSettings.led_brightness < 8) {
      GlobalSettings.led_brightness++;
      Leds::brightness = GlobalSettings.led_brightness;
      cfg_save_midi();
    }
    if (inputs[DOWN_KEY].rising() && GlobalSettings.led_brightness > 1) {
      GlobalSettings.led_brightness--;
      Leds::brightness = GlobalSettings.led_brightness;
      cfg_save_midi();
    }
  }

#ifdef SUPEROS_COMBINED
  // G# = boot the D650C firmware (original mask-ROM 303). Solid G# = SuperOS
  // is the running firmware (the d650c menu blinks it). On press: flush
  // pending saves (same set as transport stop), then reboot.
  Leds::Set(GSHARP_KEY_LED, true);
  if (inputs[GSHARP_KEY].rising()) {
    if (engine.stale) engine.Save();
    if (engine.track_stale) engine.SaveTrack();
    midi_flush_pending_saves();
    midi_flush_pending_pattern_saves(engine);
    combined_switch_firmware(FW_D650);   // does not return
  }
#endif

  if (inputs[CLEAR_KEY].rising()) {
    if (s_cfg_suppress_clear_exit)
      s_cfg_suppress_clear_exit = false;
    else
      s_cfg_menu = CfgMenu::Off;
  }
}

// =============================================================================
// Write-mode input helpers — map matrix keys to Engine sequence edits
// =============================================================================

uint8_t check_pitch_inputs() {
  uint8_t notes = 0;
  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].held()) ++notes;
  }
  return notes;
}
bool check_time_inputs() {
  if (inputs[DOWN_KEY].held())   return true;
  if (inputs[UP_KEY].held())     return true;
  if (inputs[ACCENT_KEY].held()) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Octave resolution. OS-303 encoding has 4 octave registers (0..3) which we
// expose via the original 303's UI: DOWN-only = 0, neither = 1, UP-only = 2,
// DOWN+UP held simultaneously = 3 (the very-top register). Both bits stored
// directly in the pitch byte's oct field (bits[5:4]) so OS-303 round-trip
// preserves the user's choice.
// ---------------------------------------------------------------------------
static uint8_t resolve_octave() {
  const bool dn = inputs[DOWN_KEY].held();
  const bool up = inputs[UP_KEY].held();
  if (dn && up) return 3; // OCTAVE_DOUBLE_UP (top register)
  if (up)       return 2; // OCTAVE_UP
  if (dn)       return 0; // OCTAVE_DOWN
  return 1;               // OCTAVE_ZERO (centre)
}

// Returns the MIDI note number (36-108) written this call, or 0 if nothing was written.
// Helper: SysEx broadcast of the pitch byte for the current edit cursor (time_pos).
static void send_step_update_for_cursor(Sequence &s) {
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  uint8_t slot;
  uint8_t pb = PITCH_EMPTY;
  if (s.edit_slot_index(slot)) pb = s.pitch[slot];
  midi_send_step_update(engine.get_patsel(), tp, pb, s.time(tp), engine.get_edit_var());
}

uint8_t input_pitch(bool mod = false, bool clk_run = false) {
  Sequence &s = engine.get_edit_sequence();
  if (clk_run && engine.is_step_locked()) return 0;
  if (mod) {
    if (!clk_run) s.ensure_pitch_edit_entry();
    bool flag_changed = false;
    if (inputs[ACCENT_KEY].rising()) { engine.ToggleAccent(); flag_changed = true; }
    if (inputs[SLIDE_KEY].rising())  { engine.ToggleSlide();  flag_changed = true; }
    if (inputs[UP_KEY].rising())     { engine.NudgeOctave(1); flag_changed = true; }
    if (inputs[DOWN_KEY].rising())   { engine.NudgeOctave(-1);flag_changed = true; }
    if (flag_changed) send_step_update_for_cursor(s);
  } else {
    s.ensure_pitch_edit_entry();
  }
  for (int pi = int(ARRAY_SIZE(pitched_keys)) - 1; pi >= 0; --pi) {
    const uint8_t i = uint8_t(pi);
    if (inputs[pitched_keys[i]].rising()) {
      if (mod) {
        engine.SetPitchSemitone(i);
        send_step_update_for_cursor(s);
        return uint8_t(engine.get_midi_note());
      } else {
        const uint8_t oct   = resolve_octave();
        const uint8_t flags = (inputs[ACCENT_KEY].held() << 6) |
                              (inputs[SLIDE_KEY].held()   << 7);
        engine.SetPitch(pack_pitch(i, oct), flags);
        send_step_update_for_cursor(s);
        const uint8_t written_note = uint8_t(36 + unpack_pitch_linear(pack_pitch(i, oct)));
        s.advance_pitch_to_next_note();
        return written_note;
      }
    }
  }
  return 0;
}
void input_time(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  uint8_t written_time = 0xFF;
  uint8_t new_t = 0;
  if (inputs[DOWN_KEY].rising())        { new_t = 1; written_time = 1; }
  else if (inputs[UP_KEY].rising())     { new_t = 2; written_time = 2; }
  else if (inputs[ACCENT_KEY].rising()) { new_t = 0; written_time = 0; }
  if (written_time == 0xFF) return;

  if (!mod) { engine.AdvanceEditCursor(); ++s_time_edit_steps; }
  Sequence &s = engine.get_edit_sequence();
  const uint8_t len = s.length;
  uint8_t before_pt[MAX_STEPS];
  sequence_pack_per_time(s, before_pt);
  engine.SetTime(new_t);
  uint8_t after_pt[MAX_STEPS];
  sequence_pack_per_time(s, after_pt);

  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  // Send tp FIRST, then the diff loop. The web editor's 0x16 handler computes
  // noteIdx(blob, step) against the blob's current time_data; if a later step's
  // pitch update arrives before tp's time change is applied, noteIdx counts
  // tp as a NOTE and writes the pitch into the wrong stream slot, corrupting
  // pitch[]. Sending tp first lets the editor update time_data at tp so every
  // subsequent step computes the same K-th-NOTE index the firmware did.
  const uint8_t pat = engine.get_patsel();
  const uint8_t ev = engine.get_edit_var();
  midi_send_step_update(pat, tp, after_pt[tp], written_time, ev);
  for (uint8_t i = 0; i < len; ++i) {
    if (i != tp && before_pt[i] != after_pt[i])
      midi_send_step_update(pat, i, after_pt[i], s.time(i), ev);
  }
}


extern "C" {
  static void jumptoboot(void) {
    // avr-gcc function pointers are WORD addresses: bootloader at byte
    // 0x1F000 = word 0xF800. (A 0x1F000 literal truncates to word 0xF000 =
    // byte 0x1E000 and only reached the bootloader by sliding through the
    // erased flash in between.)
    ((void (*)(void))0xF800)();
  }
}

// =============================================================================
// setup — MIDI, GPIO, optional bootloader, EEPROM load
// =============================================================================
// Only compiled for non-USB-MIDI builds (e.g. Arduino IDE with a Serial USB
// type); the app env always defines SUPEROS_USB_MIDI and keeps USB alive.
#if !DEBUG && !defined(SUPEROS_USB_MIDI)
static void usb_shutdown_hw() {
  UDIEN = 0;
  UDCON = 1;
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
}
#endif

void setup() {
#ifdef SUPEROS_COMBINED
  // Run the constructor (NSDMI defaults) over the zeroed shared arena; net
  // state matches what `static Engine engine;` produced.
  new (g_fw_arena) Engine;
#endif
  // Keep USB alive when built for USB MIDI; otherwise tear it down as before
  // (the stock app left USB detached to avoid the idle PLL/noise).
#if !DEBUG && !defined(SUPEROS_USB_MIDI)
  usb_shutdown_hw();
#endif
  midi_init(&engine);

  for (uint8_t i = 0; i < ARRAY_SIZE(INPUTS); ++i)
    pinMode(INPUTS[i], INPUT);
  for (uint8_t i = 0; i < ARRAY_SIZE(OUTPUTS); ++i)
    pinMode(OUTPUTS[i], OUTPUT);
  for (uint8_t i = 0; i < 4; ++i)
    digitalWriteFast(select_pin[i], HIGH);

  PollInputs(inputs);
  if (inputs[TAP_NEXT].held()) jumptoboot();

#if DEBUG
  Serial.begin(9600);
#endif

  flash_persist_begin(); // mount flash-as-EEPROM (formats on first boot)
  engine.Load();
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
  midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
  Leds::brightness = GlobalSettings.led_brightness;
  Leds::BeginRefresh();
}

// =============================================================================
// Edit-mode LED feedback — current step pitch / time / flags
// =============================================================================
// Light the note LED + octave LEDs for a packed pitch (bits[3:0]=semi,
// bits[5:4]=octave button: 0 = DOWN, 1 = neither, 2 = UP, 3 = both).
static void show_pitch_leds(uint8_t packed) {
  Leds::Set(pitch_leds[packed & 0x0F], true);
  const uint8_t oct = (packed >> 4) & 0x03;
  Leds::Set(DOWN_KEY_LED, oct == 0 || oct == 3);
  Leds::Set(UP_KEY_LED,   oct == 2 || oct == 3);
}

void PrintPitch() {
  const Sequence &s = engine.edit_seq_view();
  const uint8_t pc = s.get_pitch_count();
  if (pc == 0) return;
  // Prefer the edit cursor's slot when on a NOTE; fall back to pitch_pos
  // (the held NOTE) when on TIE / REST so display still shows something.
  uint8_t slot = 0;
  bool have_slot = false;
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  if (tp < s.length && s.time(tp) == 1) {
    slot = s.pitch_index_for_note(tp);
    have_slot = (slot < pc);
  }
  if (!have_slot) {
    if (s.pitch_pos < 0 || s.pitch_pos >= int(pc)) return;
    slot = uint8_t(s.pitch_pos);
  }
  const uint8_t pb = s.pitch[slot];
  if (pb == PITCH_EMPTY) return;
  show_pitch_leds(pb & 0x3f);
  if (s.get_time() == 1) {
    Leds::Set(ACCENT_KEY_LED, (pb & (1 << 6)) != 0);
    Leds::Set(SLIDE_KEY_LED,  (pb & (1 << 7)) != 0);
  }
}
void PrintTime() {
  const uint8_t t = engine.edit_seq_view().get_time();
  Leds::Set(DOWN_KEY_LED,   t == 1);
  Leds::Set(UP_KEY_LED,     t == 2);
  Leds::Set(ACCENT_KEY_LED, t == 0);
  // Step-lock UI removed: it's RAM-only since the OS-303 layout migration
  // (no EEPROM byte for it), and the SLIDE_KEY toggle here confused users.
  Leds::Set(SLIDE_KEY_LED, false);
}

// Light the pitch-key LED of the NOTE at the edit cursor (no octave/flag
// LEDs; in TIME_MODE those belong to PrintTime's note/tie/rest display).
static void PrintCursorNoteLed() {
  const Sequence &s = engine.edit_seq_view();
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  if (tp >= s.length || s.time(tp) != 1) return;
  const uint8_t slot = s.pitch_index_for_note(tp);
  if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY)
    Leds::Set(pitch_leds[s.pitch[slot] & 0x0F], true);
}

// ---------------------------------------------------------------------------
// ProcessDirectionMode — FN + PITCH_KEY: select playback direction
// C=Forward D=Reverse E=PingPong F=Random G=HalfRand A=Brownian; FN to exit
// ---------------------------------------------------------------------------
static const InputIndex  kDirKeys[DIR_COUNT] = {C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY};
static const OutputIndex kDirLeds[DIR_COUNT] = {C_KEY_LED, D_KEY_LED, E_KEY_LED, F_KEY_LED, G_KEY_LED, A_KEY_LED};

// `persist` = Pattern Write: write reserved[0] so direction persists per-pattern.
// `persist` false = Pattern Play / Track Play: RAM-only, undo SetDirection's stale
// flag so the change does not get saved on the next flush. Boot reload of the
// per-pattern direction byte is a Phase 3 task; today this just keeps RAM and
// EEPROM consistent with the user's intent for the current session.
void ProcessDirectionMode(bool persist) {
  Leds::Set(TIME_MODE_LED, clk_count & 4);
  const uint8_t ev = engine.get_edit_var();
  // Show/edit the direction of the variation being edited. Variation 1 uses the
  // engine's live direction; variations 2/3 store it on the shadow (read each
  // tick by AdvanceShadows) and persist with the shadow.
  const uint8_t cur_dir = (ev == 0) ? uint8_t(engine.get_direction())
                                    : engine.edit_seq_view().get_direction_stored();
  for (uint8_t d = 0; d < DIR_COUNT; ++d) {
    const bool active = (cur_dir == d);
    Leds::Set(kDirLeds[d], active ? bool(clk_count & 4) : true);
    if (inputs[kDirKeys[d]].rising()) {
      if (ev == 0) {
        engine.SetDirection(SequenceDirection(d));
        if (persist) engine.get_sequence().store_direction(uint8_t(d));
        else engine.stale = false;
      } else {
        engine.get_edit_sequence().store_direction(uint8_t(d)); // var2/3 -> shadow
      }
      midi_send_direction_update(d, ev); // var-tagged so the editor follows
    }
  }
  // Exit handled in main loop FUNCTION_KEY.rising() block
}

// ---------------------------------------------------------------------------
// ProcessEdit — TAP_NEXT held: pitch/time edit UI
//
// BACK_KEY behaviour:
//   rising → step back one position (clamps at step 0, never wraps)
//   If write_mode held AND not clk_run AND PITCH_MODE: audition the note now on.
//   falling → gate/audition off (handled in main loop TAP_NEXT.falling section)
// ---------------------------------------------------------------------------
void ProcessEdit(const bool &write_mode, const bool clk_run) {
  switch (engine.get_mode()) {
  case PITCH_MODE: {
    if (write_mode) {
      const uint8_t updated_note = input_pitch(true, clk_run);
      // When user presses a pitch key while TAP is held, re-audition with the new pitch.
      if (!clk_run && updated_note) {
        uint16_t mn = uint16_t(updated_note) + total_transpose;
        if (mn > 127) mn = 127;
        const Sequence &es = engine.edit_seq_view();
        const bool acc = es.get_accent();
        const uint8_t vel = acc ? 127 : 80;
        s_tap_pitch_preview_cv = clamp_cv(int(es.get_pitch()) + total_transpose);
        s_tap_pitch_preview_accent = acc;
        s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
        midi_audition_note_on(uint8_t(mn), vel);
      }
    }
    PrintPitch();
    break;
  }
  case TIME_MODE:
    if (write_mode) {
      input_time(true, clk_run);
      PrintTime();
      PrintCursorNoteLed(); // keep the step's note visible while stepping
    }
    break;
  case NORMAL_MODE:
    break;
  }

  // BACK_KEY: step back one position
  if (inputs[BACK_KEY].rising()) {
    engine.StepBack();
    const Sequence &es = engine.edit_seq_view();
    if (!clk_run && engine.get_mode() == PITCH_MODE && es.get_time() != 0) {
      uint16_t mn = uint16_t(36 + es.get_pitch()) + total_transpose;
      if (mn > 127) mn = 127;
      const bool acc = es.get_accent();
      const uint8_t vel = acc ? 127 : 80;
      s_back_pitch_preview_cv = clamp_cv(int(es.get_pitch()) + total_transpose);
      s_tap_pitch_preview_accent = acc;
      s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
      s_back_pitch_preview_gate = true;
      midi_audition_note_on(uint8_t(mn), vel);
    }
  }
  // Always close audition on BACK falling — fixes the infinitely-held note
  // bug when TAP_NEXT and BACK are pressed/released together (TAP falling
  // closed only its own preview; the back-preview latch was orphaned).
  if (inputs[BACK_KEY].falling() && engine.get_mode() == PITCH_MODE) {
    s_back_pitch_preview_gate = false;
    midi_audition_note_off();
  }
}

// Edit-variation picker: hold TAP_NEXT in Pattern Write + normal mode and the
// three variation LEDs (C/D/E) show immediately -- current variation lit, the
// other two half-dim. Press C/D/E to choose which variation (1/2/3) all pattern
// edits apply to; the choice is broadcast to the web editor (SysEx 0x1F). The
// edit variation is latched until a new pattern is selected (SetPattern resets
// it to variation 1). Edit a different pattern's variations by selecting that
// pattern first (TAP released), then holding TAP again.
void ProcessEditVarPicker(bool clk_run) {
  const uint8_t pat = engine.get_patsel();
  if (inputs[C_KEY].rising() && engine.SetEditVar(0)) midi_send_edit_variation(pat, 0);
  if (inputs[D_KEY].rising() && engine.SetEditVar(1)) midi_send_edit_variation(pat, 1);
  if (inputs[E_KEY].rising() && engine.SetEditVar(2)) midi_send_edit_variation(pat, 2);
  const uint8_t ev = engine.get_edit_var();
  // Selected variation flashes; the other two show dim (selectable targets).
  // The blink mask works while stopped too: clk_count free-runs at 120 BPM.
  const bool blink = bool(clk_count & 4);
  Leds::Set(C_KEY_LED, ev == 0 && blink); Leds::SetDim(C_KEY_LED, ev != 0);
  Leds::Set(D_KEY_LED, ev == 1 && blink); Leds::SetDim(D_KEY_LED, ev != 1);
  Leds::Set(E_KEY_LED, ev == 2 && blink); Leds::SetDim(E_KEY_LED, ev != 2);

  // Variation 3 only: SLIDE_KEY toggles poly/mono for the active slot (stopped
  // only -- it does a settings write + shadow reload). SLIDE_KEY_LED bright =
  // poly, dim = mono (toggle available).
  const uint8_t slot = engine.abs_slot(pat);
  if (ev == 2 && !clk_run && inputs[SLIDE_KEY].rising()) {
    engine.persist_shadows();                                       // flush pending var2/3 edits
    GlobalSettings.set_var3_poly(slot, !GlobalSettings.var3_is_poly(slot));
    GlobalSettings.Save();
    engine.ReloadShadows();                                          // (de)activate poly_ for this slot
    midi_send_poly_flag(pat, GlobalSettings.var3_is_poly(slot));     // tell the web editor
  }
  Leds::Set(SLIDE_KEY_LED, ev == 2 && GlobalSettings.var3_is_poly(slot));
  Leds::SetDim(SLIDE_KEY_LED, ev == 2 && !GlobalSettings.var3_is_poly(slot));
}

// ---------------------------------------------------------------------------
// ProcessPolyEdit — variation-3 polyphonic chord editor (PITCH_MODE, var3 poly).
// View octave = held DOWN/UP (0/1/2/3). A pitch key toggles that note at the view
// octave (up to POLY_VOICES per step); ACCENT/SLIDE toggle the step flags;
// TAP_NEXT / BACK move between steps. A step plays when it holds >=1 note.
// (Moving a note across octaves = toggle it off at one octave, on at another.)
// ---------------------------------------------------------------------------
static uint8_t s_poly_step = 0;
// Latched octave for entering chord notes: tap UP/DOWN to set it (tap UP twice for
// the double-up octave); it stays until changed -- no need to hold UP/DOWN.
static uint8_t s_poly_view_oct = 1; // 0=down 1=centre 2=up 3=double-up
void ProcessPolyEdit() {
  PolyVoice &p = engine.poly_;
  const uint8_t len = p.length ? p.length : 1;
  if (s_poly_step >= len) s_poly_step = 0;
  p.ensure_chords_for_notes();   // keep the chord stream covering every NOTE event

  // TAP_NEXT advances (wraps); BACK steps back but CLAMPS at step 0 so re-listening
  // never loops around to the end. Either one auditions the landing chord below.
  bool nav = false;
  if (inputs[TAP_NEXT].rising()) { s_poly_step = uint8_t((s_poly_step + 1) % len); nav = true; }
  if (inputs[BACK_KEY].rising())  { if (s_poly_step) --s_poly_step; nav = true; }

  // Latched octave: a single UP/DOWN tap sets the octave new notes land in (tap UP
  // twice for double-up). No holding required; it stays until changed.
  if (inputs[UP_KEY].rising()   && s_poly_view_oct < 3) ++s_poly_view_oct;
  if (inputs[DOWN_KEY].rising() && s_poly_view_oct > 0) --s_poly_view_oct;
  const uint8_t view_oct = s_poly_view_oct;
  bool changed = false;

  // Pitch keys edit the chord this NOTE step pulls from the stream. On a REST the
  // first pitch press promotes it to a NOTE (which inserts a chord into the list).
  for (uint8_t k = 0; k < ARRAY_SIZE(pitched_keys); ++k) {
    if (!inputs[pitched_keys[k]].rising()) continue;
    if (p.time(s_poly_step) == 0) {
      const uint8_t cc0 = p.get_chord_count();     // promote REST -> NOTE
      p.set_time(s_poly_step, 1);
      p.ensure_chords_for_notes();
      const uint8_t cin = p.chord_index_for_step(s_poly_step);
      if (cin == cc0) { uint8_t *vn = p.chord(cin); vn[0]=vn[1]=vn[2]=vn[3]=POLY_EMPTY; } // start the fresh chord empty so only pressed notes land (no phantom default C)
    }
    if (p.time(s_poly_step) != 1) { changed = true; continue; } // TIE: holds prior chord, not editable
    const uint8_t ci = p.chord_index_for_step(s_poly_step);
    const uint8_t packed = pack_pitch(k, view_oct);
    if (p.has_note(ci, packed)) p.remove_note(ci, packed);
    else                        p.add_note(ci, packed);
    changed = true;
  }
  const bool is_note = (p.time(s_poly_step) == 1);
  const uint8_t ci = p.chord_index_for_step(s_poly_step);
  if (is_note && inputs[ACCENT_KEY].rising()) { p.toggle_accent(ci); changed = true; }
  if (is_note && inputs[SLIDE_KEY].rising())  { p.toggle_slide(ci);  changed = true; }
  if (inputs[TIME_KEY].rising()) { // cycle this step's time NOTE -> TIE -> REST (shifts the list)
    const uint8_t prv = uint8_t((s_poly_step + len - 1) % len);
    const uint8_t cur = p.time(s_poly_step);
    uint8_t nt = (cur == 1) ? 2 : (cur == 2) ? 0 : 1; // note -> tie -> rest -> note
    if (nt == 2 && p.time(prv) == 0) nt = 0;          // no tie after a rest -> skip to rest
    p.set_time(s_poly_step, nt);
    p.ensure_chords_for_notes();
    changed = true;
  }
  if (changed) {
    engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
    midi_send_poly_step(engine.get_patsel(), s_poly_step); // mirror the panel chord edit to the web
  }

  // Audition the full chord (MIDI only, var3 channel) when navigating to a step;
  // close it on release. The 303 CV is suppressed for non-CV variations at the DAC.
  if (nav) {
    if (is_note) midi_audition_chord_on(p.chord(ci), p.accent(ci), total_transpose);
    else         midi_audition_chord_off();
  }
  if (inputs[TAP_NEXT].falling() || inputs[BACK_KEY].falling()) midi_audition_chord_off();

  // ---- LEDs ---- (show the chord only on NOTE steps; TIE holds, REST is silent)
  if (is_note) {
    const uint8_t *v = p.chord(ci);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] != POLY_EMPTY && ((v[i] >> 4) & 0x03) == view_oct)
        Leds::Set(pitch_leds[v[i] & 0x0F], true);
  }
  Leds::Set(DOWN_KEY_LED, view_oct == 0 || view_oct == 3);
  Leds::Set(UP_KEY_LED,   view_oct == 2 || view_oct == 3);
  Leds::Set(ACCENT_KEY_LED, is_note && p.accent(ci));
  Leds::Set(SLIDE_KEY_LED,  is_note && p.slide(ci));
  Leds::Set(TIME_MODE_LED,  p.time(s_poly_step) == 2);                              // tie indicator
  Leds::Set(OutputIndex(s_poly_step & 0x07), bool((millis() >> 7) & 1));            // step pat-LED (blink)
  Leds::Set(OutputIndex(CSHARP_KEY_LED + ((s_poly_step & 31) >> 3)), true);          // 8-step bank
  if (s_poly_step >= 32) Leds::Set(ASHARP_KEY_LED, (millis() >> 7) & 1);
  Leds::Set(PITCH_MODE_LED, true);
}

// Default overlay: pattern select, bank A/B, mode LEDs, running step chase
void ProcessDefault(const bool &write_mode, const bool &clear_mod,
               const bool &clk_run, const bool &dial_pattern_write) {
  switch (engine.get_mode()) {
  case PITCH_MODE:
    if (clk_run) {
      PrintPitch();
      const uint8_t tp = engine.get_edit_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      // Bank indicator (C#/D#/F#/G# + A# blink for the 8-step block) disabled
      // in live pitch edit: it collides with the note-key display.
      // Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      // if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case TIME_MODE:
    // Time state + note LED stay visible in TIME_MODE whether running or
    // stopped (stopped display previously went dark between TAP presses).
    PrintTime();
    PrintCursorNoteLed();
    if (clk_run) {
      const uint8_t tp = engine.get_edit_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      // Bank indicator disabled in live time edit (see PITCH_MODE above).
      // Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      // if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case NORMAL_MODE: {
    const uint8_t bank = engine.get_patsel() >> 3;
    const bool browsing_other_group = clk_run && (s_display_group != engine.get_group());

    // ── LEDs ──
    // Suppress pattern LEDs when browsing a different group while running.
    if (!browsing_other_group) {
      for (uint8_t ci = 0; ci < s_chain_queue_len; ++ci)
        Leds::Set(OutputIndex(s_chain_queued[ci] & 0x7), true);
      if (s_chain_active && s_chain_len > 1) {
        for (uint8_t ci = 0; ci < s_chain_len; ++ci)
          Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
      } else if (s_chain_anchor_key != 0xff) {
        for (uint8_t ci = 0; ci < s_chain_len; ++ci)
          Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
      } else {
        if (engine.get_patsel() != engine.get_next())
          Leds::Set(OutputIndex(engine.get_next() & 0x7), true);
      }
      Leds::Set(OutputIndex(engine.get_patsel() & 0x7), clk_count < 12);
    }
    Leds::Set(ACCENT_KEY_LED, !bank); // A
    Leds::Set(SLIDE_KEY_LED,   bank); // B

    // Step chase: Pattern Write only. Pattern Play hides the chase to keep the
    // performance display calm.
    if (clk_run && dial_pattern_write) {
      const uint8_t tp = engine.get_edit_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }

    // ── Pattern select inputs ──
    if (clk_run && clear_mod) {
      // CLEAR held while running: pat keys reserved for global copy/paste.
    } else if (clk_run && browsing_other_group) {
      // Running in a different group: queue the group switch + pattern to start at next wrap
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          engine.QueueGroup(s_display_group);
          engine.SetPattern(bank * 8 + i, false);
          emit_chain_state();
          break;
        }
      }
    }
    if (clk_run && !browsing_other_group && !clear_mod) {
      // Running: chain building always available (whether or not a chain is currently active).
      //   two keys pressed simultaneously / hold+tap → build or queue a chain
      //   single key tap (quick press+release)       → queue single pattern (or chain pattern)
      //   single key hold (> CHAIN_HOLD_MS)          → loop that pattern when chain reaches it
      const uint32_t now = (uint32_t)millis();

      // 1. Detect hold+new-key: any rising key with another key already held → build chain
      bool chain_built = false;
      for (uint8_t ni = 0; ni < 8 && !chain_built; ++ni) {
        if (!inputs[ni].rising()) continue;
        for (uint8_t hi2 = 0; hi2 < 8; ++hi2) {
          if (hi2 == ni || !inputs[hi2].read()) continue;
          const uint8_t lo2 = (hi2 < ni) ? hi2 : ni;
          uint8_t       ht  = (hi2 > ni) ? hi2 : ni;
          if (ht - lo2 > 3) ht = lo2 + 3;
          const uint8_t new_len = ht - lo2 + 1;
          s_chain_hold_key     = 0xff; // cancel any pending tap
          s_chain_hold_crossed = false;
          s_chain_hold_target_pat = 0xff;
          if (s_chain_active && s_chain_len > 1) {
            // Already in chain: queue the new chain; current chain must finish first
            if (s_chain_queue_len == 0) { // don't overwrite an existing queue
              s_chain_queue_len = new_len;
              for (uint8_t ci = 0; ci < new_len; ++ci)
                s_chain_queued[ci] = bank * 8 + lo2 + ci;
            }
          } else {
            // Not in chain: activate new chain immediately.
            // Set pos to len-1 so the chain advance queues pats[0] as next.
            s_chain_active = true;
            s_chain_len    = new_len;
            for (uint8_t ci = 0; ci < new_len; ++ci)
              s_chain_pats[ci] = bank * 8 + lo2 + ci;
            s_chain_pos       = new_len - 1;
            s_chain_queue_len = 0;
          }
          emit_chain_state();
          chain_built = true;
          break;
        }
      }

      // 2. Track single key for tap/hold (only when pressed alone)
      if (!chain_built) {
        if (s_chain_hold_key == 0xff) {
          for (uint8_t i = 0; i < 8; ++i) {
            if (!inputs[i].rising()) continue;
            bool other = false;
            for (uint8_t j = 0; j < 8; ++j)
              if (j != i && inputs[j].read()) { other = true; break; }
            if (!other) {
              s_chain_hold_key     = i;
              s_chain_hold_ms      = now;
              s_chain_hold_crossed = false;
            }
            break;
          }
        }

        // Update hold threshold
        if (s_chain_hold_key != 0xff && !s_chain_hold_crossed &&
            (now - s_chain_hold_ms) >= CHAIN_HOLD_MS) {
          s_chain_hold_crossed    = true;
          s_chain_hold_target_pat = bank * 8 + s_chain_hold_key; // loop this when reached
        }

        // Hold key released
        if (s_chain_hold_key != 0xff && inputs[s_chain_hold_key].falling()) {
          if (!s_chain_hold_crossed) {
            // Tap: queue single pattern
            if (s_chain_active && s_chain_len > 1) {
              // In chain: queue as single → deactivates chain when reached
              s_chain_queue_len = 1;
              s_chain_queued[0] = bank * 8 + s_chain_hold_key;
            } else {
              // Not in chain: direct pattern switch
              engine.SetPattern(bank * 8 + s_chain_hold_key, false);
            }
            emit_chain_state();
          }
          // else hold released: chain continues advancing (hold_loop cleared below)
          s_chain_hold_key        = 0xff;
          s_chain_hold_crossed    = false;
          s_chain_hold_target_pat = 0xff;
        }
      }

      // Hold-to-loop: only active while key still held past threshold
      s_chain_hold_loop = (s_chain_hold_key != 0xff && s_chain_hold_crossed);

    } else if (!clk_run) {
      // Stopped: chain building.  When CLEAR is held, pat keys are reserved for
      // global copy/paste handlers below — do nothing here.
      s_chain_hold_key = 0xff; // clear stale running state
      if (clear_mod) {
        // fall through: copy/paste and other CLEAR combos own pat-key rising.
      } else if (s_chain_anchor_key == 0xff) {
        for (uint8_t i = 0; i < 8; ++i) {
          if (inputs[i].rising()) {
            s_chain_anchor_key = i;
            s_chain_bank       = bank;
            s_chain_pats[0]    = bank * 8 + i;
            s_chain_len        = 1;
            s_chain_active     = false;
            s_chain_hold_loop  = false;
            engine.SetPattern(bank * 8 + i, true);
            // Stopped: notify web editor of new active pattern (no 0x15 stream while stopped).
            midi_send_active_pattern(engine.get_patsel());
            break;
          }
        }
      } else {
        for (uint8_t i = 0; i < 8; ++i) {
          if (i == s_chain_anchor_key) continue;
          if (inputs[i].rising() && s_chain_bank == bank) {
            uint8_t lo = (s_chain_anchor_key < i) ? s_chain_anchor_key : i;
            uint8_t hi = (s_chain_anchor_key > i) ? s_chain_anchor_key : i;
            if (hi - lo > 3) hi = lo + 3;
            s_chain_len = hi - lo + 1;
            for (uint8_t ci = 0; ci < s_chain_len; ++ci)
              s_chain_pats[ci] = bank * 8 + lo + ci;
          }
        }
        if (inputs[s_chain_anchor_key].falling()) {
          if (s_chain_len > 1) {
            s_chain_active = true;
            s_chain_pos    = 0;
            engine.SetPattern(s_chain_pats[0], true);
          } else {
            s_chain_active = false;
          }
          s_chain_anchor_key = 0xff;
          emit_chain_state();
        }
      }
    }

    // Bank switch — always clears chain.  Skipped when CLEAR is held so
    // CLEAR+ACCENT (randomize) and CLEAR+SLIDE combos can take the edge.
    if (inputs[ACCENT_KEY].rising() && !clear_mod) {
      s_chain_active     = false; s_chain_len       = 0;
      s_chain_queue_len  = 0;     s_chain_anchor_key = 0xff;
      s_chain_hold_key   = 0xff;  s_chain_hold_target_pat = 0xff;
      engine.SetPattern(engine.get_patsel() % 8, !clk_run); // A
      if (!clk_run) midi_send_active_pattern(engine.get_patsel());
      emit_chain_state();
    }
    if (inputs[SLIDE_KEY].rising() && !clear_mod) {
      s_chain_active     = false; s_chain_len       = 0;
      s_chain_queue_len  = 0;     s_chain_anchor_key = 0xff;
      s_chain_hold_key   = 0xff;  s_chain_hold_target_pat = 0xff;
      engine.SetPattern(engine.get_patsel() % 8 + 8, !clk_run); // B
      if (!clk_run) midi_send_active_pattern(engine.get_patsel());
      emit_chain_state();
    }
    break;
  }
  }

  const bool pat_clr_flash = (pattern_cleared_flash_timer < PATTERN_CLEARED_FLASH_MS)
                          || s_pat_cleared_hold;
  const bool in_time  = engine.get_mode() == TIME_MODE;
  const bool in_pitch = engine.get_mode() == PITCH_MODE;
  Leds::Set(TIME_MODE_LED,     in_time  || pat_clr_flash);
  Leds::Set(PITCH_MODE_LED,    in_pitch || pat_clr_flash);
  // FUNCTION_MODE_LED = "normal mode" indicator. Lit whenever the engine is not
  // in TIME/PITCH submode. Driven off NOT-in-submode so that mode-LED edge
  // cases (e.g. brief mode flicker) never leave the indicator dark.
  Leds::Set(FUNCTION_MODE_LED, !in_time && !in_pitch && !pat_clr_flash);
  if (pat_clr_flash) Leds::Set(ASHARP_KEY_LED, true);
}

// PITCH modifier: live transpose root / octave for performance
void ProcessPitchMod() {
  // Solid FUNCTION_MODE_LED so the indicator is visible while stopped (clk_count blink mask
  // would otherwise sit at 0). On release the next-frame ProcessDefault redraws normal LEDs.
  Leds::Set(FUNCTION_MODE_LED, true);
  show_pitch_leds(pack_pitch(transpose % 12, transpose / 12));

  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].rising())
      transpose = (transpose / 12) * 12 + i;
  }
  if (inputs[DOWN_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 - 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
  if (inputs[UP_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 + 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
}

// =============================================================================
// Track Write / Track Play UI: chain write/review, per-step transpose, bank
// toggle. See the workflow comments inline.
// =============================================================================
static void ProcessTrackUI(DialMode dial, bool dial_track_write, bool clk_run,
                           uint8_t track_idx, bool fn_mod, bool clear_mod,
                           bool pitch_mod) {
  const uint8_t patsel    = engine.get_patsel();
  const uint8_t pat_bank  = (patsel >> 3) & 1;                     // 0=A, 1=B (within active group)
  // When PITCH is held in TrackWrite the user is reading/writing the
  // per-step transpose; suppress the pattern + bank LEDs so only the
  // transpose key and octave LEDs are visible.
  const bool tw_transpose_view = dial_track_write && pitch_mod;
  if (!tw_transpose_view) {
    Leds::Set(ACCENT_KEY_LED, pat_bank == 0);
    Leds::Set(SLIDE_KEY_LED,  pat_bank == 1);
    // Show the active pattern's LED both stopped and running (blink chase while running).
    Leds::Set(OutputIndex(patsel & 0x07), clk_run ? bool(clk_count < 12) : true);
  }
  // FN-mode LED stays solid in track mode so it's clearly the "normal" state.
  Leds::Set(FUNCTION_MODE_LED, true);

  // Bank toggle via ACCENT / SLIDE: switch which 8-pattern half pat-keys
  // address. Works in both TrackWrite and TrackPlay so the user can audition
  // patterns across A/B before writing. !edit_mode is intentionally NOT in
  // the gate -- edit_mode == TAP_NEXT.held, and the user may hold TAP_NEXT
  // while reaching for ACCENT/SLIDE in TrackWrite.
  if (!fn_mod && !clear_mod) {
    if (inputs[ACCENT_KEY].rising()) {
      engine.SetPattern((patsel & 0x07), true);
    } else if (inputs[SLIDE_KEY].rising()) {
      engine.SetPattern((patsel & 0x07) | 0x08, true);
    }
  }

  if (dial_track_write) {
    // Track Write workflow ("bar reset" = CLEAR):
    //   - Pat-key 0..7 rising  : switch the currently-playing pattern (same as
    //                            Pattern Play). Does NOT write to the chain.
    //   - TAP_NEXT rising      : write the currently-active pattern into
    //                            chain[cursor], advance cursor.
    //   - CLEAR rising         : "bar reset" -- arms the next TAP_NEXT to
    //                            write the LAST chain step. CLEAR does NOT
    //                            wipe the chain; writes always overwrite.
    //   - TAP_NEXT rising while armed : write at cursor, mark last, reset
    //                            cursor to 0, disarm.
    //   - RUN.falling (stop)   : saves the track to EEPROM.
    // NOTE: edit_mode == inputs[TAP_NEXT].held() globally; do NOT gate handlers
    // on !edit_mode here -- it would block the rising-edge frame for TAP_NEXT.
    if (!fn_mod && inputs[CLEAR_KEY].rising()) {
      if (clk_run) {
        // Running: CLEAR arms the next TAP_NEXT as the last chain step.
        s_track_arm_last = true;
      } else {
        // Stopped: CLEAR jumps the cursor back to step 0 and auditions
        // chain[0]'s pattern so the user can review / edit the track from
        // the beginning. Mirrors TrackPlay CLEAR semantics.
        engine.TrackResetCursor();
        if (engine.track_has_chain()) {
          engine.SetPattern(engine.TrackGetPattern(0), true);
        }
      }
      emit_track_state(dial, clk_run, track_idx);
    }
    if (!fn_mod && !clear_mod && pitch_mod) {
      // PITCH held in Track Write: pitch-keys / UP / DOWN edit the per-step
      // transpose at the current cursor. Encoding mirrors the global
      // performance transpose (semi + 12*oct, 0..47, 12 = no transpose).
      const uint8_t pos = engine.get_chain_pos();
      uint8_t cur = engine.TrackGetTranspose(pos);
      bool changed = false;
      for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
        if (inputs[pitched_keys[i]].rising()) {
          cur = uint8_t((cur / 12) * 12 + i);
          changed = true;
          break;
        }
      }
      if (inputs[DOWN_KEY].rising()) {
        uint8_t oct = constrain(int(cur) / 12 - 1, 0, 3);
        cur = uint8_t((cur % 12) + oct * 12);
        changed = true;
      }
      if (inputs[UP_KEY].rising()) {
        uint8_t oct = constrain(int(cur) / 12 + 1, 0, 3);
        cur = uint8_t((cur % 12) + oct * 12);
        changed = true;
      }
      if (changed) {
        engine.TrackSetTranspose(pos, cur);
        emit_track_state(dial, clk_run, track_idx);
      }
      // LED feedback: show the step's transpose on the pitch-key + octave LEDs.
      show_pitch_leds(pack_pitch(cur % 12, cur / 12));
    } else if (!fn_mod && !clear_mod) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          const uint8_t pat_in_bank = uint8_t((pat_bank << 3) | i);
          engine.SetPattern(pat_in_bank, !clk_run);
          emit_track_state(dial, clk_run, track_idx);
          break;
        }
      }
      if (clk_run && inputs[TAP_NEXT].rising()) {
        engine.TrackWriteCurrentStep(engine.get_patsel() & 0x0F, /*repeats=*/0);
        if (s_track_arm_last) {
          engine.TrackMarkLastStep();
          engine.TrackResetCursor();
          s_track_arm_last = false;
        } else {
          engine.TrackAdvanceCursor();
        }
        emit_track_state(dial, clk_run, track_idx);
      }
      // Stopped + chain has steps: TAP_NEXT cycles the cursor through the
      // chain so the user can review what was written (pattern + transpose).
      // BACK_KEY steps backward. Non-destructive -- writes only happen while
      // the clock is running.
      if (!clk_run && engine.track_has_chain()) {
        const uint8_t clen = engine.get_chain_len();
        uint8_t pos = engine.get_chain_pos();
        bool moved = false;
        if (inputs[TAP_NEXT].rising()) {
          pos = uint8_t((pos + 1) % clen);
          moved = true;
        } else if (inputs[BACK_KEY].rising()) {
          pos = uint8_t((pos + clen - 1) % clen);
          moved = true;
        }
        if (moved) {
          engine.p_chain_pos = pos;
          engine.SetPattern(engine.TrackGetPattern(pos), true);
          emit_track_state(dial, clk_run, track_idx);
        }
      }
    }
  } else {
    // Track Play: CLEAR rising while stopped resets playhead to chain[0].
    // While running, CLEAR is ignored -- track plays until user stops it.
    if (!fn_mod && !clk_run && inputs[CLEAR_KEY].rising()) {
      engine.TrackResetCursor();
      if (engine.track_has_chain()) {
        engine.SetPattern(engine.TrackGetPattern(0), true);
      }
      emit_track_state(dial, clk_run, track_idx);
    }
  }
}

// =============================================================================
// Step-select mode (FN + PITCH toggle): two sub-modes, pitch (default) and
// time. PITCH_KEY switches to pitch sub-mode, TIME_KEY to time sub-mode.
// Pitch sub-mode: only NOTE steps selectable, TAP_NEXT enters detail editor.
// Time sub-mode: ALL steps selectable, no enter needed; just select and edit.
// =============================================================================
static void ProcessStepSelect(bool clk_run, bool dial_pattern_write) {
  Leds::Set(FUNCTION_MODE_LED, true);
  Leds::Set(PITCH_MODE_LED, !s_step_sel_time);
  Leds::Set(TIME_MODE_LED,   s_step_sel_time);
  // Resolve the pattern being viewed. With an active chain of >= 2 patterns
  // we view chain[s_step_sel_chain_view]; otherwise we view the engine's
  // active pattern. View is independent of playback: the engine keeps
  // playing its own pattern regardless of what we view here.
  const bool chain_view_active = s_chain_active && s_chain_len >= 2;
  if (chain_view_active && s_step_sel_chain_view >= s_chain_len)
    s_step_sel_chain_view = 0;
  const uint8_t view_pat_idx = chain_view_active
      ? s_chain_pats[s_step_sel_chain_view]
      : engine.get_patsel();
  // Edit the current variation when viewing the active pattern (var2/3 live
  // in the shadow buffers); chained non-active patterns edit variation 1.
  Sequence &seq = (view_pat_idx == engine.get_patsel())
                      ? engine.get_edit_sequence()
                      : engine.pattern[view_pat_idx];
  const bool playing_matches_view = (engine.get_patsel() == view_pat_idx);
  // Variation 3 poly: this slot's var3 plays from the chord-list voice, not the
  // (silent) mono shadow `seq`. Read the picker's steps/length and edit chords
  // from poly_ so the display shows the real steps instead of stale shadow data.
  const bool poly_sel = playing_matches_view && engine.get_edit_var() == 2 && engine.poly_active_;
  PolyVoice &pv = engine.poly_;
  const uint8_t blen = poly_sel ? uint8_t(pv.length ? pv.length : 1) : seq.length;
  #define SEL_TIME(i) (poly_sel ? pv.time(uint8_t(i)) : seq.time(uint8_t(i)))

  // Sub-mode switching (outside detail editor to avoid accidental mode changes).
  if (!s_step_sel_edit) {
    if (inputs[TIME_KEY].rising() && !s_step_sel_time) {
      s_step_sel_time = true;
      s_step_sel = -1; // clear selection on mode switch
    }
    if (inputs[PITCH_KEY].rising() && s_step_sel_time) {
      s_step_sel_time = false;
      s_step_sel = -1;
    }
  }

  // ── Picker (shared between pitch and time sub-modes) ──
  if (!s_step_sel_edit) {
    // A# toggles the extended half (steps 32..63); keep the same black-key
    // offset when flipping so the picker stays on the same column.
    if (inputs[ASHARP_KEY].rising()) {
      s_step_sel_ext = !s_step_sel_ext;
      s_step_sel_base = uint8_t((s_step_sel_base & 31) + (s_step_sel_ext ? 32 : 0));
    }
    const uint8_t ext = s_step_sel_ext ? 32 : 0;
    // Bank pick
    if (inputs[CSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 0);
    if (inputs[DSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 8);
    if (inputs[FSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 16);
    if (inputs[GSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 24);
    // Step pick
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        const uint8_t cand = uint8_t(s_step_sel_base + wi);
        if (cand < blen) {
          // Pitch sub-mode: only NOTE steps. Time sub-mode: any step.
          if (s_step_sel_time || SEL_TIME(cand) == 1)
            s_step_sel = int(cand);
        }
      }
    }
    // Step LEDs: NOTE=bright, TIE=dim, REST=off
    for (uint8_t wi = 0; wi < 8; ++wi) {
      const uint8_t idx = uint8_t(s_step_sel_base + wi);
      if (idx >= blen) break;
      const uint8_t tn = SEL_TIME(idx);
      if (tn == 1) Leds::Set(OutputIndex(wi), true);
      else if (tn == 2) Leds::SetDim(OutputIndex(wi), true);
    }
    // Chase LED: only when the currently-playing pattern is the one
    // being viewed (so chain slots not currently playing stay quiet).
    // Read the playhead from the edited variation (seq), not variation 1,
    // so the chase follows the right length when editing variation 2/3.
    if (clk_run && playing_matches_view) {
      const uint8_t tp = poly_sel ? uint8_t(engine.poly_time_pos_ & (MAX_STEPS - 1))
                                   : uint8_t(seq.time_pos & (MAX_STEPS - 1));
      if ((tp & ~uint8_t(7)) == s_step_sel_base)
        Leds::Set(OutputIndex(tp & 0x7), bool(clk_count & 4));
    }
    // Cover-bank LEDs (within the visible half; A# shows extended state).
    const bool blinkb = bool((millis() >> 8) & 1);
    const uint8_t base_off = uint8_t(s_step_sel_base & 31);
    const OutputIndex sel_base_led =
        (base_off == 0)  ? CSHARP_KEY_LED :
        (base_off == 8)  ? DSHARP_KEY_LED :
        (base_off == 16) ? FSHARP_KEY_LED : GSHARP_KEY_LED;
    Leds::Set(CSHARP_KEY_LED, (blen > ext + 0)  && (sel_base_led == CSHARP_KEY_LED ? blinkb : true));
    Leds::Set(DSHARP_KEY_LED, (blen > ext + 8)  && (sel_base_led == DSHARP_KEY_LED ? blinkb : true));
    Leds::Set(FSHARP_KEY_LED, (blen > ext + 16) && (sel_base_led == FSHARP_KEY_LED ? blinkb : true));
    Leds::Set(GSHARP_KEY_LED, (blen > ext + 24) && (sel_base_led == GSHARP_KEY_LED ? blinkb : true));
    // A#: solid in the extended half, blink when extended steps exist.
    Leds::Set(ASHARP_KEY_LED, s_step_sel_ext ? true : (blen > 32 ? blinkb : false));
    // Selection flash
    if (s_step_sel >= 0 && (uint8_t(s_step_sel) & ~uint8_t(7)) == s_step_sel_base)
      Leds::Set(OutputIndex(s_step_sel & 0x7), bool((millis() >> 7) & 1));

    // Chain slot select + LEDs. Available in the pitch sub-mode always (its
    // picker leaves DOWN/UP/ACCENT/SLIDE free); in the time sub-mode only
    // while no step is selected (a selection hands those keys to the time
    // editor). Hidden entirely when the chain has < 2 patterns.
    if (chain_view_active && (!s_step_sel_time || s_step_sel < 0)) {
      bool switched = false;
      if (inputs[DOWN_KEY].rising()   && s_chain_len > 0) { s_step_sel_chain_view = 0; switched = true; }
      if (inputs[UP_KEY].rising()     && s_chain_len > 1) { s_step_sel_chain_view = 1; switched = true; }
      if (inputs[ACCENT_KEY].rising() && s_chain_len > 2) { s_step_sel_chain_view = 2; switched = true; }
      if (inputs[SLIDE_KEY].rising()  && s_chain_len > 3) { s_step_sel_chain_view = 3; switched = true; }
      if (switched) {
        // Selection belongs to the previous pattern; drop it with the view.
        s_step_sel = -1;
        s_step_sel_base = 0;
        s_step_sel_ext  = false;
      }
      // LEDs: viewed slot = solid full brightness; playing slot = chase blink
      // at full brightness; remaining chain slots = dim.
      static const OutputIndex kChainLeds[4] =
          {DOWN_KEY_LED, UP_KEY_LED, ACCENT_KEY_LED, SLIDE_KEY_LED};
      const bool blinkc = bool(clk_count & 4);
      for (uint8_t ci = 0; ci < 4 && ci < s_chain_len; ++ci) {
        const bool sel  = (s_step_sel_chain_view == ci);
        const bool play = clk_run && (s_chain_pos == ci);
        if (sel || (play && blinkc)) Leds::Set(kChainLeds[ci], true);
        else                         Leds::SetDim(kChainLeds[ci], true);
      }
    }

    if (s_step_sel_time) {
      // ── Time sub-mode: edit directly from picker, no enter needed ──
      // Edits gated to Pattern Write; in Pattern Play the picker stays
      // visible as a read-only viewer (LED feedback shows current state).
      if (s_step_sel >= 0) {
        const uint8_t si = uint8_t(s_step_sel);
        if (dial_pattern_write && poly_sel) {
          // Poly var3: edit the chord-list voice's time stream (shifts chords).
          bool tchanged = false;
          if (inputs[DOWN_KEY].rising()) { pv.set_time(si, 1); tchanged = true; }
          if (inputs[UP_KEY].rising()) {
            const uint8_t prev_t = (si > 0) ? pv.time(uint8_t(si - 1)) : 0;
            if (prev_t != 0) { pv.set_time(si, 2); tchanged = true; }
          }
          if (inputs[ACCENT_KEY].rising()) { pv.set_time(si, 0); tchanged = true; }
          if (tchanged) {
            pv.ensure_chords_for_notes();
            engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
            midi_send_poly_step(engine.get_patsel(), si); // mirror to the web
          }
        } else if (dial_pattern_write) {
          bool tchanged = false;
          uint8_t before_pt[MAX_STEPS];
          sequence_pack_per_time(seq, before_pt);
          if (inputs[DOWN_KEY].rising()) {
            sequence_write_time_with_pitch_sync(seq, si, 1);
            engine.stale = true; tchanged = true;
          }
          if (inputs[UP_KEY].rising()) {
            const uint8_t prev_t = (si > 0) ? seq.time(uint8_t(si - 1)) : 0;
            if (prev_t != 0) {
              sequence_write_time_with_pitch_sync(seq, si, 2);
              engine.stale = true; tchanged = true;
            }
          }
          if (inputs[ACCENT_KEY].rising()) {
            sequence_write_time_with_pitch_sync(seq, si, 0);
            engine.stale = true; tchanged = true;
          }
          // SLIDE_KEY step-lock toggle removed (RAM-only feature).
          if (tchanged) {
            uint8_t after_pt[MAX_STEPS];
            sequence_pack_per_time(seq, after_pt);
            const uint8_t plen = seq.length;
            const uint8_t pat = view_pat_idx;
            for (uint8_t i = 0; i < plen; ++i) {
              if (i == si || before_pt[i] != after_pt[i])
                midi_send_step_update(pat, i, after_pt[i], seq.time(i));
            }
          }
        }
        // Show time info for the selected step (always, even in Play)
        const uint8_t st = SEL_TIME(si);
        Leds::Set(DOWN_KEY_LED,   st == 1);
        Leds::Set(UP_KEY_LED,     st == 2);
        Leds::Set(ACCENT_KEY_LED, st == 0);
        Leds::Set(SLIDE_KEY_LED,  false);
      }
      if (inputs[BACK_KEY].rising()) s_step_sel = -1;
    } else {
      // ── Pitch sub-mode picker ──
      // Enter detail editor on TAP_NEXT rising with a valid selection.
      if (s_step_sel >= 0 && inputs[TAP_NEXT].rising()) {
        s_step_sel_edit = true;
        if (!clk_run) {
          const uint8_t slot = seq.pitch_index_for_note(uint8_t(s_step_sel));
          const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
          if (pb != PITCH_EMPTY) {
            const uint8_t linear = unpack_pitch_linear(pb & 0x3f);
            const bool acc = (pb & (1 << 6)) != 0;
            if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
            s_tap_pitch_preview_cv = clamp_cv(int(linear) + total_transpose);
            s_tap_pitch_preview_accent = acc;
            s_tap_pitch_preview_slide = false; // previews never slide
            s_tap_pitch_preview_gate = true;
          }
        }
      }
      // BACK clears selection.
      else if (inputs[BACK_KEY].rising()) s_step_sel = -1;
    }
  } else {
    // ── Pitch detail editor (only reachable from pitch sub-mode) ──
    // Edits gated to Pattern Write. In Pattern Play the editor still
    // shows LED feedback and audition but no pitch/flag writes occur.
    if (s_step_sel < 0) {
      s_step_sel_edit = false;
    } else if (poly_sel) {
      // ── Poly chord detail editor: edit the chord this NOTE step pulls ──
      const uint8_t ci = pv.chord_index_for_step(uint8_t(s_step_sel));
      // Latched octave (tap UP/DOWN; tap UP twice for double-up) -- no holding.
      if (inputs[UP_KEY].rising()   && s_poly_view_oct < 3) ++s_poly_view_oct;
      if (inputs[DOWN_KEY].rising() && s_poly_view_oct > 0) --s_poly_view_oct;
      const uint8_t view_oct = s_poly_view_oct;
      bool pchanged = false;
      if (dial_pattern_write) {
        for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
          if (!inputs[pitched_keys[pi]].rising()) continue;
          const uint8_t packed = pack_pitch(pi, view_oct);
          if (pv.has_note(ci, packed)) pv.remove_note(ci, packed);
          else                         pv.add_note(ci, packed);
          pchanged = true;
        }
        if (inputs[ACCENT_KEY].rising()) { pv.toggle_accent(ci); pchanged = true; }
        if (inputs[SLIDE_KEY].rising())  { pv.toggle_slide(ci);  pchanged = true; }
      }
      if (pchanged) {
        engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
        midi_send_poly_step(engine.get_patsel(), uint8_t(s_step_sel)); // mirror to the web
      }
      if (inputs[BACK_KEY].rising()) s_step_sel_edit = false;
      // LEDs: show the chord at the current view octave (like ProcessPolyEdit).
      const uint8_t *v = pv.chord(ci);
      for (uint8_t i = 0; i < POLY_VOICES; ++i)
        if (v[i] != POLY_EMPTY && ((v[i] >> 4) & 0x03) == view_oct)
          Leds::Set(pitch_leds[v[i] & 0x0F], true);
      Leds::Set(DOWN_KEY_LED, view_oct == 0 || view_oct == 3);
      Leds::Set(UP_KEY_LED,   view_oct == 2 || view_oct == 3);
      Leds::Set(ACCENT_KEY_LED, pv.accent(ci));
      Leds::Set(SLIDE_KEY_LED,  pv.slide(ci));
    } else {
      // Pin pitch_pos to the SELECTED step's pitch slot for the duration
      // of the edit so SetPitchSemitone / NudgeOctave / Toggle ops target
      // the user's pick, not the currently-playing playhead.
      const int     saved_pp = seq.pitch_pos;
      const uint8_t slot = seq.pitch_index_for_note(uint8_t(s_step_sel));
      seq.pitch_pos = int(slot);
      bool changed = false;
      bool audition = false;
      if (dial_pattern_write) {
        if (inputs[ACCENT_KEY].rising()) {
          if (slot < seq.get_pitch_count()) {
            if (seq.pitch[slot] == PITCH_EMPTY) seq.pitch[slot] = PITCH_DEFAULT;
            seq.pitch[slot] ^= (1 << 6);
            engine.stale = true; changed = true;
          }
          audition = true;
        }
        if (inputs[SLIDE_KEY].rising()) {
          if (slot < seq.get_pitch_count()) {
            if (seq.pitch[slot] == PITCH_EMPTY) seq.pitch[slot] = PITCH_DEFAULT;
            seq.pitch[slot] ^= (1 << 7);
            engine.stale = true; changed = true;
          }
        }
        if (inputs[UP_KEY].rising())     { seq.nudge_octave_buttons(+1); engine.stale = true; changed = true; audition = true; }
        if (inputs[DOWN_KEY].rising())   { seq.nudge_octave_buttons(-1); engine.stale = true; changed = true; audition = true; }
        for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
          if (inputs[pitched_keys[pi]].rising()) {
            seq.SetPitchSemitone(pi);
            engine.stale = true; changed = true; audition = true;
            break;
          }
        }
      }
      seq.pitch_pos = saved_pp;
      if (changed) {
        const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
        midi_send_step_update(view_pat_idx, uint8_t(s_step_sel),
            pb, seq.time(uint8_t(s_step_sel)));
      }
      if (inputs[BACK_KEY].rising()) s_step_sel_edit = false;
      if (!clk_run && (inputs[TAP_NEXT].rising() || audition)) {
        const uint8_t ab = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
        if (ab != PITCH_EMPTY) {
          const uint8_t lin = unpack_pitch_linear(ab & 0x3f);
          const bool acc = (ab & (1 << 6)) != 0;
          if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
          s_tap_pitch_preview_cv = clamp_cv(int(lin) + total_transpose);
          s_tap_pitch_preview_accent = acc;
          s_tap_pitch_preview_slide = false; // previews never slide
          s_tap_pitch_preview_gate = true;
        }
      }

      const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
      if (pb != PITCH_EMPTY) {
        show_pitch_leds(pb & 0x3f);
        Leds::Set(ACCENT_KEY_LED, (pb & (1 << 6)) != 0);
        Leds::Set(SLIDE_KEY_LED,  (pb & (1 << 7)) != 0);
      }
    }
  }
  // Close step-select audition gate when all relevant keys are released.
  if (!clk_run && s_tap_pitch_preview_gate &&
      !check_pitch_inputs() &&
      !inputs[UP_KEY].held() && !inputs[DOWN_KEY].held() &&
      !inputs[ACCENT_KEY].held() && !inputs[TAP_NEXT].held()) {
    s_tap_pitch_preview_gate = false;
  }
  #undef SEL_TIME
}

// =============================================================================
// Keyboard play: pitched keys play notes via the audition CV path; no pattern
// writes. Octave: latched (TIME toggles, TIME_MODE_LED lit) = tap DOWN/UP to
// step register 0..3 and it holds; unlatched = hold DOWN/UP while playing.
// Press-order stack drives legato slide: holding one key and pressing another
// keeps the gate high and forces slide CV high so the 303 portamentos between
// notes. Releasing the top note slides back to the next-most-recent held note.
// Releasing all notes drops the gate. MIDI out is per-key: every held key
// keeps its own Note On open until released, so overlaps come out as legato
// (mono synths slide) and a DAW records true note lengths.
// =============================================================================

// The button matrix has no diodes: DOWN/UP/ACCENT/SLIDE sit on scan row 2 at
// columns 0..3, sharing columns with the note rows. When two of them are held
// and a note key in a matching column is pressed, the scan reads back a
// phantom note in the same row at the other matching column (three corners of
// a rectangle conduct the fourth). The phantom rises in the same frame as the
// real key and is electrically indistinguishable from it.
static bool kb_mod_col_held(uint8_t col) {
  switch (col) {
    case 0: return inputs[DOWN_KEY].held();
    case 1: return inputs[UP_KEY].held();
    case 2: return inputs[ACCENT_KEY].held();
    default: return inputs[SLIDE_KEY].held();
  }
}
// True when a rising DOWN/UP read at matrix column `col` is a likely phantom:
// a note row has a pressed key at `col` plus a pressed key at another column
// whose row-2 modifier is held.
static bool kb_oct_tap_ghosted(uint8_t col) {
  for (uint8_t r = 0; r < 4; ++r) {
    if (r == 2) continue; // the modifier row itself
    if (!inputs[InputIndex(r * 4 + col)].held()) continue;
    for (uint8_t c = 0; c < 4; ++c)
      if (c != col && kb_mod_col_held(c) && inputs[InputIndex(r * 4 + c)].held())
        return true;
  }
  return false;
}

static void ProcessKeyboardPlay() {
  Leds::Set(PITCH_MODE_LED, true);
  Leds::Set(FUNCTION_MODE_LED, true);
  Leds::Set(TIME_MODE_LED, s_kb_oct_latch);

  // TIME toggles the octave latch.
  if (inputs[TIME_KEY].rising()) s_kb_oct_latch = !s_kb_oct_latch;

  if (s_kb_oct_latch) {
    // Tap DOWN/UP to step the latched octave; ghost-guarded so a phantom
    // octave read (accent/slide held plus overlapping notes) cannot move it.
    if (inputs[UP_KEY].rising() && !kb_oct_tap_ghosted(1) && s_kb_oct < 3) ++s_kb_oct;
    if (inputs[DOWN_KEY].rising() && !kb_oct_tap_ghosted(0) && s_kb_oct > 0) --s_kb_oct;
    Leds::Set(DOWN_KEY_LED, s_kb_oct == 0 || s_kb_oct == 3);
    Leds::Set(UP_KEY_LED,   s_kb_oct == 2 || s_kb_oct == 3);
  }

  // Falling edges: remove released keys from stack, close their MIDI notes;
  // slide back if the top changed and other keys remain.
  bool top_changed = false;
  for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
    if (!inputs[pitched_keys[pi]].falling()) continue;
    const bool was_top = (s_kb_stack_depth > 0 &&
                          s_kb_stack_key[s_kb_stack_depth - 1] == pi);
    const uint8_t off_note = kb_stack_note_of(pi);
    kb_stack_remove(pi);
    if (off_note != 0xFF) midi_kb_note_off(off_note);
    if (was_top) top_changed = true;
  }
  if (top_changed && s_kb_stack_depth > 0) {
    // CV slides back to the next-most-recent held key. Its MIDI note is
    // still open (per-key notes), so nothing to send.
    s_tap_pitch_preview_cv    = s_kb_stack_cv[s_kb_stack_depth - 1];
    s_tap_pitch_preview_slide = true;
  }

  // Ghost guard: two rising note keys in the same matrix row whose columns
  // both match held modifiers are a real+phantom pair that cannot be told
  // apart; drop both rather than play a possibly-wrong note. (With the
  // octave latch there is no need to hold DOWN/UP, so this never triggers
  // in normal latched play.)
  bool ghosted[ARRAY_SIZE(pitched_keys)] = {false};
  for (uint8_t a = 0; a < ARRAY_SIZE(pitched_keys); ++a) {
    const uint8_t ia = pitched_keys[a];
    if (ia >= 16 || !inputs[ia].rising() || !kb_mod_col_held(ia & 3)) continue;
    for (uint8_t b = uint8_t(a + 1); b < ARRAY_SIZE(pitched_keys); ++b) {
      const uint8_t ib = pitched_keys[b];
      if (ib >= 16 || !inputs[ib].rising() || !kb_mod_col_held(ib & 3)) continue;
      if ((ia >> 2) == (ib >> 2)) { ghosted[a] = true; ghosted[b] = true; }
    }
  }

  // Rising edges: legato if stack already non-empty, else fresh trigger.
  for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
    if (!inputs[pitched_keys[pi]].rising() || ghosted[pi]) continue;
    const uint8_t oct    = s_kb_oct_latch ? s_kb_oct : resolve_octave();
    const uint8_t packed = pack_pitch(pi, oct);
    const uint8_t linear = unpack_pitch_linear(packed);
    const uint8_t cv     = clamp_cv(int(linear) + total_transpose);
    const bool    legato = (s_kb_stack_depth > 0);
    const bool    acc    = inputs[ACCENT_KEY].held();
    const bool    sld    = inputs[SLIDE_KEY].held() || legato;
    uint16_t mn = uint16_t(36 + linear) + total_transpose;
    if (mn > 127) mn = 127;
    const uint8_t vel = acc ? 127 : 80;
    // Fresh trigger: retrig gate so envelope opens cleanly. Legato: leave
    // gate continuously high so the 303 envelope does not retrigger.
    if (!legato && s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
    s_tap_pitch_preview_cv     = cv;
    s_tap_pitch_preview_accent = acc;
    s_tap_pitch_preview_slide  = sld;
    s_tap_pitch_preview_gate   = true;
    if (kb_stack_push(pi, cv, uint8_t(mn)))
      midi_kb_note_on(uint8_t(mn), vel);
    break; // only one new note per loop iteration
  }

  // Light exactly the keys in the stack (raw held reads include phantoms and
  // are suppressed from the global pressed-button echo in keyboard mode).
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i)
    Leds::Set(pitch_leds[s_kb_stack_key[i]], true);

  // Close gate when stack drains (per-key MIDI offs already went out).
  if (s_kb_stack_depth == 0 && s_tap_pitch_preview_gate) {
    s_tap_pitch_preview_gate  = false;
    s_tap_pitch_preview_slide = false;
  }
}

// =============================================================================
// FN held: pattern length editor + length LED display (Pattern Write only).
// =============================================================================
static void ProcessLengthEditor(bool dial_pattern_write) {
  // Always-solid FUNCTION_MODE_LED so it stays visible when the clock is stopped
  // (clk_count is frozen and may sit at 0, hiding any blink mask).
  Leds::Set(FUNCTION_MODE_LED, true);

  // FN + ACCENT / FN + SLIDE: live force at the CV stage; only active while held.
  // No persistent stamp — handled below in the DAC output block via fn_mod.

  // Length display and edits are Pattern Write only; FN in Pattern Play shows
  // nothing (it is just the keyboard-mode / direction-mode modifier there).

  if (dial_pattern_write) {
    // FN hold + step press: pattern length editor (Pattern Write only)
    // Black keys set 8-step base (C#=8, D#=16, F#=24, G#=32; +32 in extended mode)
    // White keys add fine offset +1 to +8
    // A# toggles extended mode (bases += 32)

    // LED: show current length position
    // White key: remainder within current 8-step block
    // Black keys: cumulative block coverage (solid = covered, blink = extended block covered)
    const uint8_t cur_len = engine.edit_seq_view().length;
    const bool blink_w = bool((millis() >> 8) & 1); // ~2 Hz, clock-independent
    Leds::Set(OutputIndex((cur_len - 1) & 0x7), true);
    if (s_len_extended || cur_len > 32) {
      Leds::Set(ASHARP_KEY_LED, blink_w);
      Leds::Set(CSHARP_KEY_LED, cur_len > 32 ? blink_w : false);
      Leds::Set(DSHARP_KEY_LED, cur_len > 40 ? blink_w : false);
      Leds::Set(FSHARP_KEY_LED, cur_len > 48 ? blink_w : false);
      Leds::Set(GSHARP_KEY_LED, cur_len > 56 ? blink_w : false);
    } else {
      Leds::Set(ASHARP_KEY_LED, false);
      Leds::Set(CSHARP_KEY_LED, true);           // always: len >= 1
      Leds::Set(DSHARP_KEY_LED, cur_len > 8);
      Leds::Set(FSHARP_KEY_LED, cur_len > 16);
      Leds::Set(GSHARP_KEY_LED, cur_len > 24);
    }

    if (inputs[ASHARP_KEY].rising()) s_len_extended = !s_len_extended;

    const uint8_t ext_add = s_len_extended ? 32 : 0;
    // Black key → select base range only; white key below applies the length.
    // C# = range 1-8 (base 0), D# = 9-16 (base 8), F# = 17-24 (base 16), G# = 25-32 (base 24).
    // In extended mode each base += 32: C#=33-40, D#=41-48, F#=49-56, G#=57-64.
    if (inputs[CSHARP_KEY].rising()) { s_len_black_base =  0 + ext_add; s_len_black_pressed = true; }
    if (inputs[DSHARP_KEY].rising()) { s_len_black_base =  8 + ext_add; s_len_black_pressed = true; }
    if (inputs[FSHARP_KEY].rising()) { s_len_black_base = 16 + ext_add; s_len_black_pressed = true; }
    if (inputs[GSHARP_KEY].rising()) { s_len_black_base = 24 + ext_add; s_len_black_pressed = true; }
    // White keys → fine offset from base (1–8); if no black pressed yet, set 1–8 directly
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        const uint8_t base = s_len_black_pressed ? s_len_black_base : 0;
        engine.SetLength(base + wi + 1);
        midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
      }
    }
  }
}

// =============================================================================
// Global CLEAR combos (CLEAR held, no FN; Pattern Write only). All destructive:
// rotate, randomize, mutate, shift, reverse, clear-only, copy/paste.
// =============================================================================
static void ProcessClearCombos(bool clear_mod, bool fn_mod, bool dial_pattern_write,
                               bool pitch_mod, bool time_mod, bool clk_run) {
  if (!clear_mod || fn_mod || !dial_pattern_write) return;
  bool pat_changed = false;
  // CLEAR + ACCENT rising: randomize the whole pattern.
  if (inputs[ACCENT_KEY].rising()) {
    engine.RandomizeFullPattern();
    pat_changed = true;
  }
  // CLEAR + DOWN rising: rotate time data one step LEFT within length.
  // CLEAR + PITCH + DOWN rising: rotate pitch data only one step LEFT.
  if (inputs[DOWN_KEY].rising()) {
    if (pitch_mod) engine.RotatePitchLeft();
    else           engine.RotateTimeLeft();
    pat_changed = true;
  }
  // CLEAR + UP rising: rotate time data one step RIGHT within length.
  // CLEAR + PITCH + UP rising: rotate pitch data only one step RIGHT.
  if (inputs[UP_KEY].rising()) {
    if (pitch_mod) engine.RotatePitchRight();
    else           engine.RotateTimeRight();
    pat_changed = true;
  }
  // CLEAR + SLIDE rising: Mutate current pattern (small random perturbation).
  if (inputs[SLIDE_KEY].rising()) {
    engine.Mutate();
    pat_changed = true;
  }
  // CLEAR + BACK rising: shift whole pattern (pitch+time) one step LEFT.
  if (inputs[BACK_KEY].rising()) {
    engine.ShiftPatternLeft();
    pat_changed = true;
  }
  // CLEAR + TAP_NEXT rising: shift whole pattern (pitch+time) one step RIGHT.
  if (inputs[TAP_NEXT].rising() && !pitch_mod) {
    engine.ShiftPatternRight();
    pat_changed = true;
  }
  // CLEAR + F# rising: reverse entire pattern (pitch+time) within length.
  if (inputs[FSHARP_KEY].rising()) {
    engine.ReversePattern();
    pat_changed = true;
  }
  // CLEAR + G# rising: clear pitches only (keep time data).
  if (inputs[GSHARP_KEY].rising()) {
    engine.ClearPitchesOnly();
    pat_changed = true;
  }
  // CLEAR + A# rising: clear time data only (keep pitches).
  if (inputs[ASHARP_KEY].rising()) {
    engine.ClearTimesOnly();
    pat_changed = true;
  }
  // Individual-attribute randomize (CLEAR + PITCH_KEY held + white key rising):
  //   C = semitones, D = octaves, E = accents, F = slides,
  //   G = full pitch data (sem+oct+acc+slide), A = time data
  if (pitch_mod && !time_mod) {
    if (inputs[C_KEY].rising())     { engine.RandomizeSemitones();   pat_changed = true; }
    if (inputs[D_KEY].rising())     { engine.RandomizeOctaves();     pat_changed = true; }
    if (inputs[E_KEY].rising())     { engine.RandomizeAccentData();  pat_changed = true; }
    if (inputs[F_KEY].rising())     { engine.RandomizeSlideData();   pat_changed = true; }
    if (inputs[G_KEY].rising())     { engine.RandomizePitchData();   pat_changed = true; }
    if (inputs[A_KEY].rising())     { engine.RandomizeTimeData();    pat_changed = true; }
  }
  if (pat_changed) {
    // Incremental sync drains 2 steps per loop iteration so rapid
    // randomize presses can't saturate the MIDI TX ring. The 147-byte
    // 0x11 blob was blocking clock RX and causing timing drift.
    s_pat_sync_pat = engine.get_patsel();
    s_pat_sync_pos = 0;
    s_pat_sync_len = engine.get_length();
  }
  // CLEAR + C# held + pat key rising: copy pattern (current bank) to clipboard.
  // CLEAR + D# held + pat key rising: paste clipboard into that pattern slot.
  static uint8_t s_clip_buf[PATTERN_SIZE];
  static bool    s_clip_valid = false;
  if (inputs[CSHARP_KEY].held()) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (inputs[i].rising()) {
        const uint8_t bank_now = (engine.get_patsel() >> 3) & 1;
        engine.export_pattern_blob(bank_now * 8 + i, s_clip_buf);
        s_clip_valid = true;
        break;
      }
    }
  } else if (inputs[DSHARP_KEY].held()) {
    if (s_clip_valid) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          const uint8_t bank_now = (engine.get_patsel() >> 3) & 1;
          const uint8_t dst = bank_now * 8 + i;
          engine.import_pattern_blob(dst, s_clip_buf, !clk_run);
          midi_send_pattern_update(dst);
          break;
        }
      }
    }
  }
}

// =============================================================================
// CV output: running = sequenced pitch + engine gate/accent/slide; stopped =
// keys / MIDI live play / audition previews.
// =============================================================================
static void OutputDAC(bool clk_run, bool write_mode, bool track_mode, bool edit_mode,
                      bool pitch_mod, bool fn_mod, bool dial_play_mode) {
  // PITCH_KEY + ACCENT / SLIDE held: live force-accent / force-slide override
  // applied at the CV output stage. Pattern Play / Track Play only -- blocked
  // in Pattern Write and Track Write so it can't interfere with note entry.
  // Releases return to the pattern's stored flags. (FN + ACCENT / FN + SLIDE
  // variant was dropped; PITCH_KEY is the canonical force modifier.)
  const bool force_accent_live =
      pitch_mod && !fn_mod && dial_play_mode && clk_run && inputs[ACCENT_KEY].held();
  const bool force_slide_live  =
      pitch_mod && !fn_mod && dial_play_mode && clk_run && inputs[SLIDE_KEY].held();

  if (clk_run && s_keyboard_mode) {
    // Keyboard play overrides sequenced CV with the audition preview values
    // so the 303 voice tracks the player's keys while the sequencer keeps
    // running its own pattern (sequencer MIDI out is unaffected).
    DAC::SetPitch(s_tap_pitch_preview_cv);
    DAC::SetSlide(s_tap_pitch_preview_slide);
    DAC::SetAccent(s_tap_pitch_preview_accent);
    DAC::SetGate(s_tap_pitch_preview_gate &&
                 (s_tap_pitch_preview_retrig == 0));
    if (s_tap_pitch_preview_retrig) --s_tap_pitch_preview_retrig;
  } else if (clk_run) {
    // Metronome gate is cleared at the next step boundary (one-step click,
    // like the original); this is only a stuck-gate guard if the clock dies.
    if (s_metro_gate_pulse && s_metro_gate_timer > 500) s_metro_gate_pulse = false;
    // Metronome click: override pitch with fixed CV (no transpose); accent on downbeat
    const uint8_t pitch_cv = s_metro_gate_pulse
        ? s_metro_pitch_cv
        : clamp_cv(int(engine.get_pitch_scaled()) + total_transpose);
    DAC::SetPitch(pitch_cv);
    DAC::SetSlide(engine.get_slide_dac() || force_slide_live);
    DAC::SetAccent(engine.get_accent() || force_accent_live || (s_metro_gate_pulse && s_metro_is_downbeat));
    // Force-slide-live: gate stays HIGH across all non-rest steps so the
    // envelope doesn't retrigger between notes. Combined with the slide pin
    // held high, this makes every transition a continuous portamento -- the
    // "infinite slide" the user wants. Rests still drop the gate naturally
    // (engine.resting captures the current step's REST state).
    const bool gate_running = force_slide_live
        ? !engine.resting
        : engine.get_gate();
    DAC::SetGate(gate_running || s_metro_gate_pulse);
  } else {
    bool gate = midi_live_gate();
    // Live MIDI play (clock stopped): drive the CV from the received note, not the
    // sequencer's resting pitch -- otherwise every incoming note plays the resting
    // slot (C1). Note 36 maps to linear 0 (matches engine pitch / get_midi_note).
    uint8_t pitch_cv = gate
        ? clamp_cv(int(midi_live_note()) - 36 + total_transpose)
        : clamp_cv(int(engine.get_pitch()) + total_transpose);
    // Auditioning a non-CV variation (var2/var3) is MIDI-only: the 303 analog voice
    // must stay silent. Only variation-1 edits open the audition CV/gate; the MIDI
    // audition (sent on the variation's channel) is unaffected.
    const bool audition_cv = (engine.get_edit_var() == 0);
    if (audition_cv && s_tap_pitch_preview_gate) {
      pitch_cv = s_tap_pitch_preview_cv;
      gate = true;
      if (s_tap_pitch_preview_retrig) { gate = false; --s_tap_pitch_preview_retrig; }
    } else if (audition_cv && s_back_pitch_preview_gate) {
      pitch_cv = s_back_pitch_preview_cv;
      gate = true;
    } else if (audition_cv && write_mode && !track_mode && s_cfg_menu == CfgMenu::Off && !edit_mode &&
               engine.get_mode() == PITCH_MODE && check_pitch_inputs()) {
      gate = true;
    }

    bool slide_cv = inputs[SLIDE_KEY].held() || midi_live_slide();
    // Audition slide is the captured intent, not a live seq.get_slide() read.
    // Live read would pick up the stale slide bit at the post-advance pitch_pos
    // and slide from the overwritten note to the just-written one.
    if (gate && (s_tap_pitch_preview_gate || s_back_pitch_preview_gate))
      slide_cv = slide_cv || s_tap_pitch_preview_slide;

    // Only drive the pitch CV while the gate is open. With the gate closed the
    // note is released; snapping the CV back to the sequencer's resting pitch
    // slot makes an external (still-tracking) VCO jump to a different note on
    // release. Holding the last latched pitch keeps the released note steady.
    if (gate) DAC::SetPitch(pitch_cv);
    DAC::SetSlide(slide_cv);
    // Accent during audition (TAP-held edit, BACK preview, or live key-write)
    // is locked to the preview state captured at audition entry. Letting the
    // raw ACCENT_KEY drive the analog accent pin while a note is sustaining
    // spikes the envelope and retriggers the voice -- user reported this as
    // "ACCENT retriggers the audtioned note." Toggling accent via ACCENT_KEY
    // still flips the stored flag (handled in input_pitch / step-edit); the
    // user must press BACK + TAP to re-audition with the new accent.
    const bool audition_active = s_tap_pitch_preview_gate || s_back_pitch_preview_gate;
    DAC::SetAccent(audition_active
        ? s_tap_pitch_preview_accent
        : (inputs[ACCENT_KEY].held() || midi_live_accent()));
    DAC::SetGate(gate);
  }
}

// =============================================================================
// loop: poll, MIDI/clock, UI dispatch, DAC output every iteration
// =============================================================================
void loop() {
  // Scan the button matrix at a fixed 1 kHz, not once per loop iteration.
  // PollInputs blanks the LED matrix for its whole scan (~100-200us), so
  // per-iteration scanning ties the LED duty cycle to the loop period. Note
  // ticks lengthen the loop (MIDI note TX, DAC latch), which made every NOTE
  // step visibly pulse the LEDs. A fixed scan cadence keeps the blanking
  // fraction constant regardless of loop timing. Between scans, re-push the
  // last raw samples so PinState debounce/edge semantics stay per-iteration
  // (rising()/falling() still fire for exactly one loop pass).
  static elapsedMicros input_scan_timer;
  if (input_scan_timer >= 1000) {
    input_scan_timer = 0;
    Leds::PauseRefresh();
    PollInputs(inputs);
    // Phase-lock the LED refresh to the scan (same fix as the d650c port's
    // third audit): the scan blanks the matrix, and without resetting the
    // Timer3 phase the dark tail's length beats against the scan cadence
    // (visible shimmer). Re-light the matrix NOW and restart the period.
    TCNT3 = 0;
    TIFR3 = (1 << OCF3A);
    Leds::SendISR();
    Leds::ResumeRefresh();
  } else {
    for (uint8_t i = 0; i < INPUT_COUNT; ++i) inputs[i].push(inputs[i].read());
  }

#if DEBUG
  if (Serial.available() && Serial.read()) {
    const uint8_t s = engine.abs_slot(engine.get_patsel());
    uint8_t noteSteps = 0, ties = 0, notes = 0;
    for (uint8_t st = 0; st < engine.poly_.length; ++st) {
      const uint8_t t = engine.poly_.time(st);
      if (t == 1) ++noteSteps; else if (t == 2) ++ties;
      notes += engine.poly_.voice_count(st);
    }
    Serial.printf("POLY slot=%u flag=%u active=%u len=%u noteSteps=%u ties=%u notes=%u tpos=%u rest=%u chord=%u evar=%u mode=%u\n",
                  s, GlobalSettings.var3_is_poly(s), engine.poly_active_, engine.poly_.length,
                  noteSteps, ties, notes, engine.poly_time_pos_, engine.poly_resting_,
                  engine.poly_chord_pos_, engine.edit_var_, engine.get_mode());
  }
#endif

  // All modal-modifier reads sampled once per iteration. Legacy locals are
  // initialized from `ins` so existing call sites compile unchanged. Phase 1
  // and later will replace `track_mode`/`write_mode`/etc. with `ins.*` and
  // dispatch on `dial` as gating moves into per-mode handlers.
  const InputState ins = read_input_state(inputs);
  const DialMode dial  = dial_mode_of(ins);

  const bool track_mode = ins.track_sel;
  const bool write_mode = ins.write_mode;
  const bool clear_mod  = ins.clear;
  const bool edit_mode  = ins.edit;

  // Release the "pattern-cleared LEDs lit while held" flag once CLEAR is up.
  if (!clear_mod) s_pat_cleared_hold = false;

  const bool fn_mod    = ins.fn;
  const bool pitch_mod = ins.pitch;
  const bool time_mod  = ins.time;

  // Dial-mode gates: destructive / structural ops belong to PatternWrite only.
  // Force-accent / force-slide live overrides belong to play modes only.
  const bool dial_pattern_write = (dial == DialMode::PatternWrite);
  const bool dial_play_mode     = (dial == DialMode::PatternPlay) ||
                                  (dial == DialMode::TrackPlay);
  const bool dial_track_write   = (dial == DialMode::TrackWrite);
  const bool dial_track_play    = (dial == DialMode::TrackPlay);
  const bool dial_track_mode    = dial_track_write || dial_track_play;

  total_transpose = int16_t(int(transpose) + int(engine.get_pattern_transpose()));
  // In Track mode, also add the per-chain-step transpose for the slot the
  // chain cursor is currently on. Encoding: TRACK_TRANSPOSE_ZERO (=12) means
  // "no transpose", so we subtract that offset before adding.
  if (engine.track_active && engine.track_has_chain()) {
    const int8_t step_off = int8_t(engine.TrackGetTranspose(engine.get_chain_pos()))
                          - int8_t(TRACK_TRANSPOSE_ZERO);
    total_transpose = int16_t(int(total_transpose) + int(step_off));
  }

  // Read track index from the 3-bit dial (TRACK_BIT0..2). Tracks 0..7 select
  // 1-of-8 track slots. In track modes, the track index also forces the active
  // bank (bank = track >> 1, matches OS-303 layout).
  const uint8_t cur_tracknum = uint8_t(inputs[TRACK_BIT0].held()
                                  | (inputs[TRACK_BIT1].held() << 1)
                                  | (inputs[TRACK_BIT2].held() << 2));

  if (s_cfg_menu == CfgMenu::Off && !edit_mode && fn_mod &&
      inputs[CLEAR_KEY].rising()) {
    s_cfg_menu = CfgMenu::Midi;
    s_cfg_suppress_clear_exit = true;
  }

  const bool clk_run =
      inputs[RUN].held() || (midi_clk && GlobalSettings.midi_clock_receive);

  const bool prev_midi_clk = midi_clk;
  uint8_t midi_clock_pulses = 0;
  midi_poll(engine, clk_run, midi_clk, midi_clock_pulses);
  // SysEx 0x22 may have updated led_brightness; mirror into the LED driver.
  Leds::brightness = GlobalSettings.led_brightness;
  // Persist any pending SysEx config change to EEPROM on the very next idle
  // iteration. Waiting for WRITE/RUN falling loses config if the user powers
  // off without ever entering edit/run (common with web-only workflows).
  midi_flush_pending_saves();
  // Persist web-edited patterns in the background: one pattern per idle tick,
  // only after a quiet period so rapid edits coalesce. Ensures patterns pushed
  // from the web editor survive a power cycle without requiring a RUN/WRITE
  // toggle. Safe during playback - EEPROM write is ~3-100ms per pattern.
  midi_flush_pending_pattern_saves(engine);
  // Persist hardware-edited resident patterns in the background too. engine.stale
  // is otherwise only flushed on transport stop / dial-mode change, so a user who
  // writes a pattern and powers off without ever running the transport or moving
  // the dial loses the edits. Save when stopped and quiet (2s) so the flash stall
  // happens between keypresses, not on every one.
  {
    static bool     s_stale_prev = false;
    static uint32_t s_stale_ms   = 0;
    if (engine.stale && !s_stale_prev) s_stale_ms = millis();
    s_stale_prev = engine.stale;
    if (engine.stale && !clk_run && (millis() - s_stale_ms) >= 2000) engine.Save();
  }
  // Detect MIDI clock Start rising edge (midi_clk just became true this frame).
  const bool midi_clk_rose = (!prev_midi_clk && midi_clk && GlobalSettings.midi_clock_receive);

  // Determine how many clock ticks to process this iteration.
  // MIDI clock: may be >1 if multiple 0xF8 bytes arrived during a single poll
  // (e.g. interleaved with a long SysEx). DIN sync: always 0 or 1.
  uint8_t clock_ticks = 0;
  if (midi_clock_pulses > 0) {
    clock_ticks = midi_clock_pulses;
  } else if (!midi_clk) {
    clock_ticks = inputs[CLOCK].rising() ? 1 : 0;
  }
  const bool clocked = (clock_ticks > 0);

  // Save pattern data on transport stop or dial-mode change.
  // Dial-mode change covers PatternWrite -> any other dial position (replaces
  // the prior WRITE_MODE.falling() trigger and additionally handles the
  // PatternWrite <-> TrackWrite transition where WRITE_MODE stays high).
  bool dial_changed = false;
  {
    static DialMode s_last_dial = dial;
    static bool     s_last_dial_init = false;
    // First iteration also counts: if we boot with the dial in a Track mode,
    // we need to LoadTrack and set track_active just like a real transition.
    if (!s_last_dial_init || dial != s_last_dial) dial_changed = true;
    s_last_dial = dial;
    s_last_dial_init = true;
  }
  // Save patterns when the transport STOPS, from ANY clock source: the internal
  // RUN button, MIDI clock Stop, or DIN sync stop. (Previously gated to
  // `RUN.falling() && !midi_clk`, which never fired under external MIDI/DIN
  // sync, so externally-synced edits were lost on power-off.) midi_clk already
  // reflects this frame because midi_poll() ran above. Saving on dial-mode
  // change is still avoided (it would block the loop ~25-100 ms mid-playback);
  // edits not stopped through stay in RAM and are lost on power-cycle.
  const bool running_now =
      inputs[RUN].held() || (midi_clk && GlobalSettings.midi_clock_receive);
  static bool s_was_running = false;
  const bool transport_stopped = s_was_running && !running_now;
  s_was_running = running_now;
  if (transport_stopped) {
    engine.Save();
    if (engine.track_stale) engine.SaveTrack();
    midi_flush_pending_saves();
    if (dial_track_mode) emit_track_state(dial, /*clk_run=*/false, cur_tracknum & 0x07);
    // On clock stop: if browsing a different group, switch to it. The
    // group-change detector after the clock loop broadcasts it (once).
    if (s_display_group != engine.get_group())
      engine.SetGroup(s_display_group);
  }
  // Reset edit-mode UI overlays whenever the dial leaves Pattern Write
  // (covers WRITE.falling() and PatternWrite -> TrackWrite, etc).
  if (dial_changed) {
    s_len_extended     = false;
    s_len_black_pressed = false;
    s_step_sel_mode = false;
    s_step_sel_edit = false;
    s_step_sel_time = false;
    s_step_sel      = -1;
    s_step_sel_ext  = false;
    s_dir_mode      = false;
    s_scale_mode    = false;
    if (s_keyboard_mode) {
      s_keyboard_mode = false;
      s_tap_pitch_preview_gate = false;
      midi_kb_all_notes_off();
      kb_stack_clear();
    }
    // Config menu must not survive a dial move; it otherwise keeps consuming
    // keys in the new dial position until CLEAR or FN is pressed.
    s_cfg_menu = CfgMenu::Off;
    s_cfg_suppress_clear_exit = false;
    // Exit any in-progress edit mode so PITCH_MODE / TIME_MODE state cannot
    // bleed across dial positions.
    engine.SetMode(NORMAL_MODE);
    // Persist any in-RAM chain edits before switching away from track mode,
    // otherwise edits made while running are lost when the dial moves to a
    // pattern position (or back later to a different track index).
    if (engine.track_stale) engine.SaveTrack();
    s_track_arm_last = false;
    // Entering Track mode: force bank = track >> 1, load that track. Discard
    // any active Pattern-mode chain so it can't override track playback.
    if (dial_track_mode) {
      const uint8_t track_idx = cur_tracknum & 0x07;
      const uint8_t bank      = uint8_t(track_idx >> 1);
      if (bank != engine.get_group()) engine.SetGroup(bank);
      engine.LoadTrack(track_idx);
      // Pattern-mode chain state must not leak into Track mode; clear it.
      s_chain_active     = false;
      s_chain_len        = 0;
      s_chain_queue_len  = 0;
      s_chain_anchor_key = 0xff;
      s_chain_hold_key   = 0xff;
      s_chain_hold_target_pat = 0xff;
      // TrackPlay: chain advance fires on every wrap. TrackWrite: chain
      // advance stays off so each pat-key just changes the playing pattern.
      // p_select is intentionally NOT reset here -- on entering TrackPlay the
      // user should still see the last pattern that was playing (typically
      // the last one written in TrackWrite) until they press CLEAR.
      engine.track_active = dial_track_play;
    } else {
      engine.track_active = false;
    }
    emit_track_state(dial, clk_run, cur_tracknum & 0x07);
  }

  // Track-index switch while staying in Track mode: persist the outgoing
  // track, then load the new one. Each dial position is its own track slot.
  if (dial_track_mode) {
    static uint8_t s_last_track_idx = 0xFF;
    const uint8_t cur_track_idx = cur_tracknum & 0x07;
    if (s_last_track_idx != 0xFF && cur_track_idx != s_last_track_idx) {
      if (engine.track_stale) engine.SaveTrack();
      s_track_arm_last = false;
      const uint8_t bank = uint8_t(cur_track_idx >> 1);
      if (bank != engine.get_group()) engine.SetGroup(bank);
      engine.LoadTrack(cur_track_idx);
      engine.track_active = dial_track_play;
      emit_track_state(dial, clk_run, cur_track_idx);
    }
    s_last_track_idx = cur_track_idx;
  }

  // Apply chain state received from web editor (SysEx 0x1A), or re-broadcast on config request
  {
    uint8_t rx_al, rx_ap[4], rx_ql, rx_qp[4];
    if (midi_get_received_chain(&rx_al, rx_ap, &rx_ql, rx_qp)) {
      if (rx_al == 0xff) {
        // Config request sentinel: just re-broadcast current chain state, no change
        emit_chain_state();
      } else {
        // Apply new chain state from web
        if (rx_al > 1) {
          s_chain_active = true;
          s_chain_len    = rx_al;
          for (uint8_t ci = 0; ci < rx_al; ++ci) s_chain_pats[ci] = rx_ap[ci];
          if (!clk_run) {
            s_chain_pos = 0;
            engine.SetPattern(rx_ap[0], true);
          } else {
            s_chain_pos = s_chain_len - 1; // chain advance will queue pats[0] next
          }
        } else if (rx_al == 1) {
          s_chain_active = false;
          s_chain_len    = 1;
          s_chain_pats[0] = rx_ap[0];
          engine.SetPattern(rx_ap[0], !clk_run);
        } else {
          s_chain_active = false;
          s_chain_len    = 0;
        }
        s_chain_queue_len = rx_ql;
        for (uint8_t ci = 0; ci < rx_ql; ++ci) s_chain_queued[ci] = rx_qp[ci];
        s_chain_anchor_key      = 0xff;
        s_chain_hold_key        = 0xff;
        s_chain_hold_loop       = false;
        s_chain_hold_target_pat = 0xff;
        emit_chain_state();
      }
    }
  }

  // Ignore RUN button while MIDI clock is actively driving the sequencer:
  // pressing RUN mid-playback would call engine.Reset() and stutter.
  const bool run_rising_effective = inputs[RUN].rising() && !midi_clk;
  if (run_rising_effective || midi_clk_rose) {
    // midi_poll already called engine.Reset() on MIDI Start; only reset for hardware button.
    if (!midi_clk_rose) engine.Reset();
    // Restart chain from first pattern on every start (hardware or MIDI clock).
    if (s_chain_active && s_chain_len > 1) {
      s_chain_pos       = 0;
      s_chain_queue_len = 0;
      engine.SetPattern(s_chain_pats[0], true);
    }
    emit_chain_state();
    if (dial_track_mode) emit_track_state(dial, /*clk_run=*/true, cur_tracknum & 0x07);
  }

  // While stopped, keep the shadow voices loaded for the active slot so editing
  // variation 2/3 targets (and displays) the selected pattern's data. While
  // running, the clock-tick loop handles reload (with note-off) at the switch.
  if (!clk_run && engine.ShadowsNeedReload()) {
    engine.persist_shadows();
    engine.ReloadShadows();
    engine.edit_var_ = 0;
  }

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_scale_mode) {
    ProcessScaleMode();
  } else if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (!clk_run && dial_pattern_write && engine.get_mode() == PITCH_MODE &&
             engine.edit_var_ == 2 && engine.poly_active_) {
    ProcessPolyEdit();
  } else if (edit_mode && !fn_mod && !clk_run && engine.get_mode() != NORMAL_MODE) {
    ProcessEdit(write_mode, clk_run);
  } else if (s_dir_mode) {
    ProcessDirectionMode(dial_pattern_write);
  } else if (dial_track_mode) {
    ProcessTrackUI(dial, dial_track_write, clk_run,
                   uint8_t(cur_tracknum & 0x07), fn_mod, clear_mod, pitch_mod);
  } else {
    // Reset step-select detail-editor sub-state whenever we're not currently in step-select.
    if (!s_step_sel_mode && s_step_sel_edit) { s_step_sel_edit = false; s_step_sel_time = false; }
    // FN + TIME_KEY rising → enter direction mode (allowed in pattern write mode; the
    // TIME_MODE set at line ~1050 is gated by !fn_mod).
    if (fn_mod && inputs[TIME_KEY].rising()) {
      s_dir_mode = true;
      // Overlay modes sit on top of NORMAL_MODE: drop any active PITCH/TIME
      // submode so exiting the overlay always returns to normal.
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (fn_mod && inputs[ACCENT_KEY].rising() && dial_pattern_write) {
      s_scale_mode = true;
      s_scale_fn_entry = true; // FN is down from the entry combo; its release must not exit
      s_scale_cycle_root = 0xFF;
      s_scale_cycle_idx = 0;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (fn_mod && inputs[PITCH_KEY].rising() && !s_step_sel_mode && dial_pattern_write) {
      s_step_sel_mode = true;
      s_step_sel_chain_view = 0; // always default to first chain slot on entry
      s_step_sel_base = 0;
      s_step_sel_ext  = false;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (s_step_sel_mode) {
      ProcessStepSelect(clk_run, dial_pattern_write);
    } else if (s_keyboard_mode && (dial == DialMode::PatternPlay)) {
      ProcessKeyboardPlay();
    } else if (pitch_mod && !fn_mod && !clear_mod && !write_mode && !s_keyboard_mode) {
      ProcessPitchMod();
    } else if (time_mod) {
      Leds::Set(FUNCTION_MODE_LED, true);
      // TODO: performance time effects
    } else if (fn_mod) {
      ProcessLengthEditor(dial_pattern_write);
    } else if (edit_mode && dial_pattern_write && !fn_mod && !clear_mod &&
               !s_metronome_active && engine.get_mode() == NORMAL_MODE) {
      // Hold TAP_NEXT in Pattern Write/normal mode: edit-variation picker.
      ProcessEditVarPicker(clk_run);
    } else {
      ProcessDefault(write_mode, clear_mod, clk_run, dial_pattern_write);
    }
  }

  // ── Pattern chain advance ──
  // Pattern-mode chains never run in Track mode; engine-side track_advance_chain
  // is the chain driver there.
  if (s_chain_active && s_chain_len > 1 && clk_run && !dial_track_mode) {
    const uint8_t cur = engine.get_patsel();
    bool chain_state_changed = false;

    // Activate queued item when its first pattern starts playing
    if (s_chain_queue_len > 0 && cur == s_chain_queued[0]) {
      if (s_chain_queue_len > 1) {
        // Promote full queued chain
        for (uint8_t ci = 0; ci < s_chain_queue_len; ++ci)
          s_chain_pats[ci] = s_chain_queued[ci];
        s_chain_len = s_chain_queue_len;
        s_chain_pos = 0;
      } else {
        // Single pattern: deactivate chain, just play this pattern
        s_chain_active = false;
        s_chain_len    = 0;
      }
      s_chain_queue_len    = 0;
      chain_state_changed  = true;
    }

    if (s_chain_active && s_chain_len > 1) {
      // Update position in active chain
      for (uint8_t ci = 0; ci < s_chain_len; ++ci)
        if (s_chain_pats[ci] == cur) { s_chain_pos = ci; break; }

      // Queue next: hold-loop (only when chain reaches held pattern) > queued chain > advance
      uint8_t next_pat;
      if (s_chain_hold_loop && (s_chain_hold_target_pat == 0xff || cur == s_chain_hold_target_pat)) {
        // Loop: either any pattern (legacy) or specifically the held one
        next_pat = cur;
      } else if (s_chain_queue_len > 0 && s_chain_pos == s_chain_len - 1) {
        // At the last pattern of the current chain: hand off to the queued chain
        next_pat = s_chain_queued[0];
      } else {
        next_pat = s_chain_pats[(s_chain_pos + 1) % s_chain_len];
      }
      engine.SetPattern(next_pat);
    }

    if (chain_state_changed) emit_chain_state();
  }

  // show all pressed buttons
  if (s_cfg_menu == CfgMenu::Off) {
    for (uint8_t i = 0; i < 16; ++i) {
      const InputIndex b = switched_leds[i].button;
      if (!inputs[b].held()) continue;
      if (b == UP_KEY && inputs[C_KEY2].held()) continue;
      // Keyboard mode: raw held reads of note keys include matrix phantoms;
      // ProcessKeyboardPlay lights the real (stack) keys instead.
      if (s_keyboard_mode &&
          (b <= C_KEY2 || b == CSHARP_KEY || b == DSHARP_KEY ||
           b == FSHARP_KEY || b == GSHARP_KEY))
        continue;
      Leds::Set(OutputIndex(i), true);
    }
    // A# is a direct LED (switched_leds[17]) not covered by the 0-15 loop above
    if (inputs[ASHARP_KEY].held() && !s_keyboard_mode)
      Leds::Set(ASHARP_KEY_LED, true);
  }

  // Metronome: auto-exit if transport stopped or write mode released
  if (s_metronome_active && (!clk_run || !write_mode)) {
    s_metronome_active = false;
    s_metro_gate_pulse = false;
    midi_metronome_stop();
  }
  // Per-frame: latch TAP state for metronome recording at next beat boundary
  if (s_metronome_active) {
    if (inputs[TAP_NEXT].falling()) s_metro_tap_released_since_last_beat = true;
  }

  Leds::Swap();

  // Pattern group dial (positions 1-2=group0, 3-4=group1, 5-6=group2, 7=group3)
  // Debounced: require GROUP_DEBOUNCE_FRAMES consecutive frames before accepting a new group.
  if (!inputs[TRACK_SEL].held()) {
    const uint8_t new_group = uint8_t(cur_tracknum <= 1 ? 0 : cur_tracknum <= 3 ? 1 : cur_tracknum <= 5 ? 2 : 3);
    if (s_prev_tracknum == 0xff) {
      // First frame: initialize without debounce
      s_display_group = new_group;
      s_group_debounce_val = new_group;
      s_group_debounce_count = GROUP_DEBOUNCE_FRAMES;
      if (new_group != engine.get_group())
        engine.SetGroup(new_group);
    } else if (new_group != s_display_group) {
      if (new_group == s_group_debounce_val) {
        s_group_debounce_count++;
        if (s_group_debounce_count >= GROUP_DEBOUNCE_FRAMES) {
          s_display_group = new_group;
          s_group_debounce_val = 0xff;
          s_group_debounce_count = 0;
          if (!clk_run)
            engine.SetGroup(new_group);
        }
      } else {
        s_group_debounce_val = new_group;
        s_group_debounce_count = 1;
      }
    }
    s_prev_tracknum = cur_tracknum;
  }

  if (inputs[FUNCTION_KEY].rising() && !edit_mode) {
    if (s_keyboard_mode) {
      s_keyboard_mode = false;
      kb_stack_clear();
      s_tap_pitch_preview_gate = false;
      midi_kb_all_notes_off();
    } else if (s_step_sel_mode) {
      s_step_sel_mode = false;
      s_step_sel_edit = false;
      s_step_sel_time = false;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (s_dir_mode) {
      s_dir_mode = false;
    } else if (s_scale_mode) {
      // FN is also the scale-preset modifier; the exit fires on FN FALLING
      // (tap with no preset note) inside ProcessScaleMode, not on rising.
    } else if (s_cfg_menu == CfgMenu::Midi) {
      s_cfg_menu = CfgMenu::Off;
    } else if (s_cfg_menu == CfgMenu::Off) {
      engine.SetMode(NORMAL_MODE, !clk_run);
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    // PITCH_MODE / TIME_MODE entry: Pattern Write only, and never while an
    // overlay mode owns the keys (direction / scale / step-select). Without
    // the overlay gate, pressing PITCH or TIME inside an overlay silently
    // queued a submode that popped up the moment the overlay exited (and in
    // step-select made bare step presses open the audition gate).
    const bool overlay_mode = s_dir_mode || s_scale_mode || s_step_sel_mode;
    const bool in_poly_edit = (engine.get_mode() == PITCH_MODE && engine.edit_var_ == 2 && engine.poly_active_);
    if (inputs[TIME_KEY].rising()  && dial_pattern_write && !clear_mod && !fn_mod && !edit_mode && !in_poly_edit && !overlay_mode) { engine.SetMode(TIME_MODE, !clk_run); s_time_edit_steps = 0; }
    if (inputs[PITCH_KEY].rising() && dial_pattern_write && !fn_mod && !edit_mode && !clear_mod && !overlay_mode) engine.SetMode(PITCH_MODE, !clk_run);

    // Keyboard play mode toggle: FN + PITCH_KEY rising while dial is in Pattern Play.
    if (fn_mod && inputs[PITCH_KEY].rising() && (dial == DialMode::PatternPlay) &&
        !edit_mode && !clear_mod) {
      s_keyboard_mode = !s_keyboard_mode;
      kb_stack_clear();
      if (!s_keyboard_mode) {
        s_tap_pitch_preview_gate = false;
        midi_kb_all_notes_off();
      }
    }

    // CLEAR + TIME_KEY: toggle metronome tap-write (running + Pattern Write + NORMAL_MODE).
    if (clear_mod && inputs[TIME_KEY].rising() && !fn_mod &&
        clk_run && dial_pattern_write && engine.get_mode() == NORMAL_MODE) {
      s_metronome_active = !s_metronome_active;
        if (!s_metronome_active) {
          midi_metronome_stop();
        } else {
          s_metro_prev_note    = false;
          s_metro_has_activity = false;
          // Clear time only; pitch stream is preserved so newly-tapped NOTE
          // steps consume the user's existing pitches in stream order.
          Sequence &seq = engine.get_sequence();
          const uint8_t len = engine.get_length();
          for (uint8_t i = 0; i < len; ++i) sequence_set_time_at(seq, i, 0);
          engine.stale = true;
          midi_send_pattern_update(engine.get_patsel());
        }
    }

    // CLEAR rising with a pat key held: clear that pattern (only clear path).
    // Pattern Write only -- destructive op.
    if (inputs[CLEAR_KEY].rising() && !fn_mod && !edit_mode && dial_pattern_write) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].held()) {
          const uint8_t pat = uint8_t((engine.get_patsel() >> 3) * 8 + i);
          engine.ClearPattern(pat);
          pattern_cleared_flash_timer = 0;
          s_pat_cleared_hold = true;
          midi_send_pattern_update(pat);
          break;
        }
      }
    }

    // Global CLEAR combos: rotate / randomize / mutate / shift / reverse /
    // clear-only / copy-paste. Destructive, Pattern Write only.
    ProcessClearCombos(clear_mod, fn_mod, dial_pattern_write, pitch_mod,
                       time_mod, clk_run);

    if (inputs[FUNCTION_KEY].falling()) {
      step_counter = false;
      s_len_black_pressed = false;
      s_len_extended = false;
    }
  }

  midi_leader_transport(clocked, clk_run, midi_clk,
                        inputs[RUN].rising(), inputs[RUN].falling());

  // Process every accumulated clock tick. Normally clock_ticks is 0 or 1; if
  // multiple piled up (e.g. SysEx on the same DIN port as the clock) we must
  // drain them in this iteration so the sequencer plays at the actual clock
  // rate. Earlier prototypes capped at 1/iteration but that fell behind when
  // the main loop was slower than the clock rate (audible tempo drop).
  for (uint8_t ct = 0; ct < clock_ticks; ++ct) {
    ++clk_count %= 24;

    if (clk_run) {
      const bool step_boundary = engine.Clock();
      if (step_boundary) {
        if (engine.ShadowsNeedReload()) { // slot/group changed
          engine.persist_shadows();           // flush any RAM shadow edits first
          midi_shadows_all_notes_off(engine); // close old notes on their old channels
          engine.ReloadShadows();
          engine.edit_var_ = 0;               // new pattern/group -> edit variation 1
        }
        engine.AdvanceShadows();              // advance shadows + compute their gate state
        // Wrap-only anchor: send the playhead position via 0x15 only when
        // time_pos transitions back to 0. The web editor counts MIDI clock
        // bytes to interpolate steps between anchors, so we avoid the
        // per-16th SysEx burst that coupled into the audio rail.
        {
          static uint8_t s_anchor_prev_tp = 0xFF;
          const uint8_t tp = engine.get_time_pos();
          if (tp == 0 && s_anchor_prev_tp != 0) {
            midi_send_step_position(engine.get_patsel(), 0);
            // Track view: cursor / next_p may have advanced on this wrap.
            if (dial_track_mode)
              emit_track_state(dial, /*clk_run=*/true, cur_tracknum & 0x07);
          }
          s_anchor_prev_tp = tp;
        }
        // Metronome tap-write: record time data for the step that just played
        if (s_metronome_active) {
          const uint8_t len = engine.get_length();
          const uint8_t write_step =
              uint8_t((engine.get_time_pos() + len - 1) % len);
          // Use current held() state at the beat boundary (not accumulated).
          // This correctly handles: long hold then release before boundary → REST,
          // and fast re-press (released + new press this beat) → NOTE (not TIE).
          const bool tap_now   = inputs[TAP_NEXT].held();
          const bool tap_broke = s_metro_tap_released_since_last_beat;
          const uint8_t tval = tap_now
              ? uint8_t((s_metro_prev_note && !tap_broke) ? 2 : 1)  // TIE only if continuous hold
              : 0;                                                     // not held at boundary: REST
          if (tval != 0) s_metro_has_activity = true;
          s_metro_prev_note = (tval != 0);
          s_metro_tap_released_since_last_beat = false;
          Sequence &wseq = engine.get_sequence();
          sequence_write_time_with_pitch_sync(wseq, write_step, tval);
          engine.stale = true;
          {
            uint8_t pb = PITCH_EMPTY;
            if (tval == 1) {
              const uint8_t slot = wseq.pitch_index_for_note(write_step);
              if (slot < wseq.get_pitch_count()) pb = wseq.pitch[slot];
            }
            midi_send_step_update(engine.get_patsel(), write_step, pb, tval);
          }
          // Original D650C time-write metronome (measured over MIDI on the
          // emulator): low C (DAC code 23) every 8th note (every 2nd step),
          // uniform velocity, no accent, gate held for one full step.
          if ((engine.get_time_pos() % 2) == 0) {
            s_metro_gate_pulse = true;
            s_metro_gate_timer = 0;
            s_metro_pitch_cv = 23;   // untransposed low C (factory code)
            midi_metronome_tick();
          } else {
            s_metro_gate_pulse = false;   // gate off on the in-between step
            midi_metronome_stop();
          }
          // Exit at pattern wrap (step 0) if any TAP activity occurred this pass
          if (engine.get_time_pos() == 0 && s_metro_has_activity) {
            s_metronome_active = false;
            midi_metronome_stop();
          }
        }
      }
      // Every clock tick: MIDI note on/off follows the analog gate (all 3 voices),
      // so MIDI sustain matches the 303 hardware gate exactly. The tick's sends are
      // batched and flushed as one burst (USB in a single packet, then DIN) so all
      // variations' notes land together instead of serializing per voice.
      midi_tick_begin();
      midi_seq_gate_tick(engine, total_transpose);
      midi_shadows_gate_tick(engine, total_transpose);
      midi_tick_flush(); // all variations' note-ONs first, then the queued offs
    }
  }

  // Idle tempo blink (matches the d650c's stopped-transport fallback): DIN and
  // MIDI masters gate their clock while stopped, which froze clk_count and the
  // pattern LED. When the transport is stopped and no clock tick has arrived
  // for a while, free-run the blink counter at 120 BPM (24 ppqn = 20.833 ms)
  // so the selected pattern LED keeps flashing like a real 303. Blink only:
  // sequencing still runs exclusively off real ticks.
  {
    static uint32_t s_last_real_tick_ms = 0;
    static uint32_t s_idle_blink_us = 0;
    if (clocked) {
      s_last_real_tick_ms = millis();
    } else if (!clk_run && (uint32_t)(millis() - s_last_real_tick_ms) > 400) {
      const uint32_t now_us = micros();
      if ((int32_t)(now_us - s_idle_blink_us) >= 20833) {
        // resync after a long gap instead of bursting the backlog
        if ((uint32_t)(now_us - s_idle_blink_us) > 41666u) s_idle_blink_us = now_us;
        else s_idle_blink_us += 20833;
        ++clk_count %= 24;
      }
    }
  }

  // Single group-change broadcast point: catches queued switches applied at
  // wrap AND the direct SetGroup calls above (dial, transport stop, MIDI PC).
  {
    static uint8_t s_prev_group = 0;
    if (engine.get_group() != s_prev_group) {
      s_prev_group = engine.get_group();
      s_display_group = engine.get_group();
      midi_send_group_update(engine.get_group());
      for (uint8_t _pi = 0; _pi < NUM_PATTERNS; ++_pi)
        midi_send_pattern_update(_pi);
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    // FN + UP_KEY rising: toggle triplet step mode on the active pattern.
    // Pattern Write only. UP_KEY isn't otherwise bound under FN. When the
    // toggle turns triplets ON, length is clamped to <=24 (max in triplet
    // mode = ~2 bars of 4/4: 4 quarters * 3 trips * 2 bars = 24 steps).
    if (inputs[UP_KEY].rising() && fn_mod && !pitch_mod && dial_pattern_write) {
      Sequence &seq = engine.get_edit_sequence();
      const bool now_triplet = !seq.is_triplet_mode();
      seq.set_triplet_mode(now_triplet);
      if (now_triplet && seq.length > 24) {
        seq.length = 24;
        midi_send_length_update(engine.get_patsel(), seq.length, engine.get_edit_var());
      }
      engine.stale = true;
    }
    // While FN is held in Pattern Write, light UP_KEY_LED to indicate the
    // current triplet state of the active pattern (lit = triplets, off = 16ths).
    if (fn_mod && dial_pattern_write) {
      Leds::Set(UP_KEY_LED, engine.edit_seq_view().is_triplet_mode());
    }
    // FN + DOWN_KEY: tap-to-count pattern length. Pattern Write only.
    // First tap resets length to 1; each subsequent tap adds one step.
    if (inputs[DOWN_KEY].rising() && fn_mod && !pitch_mod && dial_pattern_write) {
      if (!step_counter) {
        step_counter = true;
        s_len_extended = false; // clear extended state on fresh count
        engine.get_edit_sequence().pitch_pos = 0;
        engine.SetLength(1);
      } else {
        uint8_t cl = engine.edit_seq_view().length;
        engine.SetLength(cl < 64 ? cl + 1 : 64);
      }
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
    }

    // FN + BACK_KEY: length -1, FN + TAP_NEXT: length +1. Pattern Write only.
    // Skipped while PITCH_KEY is held -- BACK is consumed by step-select mode.
    if (fn_mod && !pitch_mod && dial_pattern_write && inputs[BACK_KEY].rising()) {
      uint8_t cl = engine.edit_seq_view().length;
      engine.SetLength(cl > 1 ? cl - 1 : 1);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
    }
    if (fn_mod && !pitch_mod && dial_pattern_write && inputs[TAP_NEXT].rising()) {
      uint8_t cl = engine.edit_seq_view().length;
      engine.SetLength(cl < 64 ? cl + 1 : 64);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
    }

    if (inputs[TAP_NEXT].rising() && !fn_mod && !engine.in_poly_pitch_edit()) {
      if (write_mode && !clk_run) {
        if (engine.get_mode() == PITCH_MODE) {
          engine.get_edit_sequence().ensure_pitch_edit_entry();
          // Audition the current pitch slot. PITCH_MODE walks the pitch stream
          // independently of time data, so don't gate on time(time_pos)==1
          // (that gate broke audition for patterns with no NOTE events yet).
          Sequence &auds = engine.get_edit_sequence();
          const uint8_t pc = auds.get_pitch_count();
          if (pc > 0 && auds.pitch_pos >= 0 && auds.pitch_pos < int(pc) &&
              auds.pitch[auds.pitch_pos] != PITCH_EMPTY) {
            uint16_t mn = uint16_t(36 + auds.get_pitch()) + total_transpose;
            if (mn > 127) mn = 127;
            const bool acc = auds.get_accent();
            const uint8_t vel = acc ? 127 : 80;
            s_tap_pitch_preview_cv = clamp_cv(int(auds.get_pitch()) + total_transpose);
            s_tap_pitch_preview_accent = acc;
            s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
            s_tap_pitch_preview_gate = true;
            midi_audition_note_on(uint8_t(mn), vel);
          }
        } else if (engine.get_mode() == TIME_MODE) {
          engine.AdvanceEditCursor();
          // Preview the note at the new cursor step (NOTE steps only).
          const Sequence &ts = engine.edit_seq_view();
          const uint8_t tp = uint8_t(ts.time_pos & (MAX_STEPS - 1));
          if (tp < ts.length && ts.time(tp) == 1) {
            const uint8_t slot = ts.pitch_index_for_note(tp);
            if (slot < ts.get_pitch_count() && ts.pitch[slot] != PITCH_EMPTY) {
              const uint8_t pb  = ts.pitch[slot];
              const uint8_t lin = unpack_pitch_linear(pb & 0x3f);
              const bool    acc = (pb & (1 << 6)) != 0;
              uint16_t mn = uint16_t(36 + lin) + total_transpose;
              if (mn > 127) mn = 127;
              if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
              s_tap_pitch_preview_cv     = clamp_cv(int(lin) + total_transpose);
              s_tap_pitch_preview_accent = acc;
              s_tap_pitch_preview_slide  = false; // previews never slide
              s_tap_pitch_preview_gate   = true;
              midi_audition_note_on(uint8_t(mn), acc ? 127 : 80);
            }
          }
        }
      }
    }
    // BACK in regular write (TAP not held): step cursor back through the
    // pitch stream. Clamp at 0 instead of wrapping -- pressing BACK on the
    // first step should stay there, not jump to the last note (which made
    // the next TAP play the last step and trigger the wrap-back-to-0 auto
    // exit). first_step=true so the auto-exit check `!first_step && pitch_pos==0`
    // doesn't fire while the user is parked on step 0.
    if (inputs[BACK_KEY].rising() && !fn_mod && write_mode && !clk_run && !edit_mode &&
        !s_step_sel_mode && engine.get_mode() == PITCH_MODE) {
      Sequence &s = engine.get_edit_sequence();
      const uint8_t pc = s.get_pitch_count();
      if (pc > 0 && s.pitch_pos > 0) {
        s.pitch_pos = s.pitch_pos - 1;
        s.sync_time_pos_to_pitch_pos();
        s.first_step = true;
      } else if (pc > 0) {
        // Already on step 0: keep cursor pinned, refresh time_pos display,
        // and ensure first_step stays true so we don't auto-exit.
        s.pitch_pos = 0;
        s.sync_time_pos_to_pitch_pos();
        s.first_step = true;
      }
    }
    if (inputs[TAP_NEXT].falling() && !engine.in_poly_pitch_edit()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off(); // close any open audition note
      // PITCH_MODE: on release, step to the next pitch-stream slot and exit
      // after a full pass. Wrap at the stream's end (pitch_count), not at
      // pattern length: REST/TIE steps own no pitch slot, so wrapping at
      // length made every note past the last one a silent blank TAP press.
      if (!clk_run && write_mode && engine.get_mode() == PITCH_MODE) {
        Sequence &es = engine.get_edit_sequence();
        es.first_step = false;
        const uint8_t epc = es.get_pitch_count();
        ++es.pitch_pos;
        if (es.pitch_pos >= int(epc) || es.pitch_pos >= int(es.length))
          es.pitch_pos = 0;
        es.sync_time_pos_to_pitch_pos();
        if (es.pitch_pos == 0 && !es.first_step)
          engine.SetMode(NORMAL_MODE, true);
      }
      if (!clk_run && engine.get_mode() == TIME_MODE &&
          int(engine.edit_seq_view().time_pos) >= int(engine.edit_seq_view().length) - 1)
        engine.SetMode(NORMAL_MODE, true);
    }
  }

  // regular pattern write mode (no TAP_NEXT).  Suppressed in direction mode so pitched
  // keys select a direction instead of writing notes into the active pattern.
  // Also suppressed in step-select mode (FN + PITCH held) so its inputs don't leak into writes.
  if (s_cfg_menu == CfgMenu::Off && !edit_mode && write_mode && !track_mode && !s_dir_mode
      && !s_step_sel_mode && !s_scale_mode && !engine.in_poly_pitch_edit()) {

    if (engine.get_mode() == TIME_MODE) {
      // SLIDE_KEY step-lock toggle removed (RAM-only after OS-303 migration).
      if (clk_run) {
        input_time(true, true);
      } else if (!fn_mod && check_time_inputs() &&
                 s_time_edit_steps < engine.edit_seq_view().length) {
        input_time(false, false);
        if (s_time_edit_steps >= engine.edit_seq_view().length)
          engine.SetMode(NORMAL_MODE, true);
      } else if (!clk_run && s_time_edit_steps >= engine.edit_seq_view().length)
        engine.SetMode(NORMAL_MODE, true);
    }

    if (engine.get_mode() == PITCH_MODE && !pitch_mod) {
      const bool check = check_pitch_inputs();
      if (clk_run || check) {
        const uint8_t written_note = input_pitch(clk_run, clk_run);
        // After recording the pitch, open audition on the hardware VCO + MIDI out.
        // Set the preview CV/gate so the DAC plays the written note (not the next step).
        if (!clk_run && check && written_note) {
          uint16_t mn = uint16_t(written_note) + total_transpose;
          if (mn > 127) mn = 127;
          const uint8_t vel = inputs[ACCENT_KEY].held() ? 127 : 80;
          s_tap_pitch_preview_cv  = clamp_cv(int(written_note) - 36 + total_transpose);
          // New-note write: audition reflects the user's modifier state at
          // write time, never stored flags. Without this, the OR with
          // seq.get_slide() below would read the post-advance pitch_pos's
          // stale slide bit and slide from the overwritten note to the new
          // one.
          s_tap_pitch_preview_accent = inputs[ACCENT_KEY].held();
          s_tap_pitch_preview_slide  = inputs[SLIDE_KEY].held();
          s_tap_pitch_preview_gate = true;
          midi_audition_note_on(uint8_t(mn), vel);
        }
      }
      // Close audition when all pitch keys released
      if (!clk_run && !check) {
        s_tap_pitch_preview_gate = false;
        midi_audition_note_off();
      }
      // Exit PITCH_MODE after a full linear loop through all steps (back to step 0).
      // first_step is true until the first write+advance, preventing false exit on entry.
      if (!clk_run
          && !engine.get_sequence().first_step
          && engine.get_sequence().pitch_pos == 0)
        engine.SetMode(NORMAL_MODE, true);
    }
    // If mode exited while a write-preview gate was latched, release it on key-up.
    if (!clk_run && !edit_mode && s_tap_pitch_preview_gate && !check_pitch_inputs()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off();
    }

  }

  // Live-transpose reset on pattern switch in play modes: the performance
  // transpose is not persisted, and resets to the no-transpose default whenever
  // the user switches patterns while in Pattern Play or Track Play.
  {
    static uint8_t s_last_patsel_for_transpose = 0xff;
    const uint8_t cur_patsel = engine.get_patsel();
    if (s_last_patsel_for_transpose != 0xff &&
        cur_patsel != s_last_patsel_for_transpose &&
        dial_play_mode) {
      transpose = 12;
    }
    s_last_patsel_for_transpose = cur_patsel;
  }

  OutputDAC(clk_run, write_mode, track_mode, edit_mode, pitch_mod, fn_mod,
            dial_play_mode);

  if (inputs[RUN].falling() && !midi_clk) {
    DAC::SetGate(false);
    engine.Reset();
    // Reset chain to first pattern on stop so next start begins at chain[0].
    if (s_chain_active && s_chain_len > 1) {
      s_chain_pos       = 0;
      s_chain_queue_len = 0;
      engine.SetPattern(s_chain_pats[0], true);
      emit_chain_state();
    }
  }

  // Incremental pattern sync: send 2 step updates per loop iteration.
  if (s_pat_sync_len > 0) {
    const Sequence &sseq = engine.get_sequence();
    for (uint8_t i = 0; i < 2 && s_pat_sync_pos < s_pat_sync_len; ++i, ++s_pat_sync_pos) {
      const uint8_t ti = s_pat_sync_pos;
      const uint8_t tt = sseq.time(ti);
      uint8_t pb = PITCH_EMPTY;
      if (tt == 1) {
        const uint8_t slot = sseq.pitch_index_for_note(ti);
        if (slot < sseq.get_pitch_count()) pb = sseq.pitch[slot];
      }
      midi_send_step_update(s_pat_sync_pat, ti, pb, tt);
    }
    if (s_pat_sync_pos >= s_pat_sync_len)
      s_pat_sync_len = 0;
  }

  // DAC::Send latches the buffered CV values to the hardware ports and pulses
  // the slide line for ~10us. Calling it every loop iteration (1000+/sec)
  // wastes CPU on hardware I/O and creates a high-frequency pulse train on
  // the slide line. OS-303 throttles to ~555 Hz when running and lets it
  // free-run when stopped (for responsive live audition). Same approach here.
  // On clock-tick iterations force an immediate latch (bypass the throttle):
  // MIDI note on/offs are emitted unthrottled in the clock-tick loop above, so
  // without this the analog gate latches up to ~1.8ms after the MIDI already
  // went out -- audible as MIDI-driven voices triggering before the 303.
  static elapsedMicros dac_timer;
  if (!clk_run || clocked || dac_timer > 1800) {
    DAC::Send();
    dac_timer = 0;
  }
}
