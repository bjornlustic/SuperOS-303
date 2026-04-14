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

EEPROMClass storage;
PersistentSettings GlobalSettings;

// =============================================================================
// Globals — timing, debounced inputs, engine, UI timers
// =============================================================================
static uint8_t ticks = 0;
static uint8_t clk_count = 0;
static uint8_t transpose = 12; // range is 0 to 47; 0 = no transpose

static PinState inputs[INPUT_COUNT];

static uint8_t tracknum = 0;
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
static bool s_back_pitch_preview_gate = false;
static uint8_t s_back_pitch_preview_cv = 0;

static elapsedMillis pattern_cleared_flash_timer;
static constexpr uint16_t PATTERN_CLEARED_FLASH_MS = 400;

// Metronome tap-write state (CLEAR+write+clk_run in NORMAL_MODE)
static bool s_metronome_active                  = false;
static bool s_metro_tap_released_since_last_beat = false; // TAP fell during this beat → next press = NOTE, not TIE
static bool s_metro_prev_note                   = false;
static bool s_metro_has_activity                = false;  // any TAP seen this pass; exit at wrap if true
static bool s_metro_gate_pulse                  = false;
static bool s_metro_is_downbeat                 = false;  // downbeat accent flag (every 8 steps)
static uint8_t s_metro_pitch_cv                 = 0;      // final DAC pitch for metronome click
static elapsedMillis s_metro_gate_timer;
// Ratchet gate reset: force CV gate LOW for one loop iteration so the 303 envelope
// re-triggers on the subsequent LOW→HIGH edge (needed when get_gate() stays HIGH across
// a ratchet boundary, e.g. 4x clocks 0→1 and 3→4 which have no natural gate-off).
static bool s_ratchet_gate_reset = false;

// Direction mode (FN + TIME_KEY)
static bool s_dir_mode = false;

// Step-select mode (FN + PITCH_KEY held): pick one step via black-key bank + white key.
// Chase LED lights only when playhead is within the active bank. -1 = no selection.
// `s_step_sel_edit` = entered the per-step detail editor via ACCENT_KEY rising.
static int     s_step_sel      = -1;
static uint8_t s_step_sel_base = 0; // 0, 8, 16, or 24
static bool    s_step_sel_edit = false;
static bool    s_step_sel_time = false; // true = time sub-mode, false = pitch sub-mode
static bool    s_step_sel_mode = false; // toggled: FN+PITCH enters, FN exits

// Incremental pattern sync state (drains 2 steps/loop while running)
static uint8_t s_pat_sync_pat = 0;
static uint8_t s_pat_sync_pos = 0;
static uint8_t s_pat_sync_len = 0;

// FN+write length entry state
static bool    s_len_extended     = false;
static uint8_t s_len_black_base   = 0;
static bool    s_len_black_pressed = false;

static Engine engine;

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

// =============================================================================
// Setup menu: hold FUNCTION + press CLEAR (TAP not held) → main menu.
//   C → MIDI channel (C# + white keys 1–8, D# then 9–16; CLEAR → main).
//   D → MIDI clock: TIME_MODE_LED on = internal only; LED off = MIDI clock receive.
//     Press TIME_KEY to toggle; CLEAR → main.
//   E → MIDI thru: SLIDE_KEY_LED on = thru enabled; off = disabled.
//     Press SLIDE_KEY to toggle; CLEAR → main. CLEAR in main exits menu entirely.
//   EEPROM bytes 16–19 + midi_apply_settings() — survives power cycle.
// =============================================================================
static const OutputIndex kCfgWhiteNoteLeds[8] = {
    C_KEY_LED, D_KEY_LED, E_KEY_LED, F_KEY_LED,
    G_KEY_LED, A_KEY_LED, B_KEY_LED, C_KEY2_LED};
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
static void process_config_menu() {
  if (s_cfg_menu == CfgMenu::Off) return;
  Leds::Set(FUNCTION_MODE_LED, true);

  static bool s_high_bank = false;
  if (inputs[DSHARP_KEY].rising()) s_high_bank = !s_high_bank;
  Leds::Set(DSHARP_KEY_LED, s_high_bank);

  const uint8_t dc = cfg_display_channel();
  if (s_high_bank) {
    if (dc >= 9 && dc <= 16) Leds::Set(OutputIndex((dc - 9) & 0x7), true);
  } else {
    if (dc >= 1 && dc <= 8)  Leds::Set(OutputIndex((dc - 1) & 0x7), true);
  }
  for (uint8_t i = 0; i < 8; ++i) {
    if (inputs[i].rising()) {
      GlobalSettings.midi_channel = uint8_t(i + 1 + (s_high_bank ? 8 : 0));
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
// Octave resolution: DOWN+UP together = DOUBLE_UP (3), else single modifier.
// DOUBLE_UP allows high C (key 12) to reach the 4th octave register.
// ---------------------------------------------------------------------------
static uint8_t resolve_octave() {
  const bool dn = inputs[DOWN_KEY].held();
  const bool up = inputs[UP_KEY].held();
  if (dn && up)  return 3; // OCTAVE_DOUBLE_UP
  if (up)        return 2; // OCTAVE_UP
  if (dn)        return 0; // OCTAVE_DOWN
  return 1;                // OCTAVE_ZERO (center)
}

// Returns the MIDI note number (36-108) written this call, or 0 if nothing was written.
uint8_t input_pitch(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return 0;
  if (mod) {
    // Only resolve reset when stopped — during playback Advance() owns the reset
    // flag and consuming it here would skip step 0 after pattern switch / MIDI Start.
    if (!clk_run) engine.get_sequence().ensure_pitch_edit_entry();
    // TIE and REST steps cannot be edited — only NOTE steps carry pitch/accent/slide/octave.
    if (engine.get_sequence().get_time() != 1) return 0;
    bool flag_changed = false;
    if (inputs[ACCENT_KEY].rising()) { engine.ToggleAccent(); flag_changed = true; }
    if (inputs[SLIDE_KEY].rising())  { engine.ToggleSlide();  flag_changed = true; }
    if (inputs[UP_KEY].rising())     { engine.NudgeOctave(1); flag_changed = true; }
    if (inputs[DOWN_KEY].rising())   { engine.NudgeOctave(-1);flag_changed = true; }
    if (flag_changed) {
      const uint8_t pp = uint8_t(engine.get_sequence().pitch_pos & (MAX_STEPS - 1));
      midi_send_step_update(engine.get_patsel(), pp,
          engine.get_sequence().pitch[pp],
          engine.get_sequence().get_time());
    }
  } else {
    engine.get_sequence().ensure_pitch_write_entry();
  }
  // Higher matrix indices first: high C before low C to avoid crosstalk ghosts.
  for (int pi = int(ARRAY_SIZE(pitched_keys)) - 1; pi >= 0; --pi) {
    const uint8_t i = uint8_t(pi);
    if (inputs[pitched_keys[i]].rising()) {
      if (mod) {
        engine.SetPitchSemitone(i);
        {
          const uint8_t pp = uint8_t(engine.get_sequence().pitch_pos & (MAX_STEPS - 1));
          midi_send_step_update(engine.get_patsel(), pp,
              engine.get_sequence().pitch[pp],
              engine.get_sequence().get_time());
        }
        return uint8_t(engine.get_midi_note()); // return for re-audition while TAP held
      } else {
        const uint8_t oct   = resolve_octave();
        const uint8_t flags = (inputs[ACCENT_KEY].held() << 6) |
                              (inputs[SLIDE_KEY].held()   << 7);
        const uint8_t pp = uint8_t(engine.get_sequence().pitch_pos & (MAX_STEPS - 1));
        // Write pitch into this step's slot without touching time data.
        // REST/TIE steps hold their pitch for when they later become NOTE steps.
        engine.SetPitch(i + 13 * oct, flags);
        midi_send_step_update(engine.get_patsel(), pp,
            engine.get_sequence().pitch[pp],
            engine.get_sequence().get_time());
        // Compute audition note directly from packed pitch — engine.get_midi_note()
        // returns PITCH_DEFAULT for REST/TIE steps, not what was just written.
        const uint8_t written_note = uint8_t(36 + unpack_pitch_linear(uint8_t(i + 13 * oct)));
        // Advance to the next NOTE step, skipping REST and TIE slots.
        engine.get_sequence().advance_pitch_to_next_note();
        return written_note;
      }
    }
  }
  return 0;
}
void input_time(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  uint8_t written_time = 0xFF;
  if (inputs[DOWN_KEY].rising()) {
    if (!mod) { engine.Advance(); ++s_time_edit_steps; }
    uint8_t old_t = engine.get_time();
    engine.SetTime(1); written_time = 1; // note
    if (!clk_run) engine.get_sequence().reflow_pitches_after_time_change(old_t);
  } else if (inputs[UP_KEY].rising()) {
    if (!mod) { engine.Advance(); ++s_time_edit_steps; }
    uint8_t old_t = engine.get_time();
    engine.SetTime(2); written_time = 2; // tie
    if (!clk_run) engine.get_sequence().reflow_pitches_after_time_change(old_t);
  } else if (inputs[ACCENT_KEY].rising()) {
    if (!mod) { engine.Advance(); ++s_time_edit_steps; }
    uint8_t old_t = engine.get_time();
    engine.SetTime(0); written_time = 0; // rest
    if (!clk_run) engine.get_sequence().reflow_pitches_after_time_change(old_t);
  }
  if (written_time != 0xFF) {
    const uint8_t tp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
    if (!clk_run) {
      // Stopped: reflow may have moved pitches, so resync all steps.
      // Incremental sync drains 2 steps/loop to stay gentle on the MIDI TX ring.
      s_pat_sync_pat = engine.get_patsel();
      s_pat_sync_pos = 0;
      s_pat_sync_len = engine.get_length();
    } else {
      // Running: lightweight single-step update only.
      midi_send_step_update(engine.get_patsel(), tp,
          engine.get_sequence().pitch[tp], written_time);
    }
  }
}


// =============================================================================
// Note LED map (chromatic order → switched_leds entries)
// =============================================================================
const MatrixPin note_led[] = {
  switched_leds[0],
  switched_leds[12],
  switched_leds[1],
  switched_leds[13],
  switched_leds[2],
  switched_leds[3],
  switched_leds[14],
  switched_leds[4],
  switched_leds[15],
  switched_leds[5],
  switched_leds[16 + 1],
  switched_leds[6],
  switched_leds[7],
};

extern "C" {
  static void jumptoboot(void) {
    ((int (*)(void))0x1F000)();
  }
}

// =============================================================================
// setup — MIDI, GPIO, optional bootloader, EEPROM load
// =============================================================================
void setup() {
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

  engine.Load();
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
  Leds::brightness = GlobalSettings.led_brightness;
  Leds::BeginRefresh();
}

// =============================================================================
// Edit-mode LED feedback — current step pitch / time / flags
// =============================================================================
void PrintPitch() {
  // Show pitch data for all step types so the user can see what is stored at each position.
  // Accent/slide LEDs are only lit for NOTE steps (they don't apply to REST/TIE).
  const Sequence &s = engine.get_sequence();
  const uint8_t pp = uint8_t(s.pitch_pos & (MAX_STEPS - 1));
  if (s.pitch_is_empty(pp)) return;
  const uint8_t note_k = s.get_note_key_index();
  Leds::Set(pitch_leds[note_k], true);
  const uint8_t ladder = s.get_octave();
  Leds::Set(DOWN_KEY_LED, ladder == OCTAVE_DOWN || ladder == OCTAVE_DOUBLE_UP);
  const bool redundant_up = note_k == PITCH_KEY_HIGH_C && s.get_octave_button() == 1;
  Leds::Set(UP_KEY_LED, ladder > OCTAVE_ZERO && !redundant_up);
  if (s.get_time() == 1) {
    // Accent and slide flags are only meaningful on NOTE steps.
    Leds::Set(ACCENT_KEY_LED, s.get_accent() != 0);
    Leds::Set(SLIDE_KEY_LED,  s.get_slide());
  }
}
void PrintTime() {
  Leds::Set(DOWN_KEY_LED,   engine.get_time() == 1);
  Leds::Set(UP_KEY_LED,     engine.get_time() == 2);
  Leds::Set(ACCENT_KEY_LED, engine.get_time() == 0);
  Leds::Set(SLIDE_KEY_LED,
            engine.get_sequence().step_locked(uint8_t(engine.get_sequence().time_pos)));
}

// ---------------------------------------------------------------------------
// ProcessDirectionMode — FN + PITCH_KEY: select playback direction
// C=Forward D=Reverse E=PingPong F=Random G=HalfRand A=Brownian; FN to exit
// ---------------------------------------------------------------------------
static const InputIndex  kDirKeys[DIR_COUNT] = {C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY};
static const OutputIndex kDirLeds[DIR_COUNT] = {C_KEY_LED, D_KEY_LED, E_KEY_LED, F_KEY_LED, G_KEY_LED, A_KEY_LED};
static const char *const kDirNames[DIR_COUNT] = {"Fwd","Rev","Ping","Rnd","Half","Brn"};

void ProcessDirectionMode() {
  Leds::Set(TIME_MODE_LED, clk_count & 4);
  for (uint8_t d = 0; d < DIR_COUNT; ++d) {
    const bool active = (engine.get_direction() == SequenceDirection(d));
    // Active direction blinks, others solid
    Leds::Set(kDirLeds[d], active ? bool(clk_count & 4) : true);
    if (inputs[kDirKeys[d]].rising()) {
      engine.SetDirection(SequenceDirection(d));
      midi_send_direction_update(d);
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
  // TIME_MODE edit + TIME_KEY held = ratchet view: DOWN/UP/ACCENT show & set ratchet 0/1/2.
  // Released → revert to normal time display. PITCH_MODE no longer has a nudge/ratchet sub-mode.
  const bool ratchet_mod = inputs[TIME_KEY].held();
  switch (engine.get_mode()) {
  case PITCH_MODE: {
    if (write_mode) {
      const uint8_t updated_note = input_pitch(true, clk_run);
      // When user presses a pitch key while TAP is held, re-audition with the new pitch.
      if (!clk_run && updated_note) {
        uint16_t mn = uint16_t(updated_note) + transpose;
        if (mn > 127) mn = 127;
        const uint8_t vel = engine.get_sequence().get_accent() ? 127 : 80;
        s_tap_pitch_preview_cv = uint8_t(engine.get_pitch() + 4 + transpose);
        midi_audition_note_on(uint8_t(mn), vel);
      }
    }
    PrintPitch();
    break;
  }
  case TIME_MODE:
    if (write_mode) {
      if (ratchet_mod) {
        // Ratchet view: DOWN=1x(0), UP=2x(1), ACCENT=3x(2). LED shows current value.
        const uint8_t tp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
        if (inputs[DOWN_KEY].rising())   { engine.SetRatchetAtCurrent(0); midi_send_ratchet_update(engine.get_patsel(), tp, 0); }
        if (inputs[UP_KEY].rising())     { engine.SetRatchetAtCurrent(1); midi_send_ratchet_update(engine.get_patsel(), tp, 1); }
        if (inputs[ACCENT_KEY].rising()) { engine.SetRatchetAtCurrent(2); midi_send_ratchet_update(engine.get_patsel(), tp, 2); }
        const uint8_t r = engine.get_sequence().get_ratchet_val(tp);
        Leds::Set(DOWN_KEY_LED,   r == 0);
        Leds::Set(UP_KEY_LED,     r == 1);
        Leds::Set(ACCENT_KEY_LED, r == 2);
      } else {
        if (inputs[SLIDE_KEY].rising()) {
          engine.ToggleStepLockFromTimeMode();
          const uint8_t ltp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
          midi_send_step_lock_update(engine.get_patsel(), ltp, engine.get_sequence().step_locked(ltp));
        }
        input_time(true, clk_run);
        PrintTime();
      }
    }
    break;
  case NORMAL_MODE:
    break;
  }

  // BACK_KEY: step back one position
  if (inputs[BACK_KEY].rising()) {
    engine.StepBack();
    if (!clk_run && engine.get_mode() == PITCH_MODE &&
        engine.get_sequence().get_time() != 0) {
      uint16_t mn = uint16_t(engine.get_midi_note()) + transpose;
      if (mn > 127) mn = 127;
      const uint8_t vel = engine.get_sequence().get_accent() ? 127 : 80;
      s_back_pitch_preview_cv = uint8_t(engine.get_pitch() + 4 + transpose);
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

// Default overlay: pattern select, bank A/B, mode LEDs, running step chase
void ProcessDefault(const bool &write_mode, const bool &clear_mod,
               const bool &clk_run) {
  switch (engine.get_mode()) {
  case PITCH_MODE:
    if (clk_run) {
      PrintPitch();
      const uint8_t tp = engine.get_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case TIME_MODE:
    if (clk_run) {
      PrintTime();
      { const uint8_t tp = engine.get_time_pos();
        Leds::Set(OutputIndex(tp & 0x7), true);
        Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
        if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4); }
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

    if (clk_run && write_mode) {
      const uint8_t tp = engine.get_time_pos();
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

  const bool pat_clr_flash = pattern_cleared_flash_timer < PATTERN_CLEARED_FLASH_MS;
  Leds::Set(TIME_MODE_LED,     engine.get_mode() == TIME_MODE   || pat_clr_flash);
  Leds::Set(PITCH_MODE_LED,    engine.get_mode() == PITCH_MODE  || pat_clr_flash);
  Leds::Set(FUNCTION_MODE_LED, engine.get_mode() == NORMAL_MODE && !pat_clr_flash);
  if (pat_clr_flash) Leds::Set(ASHARP_KEY_LED, true);
}

// PITCH modifier: live transpose root / octave for performance
void ProcessPitchMod() {
  // Solid FUNCTION_MODE_LED so the indicator is visible while stopped (clk_count blink mask
  // would otherwise sit at 0). On release the next-frame ProcessDefault redraws normal LEDs.
  Leds::Set(FUNCTION_MODE_LED, true);
  Leds::Set(pitch_leds[transpose % 12], true);
  Leds::Set(DOWN_KEY_LED, (transpose / 12) == OCTAVE_DOWN || (transpose / 12) == OCTAVE_DOUBLE_UP);
  Leds::Set(UP_KEY_LED,   (transpose / 12) > OCTAVE_ZERO);

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
// loop — poll, Tick, MIDI/clock, UI, DAC output every iteration
// =============================================================================
void loop() {
  Leds::PauseRefresh();
  PollInputs(inputs);
  Leds::ResumeRefresh();
  engine.Tick();

#if DEBUG
  if (Serial.available() && Serial.read()) {
    for (uint8_t i = 0; i < INPUT_COUNT/2; ++i) {
      Serial.printf("Input #%2u = %x   |  Input #%2u = %x\n",
                    i, inputs[i].state, i + INPUT_COUNT/2, inputs[i + INPUT_COUNT/2].state);
    }
  }
#endif

  const bool track_mode = inputs[TRACK_SEL].held();
  const bool write_mode = inputs[WRITE_MODE].held();
  const bool clear_mod  = inputs[CLEAR_KEY].held();
  const bool edit_mode  = inputs[TAP_NEXT].held();

  const bool fn_mod    = inputs[FUNCTION_KEY].held();
  const bool pitch_mod = inputs[PITCH_KEY].held();
  const bool time_mod  = inputs[TIME_KEY].held();

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

  // Save pattern data when exiting write mode or stopping clock
  if ((inputs[WRITE_MODE].falling() && !clk_run) ||
      (inputs[RUN].falling() && !midi_clk)) {
    engine.Save();
    // On clock stop: if browsing a different group, switch to it
    if (inputs[RUN].falling() && !midi_clk && s_display_group != engine.get_group()) {
      engine.SetGroup(s_display_group);
      midi_send_group_update(s_display_group);
      for (uint8_t _pi = 0; _pi < NUM_PATTERNS; ++_pi)
        midi_send_pattern_update(_pi);
    }
  }
  // Reset length editor state when write mode is released
  if (inputs[WRITE_MODE].falling()) {
    s_len_extended     = false;
    s_len_black_pressed = false;
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
          s_chain_pos    = s_chain_len - 1; // chain advance will queue pats[0] next
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
      }
    }
  }

  if (inputs[RUN].rising() || midi_clk_rose) {
    // midi_poll already called engine.Reset() on MIDI Start; only reset for hardware button.
    if (!midi_clk_rose) engine.Reset();
    // Restart chain from first pattern on every start (hardware or MIDI clock).
    if (s_chain_active && s_chain_len > 1) {
      s_chain_pos       = 0;
      s_chain_queue_len = 0;
      engine.SetPattern(s_chain_pats[0], true);
    }
    emit_chain_state();
  }

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (edit_mode && !fn_mod && !clk_run && engine.get_mode() != NORMAL_MODE) {
    ProcessEdit(write_mode, clk_run);
  } else if (s_dir_mode) {
    ProcessDirectionMode();
  } else {
    // Reset step-select detail-editor sub-state whenever we're not currently in step-select.
    if (!s_step_sel_mode && s_step_sel_edit) { s_step_sel_edit = false; s_step_sel_time = false; }
    // FN + TIME_KEY rising → enter direction mode (allowed in pattern write mode; the
    // TIME_MODE set at line ~1050 is gated by !fn_mod).
    if (fn_mod && inputs[TIME_KEY].rising()) {
      s_dir_mode = true;
    } else if (fn_mod && inputs[PITCH_KEY].rising() && !s_step_sel_mode) {
      s_step_sel_mode = true;
    } else if (s_step_sel_mode) {
      // Step-select mode: two sub-modes — pitch (default) and time.
      // PITCH_KEY switches to pitch sub-mode, TIME_KEY switches to time sub-mode.
      // Pitch sub-mode: only NOTE steps selectable, TAP_NEXT enters detail editor.
      // Time sub-mode: ALL steps selectable, no enter needed — just select & edit.
      Leds::Set(FUNCTION_MODE_LED, true);
      Leds::Set(PITCH_MODE_LED, !s_step_sel_time);
      Leds::Set(TIME_MODE_LED,   s_step_sel_time);
      const uint8_t blen = engine.get_length();
      Sequence &seq = engine.get_sequence();

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
        // Bank pick
        if (inputs[CSHARP_KEY].rising()) s_step_sel_base = 0;
        if (inputs[DSHARP_KEY].rising()) s_step_sel_base = 8;
        if (inputs[FSHARP_KEY].rising()) s_step_sel_base = 16;
        if (inputs[GSHARP_KEY].rising()) s_step_sel_base = 24;
        // Step pick
        for (uint8_t wi = 0; wi < 8; ++wi) {
          if (inputs[kCfgWhiteKeys[wi]].rising()) {
            const uint8_t cand = uint8_t(s_step_sel_base + wi);
            if (cand < blen) {
              // Pitch sub-mode: only NOTE steps. Time sub-mode: any step.
              if (s_step_sel_time || seq.time(cand) == 1)
                s_step_sel = int(cand);
            }
          }
        }
        // Step LEDs: NOTE=bright, TIE=dim, REST=off
        for (uint8_t wi = 0; wi < 8; ++wi) {
          const uint8_t idx = uint8_t(s_step_sel_base + wi);
          if (idx >= blen) break;
          const uint8_t tn = seq.time(idx);
          if (tn == 1) Leds::Set(OutputIndex(wi), true);
          else if (tn == 2) Leds::SetDim(OutputIndex(wi), true);
        }
        // Chase LED
        if (clk_run) {
          const uint8_t tp = engine.get_time_pos();
          if ((tp & ~uint8_t(7)) == s_step_sel_base)
            Leds::Set(OutputIndex(tp & 0x7), bool(clk_count & 4));
        }
        // Cover-bank LEDs
        const bool blinkb = bool((millis() >> 8) & 1);
        const OutputIndex sel_base_led =
            (s_step_sel_base == 0)  ? CSHARP_KEY_LED :
            (s_step_sel_base == 8)  ? DSHARP_KEY_LED :
            (s_step_sel_base == 16) ? FSHARP_KEY_LED : GSHARP_KEY_LED;
        Leds::Set(CSHARP_KEY_LED, (blen > 0)  && (sel_base_led == CSHARP_KEY_LED ? blinkb : true));
        Leds::Set(DSHARP_KEY_LED, (blen > 8)  && (sel_base_led == DSHARP_KEY_LED ? blinkb : true));
        Leds::Set(FSHARP_KEY_LED, (blen > 16) && (sel_base_led == FSHARP_KEY_LED ? blinkb : true));
        Leds::Set(GSHARP_KEY_LED, (blen > 24) && (sel_base_led == GSHARP_KEY_LED ? blinkb : true));
        // Selection flash
        if (s_step_sel >= 0 && (uint8_t(s_step_sel) & ~uint8_t(7)) == s_step_sel_base)
          Leds::Set(OutputIndex(s_step_sel & 0x7), bool((millis() >> 7) & 1));

        if (s_step_sel_time) {
          // ── Time sub-mode: edit directly from picker, no enter needed ──
          if (s_step_sel >= 0) {
            const uint8_t si = uint8_t(s_step_sel);
            bool tchanged = false;
            uint8_t old_t = seq.time(si);
            if (inputs[DOWN_KEY].rising()) {
              // NOTE
              sequence_set_time_at(seq, si, 1);
              engine.stale = true; tchanged = true;
            }
            if (inputs[UP_KEY].rising()) {
              // TIE -- blocked if previous step is a rest (no note to tie from)
              const uint8_t prev_t = (si > 0) ? seq.time(uint8_t(si - 1)) : 0;
              if (prev_t != 0) {
                sequence_set_time_at(seq, si, 2);
                engine.stale = true; tchanged = true;
              }
            }
            if (inputs[ACCENT_KEY].rising()) {
              // REST
              sequence_set_time_at(seq, si, 0);
              engine.stale = true; tchanged = true;
            }
            if (inputs[SLIDE_KEY].rising()) {
              // Toggle step lock
              seq.ToggleStepLock(si);
              engine.stale = true;
              midi_send_step_lock_update(engine.get_patsel(), si, seq.step_locked(si));
            }
            if (tchanged) {
              if (!clk_run) {
                seq.reflow_pitches_at(si, old_t);
                if (seq.time(si) == 1 && seq.pitch_is_empty(si))
                  seq.pitch[si] = PITCH_DEFAULT;
                // Reflow may have shuffled pitches; use incremental sync to
                // avoid the heavy 147-byte blob on every step-select edit.
                s_pat_sync_pat = engine.get_patsel();
                s_pat_sync_pos = 0;
                s_pat_sync_len = engine.get_length();
              } else {
                // Running: just send the edited step. No reflow, no heavy
                // computation. Matches how the web GUI edits work --
                // the web GUI does its own reflow in JS.
                if (seq.time(si) == 1 && seq.pitch_is_empty(si))
                  seq.pitch[si] = PITCH_DEFAULT;
                midi_send_step_update(engine.get_patsel(), si,
                    seq.pitch[si], seq.time(si));
              }
            }
            // Show time info for the selected step
            const uint8_t st = seq.time(si);
            Leds::Set(DOWN_KEY_LED,   st == 1);
            Leds::Set(UP_KEY_LED,     st == 2);
            Leds::Set(ACCENT_KEY_LED, st == 0);
            Leds::Set(SLIDE_KEY_LED,  seq.step_locked(si));
          }
          if (inputs[BACK_KEY].rising()) s_step_sel = -1;
        } else {
          // ── Pitch sub-mode picker ──
          // Enter detail editor on TAP_NEXT rising with a valid selection.
          if (s_step_sel >= 0 && inputs[TAP_NEXT].rising()) {
            s_step_sel_edit = true;
            // Audition the selected note when stopped.
            if (!clk_run) {
              const uint8_t pb = seq.pitch[s_step_sel];
              if (pb != PITCH_EMPTY) {
                const uint8_t linear = unpack_pitch_linear(pb & 0x3f);
                uint16_t mn = uint16_t(36 + linear) + transpose;
                if (mn > 127) mn = 127;
                const bool acc = (pb & (1 << 6)) != 0;
                const uint8_t vel = acc ? 127 : 80;
                s_tap_pitch_preview_cv = uint8_t(linear + 4 + transpose);
                s_tap_pitch_preview_accent = acc;
                s_tap_pitch_preview_gate = true;
                midi_audition_note_on(uint8_t(mn), vel);
              }
            }
          }
          // BACK clears selection.
          else if (inputs[BACK_KEY].rising()) s_step_sel = -1;
        }
      } else {
        // ── Pitch detail editor (unchanged — only reachable from pitch sub-mode) ──
        if (s_step_sel < 0) {
          s_step_sel_edit = false;
        } else {
          const int saved_pp = seq.pitch_pos;
          seq.pitch_pos = s_step_sel;
          bool changed = false;
          if (inputs[ACCENT_KEY].rising()) {
            seq.pitch[s_step_sel] ^= (1 << 6); engine.stale = true; changed = true;
          }
          if (inputs[SLIDE_KEY].rising()) {
            seq.pitch[s_step_sel] ^= (1 << 7); engine.stale = true; changed = true;
          }
          if (inputs[UP_KEY].rising())     { engine.NudgeOctave(+1);  changed = true; }
          if (inputs[DOWN_KEY].rising())   { engine.NudgeOctave(-1);  changed = true; }
          for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
            if (inputs[pitched_keys[pi]].rising()) {
              engine.SetPitchSemitone(pi);
              changed = true;
              break;
            }
          }
          seq.pitch_pos = saved_pp;
          if (changed) {
            midi_send_step_update(engine.get_patsel(), uint8_t(s_step_sel),
                seq.pitch[s_step_sel], seq.time(uint8_t(s_step_sel)));
          }
          if (inputs[BACK_KEY].rising()) s_step_sel_edit = false;
          // Re-audition current step on TAP_NEXT while in the detail editor.
          if (!clk_run && inputs[TAP_NEXT].rising()) {
            const uint8_t ab = seq.pitch[s_step_sel];
            if (ab != PITCH_EMPTY) {
              const uint8_t lin = unpack_pitch_linear(ab & 0x3f);
              uint16_t mn = uint16_t(36 + lin) + transpose;
              if (mn > 127) mn = 127;
              const bool acc = (ab & (1 << 6)) != 0;
              const uint8_t vel = acc ? 127 : 80;
              s_tap_pitch_preview_cv = uint8_t(lin + 4 + transpose);
              s_tap_pitch_preview_accent = acc;
              s_tap_pitch_preview_gate = true;
              midi_audition_note_on(uint8_t(mn), vel);
            }
          }

          const uint8_t pb = seq.pitch[s_step_sel];
          if (pb != PITCH_EMPTY) {
            const uint8_t e        = pb & 0x3f;
            const uint8_t note_k   = e % 13;
            const uint8_t oct_btn  = e / 13;
            const uint8_t linear   = unpack_pitch_linear(e);
            const uint8_t ladder   = linear / 12;
            Leds::Set(pitch_leds[note_k], true);
            Leds::Set(DOWN_KEY_LED, ladder == OCTAVE_DOWN || ladder == OCTAVE_DOUBLE_UP);
            const bool redundant_up = (note_k == PITCH_KEY_HIGH_C) && (oct_btn == 1);
            Leds::Set(UP_KEY_LED, ladder > OCTAVE_ZERO && !redundant_up);
            Leds::Set(ACCENT_KEY_LED, (pb & (1 << 6)) != 0);
            Leds::Set(SLIDE_KEY_LED,  (pb & (1 << 7)) != 0);
          }
        }
      }
    } else if (pitch_mod && !fn_mod && !clear_mod && !write_mode) {
      ProcessPitchMod();
    } else if (time_mod) {
      Leds::Set(FUNCTION_MODE_LED, true);
      // TODO: performance time effects
    } else if (fn_mod) {
      // Always-solid FUNCTION_MODE_LED so it stays visible when the clock is stopped
      // (clk_count is frozen and may sit at 0, hiding any blink mask).
      Leds::Set(FUNCTION_MODE_LED, true);

      // FN + ACCENT / FN + SLIDE: live force at the CV stage; only active while held.
      // No persistent stamp — handled below in the DAC output block via fn_mod.

      if (!write_mode) {
        // LED: white key = position within 8-step block; black keys = cumulative block coverage.
        // Steps 33-64: A# blinks and covered blocks blink to signal extended range.
        // Use millis()-based blink so it works even when the MIDI clock is stopped.
        const uint8_t cur_len = engine.get_length();
        const bool blink = bool((millis() >> 8) & 1); // ~2 Hz, clock-independent
        Leds::Set(OutputIndex((cur_len - 1) & 0x7), true);
        if (cur_len > 32) {
          Leds::Set(ASHARP_KEY_LED, blink);
          Leds::Set(CSHARP_KEY_LED, blink);            // always: we're in extended range
          Leds::Set(DSHARP_KEY_LED, cur_len > 40 ? blink : false);
          Leds::Set(FSHARP_KEY_LED, cur_len > 48 ? blink : false);
          Leds::Set(GSHARP_KEY_LED, cur_len > 56 ? blink : false);
        } else {
          Leds::Set(ASHARP_KEY_LED, false);
          Leds::Set(CSHARP_KEY_LED, true);
          Leds::Set(DSHARP_KEY_LED, cur_len > 8);
          Leds::Set(FSHARP_KEY_LED, cur_len > 16);
          Leds::Set(GSHARP_KEY_LED, cur_len > 24);
        }
      }

      if (write_mode) {
        // FN hold + step press: pattern length editor
        // Black keys set 8-step base (C#=8, D#=16, F#=24, G#=32; +32 in extended mode)
        // White keys add fine offset +1 to +8
        // A# toggles extended mode (bases += 32)

        // LED: show current length position
        // White key: remainder within current 8-step block
        // Black keys: cumulative block coverage (solid = covered, blink = extended block covered)
        const uint8_t cur_len = engine.get_length();
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
            midi_send_length_update(engine.get_patsel(), engine.get_length());
          }
        }
      }
    } else {
      ProcessDefault(write_mode, clear_mod, clk_run);
    }
  }

  // ── Pattern chain advance ──
  if (s_chain_active && s_chain_len > 1 && clk_run) {
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
      Leds::Set(OutputIndex(i), true);
    }
    // A# is a direct LED (switched_leds[17]) not covered by the 0-15 loop above
    if (inputs[ASHARP_KEY].held())
      Leds::Set(ASHARP_KEY_LED, true);
  }

  // Metronome: auto-exit if transport stopped or write mode released
  if (s_metronome_active && (!clk_run || !write_mode)) {
    s_metronome_active = false;
    midi_metronome_stop();
  }
  // Per-frame: latch TAP state for metronome recording at next beat boundary
  if (s_metronome_active) {
    if (inputs[TAP_NEXT].falling()) s_metro_tap_released_since_last_beat = true;
  }

  Leds::Swap();

  tracknum = uint8_t(inputs[TRACK_BIT0].held()
           | (inputs[TRACK_BIT1].held() << 1)
           | (inputs[TRACK_BIT2].held() << 2));

  // Pattern group dial (positions 1-2=group0, 3-4=group1, 5-6=group2, 7=group3)
  // Debounced: require GROUP_DEBOUNCE_FRAMES consecutive frames before accepting a new group.
  if (!inputs[TRACK_SEL].held()) {
    const uint8_t new_group = uint8_t(tracknum <= 1 ? 0 : tracknum <= 3 ? 1 : tracknum <= 5 ? 2 : 3);
    if (s_prev_tracknum == 0xff) {
      // First frame: initialize without debounce
      s_display_group = new_group;
      s_group_debounce_val = new_group;
      s_group_debounce_count = GROUP_DEBOUNCE_FRAMES;
      if (new_group != engine.get_group()) {
        engine.SetGroup(new_group);
        midi_send_group_update(new_group);
      }
    } else if (new_group != s_display_group) {
      if (new_group == s_group_debounce_val) {
        s_group_debounce_count++;
        if (s_group_debounce_count >= GROUP_DEBOUNCE_FRAMES) {
          s_display_group = new_group;
          s_group_debounce_val = 0xff;
          s_group_debounce_count = 0;
          if (!clk_run) {
            engine.SetGroup(new_group);
            midi_send_group_update(new_group);
            for (uint8_t _pi = 0; _pi < NUM_PATTERNS; ++_pi)
              midi_send_pattern_update(_pi);
          }
        }
      } else {
        s_group_debounce_val = new_group;
        s_group_debounce_count = 1;
      }
    }
    s_prev_tracknum = tracknum;
  }

  if (inputs[FUNCTION_KEY].rising() && !edit_mode) {
    if (s_step_sel_mode) {
      s_step_sel_mode = false;
      s_step_sel_edit = false;
      s_step_sel_time = false;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (s_dir_mode) {
      s_dir_mode = false;
    } else if (s_cfg_menu == CfgMenu::Midi) {
      s_cfg_menu = CfgMenu::Off;
    } else if (s_cfg_menu == CfgMenu::Off) {
      engine.SetMode(NORMAL_MODE, !clk_run);
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    if (inputs[TIME_KEY].rising()  && write_mode && !clear_mod && !fn_mod && !edit_mode) { engine.SetMode(TIME_MODE, !clk_run); s_time_edit_steps = 0; }
    if (inputs[PITCH_KEY].rising() && write_mode && !fn_mod && !edit_mode) engine.SetMode(PITCH_MODE, !clk_run);

    // CLEAR + TIME_KEY: toggle metronome tap-write (running + write + NORMAL_MODE).
    if (clear_mod && inputs[TIME_KEY].rising() && !fn_mod &&
        clk_run && write_mode && engine.get_mode() == NORMAL_MODE) {
      s_metronome_active = !s_metronome_active;
        if (!s_metronome_active) {
          midi_metronome_stop();
        } else {
          s_metro_prev_note          = false;
          s_metro_has_activity       = false;
          // Collect all NOTE pitches + existing stash into stash area, then
          // clear time and pitch so new taps start from a clean slate while
          // preserving pitch content for NOTE steps recorded during tap-write.
          Sequence &seq = engine.get_sequence();
          const uint8_t len = engine.get_length();
          const uint8_t max_stash = uint8_t(MAX_STEPS - len);
          uint8_t stash_buf[MAX_STEPS];
          uint8_t stash_n = 0;
          for (uint8_t i = 0; i < len && stash_n < max_stash; ++i) {
            if (seq.time(i) == 1 && !seq.pitch_is_empty(i))
              stash_buf[stash_n++] = seq.pitch[i];
          }
          const uint8_t old_stash = seq.get_stash_count();
          for (uint8_t k = 0; k < old_stash && stash_n < max_stash; ++k) {
            const uint8_t sb = seq.pitch[len + k];
            stash_buf[stash_n++] = (sb == PITCH_EMPTY) ? PITCH_DEFAULT : sb;
          }
          for (uint8_t i = 0; i < len; ++i) {
            sequence_set_time_at(seq, i, 0);
            seq.pitch[i] = PITCH_EMPTY;
          }
          for (uint8_t k = 0; k < stash_n; ++k)
            seq.pitch[len + k] = stash_buf[k];
          seq.set_stash_count(stash_n);
          engine.stale = true;
          midi_send_pattern_update(engine.get_patsel());
        }
    }

    // CLEAR rising with a pat key held: clear that pattern (only clear path).
    if (inputs[CLEAR_KEY].rising() && !fn_mod && !edit_mode) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].held()) {
          const uint8_t pat = uint8_t((engine.get_patsel() >> 3) * 8 + i);
          engine.ClearPattern(pat);
          pattern_cleared_flash_timer = 0;
          midi_send_pattern_update(pat);
          break;
        }
      }
    }

    // ── Global CLEAR combos (clear_mod held, no FN) ──
    if (clear_mod && !fn_mod) {
      bool pat_changed = false;
      // CLEAR + ACCENT rising: randomize pattern but keep ratchets.
      if (inputs[ACCENT_KEY].rising()) {
        engine.RandomizeFullPatternKeepRatchets();
        pat_changed = true;
      }
      // CLEAR + DOWN rising: rotate time data one step LEFT within length.
      if (inputs[DOWN_KEY].rising()) {
        engine.RotateTimeLeft();
        pat_changed = true;
      }
      // CLEAR + UP rising: rotate time data one step RIGHT within length.
      if (inputs[UP_KEY].rising()) {
        engine.RotateTimeRight();
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
      if (inputs[TAP_NEXT].rising()) {
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
      // Individual-attribute randomize (CLEAR + mode-key held + white key rising):
      //   CLEAR + PITCH_KEY + C = semitones, + D = octaves, + E = accents,
      //                     + F = slides,    + G = full pitch data (sem+oct+acc+slide)
      //   CLEAR + TIME_KEY  + C = time data, + D = ratchets
      if (pitch_mod && !time_mod) {
        if (inputs[C_KEY].rising()) { engine.RandomizeSemitones(); pat_changed = true; }
        if (inputs[D_KEY].rising()) { engine.RandomizeOctaves();   pat_changed = true; }
        if (inputs[E_KEY].rising()) { engine.RandomizeAccentData();pat_changed = true; }
        if (inputs[F_KEY].rising()) { engine.RandomizeSlideData(); pat_changed = true; }
        if (inputs[G_KEY].rising()) { engine.RandomizePitchData(); pat_changed = true; }
      } else if (time_mod && !pitch_mod) {
        if (inputs[C_KEY].rising()) { engine.RandomizeTimeData();   pat_changed = true; }
        if (inputs[D_KEY].rising()) { engine.RandomizeRatchetData();pat_changed = true; }
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
      static uint8_t s_clip_buf[128];
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

    if (inputs[FUNCTION_KEY].falling()) {
      step_counter = false;
      s_len_black_pressed = false;
      s_len_extended = false;
    }
  }

  midi_leader_transport(clocked, clk_run, midi_clk,
                        inputs[RUN].rising(), inputs[RUN].falling());

  // Process every accumulated clock tick so none are lost.  Normally clock_ticks
  // is 0 or 1, but when a long SysEx arrives on the same DIN MIDI port as the
  // clock, multiple 0xF8 bytes can pile up inside a single midi_poll() call.
  for (uint8_t ct = 0; ct < clock_ticks; ++ct) {
    ++clk_count %= 24;

    if (clk_run) {
      if (engine.Clock()) {
        midi_after_clock(engine, transpose);
        // Broadcast current step to web editor (SysEx 0x15)
        midi_send_step_position(engine.get_patsel(), engine.get_time_pos());
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
          sequence_set_time_at(wseq, write_step, tval);
          if (tval == 1) {
            // NOTE step: pull pitch from stash if this slot is empty
            if (wseq.pitch_is_empty(write_step)) {
              const uint8_t sc = wseq.get_stash_count();
              if (sc > 0) {
                wseq.pitch[write_step] = wseq.pitch[len + 0];
                for (uint8_t k = 1; k < sc; ++k)
                  wseq.pitch[len + k - 1] = wseq.pitch[len + k];
                wseq.pitch[len + sc - 1] = PITCH_EMPTY;
                wseq.set_stash_count(sc - 1);
              } else {
                wseq.pitch[write_step] = PITCH_DEFAULT;
              }
            }
          } else {
            // TIE or REST: slot must be empty
            wseq.pitch[write_step] = PITCH_EMPTY;
          }
          engine.stale = true;
          midi_send_step_update(engine.get_patsel(), write_step,
              engine.get_sequence().pitch[write_step], tval);
          // Hardware gate pulse so the 303 clicks on every beat
          s_metro_gate_pulse = true;
          s_metro_gate_timer = 0;
          // Downbeat every 8 steps: accent + E3 (DAC 44); offbeats: E4 (DAC 56).
          // Fixed hardware values — not affected by performance transpose.
          const bool is_beat_zero = (engine.get_time_pos() % 8 == 0);
          s_metro_is_downbeat = is_beat_zero;
          s_metro_pitch_cv = is_beat_zero ? 44 : 56;  // E3/E4 in standard 303 DAC range
          midi_metronome_tick(is_beat_zero);
          // Exit at pattern wrap (step 0) if any TAP activity occurred this pass
          if (engine.get_time_pos() == 0 && s_metro_has_activity) {
            s_metronome_active = false;
            midi_metronome_stop();
          }
        }
      } else if (engine.is_ratchet_retrigger()) {
        s_ratchet_gate_reset = true;
        midi_ratchet_retrigger(engine, transpose);
      }
    }
  }

  // After clock ticks: detect when a queued group switch was applied at wrap, then broadcast.
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
    // FN + DOWN_KEY: tap-to-count pattern length. First tap resets length to 1;
    // each subsequent tap adds one step. FN release locks in the count.
    if (inputs[DOWN_KEY].rising() && fn_mod && !pitch_mod) {
      if (!step_counter) {
        step_counter = true;
        s_len_extended = false; // clear extended state on fresh count
        engine.get_sequence().pitch_pos = 0;
        engine.SetLength(1);
      } else {
        uint8_t new_len = engine.get_length() < 64 ? engine.get_length() + 1 : 64;
        engine.SetLength(new_len);
      }
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.get_length());
    }

    // FN + BACK_KEY: length -1 (min 1). FN + TAP_NEXT: length +1 (cap 64). Any transport state.
    // Skipped while PITCH_KEY is held — BACK is consumed by step-select mode.
    if (fn_mod && !pitch_mod && inputs[BACK_KEY].rising()) {
      uint8_t new_len = engine.get_length() > 1 ? engine.get_length() - 1 : 1;
      engine.SetLength(new_len);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.get_length());
    }
    if (fn_mod && !pitch_mod && inputs[TAP_NEXT].rising()) {
      uint8_t new_len = engine.get_length() < 64 ? engine.get_length() + 1 : 64;
      engine.SetLength(new_len);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.get_length());
    }

    if (inputs[TAP_NEXT].rising() && !fn_mod) {
      if (write_mode && !clk_run) {
        if (engine.get_mode() == PITCH_MODE) {
          engine.get_sequence().ensure_pitch_write_entry();
          // Audition current step if it's a NOTE; advance happens on TAP falling.
          if (engine.get_sequence().get_time() == 1) {
            uint16_t mn = uint16_t(engine.get_midi_note()) + transpose;
            if (mn > 127) mn = 127;
            const uint8_t vel = engine.get_sequence().get_accent() ? 127 : 80;
            s_tap_pitch_preview_cv = uint8_t(engine.get_pitch() + 4 + transpose);
            s_tap_pitch_preview_gate = true;
            midi_audition_note_on(uint8_t(mn), vel);
          }
        } else if (engine.get_mode() == TIME_MODE) {
          const bool send = engine.Advance();
          engine.SyncAfterManualAdvance(send);
        }
      }
    }
    if (inputs[TAP_NEXT].falling()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off(); // close any open audition note
      // PITCH_MODE write: advance to the next NOTE step (skipping REST/TIE) on release,
      // exit after a full loop. Linear advance was leaving the cursor on REST/TIE slots
      // whose pitch byte is empty (PITCH_EMPTY), so the next audition played nothing.
      if (!clk_run && write_mode && engine.get_mode() == PITCH_MODE) {
        engine.get_sequence().advance_pitch_to_next_note();
        if (engine.get_sequence().pitch_pos ==
            int(engine.get_sequence().first_note_idx()))
          engine.SetMode(NORMAL_MODE, true);
      }
      if (!clk_run && engine.get_mode() == TIME_MODE &&
          engine.get_time_pos() >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }
  }

  // regular pattern write mode (no TAP_NEXT).  Suppressed in direction mode so pitched
  // keys select a direction instead of writing notes into the active pattern.
  // Also suppressed in step-select mode (FN + PITCH held) so its inputs don't leak into writes.
  if (s_cfg_menu == CfgMenu::Off && !edit_mode && write_mode && !track_mode && !s_dir_mode
      && !s_step_sel_mode) {

    if (engine.get_mode() == TIME_MODE) {
      if (write_mode && inputs[SLIDE_KEY].rising()) {
        engine.ToggleStepLockFromTimeMode();
        const uint8_t ltp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
        midi_send_step_lock_update(engine.get_patsel(), ltp, engine.get_sequence().step_locked(ltp));
      }
      if (clk_run || (!fn_mod && check_time_inputs())) {
        input_time(clk_run, clk_run);
      } else if (!clk_run && s_time_edit_steps >= engine.get_length())
        engine.SetMode(NORMAL_MODE, true);
    }

    if (engine.get_mode() == PITCH_MODE && !pitch_mod) {
      const bool check = check_pitch_inputs();
      if (clk_run || check) {
        const uint8_t written_note = input_pitch(clk_run, clk_run);
        // After recording the pitch, open audition on the hardware VCO + MIDI out.
        // Set the preview CV/gate so the DAC plays the written note (not the next step).
        if (!clk_run && check && written_note) {
          uint16_t mn = uint16_t(written_note) + transpose;
          if (mn > 127) mn = 127;
          const uint8_t vel = inputs[ACCENT_KEY].held() ? 127 : 80;
          s_tap_pitch_preview_cv  = uint8_t(written_note - 36 + 4 + transpose);
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
          && engine.get_sequence().pitch_pos == engine.get_sequence().first_note_idx())
        engine.SetMode(NORMAL_MODE, true);
    }
    // If mode exited while a write-preview gate was latched, release it on key-up.
    if (!clk_run && !edit_mode && s_tap_pitch_preview_gate && !check_pitch_inputs()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off();
    }

  }

  // ---------------------------------------------------------------------------
  // CV output: running = sequenced pitch + engine gate/accent/slide; stopped = keys
  // ---------------------------------------------------------------------------
  // PITCH_KEY + ACCENT / SLIDE held OR FN + ACCENT / SLIDE held (running):
  // non-destructive force modifiers applied at the CV output stage only.
  // Active only while held - releasing returns to the pattern's stored flags.
  const bool force_accent_live =
      ((pitch_mod && !fn_mod) || (fn_mod && !pitch_mod)) && clk_run && inputs[ACCENT_KEY].held();
  const bool force_slide_live  =
      ((pitch_mod && !fn_mod) || (fn_mod && !pitch_mod)) && clk_run && inputs[SLIDE_KEY].held();

  if (clk_run) {
    // Expire short metronome gate pulse after 25 ms
    if (s_metro_gate_pulse && s_metro_gate_timer > 25) s_metro_gate_pulse = false;
    // Metronome click: override pitch with fixed CV (no transpose); accent on downbeat
    const uint8_t pitch_cv = s_metro_gate_pulse
        ? s_metro_pitch_cv
        : uint8_t(engine.get_pitch() + 4 + transpose);
    DAC::SetPitch(pitch_cv);
    DAC::SetSlide(engine.get_slide_dac() || force_slide_live);
    DAC::SetAccent(engine.get_accent() || force_accent_live || (s_metro_gate_pulse && s_metro_is_downbeat));
    DAC::SetGate(s_ratchet_gate_reset ? false : (engine.get_gate() || s_metro_gate_pulse));
    s_ratchet_gate_reset = false;
  } else {
    uint8_t pitch_cv = uint8_t(engine.get_pitch() + 4 + transpose);
    bool gate = midi_live_gate();
    if (s_tap_pitch_preview_gate) {
      pitch_cv = s_tap_pitch_preview_cv;
      gate = true;
    } else if (s_back_pitch_preview_gate) {
      pitch_cv = s_back_pitch_preview_cv;
      gate = true;
    } else if (write_mode && !track_mode && s_cfg_menu == CfgMenu::Off && !edit_mode &&
               engine.get_mode() == PITCH_MODE && check_pitch_inputs()) {
      gate = true;
    }

    bool slide_cv = inputs[SLIDE_KEY].held() || midi_live_slide();
    if (gate && (s_tap_pitch_preview_gate || s_back_pitch_preview_gate ||
                 (write_mode && engine.get_mode() == PITCH_MODE && check_pitch_inputs())))
      slide_cv = slide_cv || engine.get_sequence().get_slide();

    DAC::SetPitch(pitch_cv);
    DAC::SetSlide(slide_cv);
    DAC::SetAccent(inputs[ACCENT_KEY].held() || midi_live_accent() ||
                   (s_tap_pitch_preview_gate && s_tap_pitch_preview_accent));
    DAC::SetGate(gate);
  }

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
    midi_send_step_position(engine.get_patsel(), 0);
  }

  // Incremental pattern sync: send 2 step updates per loop iteration.
  if (s_pat_sync_len > 0) {
    const Sequence &sseq = engine.get_sequence();
    for (uint8_t i = 0; i < 2 && s_pat_sync_pos < s_pat_sync_len; ++i, ++s_pat_sync_pos)
      midi_send_step_update(s_pat_sync_pat, s_pat_sync_pos,
          sseq.pitch[s_pat_sync_pos], sseq.time(s_pat_sync_pos));
    if (s_pat_sync_pos >= s_pat_sync_len)
      s_pat_sync_len = 0; // done
  }

  ++ticks;
  DAC::Send();
}
