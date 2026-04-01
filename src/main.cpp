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
static bool step_counter = false;
static bool midi_clk = false;
static bool wrap_edit = false;
static uint8_t s_time_edit_steps = 0; // counts writes in the current TIME_MODE edit session

/// Stopped-clock CV preview: audition paths set these; unified DAC block applies them.
static bool s_tap_pitch_preview_gate = false;
static uint8_t s_tap_pitch_preview_cv = 0;
static bool s_back_pitch_preview_gate = false;
static uint8_t s_back_pitch_preview_cv = 0;

static elapsedMillis pattern_cleared_flash_timer;
static constexpr uint16_t PATTERN_CLEARED_FLASH_MS = 400;

// Metronome tap-write state (CLEAR+write+clk_run in NORMAL_MODE)
static bool s_metronome_active                  = false;
static bool s_metro_tap_held_this_beat          = false;
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

// Direction mode (FN + PITCH_KEY)
static bool s_dir_mode = false;

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
static bool    s_chain_hold_loop  = false;// true this frame when a chain key is held for looping

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

enum class CfgMenu : uint8_t { Off, Main, MidiCh, MidiChHigh, MidiClk, MidiThru };
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

static void process_config_menu() {
  switch (s_cfg_menu) {
  case CfgMenu::Off:
    return;
  case CfgMenu::Main:
    Leds::Set(FUNCTION_MODE_LED, true);
    if (inputs[CLEAR_KEY].rising()) {
      if (s_cfg_suppress_clear_exit)
        s_cfg_suppress_clear_exit = false;
      else {
        s_cfg_menu = CfgMenu::Off;
        break;
      }
    }
    if (inputs[C_KEY].rising()) s_cfg_menu = CfgMenu::MidiCh;
    if (inputs[D_KEY].rising()) s_cfg_menu = CfgMenu::MidiClk;
    if (inputs[E_KEY].rising()) s_cfg_menu = CfgMenu::MidiThru;
    break;
  case CfgMenu::MidiCh: {
    Leds::Set(CSHARP_KEY_LED, true);
    const uint8_t dc = cfg_display_channel();
    if (dc <= 8) Leds::Set(kCfgWhiteNoteLeds[dc - 1], true);
    else         Leds::Set(kCfgWhiteNoteLeds[dc - 9], true);
    if (inputs[DSHARP_KEY].rising()) s_cfg_menu = CfgMenu::MidiChHigh;
    if (inputs[CLEAR_KEY].rising())  s_cfg_menu = CfgMenu::Main;
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        GlobalSettings.midi_channel = uint8_t(wi + 1);
        cfg_save_midi();
      }
    }
    break;
  }
  case CfgMenu::MidiChHigh:
    if (inputs[CLEAR_KEY].rising())  s_cfg_menu = CfgMenu::Main;
    if (inputs[DSHARP_KEY].rising()) s_cfg_menu = CfgMenu::MidiCh;
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        GlobalSettings.midi_channel = uint8_t(wi + 9);
        cfg_save_midi();
        s_cfg_menu = CfgMenu::MidiCh;
      }
    }
    break;
  case CfgMenu::MidiClk:
    Leds::Set(TIME_MODE_LED, !GlobalSettings.midi_clock_receive);
    if (inputs[TIME_KEY].rising()) {
      GlobalSettings.midi_clock_receive = !GlobalSettings.midi_clock_receive;
      cfg_save_midi();
    }
    if (inputs[CLEAR_KEY].rising()) s_cfg_menu = CfgMenu::Main;
    break;
  case CfgMenu::MidiThru:
    // SLIDE_KEY_LED on = MIDI thru enabled; off = disabled.
    Leds::Set(SLIDE_KEY_LED, GlobalSettings.midi_thru);
    if (inputs[SLIDE_KEY].rising()) {
      GlobalSettings.midi_thru = !GlobalSettings.midi_thru;
      cfg_save_midi();
    }
    if (inputs[CLEAR_KEY].rising()) s_cfg_menu = CfgMenu::Main;
    break;
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
  engine.get_sequence().ensure_pitch_edit_entry();
  if (mod) {
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
        // Write current step, notify web editor, capture MIDI note, then advance.
        engine.SetPitch(i + 13 * oct, flags);
        {
          const uint8_t pp = uint8_t(engine.get_sequence().pitch_pos & (MAX_STEPS - 1));
          midi_send_step_update(engine.get_patsel(), pp,
              engine.get_sequence().pitch[pp],
              engine.get_sequence().get_time());
        }
        const uint8_t written_note = uint8_t(engine.get_midi_note()); // before advance
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
    engine.get_sequence().reflow_pitches_after_time_change(old_t);
  } else if (inputs[UP_KEY].rising()) {
    if (!mod) { engine.Advance(); ++s_time_edit_steps; }
    uint8_t old_t = engine.get_time();
    engine.SetTime(2); written_time = 2; // tie
    engine.get_sequence().reflow_pitches_after_time_change(old_t);
  } else if (inputs[ACCENT_KEY].rising()) {
    if (!mod) { engine.Advance(); ++s_time_edit_steps; }
    uint8_t old_t = engine.get_time();
    engine.SetTime(0); written_time = 0; // rest
    engine.get_sequence().reflow_pitches_after_time_change(old_t);
  }
  if (written_time != 0xFF) {
    // reflow_pitches_after_time_change may have reshuffled pitch bytes across many
    // NOTE steps; send the whole pattern so the web GUI stays fully in sync.
    midi_send_pattern_update(engine.get_patsel());
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
}

// =============================================================================
// Edit-mode LED feedback — current step pitch / time / flags
// =============================================================================
void PrintPitch() {
  // Only light LEDs on NOTE steps — TIE and REST steps are invisible in pitch mode.
  if (engine.get_sequence().get_time() != 1) return;
  const uint8_t semitone = engine.get_semitone();
  if (semitone != PITCH_EMPTY) {
    const Sequence &s = engine.get_sequence();
    const uint8_t note_k = s.get_note_key_index();
    Leds::Set(pitch_leds[note_k], true);

    Leds::Set(ACCENT_KEY_LED, s.get_accent() != 0);
    Leds::Set(SLIDE_KEY_LED,  s.get_slide());
    const uint8_t ladder = s.get_octave();
    Leds::Set(DOWN_KEY_LED,
              ladder == OCTAVE_DOWN || ladder == OCTAVE_DOUBLE_UP);
    const bool redundant_up =
        note_k == PITCH_KEY_HIGH_C && s.get_octave_button() == 1;
    Leds::Set(UP_KEY_LED, ladder > OCTAVE_ZERO && !redundant_up);
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
  Leds::Set(PITCH_MODE_LED, clk_count & 4);
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
  switch (engine.get_mode()) {
  case PITCH_MODE: {
    if (write_mode) {
      const uint8_t updated_note = input_pitch(true, clk_run);
      // When user presses a pitch key while TAP is held, re-audition with the new pitch
      // (the initial TAP audition plays the old/default pitch before the key is pressed).
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
      if (inputs[SLIDE_KEY].rising()) {
        engine.ToggleStepLockFromTimeMode();
        const uint8_t ltp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
        midi_send_step_lock_update(engine.get_patsel(), ltp, engine.get_sequence().step_locked(ltp));
      }
      input_time(true, clk_run);
    }
    PrintTime();
    // fall through
  case NORMAL_MODE:
    break;
  }

  // BACK_KEY: step back one position
  if (inputs[BACK_KEY].rising()) {
    engine.StepBack();
    // Audition the note we are now on if PITCH_MODE + not running
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
  if (inputs[BACK_KEY].falling()) {
    if (!write_mode && !clk_run && engine.get_mode() == PITCH_MODE) {
      s_back_pitch_preview_gate = false;
      midi_audition_note_off();
    }
  }
}

// Default overlay: pattern select, bank A/B, mode LEDs, running step chase
void ProcessDefault(const bool &write_mode, const bool &clear_mod,
               const bool &clk_run) {
  switch (engine.get_mode()) {
  case PITCH_MODE:
    if (clk_run) PrintPitch(); // live pitch chase while sequencer runs; TAP overlay handles stopped clock
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case TIME_MODE:
    if (clk_run) PrintTime(); // live step chase while sequencer runs; TAP overlay handles stopped clock
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case NORMAL_MODE: {
    const uint8_t bank = engine.get_patsel() >> 3;

    // ── LEDs ──
    // flash LED for current pattern
    Leds::Set(OutputIndex(engine.get_patsel() & 0x7), clk_count < 12);
    // chain members (or queued pattern when no chain) shown solid
    if (s_chain_active && s_chain_len > 1) {
      for (uint8_t ci = 0; ci < s_chain_len; ++ci)
        if (s_chain_pats[ci] != engine.get_patsel())
          Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
    } else if (s_chain_anchor_key != 0xff) {
      // while building: show tentative chain solid
      for (uint8_t ci = 0; ci < s_chain_len; ++ci)
        Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
    } else {
      if (engine.get_patsel() != engine.get_next())
        Leds::Set(OutputIndex(engine.get_next() & 0x7), true);
    }
    Leds::Set(ACCENT_KEY_LED, !bank); // A
    Leds::Set(SLIDE_KEY_LED,   bank); // B

    if (clk_run && write_mode) {
      Leds::Set(OutputIndex(engine.get_time_pos() & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + (engine.get_time_pos() >> 3)), true);
    }

    // ── Pattern select inputs ──
    if (clk_run && s_chain_active && s_chain_len > 1) {
      // Running in chain: detect hold-to-loop, key press clears chain
      s_chain_hold_loop = false;
      for (uint8_t ci = 0; ci < s_chain_len; ++ci) {
        if (s_chain_pats[ci] == engine.get_patsel() &&
            inputs[s_chain_pats[ci] & 0x7].held()) {
          s_chain_hold_loop = true;
          break;
        }
      }
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          s_chain_active     = false;
          s_chain_len        = 0;
          s_chain_anchor_key = 0xff;
          engine.SetPattern(bank * 8 + i, false);
        }
      }
    } else if (!clk_run) {
      // Stopped: chain building
      if (s_chain_anchor_key == 0xff) {
        // No anchor yet — first key press starts one
        for (uint8_t i = 0; i < 8; ++i) {
          if (inputs[i].rising()) {
            if (clear_mod) {
              engine.ClearPattern(bank * 8 + i);
              pattern_cleared_flash_timer = 0;
              midi_send_pattern_update(bank * 8 + i);
            } else {
              s_chain_anchor_key = i;
              s_chain_bank       = bank;
              s_chain_pats[0]    = bank * 8 + i;
              s_chain_len        = 1;
              s_chain_active     = false;
              s_chain_hold_loop  = false;
              engine.SetPattern(bank * 8 + i, true);
            }
            break;
          }
        }
      } else {
        // Anchor is held — watch for taps on other keys to extend chain
        for (uint8_t i = 0; i < 8; ++i) {
          if (i == s_chain_anchor_key) continue;
          if (inputs[i].rising() && s_chain_bank == bank) {
            uint8_t lo = (s_chain_anchor_key < i) ? s_chain_anchor_key : i;
            uint8_t hi = (s_chain_anchor_key > i) ? s_chain_anchor_key : i;
            if (hi - lo > 3) hi = lo + 3; // cap at 4 patterns
            s_chain_len = hi - lo + 1;
            for (uint8_t ci = 0; ci < s_chain_len; ++ci)
              s_chain_pats[ci] = bank * 8 + lo + ci;
          }
        }
        // Anchor released: commit chain if len > 1, else single-select
        if (inputs[s_chain_anchor_key].falling()) {
          if (s_chain_len > 1) {
            s_chain_active = true;
            s_chain_pos    = 0;
            engine.SetPattern(s_chain_pats[0], true);
          } else {
            s_chain_active = false;
          }
          s_chain_anchor_key = 0xff;
        }
      }
    } else {
      // Running without chain: normal single-pattern select
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising())
          engine.SetPattern(bank * 8 + i, false);
      }
    }

    // Bank switch — always available; clears any active chain
    if (inputs[ACCENT_KEY].rising()) {
      s_chain_active     = false;
      s_chain_len        = 0;
      s_chain_anchor_key = 0xff;
      engine.SetPattern(engine.get_patsel() % 8, !clk_run); // A
    }
    if (inputs[SLIDE_KEY].rising()) {
      s_chain_active     = false;
      s_chain_len        = 0;
      s_chain_anchor_key = 0xff;
      engine.SetPattern(engine.get_patsel() % 8 + 8, !clk_run); // B
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
  Leds::Set(PITCH_MODE_LED, clk_count & (1 << 2));
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
  PollInputs(inputs);
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
    s_cfg_menu = CfgMenu::Main;
    s_cfg_suppress_clear_exit = true;
  }

  const bool clk_run =
      inputs[RUN].held() || (midi_clk && GlobalSettings.midi_clock_receive);

  bool clocked = false;
  bool midi_clock_pulse = false;
  midi_poll(engine, clk_run, midi_clk, midi_clock_pulse);
  if (midi_clock_pulse) clocked = true;

  if (!midi_clk) clocked = inputs[CLOCK].rising();

  // Save pattern data when exiting write mode or stopping clock
  if ((inputs[WRITE_MODE].falling() && !clk_run) ||
      (inputs[RUN].falling() && !midi_clk)) {
    engine.Save();
  }
  // Reset length editor state when write mode is released
  if (inputs[WRITE_MODE].falling()) {
    s_len_extended     = false;
    s_len_black_pressed = false;
  }

  if (inputs[RUN].rising()) engine.Reset();

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (edit_mode && !fn_mod) {
    ProcessEdit(write_mode, clk_run);
  } else if (s_dir_mode) {
    ProcessDirectionMode();
  } else {
    // FN + PITCH_KEY (key press, not hold) → enter direction mode; takes priority over transpose
    if (!write_mode && fn_mod && inputs[PITCH_KEY].rising()) {
      s_dir_mode = true;
    } else if (pitch_mod && !fn_mod) {
      ProcessPitchMod();
    } else if (time_mod) {
      Leds::Set(TIME_MODE_LED, clk_count & (1 << 2));
      // TODO: performance time effects
    } else if (fn_mod) {
      Leds::Set(FUNCTION_MODE_LED, clk_count & (1 << 2));

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
        // FN+WRITE: pattern length editor
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
  // Keep the next chain pattern queued; on hold-loop, re-queue current.
  if (s_chain_active && s_chain_len > 1 && clk_run) {
    const uint8_t cur = engine.get_patsel();
    for (uint8_t ci = 0; ci < s_chain_len; ++ci) {
      if (s_chain_pats[ci] == cur) { s_chain_pos = ci; break; }
    }
    const uint8_t next_ci = s_chain_hold_loop
      ? s_chain_pos
      : (s_chain_pos + 1) % s_chain_len;
    engine.SetPattern(s_chain_pats[next_ci]);
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
    if (inputs[TAP_NEXT].held())    s_metro_tap_held_this_beat          = true;
    if (inputs[TAP_NEXT].falling()) s_metro_tap_released_since_last_beat = true;
  }

  Leds::Send(ticks);

  tracknum = uint8_t(inputs[TRACK_BIT0].held()
           | (inputs[TRACK_BIT1].held() << 1)
           | (inputs[TRACK_BIT2].held() << 2));

  if (inputs[FUNCTION_KEY].rising()) {
    if (s_dir_mode) {
      s_dir_mode = false;
    } else if (s_cfg_menu == CfgMenu::Main) {
      s_cfg_menu = CfgMenu::Off;
    } else if (s_cfg_menu == CfgMenu::Off) {
      engine.SetMode(NORMAL_MODE, !clk_run);
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    if (inputs[TIME_KEY].rising()  && write_mode) { engine.SetMode(TIME_MODE, !clk_run); s_time_edit_steps = 0; }
    if (inputs[PITCH_KEY].rising() && write_mode && !fn_mod) engine.SetMode(PITCH_MODE, !clk_run);

    if (inputs[CLEAR_KEY].rising() && !fn_mod) {
      uint8_t clear_pat = 0xFF;
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].held()) {
          clear_pat = uint8_t((engine.get_patsel() >> 3) * 8 + i);
          break;
        }
      }
      if (clear_pat != 0xFF) {
        engine.ClearPattern(clear_pat);
        pattern_cleared_flash_timer = 0;
        midi_send_pattern_update(clear_pat);
      } else if (clk_run && write_mode && engine.get_mode() == NORMAL_MODE) {
        // No pattern key held: toggle metronome tap-write mode
        s_metronome_active = !s_metronome_active;
        if (!s_metronome_active) {
          midi_metronome_stop();
        } else {
          s_metro_tap_held_this_beat = false;
          s_metro_prev_note          = false;
          s_metro_has_activity       = false;
          // Instantly clear all time data to REST so user starts fresh
          Sequence &seq = engine.get_sequence();
          const uint8_t len = engine.get_length();
          for (uint8_t i = 0; i < len; ++i)
            sequence_set_time_at(seq, i, 0);
          engine.stale = true;
          midi_send_pattern_update(engine.get_patsel());
        }
      }
    }

    if (inputs[FUNCTION_KEY].falling()) {
      step_counter = false;
      s_len_black_pressed = false;
      s_len_extended = false;
    }
  }

  if (clocked) ++clk_count %= 24;

  midi_leader_transport(clocked, clk_run, midi_clk,
                        inputs[RUN].rising(), inputs[RUN].falling());

  if (clocked && clk_run) {
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
        s_metro_tap_held_this_beat          = false;
        s_metro_tap_released_since_last_beat = false;
        sequence_set_time_at(engine.get_sequence(), write_step, tval);
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
        s_metro_pitch_cv = is_beat_zero ? 32 : 44;  // E3/E4 in standard 303 DAC range
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

  if (s_cfg_menu == CfgMenu::Off) {
    // FN + DOWN_KEY: tap-to-count pattern length. First tap resets length to 1;
    // each subsequent tap adds one step. FN release locks in the count.
    if (inputs[DOWN_KEY].rising() && fn_mod) {
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

    if (inputs[TAP_NEXT].rising()) {
      if (write_mode) {
        if (!clk_run) {
          if (engine.get_mode() == PITCH_MODE) {
            engine.get_sequence().ensure_pitch_edit_entry();
            // Audition current NOTE step; advance to next note happens on TAP falling.
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
      } else {
        if (!clk_run) {
          const bool g = engine.Advance();
          if (g) {
            uint16_t mn = uint16_t(engine.get_midi_note()) + transpose;
            if (mn > 127) mn = 127;
            const uint8_t vel = engine.get_sequence().get_accent() ? 127 : 80;
            s_tap_pitch_preview_cv = uint8_t(engine.get_pitch() + 4 + transpose);
            s_tap_pitch_preview_gate = true;
            midi_audition_note_on(uint8_t(mn), vel);
          }
        } else {
          DAC::SetGate(engine.Advance());
        }
      }
    }
    if (inputs[TAP_NEXT].falling()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off(); // close any open audition note
      // PITCH_MODE write: advance on release (so user sees current note while held),
      // and exit to NORMAL_MODE when we've looped back to the first note.
      if (!clk_run && write_mode && engine.get_mode() == PITCH_MODE) {
        const uint8_t first_note = engine.get_sequence().first_note_idx();
        engine.get_sequence().advance_pitch_to_next_note();
        if (!wrap_edit && uint8_t(engine.get_sequence().pitch_pos) == first_note)
          engine.SetMode(NORMAL_MODE, true);
      }
      if (!wrap_edit && !clk_run && engine.get_mode() == TIME_MODE &&
          engine.get_time_pos() >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }
  }

  // regular pattern write mode (no TAP_NEXT)
  if (s_cfg_menu == CfgMenu::Off && !edit_mode && write_mode && !track_mode) {

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
      // Exit PITCH_MODE only after a full loop back to the first NOTE step.
      // first_step is true until the first write+advance, preventing false exit on entry.
      if (!clk_run
          && !engine.get_sequence().first_step
          && uint8_t(engine.get_sequence().pitch_pos) == engine.get_sequence().first_note_idx())
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
  if (clk_run) {
    // Expire short metronome gate pulse after 25 ms
    if (s_metro_gate_pulse && s_metro_gate_timer > 25) s_metro_gate_pulse = false;
    // Metronome click: override pitch with fixed CV (no transpose); accent on downbeat
    const uint8_t pitch_cv = s_metro_gate_pulse
        ? s_metro_pitch_cv
        : uint8_t(engine.get_pitch() + 4 + transpose);
    DAC::SetPitch(pitch_cv);
    DAC::SetSlide(engine.get_slide_dac());
    DAC::SetAccent(engine.get_accent() || (s_metro_gate_pulse && s_metro_is_downbeat));
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
    DAC::SetAccent(inputs[ACCENT_KEY].held() || midi_live_accent());
    DAC::SetGate(gate);
  }

  if (inputs[RUN].falling() && !midi_clk) {
    DAC::SetGate(false);
    engine.Reset();
    midi_send_step_position(engine.get_patsel(), 0);
  }

  ++ticks;
  DAC::Send();
}
