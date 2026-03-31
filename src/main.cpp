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

/// Stopped-clock CV preview: audition paths set these; unified DAC block applies them.
static bool s_tap_pitch_preview_gate = false;
static uint8_t s_tap_pitch_preview_cv = 0;
static bool s_back_pitch_preview_gate = false;
static uint8_t s_back_pitch_preview_cv = 0;

static elapsedMillis pattern_cleared_flash_timer;
static constexpr uint16_t PATTERN_CLEARED_FLASH_MS = 400;

static Engine engine;

// =============================================================================
// Setup menu: hold FUNCTION + press CLEAR (TAP not held) → main menu.
//   C → MIDI channel (C# + white keys 1–8, D# then 9–16; CLEAR → main).
//   D → MIDI clock: TIME_MODE_LED on = internal only; LED off = MIDI clock receive.
//     Press TIME_KEY to toggle; CLEAR → main. CLEAR in main exits menu entirely.
//   EEPROM bytes 16–17 + midi_apply_settings() — survives power cycle.
// =============================================================================
static const OutputIndex kCfgWhiteNoteLeds[8] = {
    C_KEY_LED, D_KEY_LED, E_KEY_LED, F_KEY_LED,
    G_KEY_LED, A_KEY_LED, B_KEY_LED, C_KEY2_LED};
static const InputIndex kCfgWhiteKeys[8] = {
    C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY, B_KEY, C_KEY2};

enum class CfgMenu : uint8_t { Off, Main, MidiCh, MidiChHigh, MidiClk };
static CfgMenu s_cfg_menu = CfgMenu::Off;
static bool s_cfg_suppress_clear_exit = false;

static uint8_t cfg_display_channel() {
  uint8_t c = GlobalSettings.midi_channel;
  if (c == 0 || c > 16) c = 1;
  return c;
}

static void cfg_save_midi() {
  GlobalSettings.save_midi_to_storage();
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive);
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

void input_pitch(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  engine.get_sequence().ensure_pitch_edit_entry();
  if (mod) {
    if (inputs[ACCENT_KEY].rising()) engine.ToggleAccent();
    if (inputs[SLIDE_KEY].rising())  engine.ToggleSlide();
    if (inputs[UP_KEY].rising())     engine.NudgeOctave(1);
    if (inputs[DOWN_KEY].rising())   engine.NudgeOctave(-1);
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
      } else {
        const uint8_t oct   = resolve_octave();
        const uint8_t flags = (inputs[ACCENT_KEY].held() << 6) |
                              (inputs[SLIDE_KEY].held()   << 7);
        // Write current step, notify web editor, then advance.
        engine.SetPitch(i + 13 * oct, flags);
        {
          const uint8_t pp = uint8_t(engine.get_sequence().pitch_pos & (MAX_STEPS - 1));
          midi_send_step_update(engine.get_patsel(), pp,
              engine.get_sequence().pitch[pp],
              engine.get_sequence().get_time());
        }
        engine.get_sequence().advance_pitch_to_next_note();
        break;
      }
    }
  }
}
void input_time(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  uint8_t written_time = 0xFF;
  if (inputs[DOWN_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(1); written_time = 1; // note
  }
  if (inputs[UP_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(2); written_time = 2; // tie
  }
  if (inputs[ACCENT_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(0); written_time = 0; // rest
  }
  if (written_time != 0xFF) {
    const uint8_t tp = uint8_t(engine.get_sequence().time_pos & (MAX_STEPS - 1));
    midi_send_step_update(engine.get_patsel(), tp,
        engine.get_sequence().pitch[tp], written_time);
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
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive);
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
    if (write_mode)
      input_pitch(true, clk_run);
    PrintPitch();
    break;
  }
  case TIME_MODE:
    if (write_mode) {
      if (inputs[SLIDE_KEY].rising())
        engine.ToggleStepLockFromTimeMode();
      input_time(true, clk_run);
    }
    PrintTime();
    // fall through
  case NORMAL_MODE:
    break;
  }

  // BACK_KEY: step back one position (not a full reset)
  if (inputs[BACK_KEY].rising()) {
    engine.StepBack();
    // Audition the note we are now on if write mode + PITCH_MODE + not running
    if (write_mode && !clk_run && engine.get_mode() == PITCH_MODE &&
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
    if (write_mode && !clk_run && engine.get_mode() == PITCH_MODE) {
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
    // PrintTime() is handled by ProcessEdit (TAP held); nothing to show during live run without TAP
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case NORMAL_MODE:
    // flash LED for current pattern
    Leds::Set(OutputIndex(engine.get_patsel() & 0x7), clk_count < 12);
    // solid LED for queued pattern
    if (engine.get_patsel() != engine.get_next())
      Leds::Set(OutputIndex(engine.get_next() & 0x7), true);
    Leds::Set(ACCENT_KEY_LED, !(engine.get_patsel() >> 3)); // A
    Leds::Set(SLIDE_KEY_LED,   (engine.get_patsel() >> 3)); // B

    if (clk_run && write_mode) {
      Leds::Set(OutputIndex(engine.get_time_pos() & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + (engine.get_time_pos() >> 3)), true);
    }
    // Pattern select inputs
    for (uint8_t i = 0; i < 8; ++i) {
      if (inputs[i].rising()) {
        const uint8_t patsel = (engine.get_patsel() >> 3) * 8 + i;
        if (clear_mod) {
          engine.ClearPattern(patsel);
          pattern_cleared_flash_timer = 0;
        } else
          engine.SetPattern(patsel, !clk_run);
      }
    }
    if (inputs[ACCENT_KEY].rising())
      engine.SetPattern(engine.get_patsel() % 8, !clk_run); // A
    if (inputs[SLIDE_KEY].rising())
      engine.SetPattern(engine.get_patsel() % 8 + 8, !clk_run); // B
    break;
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

  if (inputs[RUN].rising()) engine.Reset();

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (edit_mode) {
    ProcessEdit(write_mode, clk_run);
  } else {
    if (pitch_mod) {
      ProcessPitchMod();
    } else if (time_mod) {
      Leds::Set(TIME_MODE_LED, clk_count & (1 << 2));
      // TODO: performance time effects
    } else if (fn_mod) {
      Leds::Set(FUNCTION_MODE_LED, clk_count & (1 << 2));

      if (write_mode) {
        // show step length on LEDs
        Leds::Set(OutputIndex((engine.get_length() - 1) & 0x7), true);
        Leds::Set(CSHARP_KEY_LED, true);
        Leds::Set(DSHARP_KEY_LED, (engine.get_length() - 1) >> 3);
        Leds::Set(FSHARP_KEY_LED, (engine.get_length() - 1) >> 4);
        Leds::Set(GSHARP_KEY_LED, (engine.get_length() - 1) >= 24);

        if (inputs[DOWN_KEY].rising()) {
          if (step_counter)
            step_counter = engine.BumpLength();
          else {
            engine.SetLength(1);
            step_counter = true;
          }
        }
      }

    } else {
      ProcessDefault(write_mode, clear_mod, clk_run);
    }
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

  Leds::Send(ticks);

  tracknum = uint8_t(inputs[TRACK_BIT0].held()
           | (inputs[TRACK_BIT1].held() << 1)
           | (inputs[TRACK_BIT2].held() << 2));

  if (inputs[FUNCTION_KEY].rising()) {
    if (s_cfg_menu == CfgMenu::Main)
      s_cfg_menu = CfgMenu::Off;
    else if (s_cfg_menu == CfgMenu::Off)
      engine.SetMode(NORMAL_MODE, !clk_run);
  }

  if (s_cfg_menu == CfgMenu::Off) {
    if (inputs[TIME_KEY].rising()  && write_mode) engine.SetMode(TIME_MODE,  !clk_run);
    if (inputs[PITCH_KEY].rising() && write_mode) engine.SetMode(PITCH_MODE, !clk_run);

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
      }
    }

    if (inputs[FUNCTION_KEY].falling()) step_counter = false;
  }

  if (clocked) ++clk_count %= 24;

  midi_leader_transport(clocked, clk_run, midi_clk,
                        inputs[RUN].rising(), inputs[RUN].falling());

  if (clocked && clk_run) {
    if (engine.Clock()) {
      midi_after_clock(engine, transpose);
      // Broadcast current step to web editor (SysEx 0x15)
      midi_send_step_position(engine.get_patsel(), engine.get_time_pos());
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
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
      if (write_mode && inputs[SLIDE_KEY].rising())
        engine.ToggleStepLockFromTimeMode();
      if (clk_run || check_time_inputs()) {
        input_time(clk_run, clk_run);
      } else if (!clk_run && engine.get_time_pos() >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }

    if (engine.get_mode() == PITCH_MODE) {
      const bool check = check_pitch_inputs();
      if (clk_run || check) {
        input_pitch(clk_run, clk_run);
        // After recording the pitch, open a MIDI audition note
        if (!clk_run && check) {
          uint16_t mn = uint16_t(engine.get_midi_note()) + transpose;
          if (mn > 127) mn = 127;
          const uint8_t vel = inputs[ACCENT_KEY].held() ? 127 : 80;
          midi_audition_note_on(uint8_t(mn), vel);
        }
      }
      // Close audition when all pitch keys released
      if (!clk_run && !check) {
        midi_audition_note_off();
      }
      if (!clk_run && engine.get_sequence().pitch_pos >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }

    // BACK without TAP_NEXT held: same step-back as edit overlay (clock stopped)
    if (!clk_run && (engine.get_mode() == PITCH_MODE || engine.get_mode() == TIME_MODE) &&
        inputs[BACK_KEY].rising()) {
      engine.StepBack();
      if (engine.get_mode() == PITCH_MODE && engine.get_sequence().get_time() != 0) {
        uint16_t mn = uint16_t(engine.get_midi_note()) + transpose;
        if (mn > 127) mn = 127;
        const uint8_t vel = engine.get_sequence().get_accent() ? 127 : 80;
        s_back_pitch_preview_cv = uint8_t(engine.get_pitch() + 4 + transpose);
        s_back_pitch_preview_gate = true;
        midi_audition_note_on(uint8_t(mn), vel);
      }
    }
    if (!clk_run && engine.get_mode() == PITCH_MODE && inputs[BACK_KEY].falling()) {
      s_back_pitch_preview_gate = false;
      midi_audition_note_off();
    }
  }

  // ---------------------------------------------------------------------------
  // CV output: running = sequenced pitch + engine gate/accent/slide; stopped = keys
  // ---------------------------------------------------------------------------
  if (clk_run) {
    DAC::SetPitch(engine.get_pitch() + 4 + transpose);
    DAC::SetSlide(engine.get_slide_dac());
    DAC::SetAccent(engine.get_accent());
    DAC::SetGate(engine.get_gate());
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
