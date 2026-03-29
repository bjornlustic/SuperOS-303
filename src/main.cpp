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
/// FN+CLEAR opens Main on CLEAR's rising edge; ignore that same edge for "CLEAR exits Main".
static bool s_cfg_suppress_clear_exit = false;

static uint8_t cfg_display_channel() {
  uint8_t c = GlobalSettings.midi_channel;
  if (c == 0 || c > 16)
    c = 1;
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
    if (inputs[C_KEY].rising())
      s_cfg_menu = CfgMenu::MidiCh;
    if (inputs[D_KEY].rising())
      s_cfg_menu = CfgMenu::MidiClk;
    break;
  case CfgMenu::MidiCh: {
    Leds::Set(CSHARP_KEY_LED, true);
    const uint8_t dc = cfg_display_channel();
    if (dc <= 8)
      Leds::Set(kCfgWhiteNoteLeds[dc - 1], true);
    else
      Leds::Set(kCfgWhiteNoteLeds[dc - 9], true);
    if (inputs[DSHARP_KEY].rising())
      s_cfg_menu = CfgMenu::MidiChHigh;
    if (inputs[CLEAR_KEY].rising())
      s_cfg_menu = CfgMenu::Main;
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        GlobalSettings.midi_channel = uint8_t(wi + 1);
        cfg_save_midi();
      }
    }
    break;
  }
  case CfgMenu::MidiChHigh:
    if (inputs[CLEAR_KEY].rising())
      s_cfg_menu = CfgMenu::Main;
    if (inputs[DSHARP_KEY].rising())
      s_cfg_menu = CfgMenu::MidiCh;
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        GlobalSettings.midi_channel = uint8_t(wi + 9);
        cfg_save_midi();
        s_cfg_menu = CfgMenu::MidiCh;
      }
    }
    break;
  case CfgMenu::MidiClk:
    // TIME_MODE_LED on = internal (ignore MIDI Clock/Start/Stop); off = MIDI clock receive.
    Leds::Set(TIME_MODE_LED, !GlobalSettings.midi_clock_receive);
    if (inputs[TIME_KEY].rising()) {
      GlobalSettings.midi_clock_receive = !GlobalSettings.midi_clock_receive;
      cfg_save_midi();
    }
    if (inputs[CLEAR_KEY].rising())
      s_cfg_menu = CfgMenu::Main;
    break;
  }
}

// =============================================================================
// Write-mode input helpers — map matrix keys to Engine sequence edits
// =============================================================================

uint8_t check_pitch_inputs() {
  uint8_t notes = 0;
  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].held()) {
      ++notes;
    }
  }
  return notes;
}
bool check_time_inputs() {
  if (inputs[DOWN_KEY].held()) { return true; }
  if (inputs[UP_KEY].held()) { return true; }
  if (inputs[ACCENT_KEY].held()) { return true; }
  return false;
}
void input_pitch(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  if (mod) {
    if (inputs[ACCENT_KEY].rising()) engine.ToggleAccent();
    if (inputs[SLIDE_KEY].rising()) engine.ToggleSlide();
    if (inputs[UP_KEY].rising()) engine.NudgeOctave(1);
    if (inputs[DOWN_KEY].rising()) engine.NudgeOctave(-1);
  }
  // Higher matrix indices first: [8] high C before [1] low C so crosstalk ghosts
  // on the same read column do not record low C + UP instead of high C.
  for (int pi = int(ARRAY_SIZE(pitched_keys)) - 1; pi >= 0; --pi) {
    const uint8_t i = uint8_t(pi);
    if (inputs[pitched_keys[i]].rising()) {
      if (mod) {
        engine.SetPitchSemitone(i);
      } else {
        // Snapshot octave/flags at the moment the key rises, before advancing,
        // so the flags reflect the actual held modifier state for this note.
        const uint8_t oct = 1 - inputs[DOWN_KEY].held() + inputs[UP_KEY].held();
        const uint8_t flags = (inputs[ACCENT_KEY].held() << 6) |
                              (inputs[SLIDE_KEY].held() << 7);
        engine.get_sequence().AdvancePitch();
        engine.SetPitch(i + 13 * oct, flags);
        // Stop after the first rising key — matrix crosstalk can cause ghost
        // rising() signals on other keys in the same scan cycle. Processing only
        // the first key found prevents a phantom note from overwriting the real one.
        break;
      }
    }
  }
}
void input_time(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  if (inputs[DOWN_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(1); // note
  }
  if (inputs[UP_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(2); // tie
  }
  if (inputs[ACCENT_KEY].rising()) {
    if (!mod) engine.Advance();
    engine.SetTime(0); // rest
  }
}


// =============================================================================
// Note LED map (chromatic order → switched_leds entries) — UI / future use
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
  // Bootloader entry (Teensy++ layout); invoked at power-up if TAP_NEXT held.
  static void jumptoboot(void) {
    ((int (*)(void))0x1F000)();
  }
}

// =============================================================================
// setup — MIDI, GPIO, optional bootloader, EEPROM load
// =============================================================================
void setup() {
  midi_init(&engine);

  for (uint8_t i = 0; i < ARRAY_SIZE(INPUTS); ++i) {
    pinMode(INPUTS[i], INPUT); // pullup?
  }
  for (uint8_t i = 0; i < ARRAY_SIZE(OUTPUTS); ++i) {
    pinMode(OUTPUTS[i], OUTPUT);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWriteFast(select_pin[i], HIGH);
  }

  PollInputs(inputs);
  if (inputs[TAP_NEXT].held()) {
    jumptoboot();
  }

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
  const uint8_t semitone = engine.get_semitone();
  if (semitone != PITCH_EMPTY) {

    const Sequence &s = engine.get_sequence();
    const uint8_t note_k = s.get_note_key_index();
    Leds::Set(pitch_leds[note_k], true);

    Leds::Set(ACCENT_KEY_LED, s.get_accent() != 0);
    Leds::Set(SLIDE_KEY_LED, s.get_slide());
    const uint8_t ladder = s.get_octave();
    Leds::Set(DOWN_KEY_LED,
              ladder == OCTAVE_DOWN || ladder == OCTAVE_DOUBLE_UP);
    // Linear ladder lights UP for high C + center octave; UP key was not pressed — redundant.
    const bool redundant_up =
        note_k == PITCH_KEY_HIGH_C && s.get_octave_button() == 1;
    Leds::Set(UP_KEY_LED, ladder > OCTAVE_ZERO && !redundant_up);
  }
  // empty step: no LEDs lit — pattern is blank here
}
void PrintTime() {
  Leds::Set(DOWN_KEY_LED, engine.get_time() == 1);
  Leds::Set(UP_KEY_LED, engine.get_time() == 2);
  Leds::Set(ACCENT_KEY_LED, engine.get_time() == 0);
  Leds::Set(SLIDE_KEY_LED,
            engine.get_sequence().step_locked(uint8_t(engine.get_sequence().time_pos)));
}

// TAP_NEXT held: pitch/time edit UI; BACK resets sequence position
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
    // fall through: TIME_MODE shares NORMAL_MODE exit (no extra handling)
  case NORMAL_MODE:
    break;
  }
  if (inputs[BACK_KEY].rising())
    engine.Reset();
}

// Default overlay: pattern select, bank A/B, mode LEDs, running step chase
void ProcessDefault(const bool &write_mode, const bool &clear_mod,
               const bool &clk_run) {
  switch (engine.get_mode()) {
  case PITCH_MODE:
    PrintPitch();
    if (!write_mode)
      engine.SetMode(NORMAL_MODE); // you're not supposed to be in here
    break;

  case TIME_MODE:
    PrintTime();
    if (!write_mode)
      engine.SetMode(NORMAL_MODE); // you're not supposed to be in here
    break;

  case NORMAL_MODE:
    // flash LED for current pattern
    Leds::Set(OutputIndex(engine.get_patsel() & 0x7), clk_count < 12);
    // solid LED for queued pattern
    if (engine.get_patsel() != engine.get_next())
      Leds::Set(OutputIndex(engine.get_next() & 0x7), true);
    Leds::Set(ACCENT_KEY_LED, !(engine.get_patsel() >> 3)); // A
    Leds::Set(SLIDE_KEY_LED, (engine.get_patsel() >> 3));   // B

    if (clk_run && write_mode) {
      // chasing light for pattern step
      Leds::Set(OutputIndex(engine.get_time_pos() & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + (engine.get_time_pos() >> 3)), true);
    }
    // Inputs for Pattern Select
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

  // no modifier - show current mode; flash for pattern clear
  const bool pat_clr_flash = pattern_cleared_flash_timer < PATTERN_CLEARED_FLASH_MS;
  Leds::Set(TIME_MODE_LED, engine.get_mode() == TIME_MODE || pat_clr_flash);
  Leds::Set(PITCH_MODE_LED, engine.get_mode() == PITCH_MODE || pat_clr_flash);
  Leds::Set(FUNCTION_MODE_LED, engine.get_mode() == NORMAL_MODE && !pat_clr_flash);
  if (pat_clr_flash) Leds::Set(ASHARP_KEY_LED, true);
}

// PITCH modifier: live transpose root / octave for performance
void ProcessPitchMod() {
  Leds::Set(PITCH_MODE_LED, clk_count & (1 << 2));
  Leds::Set(pitch_leds[transpose % 12], true);
  Leds::Set(DOWN_KEY_LED, (transpose / 12) == OCTAVE_DOWN || (transpose / 12) == OCTAVE_DOUBLE_UP);
  Leds::Set(UP_KEY_LED, (transpose / 12) > OCTAVE_ZERO);

  // TODO: beat-synced transpose change?

  // check pitch keys to set new root note
  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].rising()) {
      transpose = (transpose / 12) * 12 + i;
    }
  }
  // check octave keys to jump by 12
  if (inputs[DOWN_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 - 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
  if (inputs[UP_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 + 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
  // TODO: other pitch effects?
}

// =============================================================================
// loop — poll, Tick, MIDI/clock, UI, DAC output every iteration
// =============================================================================
void loop() {
  // Poll all inputs... every single tick
  //if ((ticks & 0x03) == 0)
  PollInputs(inputs);
  engine.Tick();

#if DEBUG
  if (Serial.available() && Serial.read()) {
    for (uint8_t i = 0; i < INPUT_COUNT/2; ++i) {
      Serial.printf("Input #%2u = %x   |  Input #%2u = %x\n", i, inputs[i].state, i + INPUT_COUNT/2, inputs[i + INPUT_COUNT/2].state);
    }
  }
#endif

  const bool track_mode = inputs[TRACK_SEL].held();
  const bool write_mode = inputs[WRITE_MODE].held();
  const bool clear_mod = inputs[CLEAR_KEY].held();
  const bool edit_mode = inputs[TAP_NEXT].held();

  // todo: transpose, performance stuff, config menus
  const bool fn_mod = inputs[FUNCTION_KEY].held();
  const bool pitch_mod = inputs[PITCH_KEY].held();
  const bool time_mod = inputs[TIME_KEY].held();

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
  if (midi_clock_pulse)
    clocked = true;

  // DIN sync clock @ 24ppqn
  if (!midi_clk) {
    clocked = inputs[CLOCK].rising();
  }

  // Save pattern data - only if clock isn't running, to prevent stuttering
  // - when exiting write mode
  // - when stopping the clock
  if ((inputs[WRITE_MODE].falling() && !clk_run) ||
      (inputs[RUN].falling() && !midi_clk)) {
    engine.Save();
  }

  if (inputs[RUN].rising()) {
    //Serial.println("CLOCK RUN STARTED");
    engine.Reset();
  }

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (edit_mode) { // holding WRITE/NEXT/TAP
    ProcessEdit(write_mode, clk_run);
  } else {
    // Flash lights for modifiers
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

        // tap in number of steps
        if (inputs[DOWN_KEY].rising()) {
          if (step_counter)
            step_counter = engine.BumpLength();
          else {
            engine.SetLength(1);
            step_counter = true;
          }
        }
        // TODO: modify length with pitch keys
      }

    } else {
      ProcessDefault(write_mode, clear_mod, clk_run);
    }
  }

  // show all pressed buttons
  if (s_cfg_menu == CfgMenu::Off) {
    for (uint8_t i = 0; i < 16; ++i) {
      const InputIndex b = switched_leds[i].button;
      if (!inputs[b].held())
        continue;
      // High C already implies upper register; skip UP LED (often ghosts with C_KEY2 on same mux).
      if (b == UP_KEY && inputs[C_KEY2].held())
        continue;
      Leds::Set(OutputIndex(i), true);
    }
  }

  Leds::Send(ticks); // hardware output, framebuffer reset

  tracknum = uint8_t(inputs[TRACK_BIT0].held()
           | (inputs[TRACK_BIT1].held() << 1)
           | (inputs[TRACK_BIT2].held() << 2));

  // --- other input handling
  if (inputs[FUNCTION_KEY].rising()) {
    if (s_cfg_menu == CfgMenu::Main)
      s_cfg_menu = CfgMenu::Off;
    else if (s_cfg_menu == CfgMenu::Off)
      engine.SetMode(NORMAL_MODE, !clk_run);
  }

  if (s_cfg_menu == CfgMenu::Off) {
    if (inputs[TIME_KEY].rising() && write_mode) engine.SetMode(TIME_MODE, !clk_run);
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

  if (clocked) {
    ++clk_count %= 24;
  }

  midi_leader_transport(clocked, clk_run, midi_clk, inputs[RUN].rising(),
                        inputs[RUN].falling());

  if (clocked && clk_run) {
    if (engine.Clock())
      midi_after_clock(engine, transpose);
  }

  if (s_cfg_menu == CfgMenu::Off) {
    if (inputs[TAP_NEXT].rising()) {
      if (write_mode) {
        if (!clk_run) {
          if (engine.get_mode() == PITCH_MODE) {
            // Audition current pitch step, then advance to the next pitch slot
            if (!engine.get_sequence().step_is_empty()) {
              DAC::SetPitch(engine.get_pitch());
              DAC::SetGate(true);
            }
            engine.get_sequence().AdvancePitch();
          } else if (engine.get_mode() == TIME_MODE) {
            const bool send = engine.Advance();
            engine.SyncAfterManualAdvance(send);
          }
        }
        // With clock running, TAP does nothing in write mode (Clock() drives advance)
      } else {
        DAC::SetGate(engine.Advance());
      }
    }
    if (inputs[TAP_NEXT].falling()) {
      DAC::SetGate(false);
      if (!wrap_edit && !clk_run && engine.get_mode() == TIME_MODE &&
          engine.get_time_pos() >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }
  }

  // regular pattern write mode
  if (s_cfg_menu == CfgMenu::Off && !edit_mode && write_mode && !track_mode) {

    // ok, dilemma... we want to advance first, then record the step.

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
      DAC::SetGate(check);
      if (clk_run || check) {
        input_pitch(clk_run, clk_run);
      } else if (!clk_run && engine.get_sequence().pitch_pos >= engine.get_length() - 1)
        engine.SetMode(NORMAL_MODE, true);
    }

  }

  // ---------------------------------------------------------------------------
  // CV output: running = sequenced pitch + engine gate/accent/slide; stopped = keys
  // ---------------------------------------------------------------------------
  if (clk_run) {
    // send sequence step
    DAC::SetPitch(engine.get_pitch() + 4 + transpose);
    // Slide CV: portamento during the *destination* note only (Robin Whittle 303 slide model).
    DAC::SetSlide(engine.get_slide_dac());
    DAC::SetAccent(engine.get_accent());
    DAC::SetGate(engine.get_gate());
  } else {
    // not run mode - send notes from keys
    DAC::SetPitch(engine.get_pitch() + 4 + transpose);
    DAC::SetSlide(inputs[SLIDE_KEY].held());
    DAC::SetAccent(inputs[ACCENT_KEY].held());
  }

  // catch falling edge of RUN
  if (inputs[RUN].falling() && !midi_clk) {
    //Serial.println("CLOCK STOPPED");
    DAC::SetGate(false);
    engine.Reset();
  }

  ++ticks;

  // send DAC every other tick...
  //if (0 == (ticks & 0x1))
  DAC::Send();
}
