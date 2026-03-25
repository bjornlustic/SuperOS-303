// Copyright (c) 2026, Nicholas J. Michalek
/* 
 * Sequencer engine logic and data model for TB-303 CPU
 */

#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "drivers.h"

//
// *** Utilities ***
//
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define CONSTRAIN(x, lb, ub) do { if (x < (lb)) x = lb; else if (x > (ub)) x = ub; } while (0)

static constexpr int MAX_STEPS = 64;
static constexpr int NUM_PATTERNS = 16;

enum SequencerMode {
  NORMAL_MODE,
  PITCH_MODE,
  TIME_MODE,
};

enum OctaveState {
  OCTAVE_DOWN,
  OCTAVE_ZERO,
  OCTAVE_UP,
  OCTAVE_DOUBLE_UP,
};

struct Sequence {
  // --- sequence data - 64 bytes
  // for DAC pitch, 0 is a low G#; 4 is lowest C; and middle C is 28
  // We're gonna store pitch as if the lowest C is 0, so it needs +4 when sent to DAC
  uint8_t pitch[MAX_STEPS]; // 6-bit Pitch, Accent, and Slide
  uint8_t time_data[MAX_STEPS/2];  // 0=rest, 1=note, 2=tie, 3=triplets?
  // time is stored as nibbles, so there's actually a lot of padding
  uint8_t reserved[MAX_STEPS/2 - 1];
  uint8_t length = 16;
  // --- end sequence data

  // state
  int pitch_pos, time_pos;
  bool reset; // hold plz

  // --- functions
  // 6-bit pitch, 0 == low C
  const uint8_t get_pitch() const {
    if (step_is_empty()) return (OCTAVE_ZERO * 12); // silent default, gate will be off
    return pitch[pitch_pos] & 0x3f;
  }
  const uint8_t get_octave() const {
    if (step_is_empty()) return OCTAVE_ZERO;
    return get_pitch() / 12;
  }
  // semitone index (0–11) for LED display. Returns 0xFF if step is unwritten.
  const uint8_t get_semitone() const {
    if (step_is_empty()) return PITCH_EMPTY;
    return get_pitch() % 12;
  }
  const uint8_t get_accent() const {
    if (step_is_empty()) return 0;
    return pitch[pitch_pos] & (1<<6);
  }
  const bool get_slide(uint8_t step) const {
    if (pitch_is_empty(step)) return false;
    return pitch[step] & (1<<7);
  }
  const bool get_slide() const {
    return get_slide(pitch_pos);
  }
  const bool is_sliding() const {
    return (pitch_pos < length) && get_slide(pitch_pos+1);
  }
  // True if the CURRENT step is a tie.
  const bool is_tie() const {
    return time(time_pos) == 2;
  }
  // True if the NEXT step is a tie.
  const bool next_is_tie() const {
    const uint8_t next = (time_pos + 1) % length;
    return time(next) == 2;
  }
  // True if the NEXT step has the slide flag set.
  // Used to suppress gate-off on the current step (gate stays high into the slid step).
  const bool next_has_slide() const {
    const uint8_t next_tp = (time_pos + 1) % length;
    if (time(next_tp) != 1) return false; // next step is not a note (tie/rest) — no slide flag
    const uint8_t next_pp = pitch_pos + 1;
    if (next_pp >= length) return false; // out of bounds
    return get_slide(next_pp);
  }
  const bool is_tied() const {
    return next_is_tie();
  }
  inline uint8_t time(uint8_t idx) const {
    return (time_data[idx >> 1] >> 4*(idx & 1)) & 0xf;
  }
  const uint8_t get_time() const {
    return time(time_pos);
  }

  void SetTime(uint8_t t) {
    const uint8_t upper = time_pos & 1;
    uint8_t &data = time_data[time_pos >> 1];
    data = (~(0x0f << 4*upper) & data) | (t << 4*upper);
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    pitch[pitch_pos] = (p & 0x3f) | (flags & 0xc0);
  }
  void SetPitchSemitone(uint8_t p) {
    init_if_empty(); 
    pitch[pitch_pos] =
        ((get_octave() * 12 + p) & 0x3f) | (pitch[pitch_pos] & 0xc0);
  }
  void SetLength(uint8_t len) { length = constrain(len, 1, MAX_STEPS); }
  void SetOctave(int oct) {
    init_if_empty();
    CONSTRAIN(oct, 0, 3);
    pitch[pitch_pos] =
        ((uint8_t)oct * 12 + get_semitone()) | (pitch[pitch_pos] & 0xc0);
  }

  void ToggleSlide() { init_if_empty(); pitch[pitch_pos] ^= (1 << 7); }
  void ToggleAccent() { init_if_empty(); pitch[pitch_pos] ^= (1 << 6); }
  void SetSlide(bool on) {
    init_if_empty();
    pitch[pitch_pos] = (pitch[pitch_pos] & ~(1 << 7)) | (on << 7);
  }
  void SetAccent(bool on) {
    init_if_empty();
    pitch[pitch_pos] = (pitch[pitch_pos] & ~(1 << 6)) | (on << 6);
  }

  bool BumpLength() {
    if (++length == MAX_STEPS) return false;
    return true;
  }

  void RegenTime() {
    time_data[time_pos] = random();
  }
  void RegenPitch() {
    pitch[pitch_pos] = random();
  }

  void Reset() {
    pitch_pos = 0;
    time_pos = 0;
    reset = true;
  }
  static constexpr uint8_t PITCH_EMPTY = 0xFF; // unwritten step sentinel
  static constexpr uint8_t PITCH_DEFAULT = (OCTAVE_ZERO*12); // clean default: C, octave zero, no flags

  bool pitch_is_empty(uint8_t pos) const { return pitch[pos] == PITCH_EMPTY; }
  bool step_is_empty() const { return pitch_is_empty(pitch_pos); }

  void init_if_empty() {
    if (step_is_empty()) pitch[pitch_pos] = PITCH_DEFAULT;
  }

  void Clear() {
    for (uint8_t i = 0; i < MAX_STEPS; ++i) {
      pitch[i] = PITCH_DEFAULT;
      time_data[i>>1] = 0; // all rests
    }
    length = 8;
  }

  // returns false for rests
  bool Advance() {
    if (reset) {
      reset = false;
      return true;
    }
    ++time_pos %= length;
    if (time_pos == 0)
      pitch_pos = 0;
    else if (time(time_pos) == 1)
      ++pitch_pos;
    return time(time_pos);
  }

  // used in write mode — advances to next pitch slot, stops at length-1 (no wrap)
  void AdvancePitch() {
    if (reset) {
      reset = false;
    } else if (pitch_pos < length - 1) {
      ++pitch_pos;
    }
  }
};

// --- EEPROM data layout
static constexpr int SETTINGS_SIZE = 128;
static constexpr int PATTERN_SIZE = MAX_STEPS * 2;

const char* const sig_pew = "PewPewPew!!!";

extern EEPROMClass storage;

struct PersistentSettings {
  char signature[16];

  void Load() {
    storage.get(0, signature);
  }
  void Save() {
    storage.put(0, signature);
  }
  bool Validate() const {
    if (0 == strncmp(signature, sig_pew, 12))
      return true;

    strcpy((char*)signature, sig_pew);
    return false;
  }
};

extern PersistentSettings GlobalSettings;

void WritePattern(Sequence &seq, int idx) {
  uint8_t *src = seq.pitch;
  idx *= PATTERN_SIZE;
  for (uint8_t i = 0; i < PATTERN_SIZE; ++i) {
    storage.update(SETTINGS_SIZE + idx + i, src[i]);
  }
}
void ReadPattern(Sequence &seq, int idx) {
  uint8_t *dst = seq.pitch;
  idx *= PATTERN_SIZE;
  for (uint8_t i = 0; i < PATTERN_SIZE; ++i) {
    dst[i] = storage.read(SETTINGS_SIZE + idx + i);
  }
}

// μPD650C-133 timing constants from Sonic Potions paper (gate/ISR timing)
static constexpr uint32_t ISR_PERIOD_US       = 1800; // fixed 1.8ms interrupt period
static constexpr uint32_t GATE_ON_LATENCY_US  =  990; // fixed ISR processing: mean of 0.917–1.062ms
static constexpr uint32_t GATE_OFF_LATENCY_US = 2350; // fixed ISR processing: mean of 2.3–2.4ms

struct Engine {
  // pattern storage
  Sequence pattern[NUM_PATTERNS]; // 32 steps each
  uint8_t p_select = 0;
  uint8_t next_p = 0; // queued pattern
                      // TODO: start & end for chains

  SequencerMode mode_ = NORMAL_MODE;

  int8_t clk_count = -1;

  // slide_on: gate-hold flag — true when the CURRENT step has slide, so the current
  //           step's gate is not dropped (paper §4.3: "gate stays high in front of a slid step").
  //           Also stays high through ties and rests per paper §6 / Fig. 17.
  // slide_cv: slide signal flag — true when the CURRENT step has the slide attribute,
  //           meaning the slide CV line is asserted (paper §6: "slide signal set high
  //           at the beginning of the slid step").
  bool slide_on    = false; // gate-hold: suppress gate-off so gate stays high into next step
  bool slide_cv    = false; // slide CV: current step has slide attribute, assert slide signal
  bool was_sliding = false; // true if gate was held high INTO the current step (needs re-latch)
  bool gate_hold = false;   // keep gate high when current step is tied into the next
  bool stale = false;
  bool resting = false; // hey shutup

  // ISR simulation: free-running timer that wraps at ISR_PERIOD_US, mirroring the
  // original CPU's fixed 1.8ms interrupt. Gate on/off are each scheduled from the
  // ISR edge nearest to their respective clock edges (0 and 3), giving each its own
  // independent sawtooth beat pattern as seen in Figs 7–10 of the paper.
  elapsedMicros isr_timer;
  elapsedMicros gate_on_timer;   // started at clock 0
  elapsedMicros gate_off_timer;  // started at clock 3
  uint32_t gate_on_delay_us  = 0;
  uint32_t gate_off_delay_us = 0;
  bool gate_pending_on  = false;
  bool gate_pending_off = false;
  bool gate_state       = false; // actual driven gate value
  bool accent_state     = false; // latched accent for full step duration

  // actions
  void Load() {
    Serial.println("Loading from EEPROM...");

    // TODO: settings and calibration
    GlobalSettings.Load();
    if (GlobalSettings.Validate()) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        ReadPattern(pattern[i], i);
        if (0 == pattern[i].length) pattern[i].SetLength(8);
      }
    } else {
      Serial.println("EEPROM data invalid, initializing...");
      // initialize memory with defaults or zeroes
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        pattern[i].Clear();
      }
      GlobalSettings.Save();
      stale = true;
      Save();
    }

#if DEBUG
    Serial.println("First pattern:");
    for (uint8_t i = 0; i < 64; ++i) {
      Serial.printf("%2x ", pattern[0].pitch[i]);
    }
    Serial.print("\n");
#endif
  }
  void Save(int pidx = -1) {
    if (!stale) return;
    Serial.print("Saving to EEPROM... ");
    if (pidx < 0) {
      // save all
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        Serial.print(".");
        WritePattern(pattern[i], i);
      }
    } else
      WritePattern(pattern[pidx], pidx);

    stale = false;
    Serial.println("DONE!");
  }

  // Called every loop() iteration. Fires pending gate on/off transitions once
  // their ISR-relative delay has elapsed, reproducing the original CPU latency.
  void Tick() {
    if (gate_pending_on && gate_on_timer >= gate_on_delay_us) {
      gate_pending_on = false;
      if (!resting) {
        gate_state   = true;
        accent_state = get_sequence().get_accent() != 0;
        // Notify DAC to latch the new pitch into the R2R DAC.
        // If the gate was held high into this step (was_sliding), the slide signal is
        // already high and we need the 8.75μs re-latch to update the DAC.
        // Otherwise use the 44μs new-note latch pulse.
        if (was_sliding) {
          DAC::NotifySlideRelatch();
        } else {
          DAC::NotifyNewNote();
        }
      }
    }
    if (gate_pending_off && gate_off_timer >= gate_off_delay_us) {
      gate_pending_off = false;
      if (!slide_on) {
        gate_state = false;
        // accent_state holds through full step; cleared at next gate-on or reset
      }
    }
  }

  // returns false for rests
  bool Advance() {
    bool result = get_sequence().Advance();
    // jump to next pattern at end of current one
    if (0 == get_sequence().time_pos && next_p != p_select) {
      p_select = next_p;
      get_sequence().Reset();
      result = get_sequence().Advance();
    }

    // was_sliding: capture whether the gate was held high INTO this step (from prev step's slide).
    // Used in Tick() to decide between 44μs new-note latch and 8.75μs re-latch.
    was_sliding = slide_on;

    // slide_cv: the CURRENT step has the slide attribute.
    // The slid step IS the step with the slide flag. The slide CV asserts here,
    // and the gate of THIS step is held high into the next step.
    slide_cv = result && get_sequence().get_slide();

    // slide_on: gate-hold — suppress gate-off on THIS step so gate stays high into the next.
    // Set from the CURRENT step's own slide flag (not a lookahead).
    // Also stays high through ties and rests.
    if (result) {
      slide_on = slide_cv || get_sequence().is_tie() || get_sequence().next_is_tie();
      gate_hold = get_sequence().is_tied();
    } else {
      gate_hold = false;
      // On a rest: slide_on carried forward unchanged — cleared when the next non-slid
      // note arrives and sets slide_on = false.
    }
    resting = !result;
    return result;
  }

  // returns true for new pitch step
  bool Clock() {
    bool send_note = false;
    ++clk_count %= 6;

    if (clk_count == 0) { // sixteenth note advance
      send_note = Advance();
      resting = !send_note;
      accent_state = false; // clear before new step latches it in Tick()

      // need to take a second look at fig 7-10 to see the isr processing
      // we are using sawtooth jitter as it looks to go up and down but we
      // need a way to actually test this against another 303 cpu.
      if (send_note) {
        const uint32_t phase = isr_timer % ISR_PERIOD_US;
        const uint32_t time_to_next_isr = ISR_PERIOD_US - phase;
        gate_on_delay_us = time_to_next_isr + GATE_ON_LATENCY_US;
        gate_pending_on  = true;
        gate_on_timer    = 0;
      } else {
        // Rest: cancel any pending transitions and drop gate immediately
        gate_pending_on  = false;
        gate_pending_off = false;
        gate_state       = false;
      }
    }

    if (clk_count == 3) {
      // need to take a second look at figs 9 - 10 to see the isr processing.
      if (!resting && !slide_on) {
        const uint32_t phase = isr_timer % ISR_PERIOD_US;
        const uint32_t time_to_next_isr = ISR_PERIOD_US - phase;
        gate_off_delay_us = time_to_next_isr + GATE_OFF_LATENCY_US;
        gate_pending_off  = true;
        gate_off_timer    = 0;
      }
    }

    return send_note;
  }

  void Reset() {
    get_sequence().Reset();
    clk_count   = -1;
    slide_on    = false;
    slide_cv    = false;
    was_sliding = false;
    resting     = true;
    gate_pending_on  = false;
    gate_pending_off = false;
    gate_state       = false;
    accent_state     = false;
  }

  // After Advance() from TAP_NEXT in pattern write: Clock() does not run, so resting/slide
  // must match the new step or gate/CV stays wrong and causes noise.
  void SyncAfterManualAdvance(bool send_note) {
    resting = !send_note;
    if (!send_note) {
      slide_on     = false;
      slide_cv     = false;
      was_sliding  = false;
      gate_state   = false;
      accent_state = false;
    } else {
      // Manual advance: no ISR latency simulation, fire gate immediately
      gate_pending_on  = false;
      gate_pending_off = false;
      gate_state       = true;
      accent_state     = get_sequence().get_accent() != 0;
    }
  }

  void Generate() {
    if (mode_ == PITCH_MODE)
      get_sequence().RegenPitch();
    else if (mode_ == TIME_MODE)
      get_sequence().RegenTime();
  }

  void ClearPattern(uint8_t idx) {
    pattern[idx].Clear();
    stale = true;
    if (idx == p_select) {
      Reset();
      mode_ = NORMAL_MODE;
    }
  }

  // getters
  SequencerMode get_mode() const { return mode_; }

  Sequence &get_sequence() { return pattern[p_select]; }
  const Sequence &get_sequence() const { return pattern[p_select]; }
  const Sequence &get_pattern(uint8_t idx) const { return pattern[idx & 0xf]; }

  // Gate state is driven by Tick() using ISR-latency scheduling.
  // slide_on keeps gate high through tied/slid notes as before.
  bool get_gate() const {
    return (gate_state || slide_on || gate_hold) && !resting;
  }
  // Accent is latched for the full step duration once the gate fires.
  bool get_accent() const {
    return !resting && accent_state;
  }
  uint8_t get_semitone() const {
    return get_sequence().get_semitone();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch();
  }
  // MIDI note number: OCTAVE_DOWN C = 36 (C2), OCTAVE_ZERO C = 48 (C3)
  uint8_t get_midi_note() const {
    if (get_sequence().step_is_empty()) return 48;
    const uint8_t semitone = get_sequence().get_semitone();
    const uint8_t oct = get_sequence().get_octave();
    return 36 + semitone + (oct * 12);
  }
  bool get_slide() const {
    return slide_cv;
  }
  // Slide CV line: high ONLY on the step being slid INTO (was_sliding=true).
  // The step with the slide flag does NOT assert slide — it only holds its gate high.
  bool get_slide_dac() const {
    return was_sliding;
  }
  uint8_t get_time_pos() const {
    return get_sequence().time_pos;
  }
  uint8_t get_patsel() const {
    return p_select;
  }
  uint8_t get_next() const {
    return next_p;
  }
  const uint8_t get_time() const {
    return get_sequence().get_time();
  }
  const uint8_t get_length() const {
    return get_sequence().length;
  }

  // setters
  void SetPattern(uint8_t p_, bool override = false) {
    next_p = p_ & 0xf; // p_ % 16;
    if (override) p_select = next_p;
  }
  void SetLength(uint8_t len) {
    get_sequence().SetLength(len);
    stale = true;
  }
  bool BumpLength() {
    stale = true;
    return get_sequence().BumpLength();
  }
  void SetMode(SequencerMode m, bool reset = false) {
    if (reset && m != mode_) Reset();
    mode_ = m;
  }
  void NudgeOctave(int dir) {
    get_sequence().SetOctave(int(get_sequence().get_octave()) + dir);
  }
  // change pitch, preserving flags
  void SetPitchSemitone(uint8_t p) {
    get_sequence().SetPitchSemitone(p);
    stale = true;
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    get_sequence().SetPitch(p, flags);
    stale = true;
  }
  void SetTime(uint8_t t) {
    get_sequence().SetTime(t);
    stale = true;
  }

  void ToggleSlide() {
    if (mode_ == PITCH_MODE)
      get_sequence().ToggleSlide();
    stale = true;
  }
  void ToggleAccent() {
    if (mode_ == PITCH_MODE)
      get_sequence().ToggleAccent();
    stale = true;
  }

};
