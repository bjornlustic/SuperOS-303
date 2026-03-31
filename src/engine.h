// Copyright (c) 2026, Nicholas J. Michalek
/*
 * engine.h — TB-303 pattern model + EEPROM; Engine handles patterns, clock, and gate.
 *
 * EEPROM versions:
 *   "PewPewPew!!2" — original: pitch slots packed sequentially by NOTE event (pitch_pos ≠ time_pos)
 *   "PewPewPew!!3" — current:  1:1 pitch/time model — pitch[i] belongs to time step i
 */

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

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

static constexpr uint8_t PITCH_EMPTY = 0xFF;
// Packed step pitch: key_idx (0..12, chromatic incl. high C) + 13 * octave_btn (0..3).
// Decodes to linear CV semitone via unpack_pitch_linear(); avoids C vs C2 + UP collisions.
// oct_btn 3 = DOUBLE_UP: allows high C to go one octave above the normal UP range.
static constexpr uint8_t PITCH_PACK_MAX = 12 + 13 * 3; // 51
static constexpr uint8_t PITCH_DEFAULT = 0 + 13 * 1; // packed: low C, middle octave button
/// Index of C_KEY2 in pitched_keys / pitch_leds (chromatic row incl. upper C).
static constexpr uint8_t PITCH_KEY_HIGH_C = 12;

static inline uint8_t pack_pitch(uint8_t key_idx, uint8_t oct_btn) {
  return uint8_t(key_idx + 13 * oct_btn);
}
/// Convert packed EEPROM/CV encoding to linear semitone (0..48) for DAC / MIDI.
static inline uint8_t unpack_pitch_linear(uint8_t e) {
  if (e > PITCH_PACK_MAX)
    return e & 0x3f; // corrupt or legacy linear: clamp
  const uint8_t key_idx = e % 13;
  const uint8_t oct_btn = e / 13;
  return key_idx + 12 * oct_btn;
}

// =============================================================================
// Sequence — one pattern (matches reference OS-303 layout)
// =============================================================================
static constexpr int STEP_LOCK_BYTES = (MAX_STEPS + 7) / 8;

struct Sequence {
  uint8_t pitch[MAX_STEPS];
  uint8_t time_data[MAX_STEPS / 2];
  /// Per-step live-write lock for TIME_MODE (same physical key as slide; not pitch slide).
  uint8_t step_lock[STEP_LOCK_BYTES];
  uint8_t reserved[MAX_STEPS / 2 - 1 - STEP_LOCK_BYTES];
  uint8_t length = 16;

  int pitch_pos, time_pos;
  bool reset;
  bool first_step = true; // suppresses slide_from_prev on the very first step after Reset()

  bool step_locked(uint8_t idx) const {
    idx &= (MAX_STEPS - 1);
    return (step_lock[idx >> 3] >> (idx & 7)) & 1;
  }
  void ToggleStepLock(uint8_t idx) {
    idx &= (MAX_STEPS - 1);
    step_lock[idx >> 3] ^= uint8_t(1u << (idx & 7));
  }

  /// 1:1 model: pitch[time_pos] is this step's pitch.
  /// For TIE steps, walk backward to find the last NOTE step's pitch.
  const uint8_t get_pitch() const {
    if (time(time_pos) == 2) {
      // TIE: use the last NOTE step's pitch (rest cancels the chain)
      for (uint8_t g = 1; g < length; ++g) {
        const uint8_t tp = uint8_t((time_pos + length - g) % length);
        if (time(tp) == 0) break; // rest cancels slide/tie chain
        if (time(tp) == 1) {
          return pitch_is_empty(tp) ? unpack_pitch_linear(PITCH_DEFAULT)
                                    : unpack_pitch_linear(pitch[tp] & 0x3f);
        }
      }
      return unpack_pitch_linear(PITCH_DEFAULT);
    }
    if (step_is_empty()) return unpack_pitch_linear(PITCH_DEFAULT);
    return unpack_pitch_linear(pitch[pitch_pos] & 0x3f);
  }
  /// Linear pitch / 12 — pattern-select octave LEDs (DOWN/UP) vs stored keypad octave.
  const uint8_t get_octave() const {
    if (step_is_empty()) return OCTAVE_ZERO;
    return get_pitch() / 12;
  }
  const uint8_t get_semitone() const {
    if (step_is_empty()) return PITCH_EMPTY;
    return get_pitch() % 12;
  }
  /// Index into pitched_keys / pitch_leds (0..12), distinct for low C vs high C.
  uint8_t get_note_key_index() const {
    if (step_is_empty()) return 0;
    return (pitch[pitch_pos] & 0x3f) % 13;
  }
  /// UP/DOWN octave buttons at record time: 0 down, 1 center, 2 up, 3 double-up (packed e / 13).
  uint8_t get_octave_button() const {
    if (step_is_empty()) return 1;
    return (pitch[pitch_pos] & 0x3f) / 13;
  }
  const uint8_t get_accent() const {
    if (step_is_empty()) return 0;
    return pitch[pitch_pos] & (1 << 6);
  }
  const bool get_slide(uint8_t step) const {
    if (pitch_is_empty(step)) return false;
    return pitch[step] & (1 << 7);
  }
  const bool get_slide() const { return get_slide(pitch_pos); }
  /// TB-303 slide: slide is stored on the *source* pitch step;
  /// slide CV is on the *destination* pitch step. Ties after the source do not break the glide,
  /// but any *rest* (time 0) between source and destination cancels slide.
  bool slide_from_prev() const {
    if (length <= 1) return false;
    if (time(time_pos) == 0) return false;
    if (first_step) return false;

    // Walk backward from the step before time_pos, skipping ties until we hit a note or rest.
    // Slide flag lives on the *note* step that starts the glide, not on tie slots.
    uint8_t tp = uint8_t((time_pos + length - 1) % length);
    for (uint8_t guard = 0; guard < length; ++guard) {
      const uint8_t tt = time(tp);
      if (tt == 0)
        return false; // rest breaks chain
      if (tt == 1)
        return get_slide(tp);
      // tie: keep walking backward
      tp = uint8_t((tp + length - 1) % length);
    }
    return false;
  }
  /// Next time slot is a tie (extends previous note).
  const bool is_tied() const {
    return (time_pos < length) && (time(time_pos + 1) == 2);
  }
  /// Current time slot is a tie.
  bool is_tie() const {
    return (time_pos < length) && (time(time_pos) == 2);
  }
  /// Next step (wrapped) is a tie.
  bool next_is_tie() const {
    if (length == 0) return false;
    const uint8_t n = (time_pos + 1) % length;
    return time(n) == 2;
  }
  /// True if this NOTE step follows one or more TIE steps (gate carried; MIDI should legato in).
  bool note_after_tie_run() const {
    if (length <= 1 || first_step || time(time_pos) != 1) return false;
    uint8_t tp = uint8_t((time_pos + length - 1) % length);
    bool seen_tie = false;
    for (uint8_t g = 0; g < length; ++g) {
      const uint8_t tt = time(tp);
      if (tt == 0) return false;
      if (tt == 2) {
        seen_tie = true;
        tp = uint8_t((tp + length - 1) % length);
        continue;
      }
      if (tt == 1) return seen_tie;
    }
    return false;
  }
  /// Last tie in a run: on a tie step whose next step is not a tie.
  bool tie_chain_ending() const {
    return is_tie() && !next_is_tie();
  }

  inline uint8_t time(uint8_t idx) const {
    return (time_data[idx >> 1] >> (4 * (idx & 1))) & 0xf;
  }
  inline uint8_t effective_time(uint8_t idx) const { return time(idx); }

  const uint8_t get_time() const { return time(time_pos); }

  void SetTime(uint8_t t) {
    const uint8_t upper = time_pos & 1;
    uint8_t &data = time_data[time_pos >> 1];
    data = (~(0x0f << (4 * upper)) & data) | (t << (4 * upper));
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    pitch[pitch_pos] = (p & 0x3f) | (flags & 0xc0);
  }
  void SetPitchSemitone(uint8_t p) {
    init_if_empty();
    const uint8_t e = pitch[pitch_pos] & 0x3f;
    const uint8_t oct_btn = e / 13;
    pitch[pitch_pos] =
        (pack_pitch(p, oct_btn) & 0x3f) | (pitch[pitch_pos] & 0xc0);
  }
  void SetLength(uint8_t len) { length = constrain(len, 1, MAX_STEPS); }
  void nudge_octave_buttons(int dir) {
    init_if_empty();
    const uint8_t e = pitch[pitch_pos] & 0x3f;
    const uint8_t k = e % 13;
    int o = int(e / 13) + dir;
    CONSTRAIN(o, 0, 3); // 0=DOWN, 1=CENTER, 2=UP, 3=DOUBLE_UP
    pitch[pitch_pos] =
        (pack_pitch(k, uint8_t(o)) & 0x3f) | (pitch[pitch_pos] & 0xc0);
  }

  void ToggleSlide() {
    init_if_empty();
    pitch[pitch_pos] ^= (1 << 7);
  }
  void ToggleAccent() {
    init_if_empty();
    pitch[pitch_pos] ^= (1 << 6);
  }
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

  void RegenTime() { time_data[time_pos] = random(); }
  void RegenPitch() {
    pitch[pitch_pos] =
        uint8_t(random(PITCH_PACK_MAX + 1)) | (pitch[pitch_pos] & 0xc0);
  }

  void Reset() {
    pitch_pos = 0;
    time_pos = 0;
    reset = true;
    first_step = true;
  }

  bool pitch_is_empty(uint8_t pos) const { return pitch[pos] == PITCH_EMPTY; }
  bool step_is_empty() const { return pitch_is_empty(pitch_pos); }

  void init_if_empty() {
    if (step_is_empty()) pitch[pitch_pos] = PITCH_DEFAULT;
  }

  void Clear() {
    for (uint8_t i = 0; i < MAX_STEPS; ++i) {
      pitch[i] = PITCH_DEFAULT;
      time_data[i >> 1] = 0;
    }
    for (uint8_t i = 0; i < STEP_LOCK_BYTES; ++i)
      step_lock[i] = 0;
    length = 8;
  }

  uint8_t first_note_idx() const {
    for (uint8_t j = 0; j < length; ++j)
      if (time(j) == 1)
        return j;
    return 0;
  }
  uint8_t next_note_step_idx(uint8_t from) const {
    for (uint8_t k = 1; k <= length; ++k) {
      const uint8_t j = uint8_t((unsigned(from) + k) % unsigned(length));
      if (time(j) == 1)
        return j;
    }
    return from;
  }
  uint8_t prev_note_step_idx(uint8_t from) const {
    for (uint8_t k = 1; k <= length; ++k) {
      const uint8_t j =
          uint8_t((unsigned(from) + unsigned(length) - k) % unsigned(length));
      if (time(j) == 1)
        return j;
    }
    return from;
  }

  /// Clear PITCH_MODE entry reset and land on first NOTE step (skips leading rests/ties).
  void ensure_pitch_edit_entry() {
    if (reset) {
      reset = false;
      pitch_pos = int(first_note_idx());
      time_pos = pitch_pos;
    }
    if (time(uint8_t(pitch_pos & (MAX_STEPS - 1))) != 1) {
      pitch_pos = int(next_note_step_idx(uint8_t(pitch_pos & (MAX_STEPS - 1))));
      time_pos = pitch_pos;
    }
  }

  /// After recording/audition: move to next NOTE step only (rest/tie slots invisible in pitch edit).
  void advance_pitch_to_next_note() {
    first_step = false;
    pitch_pos = int(next_note_step_idx(uint8_t(pitch_pos & (MAX_STEPS - 1))));
    time_pos = pitch_pos;
  }

  /// 1:1 model: pitch_pos always equals time_pos.
  /// On a NOTE step, pitch[time_pos] is played. On TIE, get_pitch() walks backward.
  /// On REST, gate is low regardless.
  bool Advance() {
    if (reset) {
      reset = false;
      pitch_pos = 0;
      time_pos = 0;
      return time(0) != 0;
    }
    first_step = false;
    ++time_pos %= length;
    pitch_pos = time_pos; // 1:1 mapping
    return time(time_pos) != 0;
  }

  /// PITCH_MODE: advance to next NOTE step (used by TAP write).
  void AdvancePitch() { advance_pitch_to_next_note(); }

  /// TIME_MODE step back: one physical step toward 0 (unchanged TB behaviour).
  bool StepBack() {
    if (reset || (pitch_pos == 0 && time_pos == 0)) {
      return false;
    }
    if (pitch_pos > 0) {
      --pitch_pos;
      time_pos = pitch_pos;
    } else {
      pitch_pos = 0;
      time_pos = 0;
    }
    return true;
  }

  /// PITCH_MODE: previous NOTE step (not raw index).
  bool StepBackPitchByNote() {
    ensure_pitch_edit_entry();
    const uint8_t first = first_note_idx();
    const uint8_t cur = uint8_t(pitch_pos & (MAX_STEPS - 1));
    if (cur == first)
      return false;
    pitch_pos = int(prev_note_step_idx(cur));
    time_pos = pitch_pos;
    return true;
  }
};

/// Set time nibble for step `idx` (0=rest, 1=note, 2=tie).
inline void sequence_set_time_at(Sequence &s, uint8_t idx, uint8_t t) {
  idx &= uint8_t(MAX_STEPS - 1);
  const uint8_t upper = idx & 1u;
  uint8_t &cell = s.time_data[idx >> 1u];
  cell = uint8_t((cell & ~(0x0fu << (4u * upper))) | ((t & 0x0fu) << (4u * upper)));
}

/// TB-303–valid time fixes: no rest→tie, no leading tie, no all-ties (wrap-aware).
inline void normalize_pattern_times(Sequence &s) {
  const uint8_t L = s.length;
  if (L < 1)
    return;

  bool all_tie = true;
  for (uint8_t i = 0; i < L; ++i) {
    if (s.time(i) != 2) {
      all_tie = false;
      break;
    }
  }
  if (all_tie) {
    sequence_set_time_at(s, 0, 1);
    if (s.pitch_is_empty(0))
      s.pitch[0] = PITCH_DEFAULT;
  }

  if (s.time(0) == 2) {
    sequence_set_time_at(s, 0, 1);
    if (s.pitch_is_empty(0))
      s.pitch[0] = PITCH_DEFAULT;
  }

  for (uint8_t i = 0; i < L; ++i) {
    const uint8_t nxt = uint8_t((unsigned(i) + 1u) % unsigned(L));
    if (s.time(i) == 0 && s.time(nxt) == 2) {
      sequence_set_time_at(s, i, 1);
      if (s.pitch_is_empty(i))
        s.pitch[i] = PITCH_DEFAULT;
    }
  }
}

// =============================================================================
// EEPROM
// =============================================================================
static constexpr int SETTINGS_SIZE = 128;
static constexpr int PATTERN_SIZE = MAX_STEPS * 2;

// Bump version when pattern EEPROM layout/meaning changes.
const char *const sig_pew_v2 = "PewPewPew!!2"; // old: sequential pitch slots
const char *const sig_pew    = "PewPewPew!!3"; // current: 1:1 pitch/time

extern EEPROMClass storage;

struct PersistentSettings {
  char signature[16];
  /// MIDI input channel: 0 = omni, 1–16 = listen on that channel only.
  uint8_t midi_channel = 1;
  /// When false, MIDI Clock / Start / Stop are ignored (internal + DIN CLOCK jack only).
  bool midi_clock_receive = true;

  static constexpr int kEepromMidiChannel = 16;
  static constexpr int kEepromMidiFlags = 17;

  void Load() { storage.get(0, signature); }

  void Save() { storage.put(0, signature); }

  bool Validate() const {
    if (0 == strncmp(signature, sig_pew, 12))
      return true;
    // Accept v2 signature for migration path
    if (0 == strncmp(signature, sig_pew_v2, 12))
      return true;
    strcpy((char *)signature, sig_pew);
    return false;
  }

  bool IsV2() const {
    return 0 == strncmp(signature, sig_pew_v2, 12);
  }

  void load_midi_from_storage() {
    const uint8_t ch = storage.read(kEepromMidiChannel);
    const uint8_t fl = storage.read(kEepromMidiFlags);
    if (ch <= 16)
      midi_channel = ch;
    else
      midi_channel = 1;
    if (fl <= 1)
      midi_clock_receive = (fl != 0);
    else
      midi_clock_receive = true;
  }

  void save_midi_to_storage() {
    storage.update(kEepromMidiChannel, midi_channel);
    storage.update(kEepromMidiFlags, midi_clock_receive ? 1 : 0);
  }
};

extern PersistentSettings GlobalSettings;

inline void WritePattern(Sequence &seq, int idx) {
  uint8_t *src = seq.pitch;
  idx *= PATTERN_SIZE;
  for (uint8_t i = 0; i < PATTERN_SIZE; ++i)
    storage.update(SETTINGS_SIZE + idx + i, src[i]);
}
inline void ReadPattern(Sequence &seq, int idx) {
  uint8_t *dst = seq.pitch;
  idx *= PATTERN_SIZE;
  for (uint8_t i = 0; i < PATTERN_SIZE; ++i)
    dst[i] = storage.read(SETTINGS_SIZE + idx + i);
}

/// Migrate a pattern from v2 (sequential pitch slots by NOTE event) to v3 (1:1 pitch/time).
/// Reads note pitches in order from the old array, assigns them to NOTE time positions.
/// TIE and REST positions get PITCH_DEFAULT (preserved but not played).
inline void migrate_pattern_v2_to_v3(Sequence &s) {
  uint8_t old_pitch[MAX_STEPS];
  memcpy(old_pitch, s.pitch, MAX_STEPS);

  // Fill all positions with PITCH_DEFAULT first
  for (uint8_t i = 0; i < MAX_STEPS; ++i)
    s.pitch[i] = PITCH_DEFAULT;

  uint8_t note_idx = 0;
  for (uint8_t i = 0; i < s.length; ++i) {
    const uint8_t t = s.time(i);
    if (t == 1) {
      // NOTE step: pull the next sequential pitch from old array
      if (note_idx < MAX_STEPS && old_pitch[note_idx] != PITCH_EMPTY)
        s.pitch[i] = old_pitch[note_idx];
      ++note_idx;
    }
    // TIE / REST: leave PITCH_DEFAULT — pitch is preserved but not played
  }
}

// =============================================================================
// Engine — patterns + clock + gate
// =============================================================================
struct Engine {
  // pattern storage
  Sequence pattern[NUM_PATTERNS]; // 64 steps each
  uint8_t p_select = 0;
  uint8_t next_p = 0; // queued pattern
                      // TODO: start & end for chains

  SequencerMode mode_ = NORMAL_MODE;

  int8_t clk_count = -1;

  bool slide_gate = false; // tie/slide: hold gate across 16ths (firstpr.com 303 slide / gate)
  bool stale = false;
  bool resting = false;

  // Updated when the 16th advances (Clock, Reset, manual advance).
  uint32_t step_start_us_ = 0;

  void Load() {
#if DEBUG
    Serial.println("Loading from EEPROM...");
#endif
    GlobalSettings.Load();
    bool valid = GlobalSettings.Validate();
    bool needsMigration = GlobalSettings.IsV2();

    if (valid) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        ReadPattern(pattern[i], i);
        if (0 == pattern[i].length) pattern[i].SetLength(8);
        normalize_pattern_times(pattern[i]);
      }
      GlobalSettings.load_midi_from_storage();

      // Migrate v2 → v3 if needed
      if (needsMigration) {
#if DEBUG
        Serial.println("Migrating patterns v2->v3 (1:1 pitch/time model)...");
#endif
        for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
          migrate_pattern_v2_to_v3(pattern[i]);
          normalize_pattern_times(pattern[i]);
        }
        // Update signature to v3
        strcpy(GlobalSettings.signature, sig_pew);
        GlobalSettings.Save();
        stale = true;
        Save();
      }
    } else {
#if DEBUG
      Serial.println("EEPROM data invalid, initializing...");
#endif
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        pattern[i].Clear();
      GlobalSettings.midi_channel = 1;
      GlobalSettings.midi_clock_receive = true;
      strcpy(GlobalSettings.signature, sig_pew);
      GlobalSettings.Save();
      GlobalSettings.save_midi_to_storage();
      stale = true;
      Save();
    }
  }

  void Save(int pidx = -1) {
    if (!stale) return;
    if (pidx < 0) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        WritePattern(pattern[i], i);
    } else
      WritePattern(pattern[pidx], pidx);
    stale = false;
  }

  void Tick() {}

  // After Advance() outside Clock() — re-anchor step time.
  void SyncAfterManualAdvance(bool) { step_start_us_ = micros(); }

  /// Slide CV: only during the *destination* note after a slid step (not during the slid note).
  bool get_slide_dac() const { return get_sequence().slide_from_prev(); }

  // returns false for rests
  bool Advance() {
    bool result = get_sequence().Advance();
    // jump to next pattern at end of current one
    if (0 == get_sequence().time_pos && next_p != p_select) {
      p_select = next_p;
      get_sequence().Reset();
      result = get_sequence().Advance();
    }
    if (result) {
      // Gate overlap: source step holds gate open all 6 clocks so the destination note
      // starts while gate is still high (continuous legato/portamento). Tie steps also
      // hold gate. slide_from_prev() is used only by get_slide_dac() for the CV slide pin.
      slide_gate = get_sequence().get_slide() || get_sequence().is_tied();
    }
    resting = !result;
    return result;
  }

  // returns true on step advance (clock divide by 6)
  bool Clock() {
    ++clk_count %= 6;

    if (clk_count == 0) { // sixteenth note advance
      Advance();
      step_start_us_ = micros();
      return true;
    }

    return false;
  }

  void Reset() {
    get_sequence().Reset();
    clk_count = -1;
    slide_gate = false;
    resting = true;
    step_start_us_ = micros();
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

  bool get_gate() const {
    if (resting) return false;
    if (slide_gate) return true;
    return clk_count < 3;
  }
  bool get_accent() const {
    return !resting && get_sequence().get_accent();
  }
  uint8_t get_semitone() const {
    return get_sequence().get_semitone();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch();
  }
  // MIDI note number: OCTAVE_DOWN C = 36 (C2). Uses get_pitch() so TIE steps with an
  // empty pitch slot still resolve to the carried note (must match DAC / audition).
  uint8_t get_midi_note() const {
    return uint8_t(36 + get_sequence().get_pitch());
  }
  bool get_slide() const {
    return get_sequence().get_slide();
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
    next_p = p_ & 0xf;
    if (override) {
      p_select = next_p;
    }
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
    get_sequence().nudge_octave_buttons(dir);
    stale = true;
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

  /// Step back one position in the current edit mode (TIME: linear; PITCH: previous NOTE).
  bool StepBack() {
    bool moved = (mode_ == PITCH_MODE) ? get_sequence().StepBackPitchByNote()
                                       : get_sequence().StepBack();
    if (moved) stale = true;
    return moved;
  }

  /// True when the edited step is note-locked (bit set from TIME_MODE).
  bool is_step_locked() const {
    const Sequence &s = get_sequence();
    if (mode_ == TIME_MODE)
      return s.step_locked(uint8_t(s.time_pos));
    if (mode_ == PITCH_MODE)
      return s.step_locked(uint8_t(s.pitch_pos));
    return false;
  }
  void ToggleStepLockFromTimeMode() {
    if (mode_ != TIME_MODE) return;
    get_sequence().ToggleStepLock(uint8_t(get_sequence().time_pos));
    stale = true;
  }

  /// EEPROM pattern image: 128 bytes from `pitch[0]` through `length` (see WritePattern).
  void export_pattern_blob(uint8_t idx, uint8_t *blob128) const {
    idx &= 0xf;
    memcpy(blob128, pattern[idx].pitch, PATTERN_SIZE);
  }

  /// Replace pattern RAM from host SysEx. If `persist_eeprom`, also write EEPROM.
  bool import_pattern_blob(uint8_t idx, const uint8_t *blob128, bool persist_eeprom = true) {
    idx &= 0xf;
    const uint8_t L = blob128[PATTERN_SIZE - 1];
    if (L < 1 || uint8_t(L) > MAX_STEPS)
      return false;
    memcpy(pattern[idx].pitch, blob128, PATTERN_SIZE);
    Sequence &s = pattern[idx];
    normalize_pattern_times(s);
    if (s.pitch_pos >= s.length)
      s.pitch_pos = 0;
    if (s.time_pos >= s.length)
      s.time_pos = 0;
    if (persist_eeprom)
      WritePattern(pattern[idx], idx);
    return true;
  }

  /// MIDI Note On → current playing pitch step (live). `note` 36..84 → linear 0..48 (matches CV / pack range).
  void midi_apply_note_on(uint8_t note, uint8_t velocity) {
    if (note < 36 || note > 36 + 48)
      return;
    uint8_t lin = uint8_t(note - 36);
    if (lin > 48)
      lin = 48;
    uint8_t oct_btn = lin / 12;
    if (oct_btn > 3)
      oct_btn = 3;
    uint8_t key_idx = uint8_t(lin - oct_btn * 12);
    if (key_idx > PITCH_KEY_HIGH_C)
      key_idx = PITCH_KEY_HIGH_C;
    const uint8_t pk = pack_pitch(key_idx, oct_btn);
    const uint8_t acc = (velocity >= 100) ? uint8_t(1u << 6) : 0;
    Sequence &s = get_sequence();
    const uint8_t st = uint8_t(s.pitch_pos) & (MAX_STEPS - 1);
    const uint8_t slide = s.pitch_is_empty(st) ? 0 : (s.pitch[st] & (1u << 7));
    s.pitch[st] = (pk & 0x3f) | slide | acc;
    stale = true;
  }
};
