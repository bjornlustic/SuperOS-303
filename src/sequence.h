// Copyright (c) 2026, Nicholas J. Michalek
//
// sequence.h -- TB-303 pattern data model.
// One `Sequence` = one pattern (128 bytes in RAM).
// Pure data + inline helpers, no EEPROM or Engine coupling.

#pragma once
#include <Arduino.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define CONSTRAIN(x, lb, ub) do { if (x < (lb)) x = lb; else if (x > (ub)) x = ub; } while (0)

// Fast 16-bit xorshift PRNG - avoids the expensive 32-bit random() on AVR
// which can block the main loop long enough to miss DIN sync clock pulses.
static uint16_t s_fast_rng = 1;
static inline void fast_rand_seed() { s_fast_rng = uint16_t(micros()) | 1; }
static inline uint8_t fast_rand(uint8_t n) {
  s_fast_rng ^= s_fast_rng << 7;
  s_fast_rng ^= s_fast_rng >> 9;
  s_fast_rng ^= s_fast_rng << 8;
  return uint8_t(s_fast_rng % n);
}

static constexpr int MAX_STEPS = 64;
static constexpr int MAX_USER_STEPS = 16;
static constexpr int NUM_GROUPS = 4;
static constexpr int NUM_PATTERNS = 16;
static constexpr int EEPROM_PATTERN_SIZE = 32; // compact EEPROM bytes per pattern (groups 0-2)
// Groups 0-2 use 32-byte compact; group 3 uses full 128-byte blob at a separate EEPROM base.
static constexpr int EEPROM_G4_PATTERN_SIZE = 128; // = PATTERN_SIZE, defined after
static constexpr int G4_EEPROM_BASE = 128 + (NUM_GROUPS - 1) * NUM_PATTERNS * EEPROM_PATTERN_SIZE; // 1664
static constexpr uint8_t max_steps_for_group(uint8_t g) { return g == 3 ? uint8_t(MAX_STEPS) : MAX_USER_STEPS; }

enum SequencerMode {
  NORMAL_MODE,
  PITCH_MODE,
  TIME_MODE,
};

enum SequenceDirection : uint8_t {
  DIR_FORWARD   = 0, ///< Normal forward playback
  DIR_REVERSE   = 1, ///< Reverse (end->start)
  DIR_PINGPONG  = 2, ///< Bounce forward then backward
  DIR_RANDOM    = 3, ///< Each step jumps to a random position
  DIR_HALF_RAND = 4, ///< 50% chance of random step, else forward
  DIR_BROWNIAN  = 5, ///< Random walk: each step moves +/-1 from current position
  DIR_COUNT     = 6,
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
// Sequence -- one pattern (matches reference OS-303 layout)
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

  /// Per-pattern direction stored in reserved[0] (byte 104 of the 128-byte blob).
  uint8_t get_direction_stored() const { return reserved[0]; }
  void    store_direction(uint8_t d)   { reserved[0] = (d < 5) ? d : 0; }

  /// Pitch stash count stored in reserved[21] (byte 125).
  /// The stash holds pitches displaced from NOTE steps when the NOTE count shrinks;
  /// they live in pitch[length..length+stash_count-1] within the 64-byte pitch array.
  /// Mirrors the web editor's STASH_COUNT_IDX = 125 convention.
  uint8_t get_stash_count() const { return reserved[21] < MAX_STEPS ? reserved[21] : 0; }
  void    set_stash_count(uint8_t n) { reserved[21] = (n < MAX_STEPS) ? n : uint8_t(MAX_STEPS - 1); }

  /// Ratchet value for a step (0=1x, 1=2x, 2=3x max).
  /// Packed 2 bits/step in reserved[1..16] (bytes 105-120). Mirrors web RATCHET_BASE=105.
  /// Only meaningful on NOTE steps without slide.
  uint8_t get_ratchet_val(uint8_t step) const {
    const uint8_t idx = step & (MAX_STEPS - 1);
    const uint8_t b   = uint8_t(1 + (idx >> 2)); // reserved[1..16]
    const uint8_t sh  = uint8_t((idx & 3u) << 1);
    return (reserved[b] >> sh) & 0x03u;
  }
  void set_ratchet_val(uint8_t step, uint8_t val) {
    const uint8_t idx = step & (MAX_STEPS - 1);
    const uint8_t b   = uint8_t(1 + (idx >> 2));
    const uint8_t sh  = uint8_t((idx & 3u) << 1);
    reserved[b] = uint8_t((reserved[b] & ~(0x03u << sh)) | ((val & 0x03u) << sh));
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
  /// Linear pitch / 12 - pattern-select octave LEDs (DOWN/UP) vs stored keypad octave.
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
  /// Forward-only version (used for display / edit, where step order is always linear).
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
  /// Direction-aware slide detection: checks if the step played just before the current
  /// one had a slide flag set. step_dir is +1 if we moved forward to reach this step,
  /// -1 if we moved backward (reverse / ping-pong-backward leg).
  /// For random modes (DIR_RANDOM / DIR_HALF_RAND) there is no meaningful predecessor,
  /// so always returns false.
  bool slide_from_prev_dir(uint8_t dir, int8_t step_dir) const {
    if (dir == DIR_RANDOM || dir == DIR_HALF_RAND) return false;
    if (length <= 1) return false;
    if (time(time_pos) == 0) return false;
    if (first_step) return false;

    // delta: modular offset that walks us toward the step that played just before.
    // step_dir >= 0 (moved forward): previous is at lower index -> subtract 1 (delta = length-1).
    // step_dir <  0 (moved backward): previous is at higher index -> add 1 (delta = 1).
    const unsigned delta = (step_dir >= 0) ? (unsigned(length) - 1u) : 1u;
    uint8_t tp = uint8_t((unsigned(time_pos) + delta) % unsigned(length));
    for (uint8_t guard = 0; guard < length; ++guard) {
      const uint8_t tt = time(tp);
      if (tt == 0) return false; // rest breaks slide chain
      if (tt == 1) return get_slide(tp);
      // tie: keep walking in the same direction
      tp = uint8_t((unsigned(tp) + delta) % unsigned(length));
    }
    return false;
  }
  /// Direction-aware pitch lookup for playback. For TIE steps the pitch is carried
  /// from the last NOTE step in the play direction (same as slide walk direction).
  /// step_dir: +1 = moving forward (TIE walks backward), -1 = moving backward (TIE walks forward).
  uint8_t get_pitch_dir(int8_t step_dir) const {
    if (time(time_pos) == 2) {
      const unsigned delta = (step_dir >= 0) ? (unsigned(length) - 1u) : 1u;
      for (uint8_t g = 1; g < length; ++g) {
        const uint8_t tp = uint8_t((unsigned(time_pos) + delta * g) % unsigned(length));
        if (time(tp) == 0) break; // rest cancels tie/slide chain
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
  /// Direction-aware: is the NEXT step (in the given play direction) a TIE?
  /// next_dir: +1 = stepping forward next, -1 = stepping backward next.
  /// For random modes returns false (no defined next step).
  bool is_tied_dir(uint8_t dir, int8_t next_dir) const {
    if (dir == DIR_RANDOM || dir == DIR_HALF_RAND || dir == DIR_BROWNIAN) return false;
    if (length == 0) return false;
    const uint8_t n = (next_dir >= 0)
      ? uint8_t((unsigned(time_pos) + 1u) % unsigned(length))
      : uint8_t((unsigned(time_pos) + unsigned(length) - 1u) % unsigned(length));
    return time(n) == 2;
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

  const uint8_t get_time() const { return time(time_pos); }

  void SetTime(uint8_t t) {
    const uint8_t upper = time_pos & 1;
    uint8_t &data = time_data[time_pos >> 1];
    data = (~(0x0f << (4 * upper)) & data) | (t << (4 * upper));
  }
  /// After SetTime() at time_pos, reflow pitch data so NOTE steps keep sequential
  /// pitches from the previous stream. Pass the time value at time_pos *before*
  /// SetTime was called. No-op when the step stays NOTE or stays non-NOTE.
  /// Stream includes non-NOTE slots with user-written pitch data so pitches written
  /// via PITCH_MODE survive the first TIME_MODE reflow.
  bool reflow_pitches_after_time_change(uint8_t old_time) {
    return reflow_pitches_at(uint8_t(time_pos & (MAX_STEPS - 1)), old_time);
  }
  bool reflow_pitches_at(uint8_t tp, uint8_t old_time) {
    const uint8_t new_t = time(tp);
    if ((old_time == 1) == (new_t == 1)) return false; // NOTE<->NOTE or non<->non: nothing to do

    const uint8_t len = uint8_t(length);

    // Build pitch stream from: slots that were NOTE *or* had user-written pitch data,
    // treating the changed step with its old time type (before SetTime).
    uint8_t stream[MAX_STEPS];
    uint8_t slen = 0;
    for (uint8_t i = 0; i < len; ++i) {
      const uint8_t t_i = (i == tp) ? old_time : time(i);
      const bool was_note = (t_i == 1);
      const bool has_data = !pitch_is_empty(i);
      if (was_note || has_data)
        stream[slen++] = has_data ? pitch[i] : PITCH_DEFAULT;
    }
    // Append stash tail.
    const uint8_t old_stash = get_stash_count();
    for (uint8_t k = 0; k < old_stash && slen < MAX_STEPS; ++k) {
      const uint8_t sb = pitch[len + k];
      stream[slen++] = (sb == PITCH_EMPTY) ? PITCH_DEFAULT : sb;
    }

    // Count new NOTE steps.
    uint8_t note_count = 0;
    for (uint8_t i = 0; i < len; ++i) if (time(i) == 1) ++note_count;

    // Ensure stream has enough pitches for all new NOTE slots.
    while (slen < note_count && slen < MAX_STEPS) stream[slen++] = PITCH_DEFAULT;

    // Excess beyond note_count goes to stash (capped by available tail space).
    uint8_t excess = (slen > note_count) ? uint8_t(slen - note_count) : 0;
    const uint8_t max_stash = uint8_t(MAX_STEPS - len);
    if (excess > max_stash) excess = max_stash;

    // Redistribute: NOTE slots get sequential pitches; non-NOTE slots cleared.
    uint8_t ni = 0;
    for (uint8_t i = 0; i < len; ++i) {
      if (time(i) == 1) pitch[i] = stream[ni++];
      else               pitch[i] = PITCH_EMPTY;
    }
    // Write stash tail.
    for (uint8_t k = 0; k < excess; ++k)
      pitch[len + k] = stream[note_count + k];
    set_stash_count(excess);
    return true;
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
  void SetLength(uint8_t len, uint8_t max_len = MAX_USER_STEPS) { length = constrain(len, 1, max_len); }
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

  bool BumpLength(uint8_t max_len = MAX_USER_STEPS) {
    if (++length == max_len) return false;
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
    // Clear ratchet data stored in reserved[1..16] (2 bits/step, 64 steps).
    for (uint8_t i = 1; i <= 16; ++i)
      reserved[i] = 0;
    set_stash_count(0);
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

  /// PITCH_MODE edit entry (TAP held): clear reset flag and land on first NOTE step.
  /// If no NOTE steps exist, defaults to step 0 (caller handles display guard).
  void ensure_pitch_edit_entry() {
    if (reset) {
      reset = false;
      pitch_pos = int(first_note_idx()); // first NOTE step, or 0 if none
      time_pos = pitch_pos;
    }
  }
  /// PITCH_MODE write entry: clear reset flag and land on the first NOTE step so
  /// pitch input targets playable steps rather than REST/TIE slots.
  void ensure_pitch_write_entry() {
    if (reset) {
      reset = false;
      pitch_pos = int(first_note_idx());
      time_pos = pitch_pos;
    }
  }

  /// PITCH_MODE advance: jump to the next NOTE step, skipping REST and TIE.
  /// If no other NOTE steps exist (all-REST pattern), falls back to linear +1
  /// so notes can be entered into a freshly cleared pattern without getting stuck.
  void advance_pitch_to_next_note() {
    first_step = false;
    const uint8_t cur = uint8_t(pitch_pos & (MAX_STEPS - 1));
    const uint8_t nxt = next_note_step_idx(cur);
    // nxt == cur means no other NOTE steps found; use linear advance as fallback
    pitch_pos = int(nxt != cur ? nxt : uint8_t((unsigned(cur) + 1) % unsigned(length)));
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

  /// Direction-aware advance used by Engine for non-forward modes.
  /// `pp_dir` is a persistent +/-1 for ping-pong kept in Engine.
  bool AdvanceDirectional(uint8_t dir, int8_t &pp_dir) {
    if (reset) {
      reset = false;
      pitch_pos = 0;
      time_pos = 0;
      return time(0) != 0;
    }
    first_step = false;
    switch (dir) {
    case DIR_REVERSE:
      if (time_pos <= 0) time_pos = int(length) - 1;
      else               --time_pos;
      break;
    case DIR_PINGPONG: {
      time_pos += pp_dir;
      if (time_pos >= int(length)) {
        pp_dir = -1;
        time_pos = int(length) - 1; // retrigger last step so both endpoints play
      } else if (time_pos < 0) {
        pp_dir = 1;
        time_pos = 0;               // retrigger step 0 so both endpoints play
      }
      break;
    }
    case DIR_RANDOM:
      time_pos = int(random(length));
      break;
    case DIR_HALF_RAND:
      if (random(2)) time_pos = int(random(length));
      else { ++time_pos %= length; }
      break;
    case DIR_BROWNIAN: {
      int8_t bdir = random(2) ? 1 : -1;
      pp_dir = bdir; // pass actual walk direction back to Engine (reuses pp_dir field)
      time_pos = int((unsigned(length) + unsigned(time_pos) + unsigned(bdir)) % unsigned(length));
      break;
    }
    default: // DIR_FORWARD
      ++time_pos %= length;
      break;
    }
    pitch_pos = time_pos;
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

};

/// Set time nibble for step `idx` (0=rest, 1=note, 2=tie).
inline void sequence_set_time_at(Sequence &s, uint8_t idx, uint8_t t) {
  idx &= uint8_t(MAX_STEPS - 1);
  const uint8_t upper = idx & 1u;
  uint8_t &cell = s.time_data[idx >> 1u];
  cell = uint8_t((cell & ~(0x0fu << (4u * upper))) | ((t & 0x0fu) << (4u * upper)));
}

/// TB-303-valid time fixes: no rest->tie, no leading tie, no all-ties (wrap-aware).
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
