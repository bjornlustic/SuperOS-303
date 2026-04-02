// Copyright (c) 2026, Nicholas J. Michalek
/*
 * engine.h — TB-303 pattern model + EEPROM; Engine handles patterns, clock, and gate.
 *
 * EEPROM versions:
 *   "PewPewPew!!2" — original: pitch slots packed sequentially by NOTE event
 *   "PewPewPew!!3" — 1:1 pitch/time model, 64-step, 16 patterns
 *   "PewPewPew!!4" — 1:1 pitch/time model, 16-step max, 4 groups x 16 patterns (32 bytes/pattern)
 */

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define CONSTRAIN(x, lb, ub) do { if (x < (lb)) x = lb; else if (x > (ub)) x = ub; } while (0)

static constexpr int MAX_STEPS = 64;
static constexpr int MAX_USER_STEPS = 16;
static constexpr int NUM_GROUPS = 4;
static constexpr int NUM_PATTERNS = 16;
static constexpr int EEPROM_PATTERN_SIZE = 32; // compact EEPROM bytes per pattern (supports 4 groups in 4KB)

enum SequencerMode {
  NORMAL_MODE,
  PITCH_MODE,
  TIME_MODE,
};

enum SequenceDirection : uint8_t {
  DIR_FORWARD   = 0, ///< Normal forward playback
  DIR_REVERSE   = 1, ///< Reverse (end→start)
  DIR_PINGPONG  = 2, ///< Bounce forward then backward
  DIR_RANDOM    = 3, ///< Each step jumps to a random position
  DIR_HALF_RAND = 4, ///< 50% chance of random step, else forward
  DIR_BROWNIAN  = 5, ///< Random walk: each step moves ±1 from current position
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
    // step_dir >= 0 (moved forward): previous is at lower index → subtract 1 (delta = length-1).
    // step_dir <  0 (moved backward): previous is at higher index → add 1 (delta = 1).
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
  /// Next time slot is a tie (extends previous note) — forward-only, kept for edit paths.
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
  /// After SetTime() at time_pos, reflow pitch data so NOTE steps keep sequential
  /// pitches from the previous stream. Pass the time value at time_pos *before*
  /// SetTime was called. No-op when the step stays NOTE or stays non-NOTE.
  /// Stream includes non-NOTE slots with user-written pitch data so pitches written
  /// via PITCH_MODE survive the first TIME_MODE reflow.
  void reflow_pitches_after_time_change(uint8_t old_time) {
    const uint8_t tp = uint8_t(time_pos & (MAX_STEPS - 1));
    const uint8_t new_t = time(tp);
    if ((old_time == 1) == (new_t == 1)) return; // NOTE↔NOTE or non↔non: nothing to do

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
  void SetLength(uint8_t len) { length = constrain(len, 1, MAX_USER_STEPS); }
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
    if (++length == MAX_USER_STEPS) return false;
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
  /// `pp_dir` is a persistent ±1 for ping-pong kept in Engine.
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

  /// PITCH_MODE: step back one position linearly (same as TIME_MODE).
  bool StepBackPitchByNote() {
    return StepBack();
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
static constexpr int PATTERN_SIZE = MAX_STEPS * 2; // in-RAM / SysEx blob size (128 bytes)

// Bump version when pattern EEPROM layout/meaning changes.
const char *const sig_pew_v2 = "PewPewPew!!2"; // v2: sequential pitch slots
const char *const sig_pew_v3 = "PewPewPew!!3"; // v3: 1:1 pitch/time, 64-step
const char *const sig_pew    = "PewPewPew!!4"; // v4: 1:1 pitch/time, 16-step groups

extern EEPROMClass storage;

struct PersistentSettings {
  char signature[16];
  /// MIDI input channel: 0 = omni, 1–16 = listen on that channel only.
  uint8_t midi_channel = 1;
  /// When false, MIDI Clock / Start / Stop are ignored (internal + DIN CLOCK jack only).
  bool midi_clock_receive = true;
  /// Sequence playback direction (SequenceDirection enum).
  uint8_t sequence_direction = 0; // DIR_FORWARD
  /// When true, MIDI IN messages are forwarded to MIDI OUT (software MIDI thru).
  bool midi_thru = false;

  static constexpr int kEepromMidiChannel   = 16;
  static constexpr int kEepromMidiFlags     = 17;
  static constexpr int kEepromDirection     = 18;
  static constexpr int kEepromMidiThru      = 19;

  void Load() { storage.get(0, signature); }

  void Save() { storage.put(0, signature); }

  bool Validate() const {
    if (strncmp(signature, sig_pew, 12) == 0)    return true; // v4
    if (strncmp(signature, sig_pew_v3, 12) == 0) return true; // v3: needs migration
    strcpy((char *)signature, sig_pew);
    return false;
  }

  bool IsV3() const { return strncmp(signature, sig_pew_v3, 12) == 0; }

  void load_midi_from_storage() {
    const uint8_t ch   = storage.read(kEepromMidiChannel);
    const uint8_t fl   = storage.read(kEepromMidiFlags);
    const uint8_t dir  = storage.read(kEepromDirection);
    const uint8_t thru = storage.read(kEepromMidiThru);
    midi_channel        = (ch <= 16) ? ch : 1;
    midi_clock_receive  = (fl <= 1)  ? (fl != 0) : true;
    sequence_direction  = (dir < uint8_t(DIR_COUNT)) ? dir : 0;
    midi_thru           = (thru == 1);
  }

  void save_midi_to_storage() {
    storage.update(kEepromMidiChannel, midi_channel);
    storage.update(kEepromMidiFlags, midi_clock_receive ? 1 : 0);
    storage.update(kEepromDirection, sequence_direction);
    storage.update(kEepromMidiThru, midi_thru ? 1 : 0);
  }
};

extern PersistentSettings GlobalSettings;

// Compact EEPROM format (32 bytes):
//   [0..15]  pitch[0..15]
//   [16..23] time_data[0..7]  (covers 16 steps)
//   [24..25] step_lock[0..1]
//   [26..29] ratchets for steps 0-15 (reserved[1..4])
//   [30]     bits[2:0]=direction, bits[7:3]=stash_count
//   [31]     length
inline void WritePattern(Sequence &seq, int idx, int group) {
  uint8_t compact[EEPROM_PATTERN_SIZE];
  memcpy(compact,       seq.pitch,        16);
  memcpy(compact + 16,  seq.time_data,     8);
  memcpy(compact + 24,  seq.step_lock,     2);
  memcpy(compact + 26,  seq.reserved + 1,  4); // ratchets for steps 0-15
  uint8_t stash = (seq.reserved[21] < MAX_USER_STEPS) ? seq.reserved[21] : 0;
  compact[30] = (seq.reserved[0] & 0x07) | uint8_t(stash << 3);
  compact[31] = seq.length;
  int off = SETTINGS_SIZE + (group * NUM_PATTERNS + idx) * EEPROM_PATTERN_SIZE;
  for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
    storage.update(off + i, compact[i]);
}
inline void ReadPattern(Sequence &seq, int idx, int group) {
  int off = SETTINGS_SIZE + (group * NUM_PATTERNS + idx) * EEPROM_PATTERN_SIZE;
  uint8_t compact[EEPROM_PATTERN_SIZE];
  for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
    compact[i] = storage.read(off + i);
  memset(seq.pitch,     PITCH_EMPTY, MAX_STEPS);
  memset(seq.time_data, 0,           sizeof(seq.time_data));
  memset(seq.step_lock, 0,           sizeof(seq.step_lock));
  memset(seq.reserved,  0,           sizeof(seq.reserved));
  memcpy(seq.pitch,        compact,      16);
  memcpy(seq.time_data,    compact + 16,  8);
  memcpy(seq.step_lock,    compact + 24,  2);
  memcpy(seq.reserved + 1, compact + 26,  4); // ratchets
  seq.reserved[0]  = compact[30] & 0x07;        // direction
  seq.reserved[21] = (compact[30] >> 3) & 0x1F; // stash_count
  seq.length = compact[31];
}

// Migrate v3 (128-byte patterns, single group) -> v4 (32-byte compact, 4 groups).
// Reads old layout from EEPROM using raw byte offsets; writes compact format.
// Processes patterns 0-15 in order (safe: new offsets never overlap unread old data).
inline void migrate_v3_to_v4() {
  uint8_t buf[128];
  // Init groups 1-3 as blank first (may overlap old offsets above pattern 0)
  // Actually, old pattern p was at SETTINGS_SIZE + p*128, new is at SETTINGS_SIZE + p*32.
  // For p=0..15: new end <= 128 + 15*32 + 31 = 607 < old start of p=1 (256). Safe to process in order.
  uint8_t blank[EEPROM_PATTERN_SIZE];
  memset(blank, 0, sizeof(blank));
  for (uint8_t i = 0; i < 16; ++i) blank[i] = PITCH_EMPTY;
  blank[31] = 8;

  for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
    // Read old 128-byte blob from v3 location
    int old_off = SETTINGS_SIZE + p * 128;
    for (uint8_t i = 0; i < 128; ++i)
      buf[i] = storage.read(old_off + i);

    uint8_t compact[EEPROM_PATTERN_SIZE];
    memcpy(compact,      buf,       16); // pitch[0..15]
    memcpy(compact + 16, buf + 64,   8); // time_data[0..7]
    memcpy(compact + 24, buf + 96,   2); // step_lock[0..1]
    memcpy(compact + 26, buf + 105,  4); // ratchets (reserved[1..4])
    uint8_t dir   = buf[104] & 0x07;
    uint8_t stash = (buf[125] < MAX_USER_STEPS) ? buf[125] : 0;
    compact[30] = dir | uint8_t(stash << 3);
    uint8_t L = buf[127];
    compact[31] = (L >= 1 && L <= MAX_USER_STEPS) ? L : 8;

    // Write to group 0 location
    int new_off = SETTINGS_SIZE + p * EEPROM_PATTERN_SIZE;
    for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
      storage.update(new_off + i, compact[i]);
  }
  // Initialize groups 1-3 with blank patterns
  for (uint8_t g = 1; g < NUM_GROUPS; ++g) {
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      int off = SETTINGS_SIZE + (g * NUM_PATTERNS + p) * EEPROM_PATTERN_SIZE;
      for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
        storage.update(off + i, blank[i]);
    }
  }
  strcpy(GlobalSettings.signature, sig_pew);
  GlobalSettings.Save();
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
  Sequence pattern[NUM_PATTERNS];
  uint8_t p_select = 0;
  uint8_t next_p = 0; // queued pattern
  uint8_t group_ = 0; // active pattern group (0-3)
                      // TODO: start & end for chains

  SequencerMode      mode_      = NORMAL_MODE;
  SequenceDirection  direction_ = DIR_FORWARD;
  int8_t  pp_dir_        = 1;  ///< Ping-pong direction: +1 or -1
  uint8_t advance_count_ = 0;  ///< Steps since last pattern boundary (non-forward modes)
  /// Queued direction change (applied at next pattern wrap while clock is running).
  SequenceDirection  next_direction_          = DIR_FORWARD;
  bool               direction_change_pending_ = false;
  /// Direction of the last step advance: +1 = moved forward, -1 = moved backward.
  /// Used by slide_from_prev_dir and get_pitch_dir after Clock() to resolve pitch/slide.
  int8_t last_step_dir_ = 1;

  int8_t clk_count = -1;

  bool slide_gate = false; // tie/slide: hold gate across 16ths (firstpr.com 303 slide / gate)
  bool stale = false;
  bool resting = false;

  // Updated when the 16th advances (Clock, Reset, manual advance).
  uint32_t step_start_us_ = 0;

  uint8_t get_group() const { return group_; }

  // Switch to a different pattern group: saves current, loads new.
  void SetGroup(uint8_t g) {
    if (g >= NUM_GROUPS || g == group_) return;
    stale = true;
    Save();
    group_ = g;
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
      ReadPattern(pattern[i], i, group_);
      if (!pattern[i].length) pattern[i].SetLength(8);
      normalize_pattern_times(pattern[i]);
    }
    stale = false;
  }

  void Load() {
    GlobalSettings.Load();
    bool valid = GlobalSettings.Validate();

    if (valid && GlobalSettings.IsV3()) {
      migrate_v3_to_v4();
    }

    if (valid) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        ReadPattern(pattern[i], i, group_);
        if (!pattern[i].length) pattern[i].SetLength(8);
        normalize_pattern_times(pattern[i]);
      }
      GlobalSettings.load_midi_from_storage();
      direction_ = DIR_FORWARD;
    } else {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        pattern[i].Clear();
      GlobalSettings.midi_channel = 1;
      GlobalSettings.midi_clock_receive = true;
      GlobalSettings.midi_thru = false;
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
        WritePattern(pattern[i], i, group_);
    } else
      WritePattern(pattern[pidx], pidx, group_);
    stale = false;
  }

  void Tick() {}

  // After Advance() outside Clock() — re-anchor step time.
  void SyncAfterManualAdvance(bool) { step_start_us_ = micros(); }

  /// Slide CV: only during the *destination* note after a slid step (not during the slid note).
  /// Uses last_step_dir_ so callers (midi_after_clock) get the correct directional result.
  bool get_slide_dac() const {
    return get_sequence().slide_from_prev_dir(uint8_t(direction_), last_step_dir_);
  }

  // returns false for rests
  bool Advance() {
    bool result;
    // Track the direction of movement for this step (used by slide/tie/pitch after the call).
    int8_t step_dir     = 1; // direction we just moved: +1 = forward, -1 = backward
    int8_t next_step_dir = 1; // direction of the *next* step (for is_tied_dir check)

    if (direction_ == DIR_FORWARD) {
      step_dir     = 1;
      next_step_dir = 1;
      result = get_sequence().Advance();
      // Apply queued direction change and/or pattern switch at wrap (time_pos == 0).
      // first_step is still true after a reset-first-step (not a real wrap); guard against it.
      if (0 == get_sequence().time_pos && !get_sequence().first_step) {
        if (direction_change_pending_) {
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
        }
        if (next_p != p_select) {
          p_select = next_p;
          get_sequence().Reset();
          direction_change_pending_ = false;
          pp_dir_ = 1; advance_count_ = 0;
          result = get_sequence().Advance();
          step_dir = next_step_dir = 1;
        }
      }
    } else {
      // Capture step_dir before AdvanceDirectional may flip pp_dir_ (ping-pong / brownian).
      if (direction_ == DIR_REVERSE)                                 step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) step_dir = pp_dir_;
      else                                                           step_dir = 1; // random: doesn't matter

      result = get_sequence().AdvanceDirectional(uint8_t(direction_), pp_dir_);

      // After advance: pp_dir_ carries the outgoing direction for ping-pong / brownian.
      if (direction_ == DIR_REVERSE)                                 next_step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) next_step_dir = pp_dir_;
      else                                                           next_step_dir = 1; // random: is_tied_dir returns false anyway

      ++advance_count_;
      if (advance_count_ >= get_sequence().length) {
        advance_count_ = 0;
        if (direction_change_pending_) {
          const SequenceDirection old_dir = direction_;
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
          // PINGPONG wraps at two different positions (step 0 on the backward leg,
          // step length-1 on the forward leg), so time_pos is indeterminate when the
          // direction change fires.  Force the pre-start position that the new
          // direction expects so it always lands on the correct first step.
          // REVERSE always wraps at step 0, which already aligns correctly, so
          // no adjustment is needed there.
          if (old_dir == DIR_PINGPONG) {
            const int len = int(get_sequence().length);
            if (direction_ == DIR_FORWARD) {
              // The direction change fires on a pingpong endpoint retrigger step.
              // Pre-positioning at len-1 + first_step would add a phantom step before
              // step 0, shifting forward phase by 1.  Instead, play step 0 directly on
              // this tick so timing stays phase-locked with the advance_count_ boundary.
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
              result        = get_sequence().time(0) != 0;
              step_dir      = 1;
              next_step_dir = 1;
            } else if (direction_ == DIR_REVERSE) {
              // AdvanceDirectional(REVERSE): time_pos <= 0 → length-1; force 0.
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
            } else {
              // Switching to another non-forward mode: clean reset to step 0.
              get_sequence().Reset();
            }
          }
        }
        if (next_p != p_select) {
          p_select = next_p;
          get_sequence().Reset();
          advance_count_ = 0;
          direction_change_pending_ = false;
          pp_dir_ = 1;
        }
      }
    }

    last_step_dir_ = step_dir;

    if (result) {
      // Direction-aware slide_gate: hold gate when current step has slide flag,
      // next step (in play direction) is a TIE, or current TIE was preceded by a slide.
      const bool next_is_tie = get_sequence().is_tied_dir(uint8_t(direction_), next_step_dir);
      const bool tie_slide   = get_sequence().is_tie() &&
                               get_sequence().slide_from_prev_dir(uint8_t(direction_), step_dir);
      // A REST immediately after a slide source cancels the slide — don't hold gate.
      const uint8_t next_pos = uint8_t(
          (unsigned(get_sequence().time_pos) +
           (next_step_dir >= 0 ? 1u : unsigned(get_sequence().length) - 1u)) %
          unsigned(get_sequence().length));
      const bool next_is_rest = (get_sequence().time(next_pos) == 0);
      slide_gate = (!next_is_rest && get_sequence().get_slide()) || next_is_tie || tie_slide;
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
    // Apply any queued direction change immediately on stop/reset so the LED
    // state and the actual direction_ stay in sync for the next run.
    if (direction_change_pending_) {
      direction_ = next_direction_;
      direction_change_pending_ = false;
    }
    get_sequence().Reset();
    clk_count = -1;
    slide_gate = false;
    resting = true;
    step_start_us_ = micros();
    advance_count_ = 0;
    pp_dir_ = 1;
    last_step_dir_ = 1;
  }

  /// Ratchet distribution: ~50% none (1x), ~33% 2x, ~17% 3x.
  static uint8_t random_ratchet_val() {
    const uint8_t r = uint8_t(random(12));
    return r < 6 ? 0 : r < 10 ? 1 : 2;
  }

  /// Randomize entire pattern: both time data and pitches (NORMAL_MODE).
  void RandomizeFullPattern() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    // First pass: random time data, no ties after rests, no leading tie
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      uint8_t t;
      if (i == 0)       t = uint8_t(random(2)) ? 1 : 0;  // note or rest
      else if (prev==0) t = uint8_t(random(2)) ? 1 : 0;  // after rest: note or rest
      else              t = uint8_t(random(3));            // note, tie, or rest
      s.time_pos = i;
      s.SetTime(t);
      prev = t;
    }
    normalize_pattern_times(s);
    // Second pass: random pitches for NOTE steps; also randomize ratchets
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1) {
        const uint8_t pk  = uint8_t(random(PITCH_PACK_MAX + 1));
        const uint8_t acc = uint8_t(random(2)) ? 0x40 : 0;
        const uint8_t sl  = uint8_t(random(2)) ? 0x80 : 0;
        s.pitch[i] = pk | acc | sl;
        // Ratchet: 0=1x (most common), 1=2x, 2=3x, 3=4x; skip if slide
        s.set_ratchet_val(i, sl ? 0 : random_ratchet_val());
      } else {
        s.pitch[i] = PITCH_DEFAULT;
        s.set_ratchet_val(i, 0);
      }
    }
    stale = true;
  }

  /// Randomize pitches only — keeps time data intact (PITCH_MODE).
  void RandomizePitchData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1) {
        const uint8_t pk  = uint8_t(random(PITCH_PACK_MAX + 1));
        const uint8_t acc = uint8_t(random(2)) ? 0x40 : 0;
        const uint8_t sl  = uint8_t(random(2)) ? 0x80 : 0;
        s.pitch[i] = pk | acc | sl;
        s.set_ratchet_val(i, sl ? 0 : random_ratchet_val());
      }
    }
    stale = true;
  }

  /// Randomize time data only — keeps pitches intact (TIME_MODE).
  void RandomizeTimeData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      uint8_t t;
      if (i == 0)       t = uint8_t(random(2)) ? 1 : 0;
      else if (prev==0) t = uint8_t(random(2)) ? 1 : 0;
      else              t = uint8_t(random(3));
      s.time_pos = i;
      s.SetTime(t);
      // Clear ratchet on any step that is no longer a plain NOTE
      if (t != 1) s.set_ratchet_val(i, 0);
      prev = t;
    }
    normalize_pattern_times(s);
    stale = true;
  }

  /// Randomize ratchet values only — keeps all other data intact.
  void RandomizeRatchetData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !(s.pitch[i] & 0x80)) {
        // NOTE step without slide: randomize ratchet
        s.set_ratchet_val(i, random_ratchet_val());
      } else {
        s.set_ratchet_val(i, 0);
      }
    }
    stale = true;
  }

  /// Randomize accent flags only (~20% probability) — keeps pitch, slide, and time intact.
  void RandomizeAccentData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (random(5) == 0)  // ~20%
          s.pitch[i] |=  0x40;
        else
          s.pitch[i] &= ~0x40;
      }
    }
    stale = true;
  }

  /// Randomize slide flags only (~15% probability) — keeps pitch, accent, and time intact.
  /// Clears ratchet on any step that gains a slide flag (slide and ratchet are mutually exclusive).
  void RandomizeSlideData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (random(20) < 3) { // ~15%
          s.pitch[i] |=  0x80;
          s.set_ratchet_val(i, 0);
        } else {
          s.pitch[i] &= ~0x80;
        }
      }
    }
    stale = true;
  }

  // Direction accessors
  /// Returns the "user-visible" direction: the pending one if a change is queued,
  /// otherwise the active one. Used for LED display so the newly-selected direction
  /// lights up immediately even if it hasn't taken effect yet.
  SequenceDirection get_direction() const {
    return direction_change_pending_ ? next_direction_ : direction_;
  }
  void SetDirection(SequenceDirection d) {
    stale = true;
    if (clk_count == -1) {
      // Clock not running: apply immediately (no wrap boundary to wait for).
      direction_ = d;
      pp_dir_ = 1;
      advance_count_ = 0;
      direction_change_pending_ = false;
    } else {
      // Clock running: queue the change to take effect at the next pattern-cycle
      // boundary, so the "1" (step 0 / downbeat) stays phase-locked to the clock.
      next_direction_ = d;
      direction_change_pending_ = true;
    }
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
    // Slide destination: never ratchet — gate arrives via held CV from source.
    // Check before ratchet logic so a destination with a ratchet value set doesn't fire.
    // Steps that are both a destination AND a source hold the gate all 6 clocks.
    if (get_slide_dac()) return slide_gate ? true : (clk_count < 3);
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    // Slide source with no ratchet: hold gate HIGH all 6 clocks to bridge into destination.
    if (slide_gate && r == 0) return true;
    // Slide source with ratchet: use ratchet pattern but pin clock 5 HIGH so the gate
    // stays up through the step boundary and glides into the destination.
    switch (r) {
      case 1: return (uint8_t(clk_count) % 3u) == 0u || (slide_gate && clk_count == 5); // 2x
      case 2: return (uint8_t(clk_count) % 2u) == 0u || (slide_gate && clk_count == 5); // 3x
      default: break;                                  // r==0 (slide_gate false) falls through
    }
    return clk_count < 3;                              // normal note
  }

  /// Returns true when this MIDI clock tick is a ratchet retrigger within the current step
  /// (i.e., not the step-advance clock 0, but a subsequent sub-note trigger).
  /// Use this in the main loop alongside Clock() to fire midi_ratchet_retrigger().
  bool is_ratchet_retrigger() const {
    if (resting) return false;
    // Slide destination: never ratchet (being slid into, no new envelope trigger expected).
    if (get_slide_dac()) return false;
    // slide_gate true = source note of a slide; ratcheting is allowed on source notes.
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    switch (r) {
      case 1: return clk_count == 3;                    // 2x
      case 2: return clk_count == 2 || clk_count == 4; // 3x
      default: return false;                            // 1x or any stale val >2
    }
  }
  bool get_accent() const {
    // Accent belongs to the source note; suppress on slide destination (no new envelope trigger).
    return !resting && !get_slide_dac() && get_sequence().get_accent();
  }
  uint8_t get_semitone() const {
    return get_sequence().get_semitone();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch_dir(last_step_dir_);
  }
  // MIDI note number: OCTAVE_DOWN C = 36 (C2). Uses get_pitch_dir() so TIE steps in
  // any play direction resolve to the note that started the tie chain.
  uint8_t get_midi_note() const {
    return uint8_t(36 + get_sequence().get_pitch_dir(last_step_dir_));
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
    uint8_t L = blob128[PATTERN_SIZE - 1];
    if (L < 1 || uint8_t(L) > MAX_STEPS)
      return false;
    if (L > MAX_USER_STEPS) L = MAX_USER_STEPS;
    memcpy(pattern[idx].pitch, blob128, PATTERN_SIZE);
    pattern[idx].length = L;
    Sequence &s = pattern[idx];
    normalize_pattern_times(s);
    if (s.pitch_pos >= s.length) s.pitch_pos = 0;
    if (s.time_pos  >= s.length) s.time_pos  = 0;
    if (persist_eeprom)
      WritePattern(pattern[idx], idx, group_);
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
