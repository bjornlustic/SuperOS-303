// Copyright (c) 2026, Nicholas J. Michalek
//
// sequence.h -- TB-303 pattern data model.
// Layout matches OS-303 v0.6 byte-for-byte so EEPROM is round-trippable
// between SuperOS-303 and stock OS-303 firmware.
//
// Pattern storage (56 bytes):
//   pitch[32]            -- 8 bits: bits[3:0]=semitone (0..12, 12=high-C button)
//                                   bits[5:4]=octave (0..3)
//                                   bit[6]=accent, bit[7]=slide
//                          NOTE-event-indexed: pitch[i] = i-th NOTE in time order.
//                          PITCH_EMPTY (0xFF) marks unwritten slots.
//   time_data[16]        -- 4-bit nibbles per time step (0=REST, 1=NOTE, 2=TIE).
//   reserved[5]          -- bytes that OS-303 round-trips untouched.
//                            reserved[0] = direction (0..5)
//                            reserved[1..4] = ratchet bitmap, 1 bit/step,
//                                             32 bits = 32 steps. Bit set = 2x.
//   transpose            -- per-pattern transpose (semitones).
//   engine_select        -- OS-303's multi-engine selector (we leave at 0).
//   length               -- 1..32.
//
// Runtime state (NOT in EEPROM, lives after the persisted block):
//   pitch_pos, time_pos, reset, first_step, pitch_count_runtime,
//   step_lock_ram (32 bits, RAM-only live-write lockout).

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

static constexpr int MAX_STEPS = 32;
static constexpr int NUM_PATTERNS = 16; // patterns per bank
static constexpr int NUM_BANKS = 4;     // banks 0..3 (was NUM_GROUPS)
static constexpr int NUM_GROUPS = NUM_BANKS; // back-compat alias
static constexpr int METADATA_SIZE = 8; // OS-303: reserved[5] + transpose + engine_select + length

enum SequencerMode {
  NORMAL_MODE,
  PITCH_MODE,
  TIME_MODE,
};

enum SequenceDirection : uint8_t {
  DIR_FORWARD   = 0,
  DIR_REVERSE   = 1,
  DIR_PINGPONG  = 2,
  DIR_RANDOM    = 3,
  DIR_HALF_RAND = 4,
  DIR_BROWNIAN  = 5,
  DIR_COUNT     = 6,
};

// Octave register (low 4-bit values for OS-303 pitch byte). DOUBLE_UP is gone:
// the OS-303 encoding has only 4 octave registers (0..3). High-C is reached by
// pressing the dedicated high-C key, which writes semi=12 instead of climbing
// to oct=4.
enum OctaveState {
  OCTAVE_DOWN      = 0,
  OCTAVE_ZERO      = 1,
  OCTAVE_UP        = 2,
  OCTAVE_DOUBLE_UP = 3,
};

static constexpr uint8_t PITCH_EMPTY     = 0xFF;          // unwritten slot sentinel
static constexpr uint8_t PITCH_DEFAULT   = 0x10;          // semi=0, oct=1 (centre C), no flags
static constexpr uint8_t PITCH_KEY_HIGH_C = 12;           // high-C key index
static constexpr uint8_t PITCH_PACK_MASK = 0x3F;          // bits[5:0] = pitch (semi | oct<<4)

// Pack/unpack: OS-303 v0.6 encoding.
//   semi = 0..12 (12 means user pressed the high-C key explicitly)
//   oct  = 0..3
// Linear semitone is `semi + 12*oct`, range 0..48 (high-C in top register = 48).
static inline uint8_t pack_pitch(uint8_t semi, uint8_t oct) {
  return uint8_t((semi & 0x0F) | ((oct & 0x03) << 4));
}
static inline uint8_t unpack_pitch_linear(uint8_t e) {
  return (e & 0x0F) + 12 * ((e >> 4) & 0x03);
}

// =============================================================================
// Sequence -- one pattern. EEPROM bytes first (56), then runtime state.
// =============================================================================
struct Sequence {
  // ----- EEPROM-persisted layout (56 bytes, must match OS-303 byte-for-byte) -----
  uint8_t pitch[MAX_STEPS];           // 32 bytes
  uint8_t time_data[MAX_STEPS / 2];   // 16 bytes (4-bit nibbles)
  uint8_t reserved[METADATA_SIZE - 3]; // 5 bytes (reserved[0]=direction, [1..4]=ratchet bits)
  uint8_t transpose     = 0;
  uint8_t engine_select = 0;
  uint8_t length        = 16;
  // ----- Runtime state (NOT persisted) -----
  int pitch_pos = 0;
  int time_pos  = 0;
  bool reset      = true;
  bool first_step = true;
  uint8_t pitch_count_runtime = 0;        // rebuilt on Load via sequence_rebuild_pitch_count
  uint8_t step_lock_ram[MAX_STEPS / 8] = {0,0,0,0}; // 32 bits, RAM-only live lockout

  // ---------------------------------------------------------------------------
  // Reserved-byte accessors. reserved[0] layout: bits[2:0] = direction (0..5),
  // bit[3] = triplet step mode (1 = triplets, 0 = 16ths). Higher bits free.
  // ---------------------------------------------------------------------------
  static constexpr uint8_t TRIPLET_FLAG = 0x08;
  uint8_t get_direction_stored() const { return reserved[0] & 0x07; }
  void    store_direction(uint8_t d) {
    const uint8_t flags = reserved[0] & ~uint8_t(0x07);
    reserved[0] = flags | ((d < DIR_COUNT) ? d : 0);
  }
  bool is_triplet_mode() const { return (reserved[0] & TRIPLET_FLAG) != 0; }
  void set_triplet_mode(bool on) {
    if (on) reserved[0] |= TRIPLET_FLAG;
    else    reserved[0] &= uint8_t(~TRIPLET_FLAG);
  }

  uint8_t get_pitch_count() const { return pitch_count_runtime; }
  void    set_pitch_count(uint8_t n) {
    pitch_count_runtime = (n <= MAX_STEPS) ? n : MAX_STEPS;
  }

  // Ratchet: 1 bit/step in reserved[1..4]. 0 = 1x (normal), 1 = 2x.
  uint8_t get_ratchet_val(uint8_t step) const {
    const uint8_t idx = step & (MAX_STEPS - 1);
    return (reserved[1 + (idx >> 3)] >> (idx & 7)) & 0x01;
  }
  void set_ratchet_val(uint8_t step, uint8_t val) {
    const uint8_t idx = step & (MAX_STEPS - 1);
    const uint8_t mask = uint8_t(1u << (idx & 7));
    if (val & 0x01) reserved[1 + (idx >> 3)] |=  mask;
    else            reserved[1 + (idx >> 3)] &= ~mask;
  }

  // Step lock (RAM-only)
  bool step_locked(uint8_t idx) const {
    idx &= (MAX_STEPS - 1);
    return (step_lock_ram[idx >> 3] >> (idx & 7)) & 1;
  }
  void ToggleStepLock(uint8_t idx) {
    idx &= (MAX_STEPS - 1);
    step_lock_ram[idx >> 3] ^= uint8_t(1u << (idx & 7));
  }

  // ---------------------------------------------------------------------------
  // Time stream accessors
  // ---------------------------------------------------------------------------
  inline uint8_t time(uint8_t idx) const {
    return (time_data[idx >> 1] >> (4 * (idx & 1))) & 0xf;
  }
  const uint8_t get_time() const { return time(time_pos); }

  uint8_t count_notes_to(uint8_t time_idx) const {
    uint8_t cnt = 0;
    const uint8_t lim = (time_idx < length) ? time_idx : length;
    for (uint8_t i = 0; i < lim; ++i)
      if (time(i) == 1) ++cnt;
    return cnt;
  }
  uint8_t pitch_index_for_note(uint8_t time_idx) const {
    return count_notes_to(time_idx);
  }

  // ---------------------------------------------------------------------------
  // Playback pitch lookup
  // ---------------------------------------------------------------------------
  uint8_t get_pitch_packed_or_default() const {
    const uint8_t pc = get_pitch_count();
    if (pc == 0) return PITCH_DEFAULT;
    if (pitch_pos < 0 || pitch_pos >= int(pc)) return PITCH_DEFAULT;
    const uint8_t b = pitch[pitch_pos];
    return (b == PITCH_EMPTY) ? PITCH_DEFAULT : (b & PITCH_PACK_MASK);
  }
  const uint8_t get_pitch() const { return unpack_pitch_linear(get_pitch_packed_or_default()); }
  uint8_t get_pitch_dir(int8_t /*step_dir*/) const { return get_pitch(); }

  const uint8_t get_octave() const  { return (get_pitch_packed_or_default() >> 4) & 0x03; }
  const uint8_t get_semitone() const {
    if (get_pitch_count() == 0) return PITCH_EMPTY;
    return get_pitch_packed_or_default() & 0x0F;
  }
  uint8_t get_note_key_index() const { return get_pitch_packed_or_default() & 0x0F; }
  uint8_t get_octave_button() const  { return (get_pitch_packed_or_default() >> 4) & 0x03; }

  const uint8_t get_accent() const {
    const uint8_t pc = get_pitch_count();
    if (pc == 0) return 0;
    int pp = pitch_pos;
    if (pp < 0 || pp >= int(pc)) return 0;
    const uint8_t b = pitch[pp];
    return (b == PITCH_EMPTY) ? 0 : (b & (1 << 6));
  }
  bool get_slide_at_slot(uint8_t slot) const {
    if (slot >= get_pitch_count()) return false;
    const uint8_t b = pitch[slot];
    if (b == PITCH_EMPTY) return false;
    return (b & 0x80) != 0;
  }
  const bool get_slide() const {
    const uint8_t pc = get_pitch_count();
    if (pc == 0) return false;
    int pp = pitch_pos;
    if (pp < 0 || pp >= int(pc)) return false;
    return get_slide_at_slot(uint8_t(pp));
  }

  uint8_t note_count() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < length; ++i)
      if (time(i) == 1) ++n;
    return n;
  }

  // ---------------------------------------------------------------------------
  // Slide detection (direction-aware)
  // ---------------------------------------------------------------------------
  bool slide_from_prev_dir(uint8_t dir, int8_t step_dir) const {
    if (dir == DIR_RANDOM || dir == DIR_HALF_RAND || dir == DIR_BROWNIAN) return false;
    if (first_step) return false;
    const uint8_t pc = get_pitch_count();
    if (pc == 0) return false;
    if (length <= 1) return false;
    const uint8_t cur_t = time(uint8_t(time_pos));
    if (cur_t == 0) return false;

    int slide_src;
    if (cur_t == 1) {
      const uint8_t nc = note_count();
      if (nc < 2) return false;
      if (step_dir >= 0)
        slide_src = (int(pitch_pos) + int(nc) - 1) % int(nc);
      else
        slide_src = (int(pitch_pos) + 1) % int(nc);
    } else {
      slide_src = pitch_pos;
    }
    if (slide_src < 0 || slide_src >= int(pc)) return false;

    const unsigned delta = (step_dir >= 0) ? (unsigned(length) - 1u) : 1u;
    uint8_t tp = uint8_t((unsigned(time_pos) + delta) % unsigned(length));
    for (uint8_t guard = 0; guard < length; ++guard) {
      const uint8_t tt = time(tp);
      if (tt == 0) return false;
      if (tt == 1) break;
      tp = uint8_t((unsigned(tp) + delta) % unsigned(length));
    }
    const uint8_t b = pitch[slide_src];
    if (b == PITCH_EMPTY) return false;
    return (b & 0x80) != 0;
  }
  bool slide_from_prev() const { return slide_from_prev_dir(uint8_t(DIR_FORWARD), 1); }

  // ---------------------------------------------------------------------------
  // Tie helpers
  // ---------------------------------------------------------------------------
  bool is_tie() const { return (time_pos < length) && (time(uint8_t(time_pos)) == 2); }
  bool next_is_tie() const {
    if (length == 0) return false;
    const uint8_t n = (time_pos + 1) % length;
    return time(n) == 2;
  }
  bool is_tied() const { return next_is_tie(); }
  bool is_tied_dir(uint8_t dir, int8_t next_dir) const {
    if (dir == DIR_RANDOM || dir == DIR_HALF_RAND || dir == DIR_BROWNIAN) return false;
    if (length == 0) return false;
    const uint8_t n = (next_dir >= 0)
      ? uint8_t((unsigned(time_pos) + 1u) % unsigned(length))
      : uint8_t((unsigned(time_pos) + unsigned(length) - 1u) % unsigned(length));
    return time(n) == 2;
  }
  bool tie_chain_ending() const { return is_tie() && !next_is_tie(); }
  bool note_after_tie_run() const {
    if (length <= 1 || first_step || time(uint8_t(time_pos)) != 1) return false;
    uint8_t tp = uint8_t((time_pos + length - 1) % length);
    bool seen_tie = false;
    for (uint8_t g = 0; g < length; ++g) {
      const uint8_t tt = time(tp);
      if (tt == 0) return false;
      if (tt == 2) { seen_tie = true; tp = uint8_t((tp + length - 1) % length); continue; }
      if (tt == 1) return seen_tie;
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // Time-step writing (raw nibble; does NOT touch pitch stream)
  // ---------------------------------------------------------------------------
  void SetTime(uint8_t t) {
    const uint8_t upper = time_pos & 1;
    uint8_t &data = time_data[time_pos >> 1];
    data = (~(0x0f << (4 * upper)) & data) | (t << (4 * upper));
  }

  // ---------------------------------------------------------------------------
  // Pitch-slot mutators
  // ---------------------------------------------------------------------------
  bool edit_slot_index(uint8_t &out_slot) const {
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return false;
    out_slot = uint8_t(pitch_pos);
    return true;
  }
  void init_if_empty() {
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    if (pitch[pitch_pos] == PITCH_EMPTY) pitch[pitch_pos] = PITCH_DEFAULT;
    if (uint8_t(pitch_pos) >= get_pitch_count()) set_pitch_count(uint8_t(pitch_pos + 1));
  }
  bool step_is_empty() const {
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return true;
    if (uint8_t(pitch_pos) >= get_pitch_count()) return true;
    return pitch[pitch_pos] == PITCH_EMPTY;
  }
  bool pitch_is_empty(uint8_t slot) const {
    if (slot >= get_pitch_count()) return true;
    return pitch[slot] == PITCH_EMPTY;
  }

  // SetPitch: caller passes the packed pitch in `p` (low 6 bits used: semi | oct<<4)
  // and the flag bits in `flags` (top 2 bits used: accent | slide).
  void SetPitch(uint8_t p, uint8_t flags) {
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    pitch[pitch_pos] = (p & PITCH_PACK_MASK) | (flags & 0xC0);
    if (uint8_t(pitch_pos) >= get_pitch_count()) set_pitch_count(uint8_t(pitch_pos + 1));
  }
  void SetPitchSemitone(uint8_t semi) {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    const uint8_t oct = (pitch[pitch_pos] >> 4) & 0x03;
    pitch[pitch_pos] = pack_pitch(semi, oct) | (pitch[pitch_pos] & 0xC0);
  }
  void nudge_octave_buttons(int dir) {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    const uint8_t semi = pitch[pitch_pos] & 0x0F;
    int o = int((pitch[pitch_pos] >> 4) & 0x03) + dir;
    CONSTRAIN(o, 0, 3);
    pitch[pitch_pos] = pack_pitch(semi, uint8_t(o)) | (pitch[pitch_pos] & 0xC0);
  }
  void ToggleSlide() {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    pitch[pitch_pos] ^= (1 << 7);
  }
  void ToggleAccent() {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    pitch[pitch_pos] ^= (1 << 6);
  }
  void SetSlide(bool on) {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    pitch[pitch_pos] = (pitch[pitch_pos] & ~(1 << 7)) | (on << 7);
  }
  void SetAccent(bool on) {
    init_if_empty();
    if (pitch_pos < 0 || pitch_pos >= MAX_STEPS) return;
    pitch[pitch_pos] = (pitch[pitch_pos] & ~(1 << 6)) | (on << 6);
  }

  // ---------------------------------------------------------------------------
  // Length
  // ---------------------------------------------------------------------------
  void SetLength(uint8_t len, uint8_t max_len = MAX_STEPS) {
    length = constrain(len, 1, max_len);
  }
  bool BumpLength(uint8_t max_len = MAX_STEPS) {
    if (++length == max_len) return false;
    return true;
  }

  // ---------------------------------------------------------------------------
  // Reset / Clear
  // ---------------------------------------------------------------------------
  void Reset() {
    pitch_pos = 0;
    time_pos = 0;
    reset = true;
    first_step = true;
  }
  void Clear() {
    for (uint8_t i = 0; i < MAX_STEPS; ++i) {
      pitch[i] = PITCH_EMPTY;
      if ((i & 1) == 0) time_data[i >> 1] = 0;
    }
    for (uint8_t i = 0; i < (METADATA_SIZE - 3); ++i)
      reserved[i] = 0;
    for (uint8_t i = 0; i < (MAX_STEPS / 8); ++i)
      step_lock_ram[i] = 0;
    transpose = 0;
    engine_select = 0;
    set_pitch_count(0);
    length = 8;
  }

  // ---------------------------------------------------------------------------
  // Time-step navigation helpers
  // ---------------------------------------------------------------------------
  uint8_t first_note_idx() const {
    for (uint8_t j = 0; j < length; ++j)
      if (time(j) == 1) return j;
    return 0;
  }
  void ensure_pitch_edit_entry() {
    if (reset) {
      reset = false;
      pitch_pos = 0;
      sync_time_pos_to_pitch_pos();
    }
  }
  void ensure_pitch_write_entry() { ensure_pitch_edit_entry(); }
  void sync_time_pos_to_pitch_pos() {
    if (length == 0) { time_pos = 0; return; }
    const uint8_t want = uint8_t(pitch_pos < 0 ? 0 : pitch_pos);
    uint8_t cur = 0;
    for (uint8_t i = 0; i < length; ++i) {
      if (time(i) == 1) {
        if (cur == want) { time_pos = int(i); return; }
        ++cur;
      }
    }
    time_pos = int(first_note_idx());
  }
  void advance_pitch_to_next_note() {
    first_step = false;
    if (length == 0) return;
    ++pitch_pos;
    if (pitch_pos >= int(length)) pitch_pos = 0;
    sync_time_pos_to_pitch_pos();
  }

  // ---------------------------------------------------------------------------
  // Playback advance
  // ---------------------------------------------------------------------------
  bool Advance() {
    if (reset) {
      reset = false;
      time_pos = 0;
      pitch_pos = 0;
      return time(0) != 0;
    }
    if (first_step && time(uint8_t(time_pos)) == 1) {
      first_step = false;
    }
    const uint8_t prev_pos = uint8_t(time_pos);
    ++time_pos %= length;
    // Wrap-back to 0 from a non-zero position completes the first pass even
    // if no NOTE was hit (cleared / all-rest patterns). Without this, the
    // engine wrap detection (`!first_step && time_pos==0`) never fires and
    // queued pattern switches stick.
    if (first_step && prev_pos != 0 && time_pos == 0) {
      first_step = false;
    }
    if (time(uint8_t(time_pos)) == 1) {
      const uint8_t pc = get_pitch_count();
      if (pc > 0 && !first_step) {
        pitch_pos = int(pitch_index_for_note(uint8_t(time_pos)));
      }
    }
    return time(uint8_t(time_pos)) != 0;
  }

  bool AdvanceDirectional(uint8_t dir, int8_t &pp_dir) {
    if (reset) {
      reset = false;
      time_pos = 0;
      pitch_pos = 0;
      return time(0) != 0;
    }
    if (first_step && time(uint8_t(time_pos)) == 1) {
      first_step = false;
    }
    const uint8_t prev_pos = uint8_t(time_pos);
    switch (dir) {
    case DIR_REVERSE:
      if (time_pos <= 0) time_pos = int(length) - 1;
      else               --time_pos;
      break;
    case DIR_PINGPONG: {
      time_pos += pp_dir;
      if (time_pos >= int(length)) {
        pp_dir = -1;
        time_pos = int(length) - 1;
      } else if (time_pos < 0) {
        pp_dir = 1;
        time_pos = 0;
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
      pp_dir = bdir;
      time_pos = int((unsigned(length) + unsigned(time_pos) + unsigned(bdir)) % unsigned(length));
      break;
    }
    default:
      ++time_pos %= length;
      break;
    }
    // Wrap-back to 0 from non-zero clears first_step (matches forward Advance).
    if (first_step && prev_pos != 0 && time_pos == 0) {
      first_step = false;
    }
    if (time(uint8_t(time_pos)) == 1) {
      const uint8_t pc = get_pitch_count();
      if (pc > 0 && !first_step) {
        pitch_pos = int(pitch_index_for_note(uint8_t(time_pos)));
      }
    }
    return time(uint8_t(time_pos)) != 0;
  }

  void AdvancePitch() { advance_pitch_to_next_note(); }

  bool StepBack() {
    if (reset || (time_pos == 0)) return false;
    --time_pos;
    if (time(uint8_t(time_pos)) == 1)
      pitch_pos = int(pitch_index_for_note(uint8_t(time_pos)));
    return true;
  }
};

// =============================================================================
// Free helpers
// =============================================================================
inline void sequence_set_time_at(Sequence &s, uint8_t idx, uint8_t t) {
  idx &= uint8_t(MAX_STEPS - 1);
  const uint8_t upper = idx & 1u;
  uint8_t &cell = s.time_data[idx >> 1u];
  cell = uint8_t((cell & ~(0x0fu << (4u * upper))) | ((t & 0x0fu) << (4u * upper)));
}

inline void sequence_rebuild_pitch_count(Sequence &s) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < s.length; ++i)
    if (s.time(i) == 1) ++n;
  s.set_pitch_count(n);
}

inline void sequence_ensure_pitch_for_notes(Sequence &s) {
  const uint8_t nc = s.note_count();
  uint8_t pc = s.get_pitch_count();
  while (pc < nc && pc < MAX_STEPS) {
    s.pitch[pc] = PITCH_DEFAULT;
    ++pc;
  }
  if (pc != s.get_pitch_count()) s.set_pitch_count(pc);
}

inline void sequence_write_time_with_pitch_sync(Sequence &s, uint8_t idx, uint8_t new_t) {
  idx &= uint8_t(MAX_STEPS - 1);
  if (idx >= s.length) return;
  const uint8_t old_t = s.time(idx);
  if (old_t == new_t) return;
  sequence_set_time_at(s, idx, new_t);
  sequence_ensure_pitch_for_notes(s);
}

inline void sequence_pack_per_time(const Sequence &s, uint8_t *out) {
  const uint8_t pc = s.get_pitch_count();
  uint8_t k = 0;
  for (uint8_t i = 0; i < s.length; ++i) {
    if (s.time(i) == 1 && k < pc) out[i] = s.pitch[k++];
    else                          out[i] = PITCH_EMPTY;
  }
}

inline void sequence_unpack_per_time(Sequence &s, const uint8_t *in) {
  uint8_t k = 0;
  for (uint8_t i = 0; i < s.length; ++i) {
    if (s.time(i) == 1) {
      const uint8_t b = in[i];
      s.pitch[k++] = (b == PITCH_EMPTY) ? PITCH_DEFAULT : b;
    }
  }
  for (uint8_t i = k; i < MAX_STEPS; ++i) s.pitch[i] = PITCH_EMPTY;
  s.set_pitch_count(k);
}

inline void normalize_pattern_times_only(Sequence &s) {
  const uint8_t L = s.length;
  if (L < 1) return;
  bool all_tie = true;
  for (uint8_t i = 0; i < L; ++i) {
    if (s.time(i) != 2) { all_tie = false; break; }
  }
  if (all_tie) sequence_set_time_at(s, 0, 1);
  if (s.time(0) == 2) sequence_set_time_at(s, 0, 1);
  for (uint8_t i = 0; i < L; ++i) {
    const uint8_t nxt = uint8_t((unsigned(i) + 1u) % unsigned(L));
    if (s.time(i) == 0 && s.time(nxt) == 2) sequence_set_time_at(s, i, 1);
  }
}

inline void normalize_pattern_times(Sequence &s) {
  const uint8_t L = s.length;
  if (L < 1) return;
  bool all_tie = true;
  for (uint8_t i = 0; i < L; ++i) {
    if (s.time(i) != 2) { all_tie = false; break; }
  }
  if (all_tie) sequence_write_time_with_pitch_sync(s, 0, 1);
  if (s.time(0) == 2) sequence_write_time_with_pitch_sync(s, 0, 1);
  for (uint8_t i = 0; i < L; ++i) {
    const uint8_t nxt = uint8_t((unsigned(i) + 1u) % unsigned(L));
    if (s.time(i) == 0 && s.time(nxt) == 2) {
      sequence_write_time_with_pitch_sync(s, i, 1);
    }
  }
}

// Helper to roll a random pitch byte (semi 0..12, oct 0..3, no flags).
static inline uint8_t fast_rand_pitch_byte() {
  const uint8_t semi = fast_rand(13);  // 0..12 inclusive
  const uint8_t oct  = fast_rand(4);   // 0..3
  return pack_pitch(semi, oct);
}
