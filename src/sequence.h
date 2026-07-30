// Copyright (c) 2026, Nicholas J. Michalek
//
// sequence.h -- TB-303 pattern data model (two-stream: pitch and time are
// independent; the K-th NOTE event in time order plays pitch[K]).
//
// Persisted pattern layout (PATTERN_SIZE = 52 bytes at MAX_STEPS = 32,
// serialized by persistent_settings.h into the flash block store):
//   pitch[32]            -- 8 bits: bits[3:0]=semitone (0..12, 12=high-C button)
//                                   bits[5:4]=octave (0..3)
//                                   bit[6]=accent, bit[7]=slide
//                          NOTE-event-indexed: pitch[i] = i-th NOTE in time order.
//                          PITCH_EMPTY (0xFF) marks unwritten slots.
//   time_data[8]         -- 2-bit cells per time step (0=REST, 1=NOTE, 2=TIE),
//                          4 steps packed per byte.
//   reserved[9]          -- reserved[0] = direction (bits[2:0]) + triplet flag (bit 3)
//                           reserved[1] = scale mask low 8 bits
//                           reserved[2] = scale enabled (bit 7) + mask high 4 bits
//                           reserved[3..8] = free padding.
//   transpose            -- per-pattern transpose, signed int8 in the byte (-24..+24).
//   engine_select        -- unused, kept 0.
//   length               -- 1..32 (triplet mode caps at 24).
//
// Runtime state (NOT persisted, lives after the persisted block):
//   pitch_pos, time_pos, reset, first_step, pitch_count_runtime,
//   step_lock_ram (64 bits, RAM-only live-write lockout).

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

// 32 steps per section. A linked A/B pair (spec 1-a) is one 64-step pattern,
// which is the user-facing "pattern" on this build. 32 (not 64) is what makes
// worst-case persistence GUARANTEED on the internal-flash combined arena: all
// 192 patterns + poly + probability + tracks + settings = 107 of 110 record
// pages (see flash_persist.h). At 64 steps the same set needs 127+ pages and
// ~33 KB of payload against 26.8 KB physical -- it cannot fit.
static constexpr int MAX_STEPS = 32;
// Triplet mode replaces 16-step pages with 12-step pages (spec 7): the same
// pages, so the length ceiling is 2 * 12 = 24 rather than 2 * 16 = 32.
static constexpr int STEPS_PER_PAGE         = 16;
static constexpr int TRIPLET_STEPS_PER_PAGE = 12;
static constexpr int TRIPLET_MAX_STEPS      = (MAX_STEPS / STEPS_PER_PAGE) * TRIPLET_STEPS_PER_PAGE;
static constexpr int NUM_PATTERNS = 16; // patterns per bank
// 4 pattern groups (I-IV), matching the panel legend. The group knob has 7
// physical detents; they pair into 4 logical groups (1-2 = I, 3-4 = II,
// 5-6 = III, 7 = IV), which is what the stock firmware has always done. Each
// group holds 8 patterns, each with an A and a B section -> 16 slots per
// group, 64 slots total.
// NOTE: the FRAM build is where 7 groups / 112 slots is planned -- see the
// "Planned for the FRAM build" note in .claude/CLAUDE.md. It does not fit the
// internal-flash arena at 64 steps.
static constexpr int NUM_BANKS = 4;     // banks 0..3 (was NUM_GROUPS)
static constexpr int NUM_GROUPS = NUM_BANKS; // back-compat alias
// Each of the 64 slots (bank*16 + pat) holds 3 variations. Variation 1 is the
// 303's own voice by default; the per-slot CV variation selects which one the
// CV/gate plays. 64 slots x 3 = 192 patterns total.
static constexpr int NUM_VARIATIONS = 3;
static constexpr int NUM_SLOTS = NUM_PATTERNS * NUM_BANKS; // 64
// reserved[9] (direction + unused padding) + transpose + engine_select + length
static constexpr int METADATA_SIZE = 12;

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

// -----------------------------------------------------------------------------
// Per-step characteristic probability (variation 1 / CV voice only). THREE
// bytes per step, grid-indexed (NOT pitch-stream indexed), so a step can arm
// accent, slide, a down transpose, AND an up transpose all at once, each with
// its own probability level 1..13 (13 = 100%). Down (-12) and up (+12 or +24)
// roll independently; if BOTH pass on a step the firmware flips a coin to pick
// which shift applies. All bytes 7-bit safe for SysEx. All-zero = unarmed.
//   byte0: bits[3:0] = accent level (0 = off, 1..13)
//          bits[7:4] = slide  level (0 = off, 1..13)
//   byte1: bits[3:0] = down transpose level (-12; 0 = off)
//          bits[7:4] = up   transpose level (0 = off)
//   byte2: bit[0]    = up-is-double (up delta = +24 when set, else +12)
// When accent/slide is armed the step's stored accent/slide flag is ignored and
// the output is purely probability-driven. Transpose shifts the step's
// programmed linear pitch before scale quantize.
// -----------------------------------------------------------------------------
static constexpr uint8_t PROB_LEVEL_MAX = 13; // level 13 always passes
static inline uint8_t prob_accent_level(uint8_t b0) { return b0 & 0x0F; }
static inline uint8_t prob_slide_level(uint8_t b0)  { return (b0 >> 4) & 0x0F; }
static inline uint8_t prob_down_level(uint8_t b1)   { return b1 & 0x0F; }
static inline uint8_t prob_up_level(uint8_t b1)     { return (b1 >> 4) & 0x0F; }
static inline bool    prob_up_double(uint8_t b2)    { return (b2 & 0x01) != 0; }
static inline uint8_t prob_pack_ac_sl(uint8_t acc_lvl, uint8_t sld_lvl) {
  return uint8_t((acc_lvl & 0x0F) | ((sld_lvl & 0x0F) << 4));
}
static inline uint8_t prob_pack_du(uint8_t down_lvl, uint8_t up_lvl) {
  return uint8_t((down_lvl & 0x0F) | ((up_lvl & 0x0F) << 4));
}
// Graceful octave degrade: the double-up shift falls back to single up, single
// shifts fall back to unchanged. Playable range is linear -12..52: the CV DAC
// spans 64 codes (code = pitch + transpose - 1, "none" = +12), giving an
// octave of headroom below bottom C and 4 semitones above double-up high C
// (the top E, linear 52 = DAC code 63).
static inline int8_t prob_degrade_transpose(uint8_t base, int8_t t) {
  int v = int(base) + t;
  if (v >= -12 && v <= 52) return t;
  if (t == 24) {
    t = 12;
    v = int(base) + t;
    if (v >= -12 && v <= 52) return t;
  }
  return 0;
}

// =============================================================================
// Sequence -- one pattern. Persisted bytes first (PATTERN_SIZE), then
// runtime state.
// =============================================================================
struct Sequence {
  // ----- Persisted layout (92 bytes, serialized as one flash record) -----
  uint8_t pitch[MAX_STEPS];           // 64 bytes
  uint8_t time_data[MAX_STEPS / 4];   // 16 bytes (2-bit cells, 4 steps/byte)
  uint8_t reserved[METADATA_SIZE - 3]; // 9 bytes (reserved[0]=direction, [1..2]=scale, [3..8]=free)
  uint8_t transpose     = 0;
  uint8_t engine_select = 0;
  uint8_t length        = 16;
  // ----- Runtime state (NOT persisted). Playheads hold 0..MAX_STEPS-1 (63);
  // int8_t is wide enough and saves 2 bytes on each of the 18 resident copies.
  int8_t pitch_pos = 0;
  int8_t time_pos  = 0;
  bool reset      = true;
  bool first_step = true;
  uint8_t pitch_count_runtime = 0;        // rebuilt on Load via sequence_rebuild_pitch_count
  uint8_t step_lock_ram[MAX_STEPS / 8] = {}; // 64 bits, RAM-only live lockout

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

  // ---------------------------------------------------------------------------
  // A/B section link (spec 1-a). Slots p and p+8 of a bank are the A and B
  // SECTIONS of one pattern; linking them makes the pair play and edit in
  // serial as a single pattern of up to 128 steps. The flag lives in
  // reserved[3] bit 0 of the A-SECTION pattern, so it persists with the
  // pattern itself: reselecting the pattern later restores "both sections
  // selected" without the performer having to remember it.
  // ---------------------------------------------------------------------------
  static constexpr uint8_t AB_LINK_FLAG = 0x01;
  bool ab_linked() const { return (reserved[3] & AB_LINK_FLAG) != 0; }
  void set_ab_linked(bool on) {
    if (on) reserved[3] |= AB_LINK_FLAG;
    else    reserved[3] &= uint8_t(~AB_LINK_FLAG);
  }

  // ---------------------------------------------------------------------------
  // Per-pattern scale. reserved[1] = mask low 8 bits, reserved[2]: bit7 =
  // enabled, bits[3:0] = mask high 4 bits. A raw mask of 0 is the "unset"
  // sentinel and reads back as all 12 classes allowed, so cleared/legacy
  // patterns default to all-notes / disabled. Non-destructive: scale quantizes
  // pitch only at output and constrains random generation; stored pitch[] is
  // never rewritten.
  // ---------------------------------------------------------------------------
  uint16_t scale_raw() const {
    return uint16_t(reserved[1] | (uint16_t(reserved[2] & 0x0F) << 8));
  }
  uint16_t scale_mask() const { const uint16_t m = scale_raw(); return m ? m : 0x0FFF; }
  bool scale_enabled() const { return (reserved[2] & 0x80) != 0; }
  void set_scale_mask(uint16_t m) {
    m &= 0x0FFF;
    reserved[1] = uint8_t(m & 0xFF);
    reserved[2] = uint8_t((reserved[2] & 0x80) | ((m >> 8) & 0x0F));
  }
  void set_scale_enabled(bool on) {
    if (on) reserved[2] |= 0x80; else reserved[2] &= uint8_t(0x7F);
  }
  void toggle_scale_class(uint8_t cls) {
    set_scale_mask(scale_mask() ^ uint16_t(1u << (cls % 12)));
  }
  bool scale_allows(uint8_t cls) const { return (scale_mask() >> (cls % 12)) & 1; }

  /// Map a linear semitone 0..48 to the nearest allowed note class (ties to the
  /// lower note, clamped 0..48). Identity when the scale is disabled.
  uint8_t scale_quantize_linear(uint8_t lin) const {
    if (!scale_enabled()) return lin;
    const uint16_t mask = scale_mask();
    if ((mask >> (lin % 12)) & 1) return lin;
    for (int d = 1; d <= 12; ++d) {
      const int lo = int(lin) - d, hi = int(lin) + d;
      if (lo >= 0  && ((mask >> (lo % 12)) & 1)) return uint8_t(lo);
      if (hi <= 48 && ((mask >> (hi % 12)) & 1)) return uint8_t(hi);
    }
    return lin;
  }

  /// Quantize the 6-bit packed pitch field (caller preserves the 0xC0 flag bits).
  uint8_t scale_apply_packed(uint8_t packed6) const {
    if (!scale_enabled()) return packed6;
    const uint8_t lin = scale_quantize_linear(unpack_pitch_linear(packed6));
    const uint8_t oct = lin / 12;
    return pack_pitch(uint8_t(lin - oct * 12), oct);
  }

  uint8_t get_pitch_count() const { return pitch_count_runtime; }
  void    set_pitch_count(uint8_t n) {
    pitch_count_runtime = (n <= MAX_STEPS) ? n : MAX_STEPS;
  }

  // Step lock (RAM-only)
  bool step_locked(uint8_t idx) const {
    idx &= (MAX_STEPS - 1);
    return (step_lock_ram[idx >> 3] >> (idx & 7)) & 1;
  }

  // ---------------------------------------------------------------------------
  // Time stream accessors
  // ---------------------------------------------------------------------------
  inline uint8_t time(uint8_t idx) const {
    return (time_data[idx >> 2] >> (2 * (idx & 3))) & 0x3;
  }
  const uint8_t get_time() const { return time(time_pos); }

  // Number of NOTE events strictly before `time_idx` = the pitch-stream slot
  // the NOTE at that step plays (the two-stream mapping).
  uint8_t pitch_index_for_note(uint8_t time_idx) const {
    uint8_t cnt = 0;
    const uint8_t lim = (time_idx < length) ? time_idx : length;
    for (uint8_t i = 0; i < lim; ++i)
      if (time(i) == 1) ++cnt;
    return cnt;
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

  // ---------------------------------------------------------------------------
  // Tie helpers
  // ---------------------------------------------------------------------------
  bool is_tie() const { return (time_pos < length) && (time(uint8_t(time_pos)) == 2); }
  bool is_tied_dir(uint8_t dir, int8_t next_dir) const {
    if (dir == DIR_RANDOM || dir == DIR_HALF_RAND || dir == DIR_BROWNIAN) return false;
    if (length == 0) return false;
    const uint8_t n = (next_dir >= 0)
      ? uint8_t((unsigned(time_pos) + 1u) % unsigned(length))
      : uint8_t((unsigned(time_pos) + unsigned(length) - 1u) % unsigned(length));
    return time(n) == 2;
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

  // ---------------------------------------------------------------------------
  // Length
  // ---------------------------------------------------------------------------
  void SetLength(uint8_t len, uint8_t max_len = MAX_STEPS) {
    length = constrain(len, 1, max_len);
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
    for (uint8_t i = 0; i < MAX_STEPS; ++i) pitch[i] = PITCH_EMPTY;
    for (uint8_t i = 0; i < (MAX_STEPS / 4); ++i) time_data[i] = 0;
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
      time_pos = int(fast_rand(uint8_t(length)));
      break;
    case DIR_HALF_RAND:
      if (fast_rand(2)) time_pos = int(fast_rand(uint8_t(length)));
      else { ++time_pos %= length; }
      break;
    case DIR_BROWNIAN: {
      int8_t bdir = fast_rand(2) ? 1 : -1;
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
  const uint8_t shift = 2u * (idx & 3u);
  uint8_t &cell = s.time_data[idx >> 2u];
  cell = uint8_t((cell & ~(0x03u << shift)) | ((t & 0x03u) << shift));
}

inline void sequence_rebuild_pitch_count(Sequence &s) {
  s.set_pitch_count(s.note_count());
}

// Extend the pitch stream so every NOTE event inside `length` has a pitch.
// Slots that still hold a real pitch are LEFT ALONE: shortening a pattern drops
// pitch_count but keeps pitch[] intact, so re-lengthening replays the original
// pitches instead of a run of default Cs (spec 5, non-volatile length changes).
// Only genuinely unwritten slots get PITCH_DEFAULT.
inline void sequence_ensure_pitch_for_notes(Sequence &s) {
  const uint8_t nc = s.note_count();
  uint8_t pc = s.get_pitch_count();
  while (pc < nc && pc < MAX_STEPS) {
    if (s.pitch[pc] == PITCH_EMPTY) s.pitch[pc] = PITCH_DEFAULT;
    ++pc;
  }
  if (pc != s.get_pitch_count()) s.set_pitch_count(pc);
}

// Spec 3-a: sequential PITCH MODE entry SIZES the pattern -- "the PATTERN
// length will automatically be set by the number of NOTES entered". While the
// cursor sits one past the last NOTE event it is appending, so each written
// pitch also claims the next time step as a NOTE and pushes the last step out.
// Once the cursor is inside existing content it overwrites instead (spec 4-d),
// and this returns false. Returns true when a step was appended.
inline bool sequence_append_note_step(Sequence &s, uint8_t max_len) {
  const uint8_t nc = s.note_count();
  if (s.pitch_pos < 0 || uint8_t(s.pitch_pos) != nc) return false;
  if (nc >= max_len) return false;
  // The pattern is still being SIZED only while everything after the cursor is
  // empty: then the last step tracks the note count exactly (a freshly cleared
  // pattern starts at length 8, and must end up at 1 after one note). If real
  // time data sits past the cursor the pattern is being edited, not written,
  // so the last step only ever grows.
  bool sizing = true;
  for (uint8_t i = nc; i < s.length; ++i)
    if (s.time(i) != 0) { sizing = false; break; }
  if (sizing || s.length < uint8_t(nc + 1)) s.length = uint8_t(nc + 1);
  sequence_set_time_at(s, nc, 1);
  s.time_pos  = int(nc);       // cursor rests on the step just written
  s.pitch_pos = int(nc) + 1;   // next press appends the one after it
  s.first_step = false;
  return true;
}

// Spec 5-b: FUNCTION + DOWN halves the length. Integer division gives the
// spec's odd ladder (31 > 15 > 7 > 3 > 1). Purely a last-step move: no time or
// pitch data is destroyed, so FUNCTION + UP brings it all back.
inline uint8_t sequence_halve_length(Sequence &s) {
  s.length = (s.length > 1) ? uint8_t(s.length / 2) : uint8_t(1);
  sequence_rebuild_pitch_count(s);
  return s.length;
}

// Spec 5-b/5-c: FUNCTION + UP doubles the length, clamped to `cap`. If the
// revealed tail holds no NOTES (nothing to restore from an earlier halve), the
// first half is COPIED into it -- the spec's "16-step pattern doubled to 32
// copies steps 1-16 into 17-32". Otherwise the retained tail simply reappears.
inline uint8_t sequence_double_length(Sequence &s, uint8_t cap) {
  const uint8_t len = s.length;
  uint8_t nlen = (len > (cap / 2)) ? cap : uint8_t(len * 2);
  if (nlen <= len) return len;
  bool tail_empty = true;
  for (uint8_t i = len; i < nlen; ++i)
    if (s.time(i) != 0) { tail_empty = false; break; }
  if (tail_empty) {
    // Mirror the pitch stream first: the copied half's NOTE events follow the
    // original's in stream order, so pitches [0, nc) repeat at [nc, 2nc).
    const uint8_t nc = s.note_count();
    for (uint8_t j = 0; j < nc && uint8_t(nc + j) < MAX_STEPS; ++j)
      s.pitch[uint8_t(nc + j)] = s.pitch[j];
    for (uint8_t i = len; i < nlen; ++i)
      sequence_set_time_at(s, i, s.time(uint8_t(i - len)));
  }
  s.length = nlen;
  sequence_rebuild_pitch_count(s);
  sequence_ensure_pitch_for_notes(s);
  return nlen;
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
  // A TIE after a REST is left alone: playback holds the gate only if a NOTE
  // opened it, so an orphaned tie is just silence. Promoting the rest back to
  // a NOTE (as this used to do) materialized a phantom note from the queued
  // pitch stream whenever the user rested a note that had a tie behind it.
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
  // Rest-then-tie stays as written; see normalize_pattern_times_only.
}

// Weighted octave: DOWN 25% / CENTRE 50% / UP 25%. Top register excluded.
static inline uint8_t fast_rand_octave_weighted() {
  const uint8_t r = fast_rand(20);
  if (r < 5)  return 0;
  if (r < 15) return 1;
  return 2;
}
static inline uint8_t fast_rand_pitch_byte_weighted() {
  return pack_pitch(fast_rand(13), fast_rand_octave_weighted());
}
// Time: NOTE 60% / REST 25% / TIE 15%, with TIE-after-REST and first-step-no-TIE guards.
static inline uint8_t fast_rand_time_weighted(uint8_t prev_t, bool is_first) {
  const uint8_t r = fast_rand(20);
  if (is_first || prev_t == 0) return (r < 12) ? 1 : 0;
  if (r < 12) return 1;
  if (r < 17) return 0;
  return 2;
}
static inline uint8_t fast_rand_accent_weighted()  { return (fast_rand(10) < 3) ? 0x40 : 0; }
static inline uint8_t fast_rand_slide_weighted()   { return (fast_rand(10) < 2) ? 0x80 : 0; }
