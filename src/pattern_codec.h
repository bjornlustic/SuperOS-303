#pragma once
// pattern_codec.h -- trimmed pattern storage codec.
//
// A pattern is 92 bytes dense (pitch[64] + time_data[16] + 12 metadata), which
// monopolises a 256-byte flash page even when the pattern holds a handful of
// notes. Stored TRIMMED, the pitch stream keeps only as many entries as are
// actually written, so PAT_PER_BLOCK patterns share one page.
//
// How many pitch bytes are "actually written" is NOT note_count: shortening a
// pattern is non-volatile (spec 5), so pitch[] legitimately holds entries past
// the last step that must come back when the pattern is lengthened again. The
// stored count is therefore 1 + the highest non-empty index, which trims only
// the genuinely untouched tail.
//
// Region layout (PAT_REGION bytes):
//   [0]                 stored pitch count n (0..MAX_STEPS), or 0xFF = dense overflow
//   [1..METADATA_SIZE]  metadata: reserved[9] + transpose + engine + length
//   [PC_TIME_OFF..]     time_data (MAX_STEPS/4)
//   [PC_PITCH_OFF..]    n pitch bytes
// A pattern needing more than PAT_SPARSE_MAX pitch bytes falls back to a dense
// 92-byte block of its own, so the worst case still works.
//
// Include AFTER the FB_* constants, Sequence and the g_flash instance.

// Flat pattern index: variation-major so all three variations of a slot are
// addressable from one id space (var 0/1 = the CV + MIDI pair, var 2 = mono var3).
inline uint16_t pattern_index_of(uint8_t abs_slot, uint8_t var) {
  const uint8_t v = (var >= 2) ? 2 : var;
  return uint16_t(v) * uint16_t(NUM_SLOTS) + abs_slot;
}

static constexpr uint8_t PC_TIME_OFF  = 1 + METADATA_SIZE;               // 13
static constexpr uint8_t PC_PITCH_OFF = uint8_t(PC_TIME_OFF + MAX_STEPS / 4);

inline void pattern_trim_encode(const Sequence &seq, uint8_t *region) {
  uint8_t n = 0;
  for (uint8_t i = MAX_STEPS; i-- > 0;)
    if (seq.pitch[i] != PITCH_EMPTY) { n = uint8_t(i + 1); break; }
  region[0] = n;
  memcpy(region + 1,           seq.reserved,  METADATA_SIZE);
  memcpy(region + PC_TIME_OFF, seq.time_data, MAX_STEPS / 4);
  memcpy(region + PC_PITCH_OFF, seq.pitch,    n);
}

inline void pattern_trim_decode(Sequence &seq, const uint8_t *region) {
  const uint8_t n = region[0];
  memcpy(seq.reserved,  region + 1,           METADATA_SIZE);
  memcpy(seq.time_data, region + PC_TIME_OFF, MAX_STEPS / 4);
  memset(seq.pitch, PITCH_EMPTY, MAX_STEPS);
  memcpy(seq.pitch, region + PC_PITCH_OFF, n);
}

inline void ReadPatternAt(Sequence &seq, uint8_t abs_slot, uint8_t var) {
  const uint16_t pidx = pattern_index_of(abs_slot, var);
  uint8_t blk[FB_PATTERN_BLK_LEN];
  if (g_flash.read(uint16_t(FB_PATTERN_BASE + pidx / PAT_PER_BLOCK), blk,
                   FB_PATTERN_BLK_LEN) != FB_PATTERN_BLK_LEN) {
    clear_pattern_bytes(seq);
    return;
  }
  const uint8_t *r = blk + (pidx % PAT_PER_BLOCK) * PAT_REGION;
  if (r[0] == 0xFF) {                                  // dense overflow block
    uint8_t dense[FB_PATTERN_LEN_ONE];
    if (g_flash.read(uint16_t(FB_PATX_BASE + pidx), dense, FB_PATTERN_LEN_ONE)
        == FB_PATTERN_LEN_ONE)
      deserialize_pattern(seq, dense);
    else
      clear_pattern_bytes(seq);
    return;
  }
  pattern_trim_decode(seq, r);
}

inline void WritePatternAt(const Sequence &seq, uint8_t abs_slot, uint8_t var) {
  const uint16_t pidx = pattern_index_of(abs_slot, var);
  const uint16_t gblk = uint16_t(FB_PATTERN_BASE + pidx / PAT_PER_BLOCK);
  uint8_t blk[FB_PATTERN_BLK_LEN];
  if (g_flash.read(gblk, blk, FB_PATTERN_BLK_LEN) != FB_PATTERN_BLK_LEN)
    memset(blk, 0, FB_PATTERN_BLK_LEN);   // fresh group: neighbours read as cleared
  uint8_t *r = blk + (pidx % PAT_PER_BLOCK) * PAT_REGION;

  uint8_t n = 0;
  for (uint8_t i = MAX_STEPS; i-- > 0;)
    if (seq.pitch[i] != PITCH_EMPTY) { n = uint8_t(i + 1); break; }

  memset(r, 0, PAT_REGION);
  if (n > PAT_SPARSE_MAX) {
    r[0] = 0xFF;                          // region points at the dense block
    uint8_t dense[FB_PATTERN_LEN_ONE];
    serialize_pattern(seq, dense);
    g_flash.write(uint16_t(FB_PATX_BASE + pidx), dense, FB_PATTERN_LEN_ONE);
  } else {
    pattern_trim_encode(seq, r);
  }
  g_flash.write(gblk, blk, FB_PATTERN_BLK_LEN);
}
