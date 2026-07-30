#pragma once
// poly_codec.h -- trimmed variation-3 poly storage codec.
//
// The dense poly blob is 227 bytes, 192 of which are chord notes: 3 bytes for
// each of 64 chord slots whether or not the pattern uses them. A 16-chord
// pattern therefore burned a whole 256-byte page for ~83 bytes of content.
// Stored trimmed, only chord_count chords are written and POLY_PER_BLOCK
// patterns share one page.
//
// Region layout (POLY_REGION bytes), offsets derived from POLY_STEPS:
//   [0]                 chord count c (0..POLY_CHORDS), or 0xFF = dense overflow
//   [PT_TIME_OFF..]     time_data    [PT_ACC_OFF..] acc_bits   [PT_SLD_OFF..] sld_bits
//   [PT_DIR_OFF]        direction    [PT_LEN_OFF]   length
//   [POLY_TRIM_FIXED..] 3 bytes per stored chord
// A pattern with more than POLY_SPARSE_MAX chords falls back to a dense
// POLY_BLOB_SIZE block of its own, so the worst case still works.
//
// Include AFTER the FB_* constants, PolyVoice and the g_flash instance.

static constexpr uint8_t PT_TIME_OFF     = 1;
static constexpr uint8_t PT_ACC_OFF      = uint8_t(PT_TIME_OFF + POLY_STEPS / 4);
static constexpr uint8_t PT_SLD_OFF      = uint8_t(PT_ACC_OFF + POLY_CHORDS / 8);
static constexpr uint8_t PT_DIR_OFF      = uint8_t(PT_SLD_OFF + POLY_CHORDS / 8);
static constexpr uint8_t PT_LEN_OFF      = uint8_t(PT_DIR_OFF + 1);
static constexpr uint8_t POLY_TRIM_FIXED = uint8_t(PT_LEN_OFF + 1); // count + streams, before the chords

inline void poly_trim_encode(const PolyVoice &pv, uint8_t *region, uint8_t c) {
  uint8_t dense[POLY_BLOB_SIZE];
  pv.serialize(dense);
  region[0] = c;
  memcpy(region + PT_TIME_OFF, dense + POLY_TIME_OFF, POLY_STEPS / 4);
  memcpy(region + PT_ACC_OFF,  dense + POLY_ACC_OFF,  POLY_CHORDS / 8);
  memcpy(region + PT_SLD_OFF,  dense + POLY_SLD_OFF,  POLY_CHORDS / 8);
  region[PT_DIR_OFF] = dense[POLY_DIR_OFF];
  region[PT_LEN_OFF] = dense[POLY_LEN_OFF];
  memcpy(region + POLY_TRIM_FIXED, dense, size_t(c) * 3);   // chords 0..c-1
}

inline void poly_trim_decode(PolyVoice &pv, const uint8_t *region) {
  const uint8_t c = region[0];
  uint8_t dense[POLY_BLOB_SIZE];
  memset(dense, 0, POLY_BLOB_SIZE);
  memcpy(dense, region + POLY_TRIM_FIXED, size_t(c) * 3);
  // Chords past the stored count read as empty voices, not as zeros (which
  // would decode to a real pitch).
  for (uint8_t i = c; i < POLY_CHORDS; ++i) {
    uint8_t *o = dense + i * 3;
    o[0] = uint8_t(POLY_EMPTY | (POLY_EMPTY << 6));
    o[1] = uint8_t((POLY_EMPTY >> 2) | (POLY_EMPTY << 4));
    o[2] = uint8_t((POLY_EMPTY >> 4) | (POLY_EMPTY << 2));
  }
  memcpy(dense + POLY_TIME_OFF, region + PT_TIME_OFF, POLY_STEPS / 4);
  memcpy(dense + POLY_ACC_OFF,  region + PT_ACC_OFF,  POLY_CHORDS / 8);
  memcpy(dense + POLY_SLD_OFF,  region + PT_SLD_OFF,  POLY_CHORDS / 8);
  dense[POLY_CNT_OFF] = c;
  dense[POLY_DIR_OFF] = region[PT_DIR_OFF];
  dense[POLY_LEN_OFF] = region[PT_LEN_OFF];
  pv.deserialize(dense);
}

inline void ReadPolyAt(PolyVoice &pv, uint8_t abs_slot) {
  uint8_t blk[FB_POLY_BLK_LEN];
  if (g_flash.read(uint16_t(FB_POLY_BASE + abs_slot / POLY_PER_BLOCK), blk,
                   FB_POLY_BLK_LEN) != FB_POLY_BLK_LEN) {
    pv.Clear();
    return;
  }
  const uint8_t *r = blk + (abs_slot % POLY_PER_BLOCK) * POLY_REGION;
  if (r[0] == 0xFF) {                              // dense overflow block
    uint8_t dense[POLY_BLOB_SIZE];
    if (g_flash.read(uint16_t(FB_POLYX_BASE + abs_slot), dense, POLY_BLOB_SIZE)
        == POLY_BLOB_SIZE)
      pv.deserialize(dense);
    else
      pv.Clear();
    return;
  }
  poly_trim_decode(pv, r);
}

inline void WritePolyAt(const PolyVoice &pv, uint8_t abs_slot) {
  const uint16_t gblk = uint16_t(FB_POLY_BASE + abs_slot / POLY_PER_BLOCK);
  uint8_t blk[FB_POLY_BLK_LEN];
  if (g_flash.read(gblk, blk, FB_POLY_BLK_LEN) != FB_POLY_BLK_LEN)
    memset(blk, 0, FB_POLY_BLK_LEN);
  uint8_t *r = blk + (abs_slot % POLY_PER_BLOCK) * POLY_REGION;

  // Store every chord up to the highest one that actually holds a voice, so a
  // stale chord_count can never truncate real content.
  uint8_t c = pv.chord_count;
  if (c > POLY_CHORDS) c = POLY_CHORDS;
  for (uint8_t i = POLY_CHORDS; i-- > c;) {
    const uint8_t *v = pv.chord(i);
    if (v[0] != POLY_EMPTY || v[1] != POLY_EMPTY ||
        v[2] != POLY_EMPTY || v[3] != POLY_EMPTY) { c = uint8_t(i + 1); break; }
  }

  memset(r, 0, POLY_REGION);
  if (c > POLY_SPARSE_MAX) {
    r[0] = 0xFF;
    uint8_t dense[POLY_BLOB_SIZE];
    pv.serialize(dense);
    g_flash.write(uint16_t(FB_POLYX_BASE + abs_slot), dense, POLY_BLOB_SIZE);
  } else {
    poly_trim_encode(pv, r, c);
  }
  g_flash.write(gblk, blk, FB_POLY_BLK_LEN);
}
