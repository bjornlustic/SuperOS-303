// flash_persist.h -- device glue between the persistence layer and the flash
// block store (flash_eeprom.h). Provides the page program/read hooks, the
// single FlashEeprom instance, and the logical block-id map.
#pragma once
#include <Arduino.h>
#include "flash_store.h"   // flash_write_page / pgm_read_byte_far / arena bounds
#include "flash_eeprom.h"

// The id space below addresses every slot on the device (4 groups x 16 = 64) in
// all three variations. It is not a storage promise: records exist only for
// what is actually written, and the live set must fit the arena
// (FE_RECORD_PAGES). Past that, write() refuses NEW blocks while existing ones
// can still be updated -- saving stops, nothing stored is lost.
static constexpr uint8_t FB_PATTERN_LEN_ONE = MAX_STEPS + (MAX_STEPS / 4) + METADATA_SIZE; // 92

// Logical block ids (must stay < FE_MAX_BLOCKS). Patterns and probability are
// both stored TRIMMED, several per page; poly, tracks and settings get a page
// each. Pattern ids use a flat variation-major index (var*NUM_SLOTS + slot,
// var 0/1 = the CV+MIDI pair, var 2 = mono var3), so all three variations share
// one id space -- see pattern_codec.h.
//   FB_PATTERN_BASE .. +47             : pattern groups, PAT_PER_BLOCK per block
//   FB_PATX_BASE .. +191               : dense pattern overflow, one per (slot,var)
//   FB_POLY_BASE .. / FB_POLYX_BASE ..  : poly var3 groups + dense overflow
//   FB_TRACK_BASE .. +3                : track pairs (2 tracks per record)
//   FB_SETTINGS                        : settings
//   FB_PROB_BASE .. / FB_PROBX_BASE .. : probability groups + dense overflow
//
// At MAX_STEPS = 32 the WORST CASE is guaranteed to fit the combined arena:
// a full pattern trims to 21 + 32 = 53 B, so 4 share a 240 B record and the
// sparse ceiling (60 - 21 = 39) can never be exceeded -- the dense overflow
// paths are kept but are unreachable. Full budget, everything armed:
// 48 pattern + 32 poly + 22 prob + 4 track + 1 settings = 107 of 110 pages.
static constexpr uint8_t  PAT_REGION     = 60;
static constexpr uint8_t  PAT_PER_BLOCK  = 4;
static constexpr uint8_t  PAT_FIXED      = uint8_t(1 + METADATA_SIZE + MAX_STEPS / 4); // 21
static constexpr uint8_t  PAT_SPARSE_MAX = PAT_REGION - PAT_FIXED;  // 39 >= MAX_STEPS
static_assert(PAT_SPARSE_MAX >= MAX_STEPS, "pattern dense overflow must be unreachable");
static constexpr uint8_t  FB_PATTERN_BLK_LEN = PAT_PER_BLOCK * PAT_REGION;   // 240
static constexpr uint16_t FB_PATTERN_COUNT   = uint16_t(3 * NUM_SLOTS);      // 192
static constexpr uint16_t FB_PATTERN_BASE = 0;
static constexpr uint16_t FB_PATTERN_BLOCKS =
    uint16_t((FB_PATTERN_COUNT + PAT_PER_BLOCK - 1) / PAT_PER_BLOCK);        // 48
static constexpr uint16_t FB_PATX_BASE  = uint16_t(FB_PATTERN_BASE + FB_PATTERN_BLOCKS);
// Poly var3, trimmed to the chords actually used (see poly_codec.h). The trim
// fixed part at 32 steps is 19 B, so a full 32-chord voice is 19 + 96 = 115 B
// and the dense overflow is unreachable (kept for safety).
static constexpr uint8_t  POLY_REGION      = 120;  // 2 per 242-byte record
static constexpr uint8_t  POLY_PER_BLOCK   = 2;
static constexpr uint8_t  POLY_SPARSE_MAX  = (POLY_REGION - 19) / 3;   // 33 >= 32 chords
static constexpr uint8_t  FB_POLY_BLK_LEN  = POLY_PER_BLOCK * POLY_REGION;  // 240
static constexpr uint16_t FB_POLY_BLOCKS   =
    uint16_t((NUM_SLOTS + POLY_PER_BLOCK - 1) / POLY_PER_BLOCK);           // 32
static constexpr uint16_t FB_POLY_BASE  = uint16_t(FB_PATX_BASE + FB_PATTERN_COUNT);
static constexpr uint16_t FB_POLYX_BASE = uint16_t(FB_POLY_BASE + FB_POLY_BLOCKS);
// Tracks are paired 2 per record (2 x 104 = 208 <= 242): 8 tracks -> 4 ids.
static constexpr uint8_t  TRACK_PER_BLOCK = 2;
static constexpr uint16_t FB_TRACK_BASE = uint16_t(FB_POLYX_BASE + NUM_SLOTS);
static constexpr uint16_t FB_TRACK_BLOCKS = uint16_t((8 + TRACK_PER_BLOCK - 1) / TRACK_PER_BLOCK);
static constexpr uint16_t FB_SETTINGS   = uint16_t(FB_TRACK_BASE + FB_TRACK_BLOCKS);
static_assert(FB_PATTERN_BLK_LEN <= FE_MAX_PAYLOAD, "pattern group must fit one record");
static_assert(PAT_FIXED + PAT_SPARSE_MAX <= PAT_REGION, "pattern region too small");

// Step-probability tables (variation 1 only). RAM and SysEx keep 3 bytes/step
// (b0 accent+slide levels, b1 down+up levels, b2 up-double), but STORAGE is
// sparse: a table is almost always a few armed steps in a field of zeros.
// Region layout (PROB_REGION bytes per slot):
//   [0]       armed-step count n, or 0xFF = stored in the dense overflow block
//   [1..8]    armed-step bitmap   [9..16] up-double bitmap
//   [17..]    2 bytes per armed step, ascending: b0, b1
static constexpr uint8_t  PROB_SLOTS_PER_BLOCK = 3;
static constexpr uint8_t  PROB_REGION          = 80;
static constexpr uint8_t  PROB_FIXED           = uint8_t(1 + 2 * (MAX_STEPS / 8)); // 9
static constexpr uint8_t  PROB_SPARSE_MAX      = MAX_STEPS;  // 9 + 2*32 = 73 <= 80: overflow unreachable
static constexpr uint8_t  FB_PROB_BLK_LEN      = PROB_SLOTS_PER_BLOCK * PROB_REGION; // 240
static constexpr uint16_t FB_PROB_BLOCKS       =
    uint16_t((NUM_SLOTS + PROB_SLOTS_PER_BLOCK - 1) / PROB_SLOTS_PER_BLOCK);
static constexpr uint16_t FB_PROB_BASE  = uint16_t(FB_SETTINGS + 1);
static constexpr uint16_t FB_PROBX_BASE = uint16_t(FB_PROB_BASE + FB_PROB_BLOCKS);
static_assert(FB_PROB_BLK_LEN <= FE_MAX_PAYLOAD, "probability group must fit one record");
static_assert(PROB_FIXED + 2 * PROB_SPARSE_MAX <= PROB_REGION, "sparse region too small");
static_assert(FB_PROBX_BASE + NUM_SLOTS <= FE_MAX_BLOCKS, "block id map overflows the index");
// The whole point of the 32-step layout: every slot fully armed (all three
// variations written full, poly on every slot, probability on every step,
// all tracks) fits the arena WORST CASE, with the one update-reserve page
// still free. Dense overflow ids never materialize (see PAT/PROB/POLY
// sparse-max asserts), so group blocks + tracks + settings is the whole set.
static_assert(FB_PATTERN_BLOCKS + FB_POLY_BLOCKS + FB_PROB_BLOCKS +
                  FB_TRACK_BLOCKS + 1 <= FE_RECORD_PAGES - 1,
              "worst-case live set must fit the arena");

// Serialized sizes.
static constexpr uint8_t FB_TRACK_LEN    = 104;  // p_chain[32] + last[8] + transpose[64]; == TRACK_BYTES (asserted in engine.h)
static constexpr uint8_t FB_SETTINGS_LEN = uint8_t(24 + NUM_SLOTS / 8); // 38: 24 + 14-byte var3 poly bitmap
static constexpr uint8_t FB_PROB_LEN     = uint8_t(3 * MAX_STEPS); // 192 <= FE_MAX_PAYLOAD

// Page hooks: the block store addresses absolute flash pages; program via the
// boot SPM service, read via far program-memory reads.
inline uint8_t fe_dev_program(uint16_t abs_page, const uint8_t *buf) {
  return flash_write_page(abs_page, buf);
}
inline void fe_dev_read(uint16_t abs_page, uint8_t *buf) {
  uint32_t a = (uint32_t)abs_page << 8;
  for (uint16_t i = 0; i < FE_PAGE; ++i) buf[i] = pgm_read_byte_far(a + i);
}

extern FlashEeprom g_flash;

// Tracks, paired TRACK_PER_BLOCK per record. A fresh pair record is 0xFF-filled
// so the never-written sibling half still reads as "fresh" (LoadTrack's uninit
// detection keys on 0xFF).
inline bool ReadTrackAt(uint8_t track, uint8_t *dst /* FB_TRACK_LEN */) {
  uint8_t blk[TRACK_PER_BLOCK * FB_TRACK_LEN];
  if (g_flash.read(uint16_t(FB_TRACK_BASE + track / TRACK_PER_BLOCK), blk,
                   sizeof(blk)) != sizeof(blk))
    return false;
  memcpy(dst, blk + (track % TRACK_PER_BLOCK) * FB_TRACK_LEN, FB_TRACK_LEN);
  return true;
}
inline bool WriteTrackAt(uint8_t track, const uint8_t *src /* FB_TRACK_LEN */) {
  const uint16_t gblk = uint16_t(FB_TRACK_BASE + track / TRACK_PER_BLOCK);
  uint8_t blk[TRACK_PER_BLOCK * FB_TRACK_LEN];
  if (g_flash.read(gblk, blk, sizeof(blk)) != sizeof(blk))
    memset(blk, 0xFF, sizeof(blk));
  memcpy(blk + (track % TRACK_PER_BLOCK) * FB_TRACK_LEN, src, FB_TRACK_LEN);
  return g_flash.write(gblk, blk, sizeof(blk));
}

// Mount the arena (formats if blank). Returns false if flash is unavailable
// (e.g. SPM service not installed) -- the app then runs without persistence.
inline bool flash_persist_begin() {
  return g_flash.begin(fe_dev_program, fe_dev_read);
}
