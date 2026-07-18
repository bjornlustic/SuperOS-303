// flash_persist.h -- device glue between the persistence layer and the flash
// block store (flash_eeprom.h). Provides the page program/read hooks, the
// single FlashEeprom instance, and the logical block-id map.
#pragma once
#include <Arduino.h>
#include "flash_store.h"   // flash_write_page / pgm_read_byte_far / arena bounds
#include "flash_eeprom.h"

// Mono patterns pack two-per-page (2x92 = 184 <= FE_MAX_PAYLOAD). var0+var1 of a
// slot share a block; variation 3 packs two slots' mono var3 per block. Only when
// a slot's variation 3 is POLY does it get its own dedicated block (a 226-byte
// poly blob). So a fully-used device costs ~105 blocks when var3 is mono (fits the
// 127 live-block cap), rising toward 137 only as poly slots accumulate.
static constexpr uint8_t FB_PATTERN_LEN_ONE = MAX_STEPS + (MAX_STEPS / 4) + METADATA_SIZE; // 92

// Logical block ids (must stay < FE_MAX_BLOCKS).
//   0 .. NUM_SLOTS-1                   : var0+var1 pairs (block s = slot s)
//   FB_MONOVAR2_BASE .. +NUM_SLOTS/2-1 : mono var3, two slots per block (block = base + slot/2, half = slot&1)
//   FB_POLY_BASE .. +NUM_SLOTS-1       : poly var3, one slot per block (poly slots only)
//   FB_TRACK_BASE .. +7                : tracks 0..7
//   FB_SETTINGS                        : settings
static constexpr uint8_t FB_PATTERN_BASE   = 0;
static constexpr uint8_t FB_MONOVAR2_BASE  = uint8_t(NUM_SLOTS);                       // 64
static constexpr uint8_t FB_POLY_BASE      = uint8_t(FB_MONOVAR2_BASE + NUM_SLOTS / 2); // 96
static constexpr uint8_t FB_TRACK_BASE     = uint8_t(FB_POLY_BASE + NUM_SLOTS);        // 160
static constexpr uint8_t FB_SETTINGS       = uint8_t(FB_TRACK_BASE + 8);               // 168
// Per-slot step-probability table, one block per slot (variation 1 only; only
// written when any step is armed). 3 bytes/step: b0 accent+slide levels,
// b1 down+up transpose levels, b2 up-double flag.
static constexpr uint8_t FB_PROB_BASE      = uint8_t(FB_SETTINGS + 1);                 // 169

// Serialized sizes.
static constexpr uint8_t FB_PATTERN_LEN  = 2 * FB_PATTERN_LEN_ONE; // 184 (a 2-pattern block)
static constexpr uint8_t FB_TRACK_LEN    = 104;  // p_chain[32] + last[8] + transpose[64]; == TRACK_BYTES (asserted in engine.h)
static constexpr uint8_t FB_SETTINGS_LEN = 32;   // 24 + 8-byte var3 poly bitmap
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

// Mount the arena (formats if blank). Returns false if flash is unavailable
// (e.g. SPM service not installed) -- the app then runs without persistence.
inline bool flash_persist_begin() {
  return g_flash.begin(fe_dev_program, fe_dev_read);
}
