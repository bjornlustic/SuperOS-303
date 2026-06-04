// flash_persist.h -- device glue between the persistence layer and the flash
// block store (flash_eeprom.h). Provides the page program/read hooks, the
// single FlashEeprom instance, and the logical block-id map.
#pragma once
#include <Arduino.h>
#include "flash_store.h"   // flash_write_page / pgm_read_byte_far / arena bounds
#include "flash_eeprom.h"

// Logical block ids (must stay < FE_MAX_BLOCKS).
//   0..63  : patterns (flat index bank*16 + pat)
//   64..71 : tracks 0..7
//   72     : settings
static constexpr uint8_t FB_PATTERN_BASE = 0;
static constexpr uint8_t FB_TRACK_BASE   = 64;
static constexpr uint8_t FB_SETTINGS     = 72;

// Serialized sizes.
static constexpr uint8_t FB_PATTERN_LEN  = MAX_STEPS + (MAX_STEPS / 4) + METADATA_SIZE; // pitch + time + meta
static constexpr uint8_t FB_TRACK_LEN    = 104;  // p_chain[32] + last[8] + transpose[64]; == TRACK_BYTES (asserted in engine.h)
static constexpr uint8_t FB_SETTINGS_LEN = 22;

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
