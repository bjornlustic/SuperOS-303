// flash_store.h -- app-side access to the boot-section SPM flash service.
//
// Phase 0 primitives for flash-as-EEPROM. The routine that actually writes
// flash lives in the boot section (src/flash_service, byte 0x1FE00); this is
// just the caller plus arena bounds. See FLASH_EEPROM_DESIGN.md.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

// Byte address of the SPM service entry trampoline (pinned by the linker).
static constexpr uint32_t FLASH_SERVICE_ENTRY = 0x1FE00UL;

// avr:51 function pointers are WORD addresses: byte 0x1FE00 -> word 0xFF00.
typedef uint8_t (*FlashServiceFn)(uint16_t page, const uint8_t *buf);
static const FlashServiceFn kFlashService = (FlashServiceFn)(FLASH_SERVICE_ENTRY >> 1);

static constexpr uint32_t FLASH_PAGE_SIZE  = 256;
static constexpr uint32_t FLASH_ARENA_BASE = 0xE000UL;   // first arena byte (64 KB arena)
static constexpr uint32_t FLASH_ARENA_END  = 0x1E000UL;  // one past the last arena byte
static constexpr uint16_t FLASH_ARENA_FIRST_PAGE = FLASH_ARENA_BASE / FLASH_PAGE_SIZE;        // 0xE0
static constexpr uint16_t FLASH_ARENA_LAST_PAGE  = (FLASH_ARENA_END / FLASH_PAGE_SIZE) - 1;   // 0x1DF

// Status codes returned by flash_write_page (0 = ok).
static constexpr uint8_t FLASH_OK            = 0;
static constexpr uint8_t FLASH_ERR_REJECTED  = 1;     // service refused the page (out of arena)
static constexpr uint8_t FLASH_ERR_NO_SERVICE = 0xFE; // SPM service not installed

// True if the SPM service trampoline is installed at the entry address. The
// entry is a JMP whose first opcode word is 0x940C (high address bits are zero).
inline bool flash_service_present() {
  return pgm_read_word_far(FLASH_SERVICE_ENTRY) == 0x940C;
}

// Erase + write one 256-byte page from a RAM buffer. Returns FLASH_OK on
// success. Halts the CPU for a few ms during the write, so call only when the
// clock is stopped (same discipline as the current EEPROM writes). Never calls
// into the boot section unless the service is actually present.
inline uint8_t flash_write_page(uint16_t page, const uint8_t *buf) {
  if (!flash_service_present()) return FLASH_ERR_NO_SERVICE;
  return kFlashService(page, buf);
}

// Read one byte from the arena (far address, >64 KB).
inline uint8_t flash_read(uint32_t addr) {
  return pgm_read_byte_far(addr);
}
