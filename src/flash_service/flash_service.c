/*
 * flash_service.c -- SPM flash-write service for SuperOS-303.
 *
 * SPM (flash self-write) can ONLY execute from the boot section, so the app
 * cannot write flash directly. This tiny service lives in the otherwise-empty
 * top of the boot section (entry at byte 0x1FE00, spanning pages 0x1FE..0x1FF)
 * and the running app calls into it to erase+write one 256-byte flash page.
 * See FLASH_EEPROM_DESIGN.md.
 *
 * Install: built as its own image and packed into service-install.syx, which
 * writes only the service pages via the existing bootloader SysEx updater. The
 * running bootloader (pages 0x1F0..0x1F3) is never touched, so there is no
 * brick risk and a failed install is retryable.
 *
 * ABI (matches avr-gcc default): page in r24:r25, buf pointer in r22:r23,
 * returns status in r24. avr:51 function pointers are WORD addresses, so the
 * app calls word 0xFF00 (= byte 0x1FE00).
 */

#include <avr/io.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define PAGE_SIZE 256

/* Emulated-EEPROM arena = 0x10000..0x1DFFF (RWW, below the 0x1E000 NRWW edge).
 * The service refuses anything outside it, so a buggy app can never erase the
 * application code (pages 0x000..0x0FF), the boot section, or the arena's
 * neighbours. The only page this service will ever write is one the caller
 * names AND that falls inside [ARENA_FIRST_PAGE, ARENA_LAST_PAGE]. */
#define ARENA_FIRST_PAGE 0x100u /* 0x10000 / 256 -> 56 KB arena base (raised from 0xE0 for the USB-MIDI app) */
#define ARENA_LAST_PAGE  0x1DFu /* 0x1DF00 / 256 -> last page fully below 0x1E000 */

/* Trampoline pinned at byte 0x1FF00 by the linker (--section-start). The app
 * ICALLs here; this JMPs to the real implementation, leaving the argument
 * registers untouched so they pass straight through. */
__attribute__((section(".service_entry"), naked, used))
void flash_service_entry(void) {
  __asm__ __volatile__("jmp flash_service_impl");
}

__attribute__((used))
uint8_t flash_service_impl(uint16_t page, const uint8_t *buf) {
  if (page < ARENA_FIRST_PAGE || page > ARENA_LAST_PAGE)
    return 1; /* out of arena -- reject */

  /* page * 256 via shift so no __mul libgcc call is needed under -nostdlib. */
  uint32_t addr = (uint32_t)page << 8;

  uint8_t sreg = SREG;
  cli(); /* interrupt vectors live in RWW and are unreadable during the write */

  eeprom_busy_wait();

  /* The arena now lives at 0x10000..0x1DFFF -- above the 64 KB boundary. SPM
   * page erase/write address the flash via RAMPZ:Z, but the plain boot_page_*
   * macros only load the 16-bit Z register and leave RAMPZ untouched, so they
   * cannot reach bit 16 and would hit the wrong (low) page. Set RAMPZ to the
   * high address byte ourselves; the macros' SPM then targets the real page.
   * (Page fill writes the temp buffer by word offset and does not use RAMPZ.) */
  RAMPZ = (uint8_t)(addr >> 16);

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  RAMPZ = 0; /* boot_page_* (extended) left RAMPZ set; restore for plain LPM */
  SREG = sreg;
  return 0;
}
