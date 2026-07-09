/*
 * Minimal MIDI SysEx Bootloader (DIN serial + USB-MIDI)
 * Target: AT90USB1286
 * Clock : 16 MHz
 *
 * The same F0 7D ... F7 update protocol is accepted from either byte
 * source: USART1 (DIN MIDI IN) or the polled USB-MIDI device in
 * usb_boot.c. Entry stays as before (hold WRITE/NEXT/TAP at power-on);
 * additionally the app can jump here without a reset after parking
 * BOOT_MAGIC in GPIOR0 (SysEx 0x4A in the D650 app), which is what makes
 * updates possible over USB-C alone.
 */

// #define F_CPU 16000000UL

#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#include "usb_boot.h"

#define APP_ADDRESS 0x0000
#define PAGE_SIZE SPM_PAGESIZE
#define SYSEX_MAX (PAGE_SIZE*2)

/* App-requested bootloader entry: survives a jmp (no reset), and GPIOR0
 * resets to 0 on any real reset, so it cannot false-positive at power-on.
 * Keep in sync with the app's 0x4A handler (emu_avr.cpp). */
#define BOOT_MAGIC 0xB7

/* Hardware enters the boot section at 0x1F000 (BOOTRST, BOOTSZ=01). This
 * stub is pinned there by the linker (.boot_entry); it establishes the C
 * environment the app or reset left behind and jumps to main. Built with
 * -nostartfiles, so no crt does this for us. */
int main(void);
__attribute__((naked, used, section(".boot_entry")))
void boot_entry(void) {
  asm volatile(
    "eor  r1, r1            \n\t"
    "ldi  r28, %0           \n\t"
    "ldi  r29, %1           \n\t"
    "out  __SP_L__, r28     \n\t"
    "out  __SP_H__, r29     \n\t"
    "jmp  main              \n\t"
    :: "M" (RAMEND & 0xFF), "M" (RAMEND >> 8));
}

static uint8_t page_buffer[PAGE_SIZE];
static uint8_t sysex_buf[SYSEX_MAX];

static volatile uint16_t sysex_index = 0;
static volatile uint8_t in_sysex = 0;

static void uart_init(void) {
  // 31250 baud @ 16 MHz → UBRR = 31
  UBRR1H = 0;
  UBRR1L = 31;

  UCSR1A = 0;
  UCSR1B = (1 << RXEN1);                  // RX only
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8N1
}

/* returns checksum of decoded bytes */
static uint8_t decode_7bit(uint8_t *in, uint16_t len, uint8_t *out) {
  uint16_t i = 0, o = 0;
  uint8_t check = 0;

  while (i < len) {
    uint8_t msb = in[i++];

    for (uint8_t b = 0; b < 7 && i < len; b++) {
      uint8_t v = in[i++];
      out[o] = v | (((msb >> b) & 1) << 7);
      check ^= out[o++];
      if (o >= PAGE_SIZE)
        return check;
    }
  }
  return check;
}

static void flash_write_page(uint16_t page) {
  uint32_t addr = (uint32_t)page * PAGE_SIZE;

  uint8_t sreg = SREG;
  cli();

  eeprom_busy_wait();

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)(page_buffer[i]) | ((uint16_t)(page_buffer[i + 1]) << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  SREG = sreg;
}

static void jump_to_app(void) {
  cli();

  // If application not blank (flash read: the old *(uint16_t*)0 read the
  // r0/r1 register file, not flash). Returns to the caller when blank.
  if (pgm_read_word(APP_ADDRESS) != 0xFFFF) {
    usb_boot_detach(); // no-op unless USB was brought up
    ((void (*)(void))APP_ADDRESS)();
  }
}

static uint8_t count = 0;
static void process_sysex(uint8_t *data, uint16_t len) {
  if (len < 2)
    return;

  if (data[0] != 0x7D)
    return;

  uint8_t cmd = data[1];

  if (cmd == 0x01 && len > 9) {
    uint16_t page = ((uint16_t)data[2] << 7) | data[3];
    uint16_t packed_len = ((uint16_t)data[4] << 7) | data[5];
    uint8_t cksum = (data[6] << 4) | data[7];

    if (packed_len + 8 > len)
      return;

    PORTD = (PORTD & 0x0F) | (count++ << 4); // cycle LEDs while writing

    // if they match, cksum will become zero
    cksum ^= decode_7bit(&data[8], packed_len, page_buffer);
    if (cksum) {
      // uh-oh bad checksum, slow blink forever
      while (1) {
        PORTD ^= 0xF0;
        _delay_ms(200);
      }
    }
    flash_write_page(page);
  } else if (cmd == 0x02) {
    jump_to_app();
  }
}

// One byte of MIDI from either transport (USART1 or USB); non-static so
// usb_boot.c can feed it.
void midi_byte(uint8_t b) {
  if (b == 0xF0) {
    in_sysex = 1;
    sysex_index = 0;
    return;
  }

  if (!in_sysex)
    return;

  if (b == 0xF7) {
    process_sysex(sysex_buf, sysex_index);
    in_sysex = 0;
    return;
  }

  if (b < 0x80 && sysex_index < SYSEX_MAX)
    sysex_buf[sysex_index++] = b;
}

int main(void) {
  // After a watchdog reset, WDRF forces WDE on and wdt_disable() alone cannot
  // clear it: the chip reset-loops every 15 ms until power-off. Clear MCUSR
  // first. (The combined firmware's G# switch reboots via the watchdog.)
  MCUSR = 0;
  wdt_disable();

  // -nostartfiles: .bss is never zeroed, so parser state is set explicitly
  // (this also covers entry by jump from the app, where RAM is app leftovers)
  sysex_index = 0;
  in_sysex = 0;
  count = 0;

  DDRF = 0xFF; // select pin outputs
  DDRB = 0x00; // button inputs
  DDRD |= 0xF0; // direct LED outputs (but also the MIDI serial lines)
  uart_init();

  // app-requested entry (SysEx 0x4A jumped here with the magic parked)
  uint8_t stay = 0;
  if (GPIOR0 == BOOT_MAGIC) {
    GPIOR0 = 0;
    stay = 1;
  }

  // check for button combo to stop the jump
  PORTF = 0x00;
  PORTF = 0x0F;
  _delay_ms(40);
  if (PINB & (1 << 1)) // hold WRITE/NEXT/TAP to stay in bootloader
    stay = 1;

  if (!stay)
    jump_to_app(); // returns only if the app section is blank

  // attach early so USB enumeration overlaps the LED show (control
  // requests just wait in the FIFO until the loop below services them)
  usb_boot_init();

  // indicate bootloader mode: TIME, PITCH, FUNCTION, and A# key LEDS, all on
  PORTD |= 0xF0;
  // flash each LED twice for sanity check
  for (uint8_t i = 0; i < 4; ++i) {
    _delay_ms(100);
    PORTD ^= (1 << (4 + i));
    _delay_ms(100);
    PORTD ^= (1 << (4 + i));
    _delay_ms(100);
    PORTD ^= (1 << (4 + i));
    _delay_ms(100);
    PORTD ^= (1 << (4 + i));
  }

  while (1) {
    if (UCSR1A & (1 << RXC1))
      midi_byte(UDR1);
    usb_boot_task();
  }
}
