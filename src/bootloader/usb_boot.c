/*
 * Polled USB-MIDI device stack for the SysEx bootloader (AT90USB1286).
 *
 * Enumerates as a one-port USB-MIDI device ("SuperOS-303 Bootloader",
 * 16C0:0485 like the app) and feeds every received MIDI byte into the
 * bootloader's midi_byte() parser, so the same update .syx works over
 * USB-C and DIN. Fully polled, no interrupts: bootload.c calls
 * usb_boot_task() from its main loop. While a flash page write is in
 * progress the OUT endpoint NAKs, so the USB path needs none of the
 * inter-message pacing the DIN path does.
 *
 * Register sequences follow PJRC's usb_midi core (known good on this
 * part). Build constraints: -nostartfiles/-nostdlib, so no .data
 * (const tables are PROGMEM) and all state is set at runtime.
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>

#include "usb_boot.h"

/* bootload.c */
extern void midi_byte(uint8_t b);

#define EP0_SIZE 64

static uint8_t usb_active; /* init ran: detach-on-jump needed */
static uint8_t usb_cfg;    /* SET_CONFIGURATION value */

static const uint8_t PROGMEM dev_desc[18] = {
  18, 1,                  /* device */
  0x00, 0x02,             /* USB 2.0 (full speed) */
  0, 0, 0,                /* class per interface */
  EP0_SIZE,
  0xC0, 0x16, 0x85, 0x04, /* VID/PID 16C0:0485, same as the app */
  0x02, 0x00,             /* bcdDevice 0.02 marks the bootloader */
  0, 1, 0,                /* no mfr string, product = 1, no serial */
  1
};

/* One-cable USB-MIDI function, same jack topology as the Teensy usb_midi
 * core: embedded+external IN/OUT jack pairs, bulk OUT (host->device) and
 * bulk IN. The IN endpoint is never written, but hosts expect it. */
static const uint8_t PROGMEM cfg_desc[101] = {
  9, 2, 101, 0, 2, 1, 0, 0xC0, 50,    /* config: 2 interfaces */
  9, 4, 0, 0, 0, 1, 1, 0, 0,          /* if0: AudioControl */
  9, 0x24, 1, 0x00, 0x01, 9, 0, 1, 1, /* CS AC header -> MS if1 */
  9, 4, 1, 0, 2, 1, 3, 0, 0,          /* if1: MIDIStreaming */
  7, 0x24, 1, 0x00, 0x01, 65, 0,      /* CS MS header, 65 bytes follow */
  6, 0x24, 2, 1, 1, 0,                /* MIDI IN jack, embedded (1) */
  6, 0x24, 2, 2, 2, 0,                /* MIDI IN jack, external (2) */
  9, 0x24, 3, 1, 3, 1, 2, 1, 0,       /* MIDI OUT jack, embedded (3) */
  9, 0x24, 3, 2, 4, 1, 1, 1, 0,       /* MIDI OUT jack, external (4) */
  9, 5, 0x01, 2, 64, 0, 0, 0, 0,      /* EP1 OUT bulk 64 */
  5, 0x25, 1, 1, 1,
  9, 5, 0x82, 2, 64, 0, 0, 0, 0,      /* EP2 IN bulk 64 */
  5, 0x25, 1, 1, 3,
};

static const uint8_t PROGMEM str0[] = { 4, 3, 0x09, 0x04 };
static const uint8_t PROGMEM str1[] = {
  2 + 2 * 22, 3,
  'S',0,'u',0,'p',0,'e',0,'r',0,'O',0,'S',0,'-',0,'3',0,'0',0,'3',0,' ',0,
  'B',0,'o',0,'o',0,'t',0,'l',0,'o',0,'a',0,'d',0,'e',0,'r',0
};

/* MIDI bytes per USB-MIDI Code Index Number */
static const uint8_t PROGMEM cin_len[16] =
  { 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1 };

void usb_boot_init(void) {
  usb_cfg = 0;
  usb_active = 1;
  UDCON = (1 << DETACH); /* clean start even if the app left USB attached */
  USBCON = 0;
  UHWCON = (1 << UIMOD) | (1 << UVREGE);
  USBCON = (1 << USBE) | (1 << FRZCLK);
  PLLCSR = 0x16; /* 16 MHz crystal -> 48 MHz USB clock */
  while (!(PLLCSR & (1 << PLOCK)))
    ;
  USBCON = (1 << USBE) | (1 << OTGPADE);
  UDCON = 0; /* attach */
}

void usb_boot_detach(void) {
  if (!usb_active)
    return;
  UDCON = (1 << DETACH);
  USBCON = 0;
  PLLCSR = 0;
  /* let the host register the drop before the app re-attaches; F_CPU is
   * 4 MHz in this build vs the real 16 MHz clock, so this is ~50 ms */
  _delay_ms(200);
}

/* control-IN data stage from PROGMEM, honouring wLength, ZLP included.
 * Far pointer + ELPM throughout: this code links above 64 KB, so plain
 * pgm_read_byte (16-bit LPM) would read app-region flash instead. */
static void ep0_send(uint_farptr_t p, uint8_t len, uint16_t wLength) {
  uint16_t n = (wLength < len) ? wLength : len;
  uint8_t chunk;
  do {
    uint8_t x;
    do {
      x = UEINTX;
    } while (!(x & ((1 << TXINI) | (1 << RXOUTI))));
    if (x & (1 << RXOUTI))
      return; /* host aborted the transfer */
    chunk = (n > EP0_SIZE) ? EP0_SIZE : (uint8_t)n;
    for (uint8_t i = 0; i < chunk; i++)
      UEDATX = pgm_read_byte_far(p++);
    n -= chunk;
    UEINTX = (uint8_t)~(1 << TXINI);
  } while (n || chunk == EP0_SIZE);
}

void usb_boot_task(void) {
  if (UDINT & (1 << EORSTI)) { /* bus reset: (re)arm EP0 */
    UDINT = (uint8_t)~(1 << EORSTI);
    UENUM = 0;
    UECONX = (1 << EPEN);
    UECFG0X = 0;    /* control */
    UECFG1X = 0x32; /* 64 bytes, single bank, alloc */
    usb_cfg = 0;
  }

  UENUM = 0;
  if (UEINTX & (1 << RXSTPI)) {
    uint8_t bmReq = UEDATX;
    uint8_t bReq = UEDATX;
    uint8_t wVal_l = UEDATX;
    uint8_t wVal_h = UEDATX;
    (void)UEDATX; /* wIndex */
    (void)UEDATX;
    uint8_t wLen_l = UEDATX;
    uint8_t wLen_h = UEDATX;
    uint16_t wLength = wLen_l | ((uint16_t)wLen_h << 8);
    UEINTX = (uint8_t)~((1 << RXSTPI) | (1 << RXOUTI) | (1 << TXINI));

    if (bReq == 6 && bmReq == 0x80) { /* GET_DESCRIPTOR */
      uint_farptr_t p = 0;
      uint8_t len = 0;
      if (wVal_h == 1) {
        p = pgm_get_far_address(dev_desc);
        len = sizeof(dev_desc);
      } else if (wVal_h == 2) {
        p = pgm_get_far_address(cfg_desc);
        len = sizeof(cfg_desc);
      } else if (wVal_h == 3 && wVal_l == 0) {
        p = pgm_get_far_address(str0);
        len = sizeof(str0);
      } else if (wVal_h == 3 && wVal_l == 1) {
        p = pgm_get_far_address(str1);
        len = sizeof(str1);
      }
      if (p)
        ep0_send(p, len, wLength);
      else
        UECONX = (1 << STALLRQ) | (1 << EPEN); /* qualifier etc. */
      return;
    }
    if (bReq == 5 && bmReq == 0) { /* SET_ADDRESS */
      UEINTX = (uint8_t)~(1 << TXINI);
      while (!(UEINTX & (1 << TXINI)))
        ; /* address applies only after the status stage */
      UDADDR = wVal_l | (1 << ADDEN);
      return;
    }
    if (bReq == 9 && bmReq == 0) { /* SET_CONFIGURATION */
      usb_cfg = wVal_l;
      UENUM = 1; /* EP1 OUT bulk 64, double bank */
      UECONX = (1 << EPEN);
      UECFG0X = 0x80;
      UECFG1X = 0x36;
      UENUM = 2; /* EP2 IN bulk 64, double bank */
      UECONX = (1 << EPEN);
      UECFG0X = 0x81;
      UECFG1X = 0x36;
      UERST = 0x06; /* reset both FIFOs */
      UERST = 0;
      UENUM = 0;
      UEINTX = (uint8_t)~(1 << TXINI);
      return;
    }
    if (bReq == 8 && bmReq == 0x80) { /* GET_CONFIGURATION */
      while (!(UEINTX & (1 << TXINI)))
        ;
      UEDATX = usb_cfg;
      UEINTX = (uint8_t)~(1 << TXINI);
      return;
    }
    if (bReq == 0 && (bmReq & 0x80)) { /* GET_STATUS */
      while (!(UEINTX & (1 << TXINI)))
        ;
      UEDATX = (bmReq == 0x80) ? 1 : 0; /* device: self-powered */
      UEDATX = 0;
      UEINTX = (uint8_t)~(1 << TXINI);
      return;
    }
    if ((bReq == 1 || bReq == 3) && bmReq <= 2) { /* CLEAR/SET_FEATURE */
      UEINTX = (uint8_t)~(1 << TXINI);
      return;
    }
    if (bReq == 11 && bmReq == 1) { /* SET_INTERFACE */
      UEINTX = (uint8_t)~(1 << TXINI);
      return;
    }
    UECONX = (1 << STALLRQ) | (1 << EPEN);
    return;
  }

  if (!usb_cfg)
    return;

  UENUM = 1; /* drain the MIDI OUT endpoint */
  if (UEINTX & (1 << RXOUTI)) {
    UEINTX = (uint8_t)~(1 << RXOUTI);
    while (UEINTX & (1 << RWAL)) {
      uint8_t b0 = UEDATX;
      uint8_t b1 = UEDATX;
      uint8_t b2 = UEDATX;
      uint8_t b3 = UEDATX;
      uint8_t n = pgm_read_byte_far(pgm_get_far_address(cin_len) + (b0 & 0x0F));
      if (n >= 1)
        midi_byte(b1);
      if (n >= 2)
        midi_byte(b2);
      if (n >= 3)
        midi_byte(b3);
    }
    UEINTX = (uint8_t)~(1 << FIFOCON); /* release the bank */
  }
}
