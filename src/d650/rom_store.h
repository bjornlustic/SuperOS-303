// rom_store.h -- user-supplied D650C mask ROM: EEPROM persistence + the SysEx
// receiver (combined build only). Ported from SuperOS-606; see src/d650/SYNC.md.
//
// The D650_ROM_IN_RAM build ships WITHOUT the (copyrighted) D650C mask ROM. The
// user dumps their own chip (or uses a dump they are legally entitled to) and
// uploads it over DIN MIDI in the following nibble format:
//
//   F0 7D 03 03 7E 03 <blk> <2048 nibbles hi,lo> <ck hi> <ck lo> F7   blk = 0,1
//   F0 7D 03 03 7F 00 F7                                              end mark
//
// Each block carries 1024 ROM bytes; <ck> is the sum of those decoded bytes
// mod 256. The web editor converts a raw 2048-byte .bin dump into this format
// (or forwards an existing .syx already in this format).
//
// Storage: internal EEPROM (combined.h map). Data at EE_ROM_DATA, sum16 at
// EE_ROM_SUM, magic byte at EE_ROM_MAGIC written LAST, so a torn upload can
// never validate at boot. The emulator runs the ROM from a 2 KB SRAM copy
// (UCOM4_ROM_RD is redirected by D650_ROM_IN_RAM; a RAM fetch is a cycle
// cheaper than the LPM it replaces, so the interpreter's timing is preserved).
#pragma once
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "../combined.h"

#define D650_ROM_SIZE 2048

static inline uint16_t rom_sum16(const uint8_t *p) {
  uint16_t s = 0;
  for (uint16_t i = 0; i < D650_ROM_SIZE; ++i) s += p[i];
  return s;
}

// EEPROM -> dst. True only if the stored ROM is present and checksums clean.
static bool rom_load(uint8_t *dst) {
  if (eeprom_read_byte(EE_ROM_MAGIC) != EE_ROM_MAGIC_VAL) return false;
  eeprom_read_block(dst, EE_ROM_DATA, D650_ROM_SIZE);
  const uint16_t want = (uint16_t)eeprom_read_byte(EE_ROM_SUM)
                      | ((uint16_t)eeprom_read_byte(EE_ROM_SUM + 1) << 8);
  return rom_sum16(dst) == want;
}

// src -> EEPROM. Blocking, worst case ~7 s of 3.3 ms byte programs; call only
// from the upload paths. The magic is invalidated first and rewritten last.
// wdt_reset() is a no-op on the nousbc build (no watchdog) and pets the 8 s
// loop watchdog on the USB combined build.
// `progress` (local addition, see SYNC.md) is called every 64 bytes so the
// caller can keep an upload indicator animating. This loop blocks for SECONDS:
// eeprom_update_byte busy-waits ~3.3 ms per byte it actually changes, and on an
// erased EEPROM all 2048 change (~6.8 s). Without the hook the main loop never
// runs, so the LED refresh ISR just holds the last published frame and the
// indicator looks frozen mid-upload.
static void rom_save(const uint8_t *src, void (*progress)() = nullptr) {
  eeprom_update_byte(EE_ROM_MAGIC, 0xFF);
  for (uint16_t i = 0; i < D650_ROM_SIZE; ++i) {
    eeprom_update_byte(EE_ROM_DATA + i, src[i]);
    // Every 16 bytes, not 64: a changed byte costs ~3.3 ms, so 64 would only
    // call back every ~211 ms and undersample the 100 ms blink phase (it
    // aliases into a slow, irregular flicker). 16 bytes is ~53 ms, comfortably
    // faster than the phase, so the fast blink actually reads as fast.
    if ((i & 0x0F) == 0) { wdt_reset(); if (progress) progress(); }
  }
  const uint16_t s = rom_sum16(src);
  eeprom_update_byte(EE_ROM_SUM, (uint8_t)(s & 0xFF));
  eeprom_update_byte(EE_ROM_SUM + 1, (uint8_t)(s >> 8));
  eeprom_update_byte(EE_ROM_MAGIC, EE_ROM_MAGIC_VAL);
  wdt_reset();
}

// ---------------------------------------------------------------------------
// Byte-fed receiver. Decodes directly into the caller's 2048-byte buffer.
// The caller must treat the buffer as garbage from ROMRX_STARTED until
// ROMRX_DONE (in live emulator mode: freeze the interpreter on STARTED and
// reload the buffer from EEPROM if the transfer errors out or times out).
// ---------------------------------------------------------------------------
enum RomRxEvent : uint8_t {
  ROMRX_NONE = 0,      // nothing notable this byte
  ROMRX_STARTED,       // confirmed ROM-block message: buffer is being written
  ROMRX_BLOCK,         // one 1024-byte block landed and checksummed clean
  ROMRX_DONE,          // end marker seen with both blocks received
  ROMRX_ERROR,         // bad nibble/checksum/sequence: receiver reset
};

struct RomRx {
  uint8_t  state = 0;      // index into the expected-byte walk below
  uint8_t  blk = 0;        // current block (0/1)
  uint16_t nib = 0;        // nibbles consumed in the data run (0..2047)
  uint8_t  hi = 0;         // latched high nibble
  uint8_t  sum = 0;        // running mod-256 sum of decoded bytes
  uint8_t  got = 0;        // bit per received block
  uint32_t last_ms = 0;    // for the caller's stall timeout

  bool busy() const { return state >= 5; }  // past the shared F0 7D 03 03 header
  void reset() { state = 0; got = 0; }

  RomRxEvent feed(uint8_t b, uint32_t now_ms) {
    if (b >= 0xF8) return ROMRX_NONE;              // realtime: legal anywhere
    last_ms = now_ms;
    if (b == 0xF0) { state = 1; return ROMRX_NONE; }
    if (state == 0) return ROMRX_NONE;             // not in a message
    if (b >= 0x80 && b != 0xF7) {                  // status aborts the message
      const RomRxEvent e = busy() ? ROMRX_ERROR : ROMRX_NONE;
      state = 0;
      return e;
    }
    switch (state) {
      case 1: state = (b == 0x7D) ? 2 : 0; return ROMRX_NONE;
      case 2: state = (b == 0x03) ? 3 : 0; return ROMRX_NONE;
      case 3: state = (b == 0x03) ? 4 : 0; return ROMRX_NONE;
      case 4:                                      // command: 7E data / 7F end
        if (b == 0x7E) { state = 5; return ROMRX_NONE; }
        if (b == 0x7F) { state = 10; return ROMRX_NONE; }
        state = 0; return ROMRX_NONE;
      case 5:                                      // block header constant
        if (b != 0x03) { state = 0; return ROMRX_ERROR; }
        state = 6; return ROMRX_NONE;
      case 6:                                      // block number
        if (b > 1) { state = 0; return ROMRX_ERROR; }
        blk = b; nib = 0; sum = 0; state = 7;
        return ROMRX_STARTED;                      // caller: buffer now dirty
      case 7:                                      // 2048 data nibbles
        if (b > 0x0F) { state = 0; return ROMRX_ERROR; }
        if (nib & 1) {
          const uint8_t by = (uint8_t)((hi << 4) | b);
          buf[((uint16_t)blk << 10) | (nib >> 1)] = by;
          sum = (uint8_t)(sum + by);
        } else {
          hi = b;
        }
        if (++nib == 2 * 1024) state = 8;
        return ROMRX_NONE;
      case 8:                                      // checksum high nibble
        if (b > 0x0F) { state = 0; return ROMRX_ERROR; }
        hi = b; state = 9; return ROMRX_NONE;
      case 9:                                      // checksum low nibble + F7 next
        if (b > 0x0F) { state = 0; return ROMRX_ERROR; }
        if ((uint8_t)((hi << 4) | b) != sum) { state = 0; return ROMRX_ERROR; }
        state = 12; return ROMRX_NONE;
      case 12:                                     // block terminator
        state = 0;
        if (b != 0xF7) return ROMRX_ERROR;
        got |= (uint8_t)(1 << blk);
        return ROMRX_BLOCK;
      case 10:                                     // end marker payload
        if (b != 0x00) { state = 0; return ROMRX_ERROR; }
        state = 11; return ROMRX_NONE;
      case 11: {                                   // end terminator
        state = 0;
        if (b != 0xF7) return ROMRX_ERROR;
        const bool ok = (got == 0x03);
        got = 0;
        return ok ? ROMRX_DONE : ROMRX_ERROR;
      }
      default: state = 0; return ROMRX_NONE;
    }
  }

  uint8_t *buf = nullptr;   // 2048-byte destination, set once by the owner
};
