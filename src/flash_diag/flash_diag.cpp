// flash_diag.cpp -- block-store persistence diagnostic (no USB).
//
// Tests the SAME path the app uses to save patterns: mount the flash block store
// (g_flash), write a scratch block, and check whether it survives a power cycle.
// Reports on the keypad LEDs:
//
//   chromatic key row (C..C2) solid = PERSISTED. The block store kept a record
//     across the reboot -> low-level storage works; the persistence bug is in the
//     engine/settings glue or the save trigger, NOT the block store.
//   UP_KEY_LED solid               = wrote the sentinel this boot. Power-cycle
//     the device (no reflash) to see if it comes back as PERSISTED.
//   DOWN_KEY_LED blinking          = g_flash.write() returned false -- the block
//     store itself refused/failed the write.
//
// Procedure: flash this (-e flash-diag), power-cycle ONCE (UP lights = sentinel
// written), power-cycle AGAIN. Row solid on the 2nd cycle = block store persists.
// UP again on the 2nd cycle = block store does NOT persist. Reflash -e app after.
//
// Uses scratch block id 169 (one past FB_SETTINGS=168, unused) so it never
// touches real saved data, and it APPENDS (mount does not wipe an existing arena).
#include <Arduino.h>
#include "pins.h"
#include "drivers.h"
#include "sequence.h"
#include "flash_persist.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

FlashEeprom g_flash;

static constexpr uint8_t FB_SCRATCH = 169;            // unused id, < FE_MAX_BLOCKS
static const uint8_t MAGIC[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x23,0x45,0x67};

enum { ST_PERSISTED, ST_WROTE, ST_WRITE_FAIL };
static uint8_t s_state = ST_WRITE_FAIL;

void setup() {
  for (uint8_t i = 0; i < ARRAY_SIZE(INPUTS); ++i)  pinMode(INPUTS[i], INPUT);
  for (uint8_t i = 0; i < ARRAY_SIZE(OUTPUTS); ++i) pinMode(OUTPUTS[i], OUTPUT);
  for (uint8_t i = 0; i < 4; ++i) digitalWriteFast(select_pin[i], HIGH);

  Leds::BeginRefresh();

  flash_persist_begin();                              // mount g_flash (formats if blank)

  uint8_t rb[8] = {0};
  const uint8_t n = g_flash.read(FB_SCRATCH, rb, 8);
  if (n == 8 && memcmp(rb, MAGIC, 8) == 0) {
    s_state = ST_PERSISTED;                           // survived a prior reboot
  } else {
    s_state = g_flash.write(FB_SCRATCH, MAGIC, 8) ? ST_WROTE : ST_WRITE_FAIL;
  }
}

void loop() {
  if (s_state == ST_PERSISTED) {
    for (uint8_t k = 0; k < ARRAY_SIZE(pitch_leds); ++k) Leds::Set(pitch_leds[k], true);
  } else if (s_state == ST_WROTE) {
    Leds::Set(UP_KEY_LED, true);
  } else {
    Leds::Set(DOWN_KEY_LED, (millis() >> 8) & 1);
  }
  Leds::Swap();
  delay(2);
}
