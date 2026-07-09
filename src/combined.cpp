// combined.cpp -- entry dispatch for the combined SuperOS + D650C build.
// Empty TU in non-combined builds.
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "combined.h"
#include "engine.h"
extern "C" {
#include "emu/ucom4.h"
#include "emu/d650_host.h"
}

uint8_t g_fw_arena[sizeof(Engine) > sizeof(d650_host) ? sizeof(Engine)
                                                      : sizeof(d650_host)];

// main.cpp and d650/emu_avr.cpp rename their setup/loop to these.
void superos_setup(); void superos_loop();
void emu_setup();     void emu_loop();

static uint8_t g_fw = FW_SUPEROS;

void setup() {
  // Defense in depth against the WDT reset loop (see bootload.c main): if an
  // older bootloader is on the chip, clear WDRF here before the 15 ms window
  // ends. Harmless on a normal boot.
  MCUSR = 0;
  wdt_disable();
  const uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_D650) ? FW_D650 : FW_SUPEROS;
  if (g_fw == FW_D650) emu_setup(); else superos_setup();
}

void loop() {
  if (g_fw == FW_D650) emu_loop(); else superos_loop();
}

void combined_switch_firmware(uint8_t fw) {
  eeprom_update_byte(EE_FW_SELECT, fw);
  // Hardware reset via watchdog: full peripheral reset, then the bootloader
  // (which wdt_disable()s on entry) falls through to the app, and setup()
  // reads the new select byte.
  cli();
  wdt_enable(WDTO_15MS);
  for (;;) {}
}
#endif
