// combined.cpp -- entry dispatch for the combined SuperOS + D650C build.
// Empty TU in non-combined builds.
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#define DRIVERS_NO_ISR   // main.cpp owns the Timer3 vector
#include "pins.h"
#include "drivers.h"
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
  // Input lockout before the reboot: a key still held through the reset lands
  // in the bootloader's stay-resident window (the TAP line is sampled 40 ms
  // after reset, and the diode-less matrix can ghost other held keys onto it)
  // and the unit sits in the bootloader looking crashed until power-off.
  // Wait until every momentary key (incl. the G#/switch key itself) has read
  // released for 250 ms straight. Dial lines are levels and are ignored.
  // Hard 4 s timeout so a stuck line can't block the switch forever.
  {
    static const uint8_t kMomentary[] = {
      C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY, B_KEY, C_KEY2,
      DOWN_KEY, UP_KEY, ACCENT_KEY, SLIDE_KEY,
      FSHARP_KEY, GSHARP_KEY, ASHARP_KEY, BACK_KEY,
      CSHARP_KEY, DSHARP_KEY,
      CLEAR_KEY, FUNCTION_KEY, PITCH_KEY, TIME_KEY,
      RUN, TAP_NEXT,
    };
    PinState keys[INPUT_COUNT];
    const uint32_t t0 = millis();
    uint32_t idle_since = millis();
    while (millis() - t0 < 4000) {
      Leds::PauseRefresh();
      PollInputs(keys);
      Leds::ResumeRefresh();
      bool any = false;
      for (uint8_t i = 0; i < sizeof(kMomentary); ++i)
        any |= keys[kMomentary[i]].read();
      const uint32_t now = millis();
      if (any) idle_since = now;
      else if (now - idle_since >= 250) break;
      delay(2);
    }
  }
  // Hardware reset via watchdog: full peripheral reset, then the bootloader
  // (which wdt_disable()s on entry) falls through to the app, and setup()
  // reads the new select byte.
  cli();
  wdt_enable(WDTO_15MS);
  for (;;) {}
}
#endif
