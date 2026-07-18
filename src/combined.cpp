// combined.cpp -- entry dispatch for the combined SuperOS + D650C build.
// Empty TU in non-combined builds.
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#define DRIVERS_NO_ISR   // main.cpp owns the Timer3 vector
#include "pins.h"
#include "drivers.h"
#include "combined.h"
#include "engine.h"
extern "C" {
#include "emu/ucom4.h"
#include "emu/d650_host.h"
}

// Pre-main boot guard. The Teensy core's usb_init() runs BEFORE setup() and
// busy-waits FOREVER on USB PLL lock (`while (!(PLLCSR & PLOCK));`). If the
// PLL misses lock -- seen intermittently when a watchdog reboot lands while
// the previous firmware's USB/PLL was still running -- the unit wedges dead
// with USB detached until power-off. Arm a 2 s watchdog from .init3 (before
// crt0 finishes, long before usb_init) so ANY pre-setup hang retries the
// boot instead of freezing. setup() disables it. The bootloader is not
// affected (it clears MCUSR + disables the WDT on entry, and this code only
// runs once it jumps to the app).
extern "C" __attribute__((naked, used, section(".init3")))
void combined_boot_guard(void) {
  MCUSR = 0;
  wdt_enable(WDTO_2S);
}

// The D650_ROM_IN_RAM build runs the mask ROM from a 2 KB SRAM copy that lives
// in the arena TAIL, just past the d650 machine (only one firmware runs per
// boot, so in SuperOS mode Engine owns the whole arena and the ROM copy is
// dormant). Size the arena to fit whichever side is larger: SuperOS's Engine,
// or the d650 machine plus the 2 KB ROM. See emu_avr.cpp (s_rom) + rom_store.h.
#if defined(D650_ROM_IN_RAM)
static constexpr unsigned kD650Side = sizeof(d650_host) + 2048u; // D650_ROM_SIZE
#else
static constexpr unsigned kD650Side = sizeof(d650_host);
#endif
uint8_t g_fw_arena[sizeof(Engine) > kD650Side ? sizeof(Engine) : kD650Side];
// midi.cpp parks its SuperOS-only SysEx TX ring at g_fw_arena + sizeof(Engine).
// If the d650 side ever stops being the larger of the two, that tail is real
// Engine state and the overlay would corrupt it -- fail the build instead.
static_assert(sizeof(Engine) + FW_ARENA_SUPEROS_TAIL <= sizeof(g_fw_arena),
              "arena tail too small for the SuperOS-only overlay (see combined.h)");

// Shared inbound USB SysEx reassembly scratch (see combined.h).
uint8_t g_usb_sysex_scratch[FW_USB_SYSEX_SCRATCH];

// main.cpp and d650/emu_avr.cpp rename their setup/loop to these.
void superos_setup(); void superos_loop();
void emu_setup();     void emu_loop();

static uint8_t g_fw = FW_SUPEROS;

// TEMP boot-stage breadcrumb for the fast-switch wedge investigation: written
// at each boot milestone, read back over ISP when the unit wedges. Remove
// when the investigation concludes. 0xA1/2/3 = SuperOS stages, 0xB1/2/3 =
// D650C stages (1 = setup entered, 2 = firmware setup done, 3 = loop ran).
#define EE_BOOT_CRUMB ((uint8_t *)0x0FF0)

void setup() {
  MCUSR = 0;
  // Permanent 8 s watchdog, petted every loop() pass. Root-cause evidence
  // (boot-stage breadcrumbs, 1 Hz heartbeat, boot counter over ISP): after a
  // fast firmware swap the host's USB port can come up hosed, and on such
  // boots the CPU intermittently FREEZES ~4-5 s in (heartbeat stops, boot
  // counter static, USB absent) -- somewhere below our code, most likely the
  // USB core against the pathological bus. The watchdog converts any such
  // freeze into an automatic recovery reboot; the fresh attach afterwards
  // has always enumerated cleanly. 8 s clears every legitimate long stall
  // (flash GC ~1 s, d650c full-store EEPROM flush ~5 s, swap lockout 4 s).
  wdt_enable(WDTO_8S);
  const uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_D650) ? FW_D650 : FW_SUPEROS;
  // TEMP wedge diagnostics: boot counter; WDT-recovery counter via the reset
  // cause the bootloader stashes in GPIOR1 before clearing MCUSR.
  eeprom_update_byte((uint8_t *)0x0FF2,
                     (uint8_t)(eeprom_read_byte((uint8_t *)0x0FF2) + 1));
  if (GPIOR1 & (1 << WDRF))
    eeprom_update_byte((uint8_t *)0x0FF3,
                       (uint8_t)(eeprom_read_byte((uint8_t *)0x0FF3) + 1));
  eeprom_update_byte(EE_BOOT_CRUMB, g_fw == FW_D650 ? 0xB1 : 0xA1);
  if (g_fw == FW_D650) emu_setup(); else superos_setup();
  eeprom_update_byte(EE_BOOT_CRUMB, g_fw == FW_D650 ? 0xB2 : 0xA2);
}

void loop() {
  wdt_reset();   // pet the permanent freeze-recovery watchdog (see setup)
  static bool s_first = true;
  if (s_first) {
    s_first = false;
    eeprom_update_byte(EE_BOOT_CRUMB, g_fw == FW_D650 ? 0xB3 : 0xA3);
  }
  // TEMP liveness heartbeat (wedge investigation): seconds-since-boot at
  // 0x0FF1, 1 Hz. Remove with the breadcrumbs.
  static uint32_t s_hb_ms = 0;
  if (millis() - s_hb_ms >= 1000) {
    s_hb_ms = millis();
    eeprom_update_byte((uint8_t *)0x0FF1, (uint8_t)(millis() / 1000));
  }
  if (g_fw == FW_D650) emu_loop(); else superos_loop();
}

void combined_switch_firmware(uint8_t fw) {
  wdt_reset();   // fresh 8 s budget for the flushes + lockout below
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
  // Shut USB and its PLL down CLEANLY before the reset (same sequence as the
  // d650c side's bootloader entry): the host sees a proper disconnect, and
  // the next boot's usb_init() starts the PLL from a cold, known state
  // instead of re-locking one that was running at the moment of reset (the
  // suspected trigger for the intermittent PLL-lock hang, see
  // combined_boot_guard above).
  cli();
  UDIEN = 0;
  UDCON |= (1 << DETACH);
  _delay_ms(30);                 // host must register the drop first
  USBCON = 0;
  PLLCSR = 0;
  // Hardware reset via watchdog: full peripheral reset, then the bootloader
  // (which wdt_disable()s on entry) falls through to the app, and setup()
  // reads the new select byte.
  wdt_enable(WDTO_15MS);
  for (;;) {}
}
#endif
