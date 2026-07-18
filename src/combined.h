// combined.h -- combined SuperOS-303 + D650C-emulator build (SUPEROS_COMBINED).
//
// One image, one firmware active per boot, selected by a byte in the internal
// EEPROM and switchable from either firmware's FUNCTION+CLEAR config menu.
// SuperOS keeps its patterns in the flash arena; the d650c side keeps ALL of
// its state (uPD444 pattern store + settings) in the 4 KB internal EEPROM,
// which SuperOS never touches.
//
// Internal EEPROM map (AT90USB1286, 4096 bytes):
//   0x000            firmware select: 0/0xFF = SuperOS, 1 = D650C
//   0x001            d650 area magic (EE_EMU_MAGIC_VAL when initialized)
//   0x002            mask-ROM magic (EE_ROM_MAGIC_VAL when a valid ROM is stored)
//   0x003..0x004     mask-ROM sum16 (LE) over the 2048 ROM bytes
//   0x008..0x017     d650 settings blob (EMU_SETTINGS_LEN = 16)
//   0x020..0x61F     d650 uPD444 packed pattern store (D650_EXT_BYTES = 1536)
//   0x620..0xE1F     user-loaded D650C mask ROM (2048 bytes; the D650_ROM_IN_RAM
//                    build lets the user upload their own dump via SysEx over
//                    USB-C or DIN -- see src/d650/rom_store.h)
//   0xE20..0xFFF     free
#pragma once
#include <stdint.h>

#define EE_FW_SELECT   ((uint8_t *)0x000)
#define EE_EMU_MAGIC   ((uint8_t *)0x001)
#define EE_ROM_MAGIC   ((uint8_t *)0x002)
#define EE_ROM_SUM     ((uint8_t *)0x003)
#define EE_EMU_SETTINGS ((uint8_t *)0x008)
#define EE_EMU_PATT    ((uint8_t *)0x020)
#define EE_ROM_DATA    ((uint8_t *)0x620)

static constexpr uint8_t FW_SUPEROS = 0;   // also 0xFF (virgin EEPROM)
static constexpr uint8_t FW_D650    = 1;
// Bump when d650 settings defaults/semantics change (0x67: pitch_base 25,
// factory pitch standard). Mismatch re-defaults settings AND re-seeds the store.
static constexpr uint8_t EE_EMU_MAGIC_VAL = 0x67;
// Written LAST by rom_save (invalidated first) so a torn ROM upload never
// validates at boot.
static constexpr uint8_t EE_ROM_MAGIC_VAL = 0x6D;

// Select the other firmware and reboot through the bootloader's app entry.
// Defined in combined.cpp; callable from either firmware's config menu.
void combined_switch_firmware(uint8_t fw);

// RAM overlay: only one firmware runs per boot, so the two biggest per-side
// objects (SuperOS Engine, d650 machine incl. the 1536-byte uPD444 store)
// share this arena. Sized in combined.cpp to max(sizeof(Engine),
// sizeof(d650_host)); zeroed BSS at every boot, exactly like the statics it
// replaces (Engine is placement-new'ed in superos_setup, d650_init memsets).
extern uint8_t g_fw_arena[];
