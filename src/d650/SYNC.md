Sources copied from ../SuperOS-303D650cEmulator
commit 16361360d7b4da1ef580ac81b6a172dcd0f3822c
"Update binary files and improve emulation performance"
Local changes are guarded by SUPEROS_COMBINED; re-sync by copying
emu_avr.cpp and emu/*.{c,h} and re-applying the guarded blocks.

Local additions for the user-uploadable mask ROM (ported from SuperOS-606,
guarded by D650_ROM_IN_RAM / D650_ROM_EMBEDDED -- re-apply after any re-sync):
  - rom_store.h (NEW, firmware-local; not in the upstream emulator)
  - emu/ucom4.h: the D650_ROM_IN_RAM branch of the UCOM4_ROM_RD macro
  - emu_avr.cpp: gated tb303_rom include, s_rom buffer in the arena tail,
    emu_setup ROM load + embedded fallback, the DIN RomRx feed in midi_in_poll,
    the stall recovery + interpreter freeze in emu_loop, and d650_init(s_rom).
