Sources copied from ../SuperOS-303D650cEmulator
commit 16361360d7b4da1ef580ac81b6a172dcd0f3822c
"Update binary files and improve emulation performance"
Local changes are guarded by SUPEROS_COMBINED; re-sync by copying
emu_avr.cpp and emu/*.{c,h} and re-applying the guarded blocks.

emu/tb303_rom.h (the reconstructed mask ROM) is intentionally NOT tracked:
it must never be distributed. Copy it from the emulator repo for a local
ROM-embedded `combined` build; the public release build is `combined-norom`,
where users upload their own ROM dump over SysEx.
