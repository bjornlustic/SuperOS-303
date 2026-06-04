Import("env")
import os, sys

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
from intelhex import IntelHex

syx_path = os.path.join(env["PROJECT_DIR"], "app-update.syx")

# First byte of the flash-as-EEPROM arena. App code must never reach it, or an
# app update would overwrite saved patterns. See FLASH_EEPROM_DESIGN.md.
FLASH_ARENA_BASE = 0xE000

def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    maxaddr = IntelHex(hex_path).maxaddr()
    if maxaddr >= FLASH_ARENA_BASE:
        sys.stderr.write(
            "makesyx: FATAL: app reaches 0x%X, into the flash-EEPROM arena "
            "(base 0x%X). An update would clobber saved patterns. Shrink the app "
            "or move the arena base.\n" % (maxaddr, FLASH_ARENA_BASE))
        env.Exit(1)
        return
    print("makesyx: %s -> app-update.syx" % hex_path)
    with open(out_path, "wb") as f:
        hex2sysex.process(hex_path, f)
    print("makesyx: wrote %d bytes" % os.path.getsize(out_path))

# Command node: always rebuild the .syx regardless of whether the .hex changed.
# AlwaysBuild marks it unconditionally out-of-date every run.
# Default adds it to the targets built by plain `pio run`.
syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)
