Import("env")
import os, sys

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
from intelhex import IntelHex

# Name the update file by env so app and combined builds don't clobber each
# other (app -> app-update.syx, combined -> combined-update.syx, ...).
syx_path = os.path.join(env["PROJECT_DIR"], "%s-update.syx" % env["PIOENV"])

# First byte of the flash-as-EEPROM arena. App code must never reach it, or an
# app update would overwrite saved patterns. See FLASH_EEPROM_DESIGN.md.
# The combined SuperOS+D650C image moves the arena up; keep in sync with
# FE_ARENA_FIRST_PAGE in src/flash_eeprom.h.
if "SUPEROS_COMBINED" in env.get("CPPDEFINES", []):
    FLASH_ARENA_BASE = 0x17200
else:
    FLASH_ARENA_BASE = 0x12000

# Bundled into every update .syx so users never have to install the SPM
# service separately (it lives at 0x1FE00, above the bootloader; the
# bootloader's cmd 0x01 writes those pages the same as any others).
SERVICE_HEX = os.path.join(
    env["PROJECT_DIR"], ".pio", "build", "flash-service", "firmware.hex")

def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    ih = IntelHex(hex_path)
    maxaddr = ih.maxaddr()
    if maxaddr >= FLASH_ARENA_BASE:
        sys.stderr.write(
            "makesyx: FATAL: app reaches 0x%X, into the flash-EEPROM arena "
            "(base 0x%X). An update would clobber saved patterns. Shrink the app "
            "or move the arena base.\n" % (maxaddr, FLASH_ARENA_BASE))
        env.Exit(1)
        return
    if not os.path.exists(SERVICE_HEX):
        sys.stderr.write(
            "makesyx: FATAL: %s not found. The flash service is bundled into "
            "the update .syx; build it first: pio run -e flash-service\n"
            % SERVICE_HEX)
        env.Exit(1)
        return
    # overlap='error': the app is capped below the arena and the service sits
    # at 0x1FE00, so any overlap means a linker-script mistake.
    ih.merge(IntelHex(SERVICE_HEX), overlap='error')
    print("makesyx: %s + flash-service -> %s"
          % (hex_path, os.path.basename(out_path)))
    with open(out_path, "wb") as f:
        hex2sysex.emit(ih, f)
    print("makesyx: wrote %d bytes" % os.path.getsize(out_path))

# Command node: always rebuild the .syx regardless of whether the .hex changed.
# AlwaysBuild marks it unconditionally out-of-date every run.
# Default adds it to the targets built by plain `pio run`.
syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)
