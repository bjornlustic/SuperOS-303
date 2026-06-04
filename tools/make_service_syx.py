Import("env")
import os, sys

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
from intelhex import IntelHex

# The SPM service lives in the empty top of the boot section. We emit a .syx
# that writes only its pages via the existing bootloader updater (cmd 0x01).
# HARD brick-safety bound: we must NEVER write a page below 0x1F4. Pages
# 0x1F0..0x1F3 hold the RUNNING bootloader (writing them bricks the device, and
# there is no ISP to recover) and lower pages hold the app / arena.
PAGE_SIZE = 256
SAFE_MIN_PAGE = 0x1F4                 # first page above the running bootloader
SAFE_MIN_ADDR = SAFE_MIN_PAGE * PAGE_SIZE  # 0x1F400
FLASH_END = 0x1FFFF

syx_path = os.path.join(env["PROJECT_DIR"], "service-install.syx")

def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    ih = IntelHex(hex_path)
    lo, hi = ih.minaddr(), ih.maxaddr()

    # HARD brick-safety guard: every byte of the image must sit above the
    # running bootloader and within flash. Refuse otherwise.
    if lo < SAFE_MIN_ADDR or hi > FLASH_END:
        sys.stderr.write(
            "make_service_syx: FATAL: image spans 0x%X..0x%X, outside the safe "
            "boot window 0x%X..0x%X. Refusing to build .syx (could overwrite the "
            "running bootloader/app and brick the device, with no ISP to "
            "recover). Check the linker section-start flags in "
            "[env:flash-service].\n" % (lo, hi, SAFE_MIN_ADDR, FLASH_END))
        env.Exit(1)
        return

    first_page = lo // PAGE_SIZE
    last_page = hi // PAGE_SIZE
    with open(out_path, "wb") as f:
        for page in range(first_page, last_page + 1):
            data = ih.tobinarray(start=page * PAGE_SIZE, size=PAGE_SIZE)
            f.write(hex2sysex.build_sysex(page, data))
        # cmd 0x02 = execute/jump-to-app (safe; no-ops if app is blank).
        f.write(bytes([0xF0, hex2sysex.MFR_ID, hex2sysex.CMD_EXECUTE, 0xF7]))
    print("make_service_syx: %s -> service-install.syx (pages 0x%X..0x%X, %d bytes)"
          % (hex_path, first_page, last_page, os.path.getsize(out_path)))

syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)
