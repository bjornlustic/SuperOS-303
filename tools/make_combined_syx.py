#!/usr/bin/env python3
# Build a self-contained release .syx for the combined firmware: the app pages
# PLUS the SPM flash-service page, all written through the standard SysEx
# bootloader (bootload.c cmd 0x01, trailing 0x02 = run app). The target CPU
# only needs the bootloader present (hold TAP_NEXT at power-on, or send
# F0 7D 4A F7 to a running app).
#
# The .syx is SPARSE: only pages that exist in the images are sent, so the
# pattern arena is never touched and the running bootloader is never written
# (hex2sysex.process would emit 0xFF fill pages across both -- do not use it
# for a merged image).
#
# Usage (after `pio run -e combined -e flash-service`):
#   python3 tools/make_combined_syx.py combined-v0.9.9.syx
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hex2sysex
from intelhex import IntelHex

PAGE = 256
ARENA_FIRST_PAGE   = 0x140  # combined build; keep in sync with flash_eeprom.h
BOOT_MIN_SAFE_PAGE = 0x1F4  # pages below hold the RUNNING bootloader
FLASH_LAST_PAGE    = 0x1FF

APP_HEX = ".pio/build/combined/firmware.hex"
SVC_HEX = ".pio/build/flash-service/firmware.hex"

def load(path, lo_page, hi_page):
    ih = IntelHex(path)
    pages = set()
    for (a, b) in ih.segments():
        pages.update(range(a // PAGE, (b - 1) // PAGE + 1))
    for p in pages:
        if not (lo_page <= p <= hi_page):
            sys.exit("FATAL: %s contains page 0x%X outside the safe window "
                     "0x%X..0x%X; refusing." % (path, p, lo_page, hi_page))
    return ih, sorted(pages)

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "combined-update-full.syx"
    app, app_pages = load(APP_HEX, 0, ARENA_FIRST_PAGE - 1)
    svc, svc_pages = load(SVC_HEX, BOOT_MIN_SAFE_PAGE, FLASH_LAST_PAGE)
    with open(out_path, "wb") as f:
        for ih, pages in ((app, app_pages), (svc, svc_pages)):
            for p in pages:
                f.write(hex2sysex.build_sysex(p, ih.tobinarray(start=p * PAGE, size=PAGE)))
        f.write(bytes([0xF0, hex2sysex.MFR_ID, hex2sysex.CMD_EXECUTE, 0xF7]))
    print("wrote %s: %d bytes, %d app pages (top 0x%X) + %d service pages (0x%X..)"
          % (out_path, os.path.getsize(out_path), len(app_pages),
             app_pages[-1] * PAGE + PAGE - 1, len(svc_pages), svc_pages[0] * PAGE))

if __name__ == "__main__":
    main()
