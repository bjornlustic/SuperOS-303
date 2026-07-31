#!/usr/bin/env python

import sys
from intelhex import IntelHex

PAGE_SIZE = 256
SYSEX_START = 0xF0
SYSEX_END = 0xF7
MFR_ID = 0x7D

CMD_WRITE_PAGE = 0x01
CMD_EXECUTE = 0x02

def pack_7bit(data):
    """
    Convert 8-bit data into MIDI 7-bit packed format.
    7 bytes → 8 bytes
    """
    out = []
    i = 0

    while i < len(data):
        chunk = data[i:i+7]
        msb = 0

        for bit, b in enumerate(chunk):
            if b & 0x80:
                msb |= (1 << bit)

        out.append(msb)

        for b in chunk:
            out.append(b & 0x7F)

        i += 7

    return out


def build_sysex(page, page_data):

    encoded = pack_7bit(page_data)
    length = len(encoded)
    checksum = 0
    for b in page_data:
        checksum ^= b

    msg = [
        SYSEX_START,
        MFR_ID,
        CMD_WRITE_PAGE,
        (page >> 7) & 0x7F,
        page & 0x7F,
        (length >> 7) & 0x7F,
        length & 0x7F,
        (checksum >> 4) & 0x0F,
        checksum & 0x0F
    ]

    msg.extend(encoded)
    msg.append(SYSEX_END)

    return bytes(msg)


# 0x1F000..0x1FDFF is reserved for the RUNNING bootloader (currently ends
# ~0x1F7B3; the flash service owns 0x1FE00+). Writing any of it from the
# bootloader bricks the device (no ISP to recover). Never emit those pages.
BOOT_FIRST_PAGE = 0x1F000 // PAGE_SIZE
BOOT_LAST_PAGE = 0x1FDFF // PAGE_SIZE


def emit(ih, outbuf):
    """Emit write-page messages for only the pages the image actually
    covers. Gap pages (e.g. between the app and the 0x1FE00 flash service)
    are never sent: writing them as 0xFF would erase the flash arena and
    the running bootloader."""
    pages = set()
    for start, stop in ih.segments():
        pages.update(range(start // PAGE_SIZE, (stop - 1) // PAGE_SIZE + 1))

    for page in sorted(pages):
        if BOOT_FIRST_PAGE <= page <= BOOT_LAST_PAGE:
            sys.stderr.write(
                "hex2sysex: skipping page 0x%X (running bootloader; writing "
                "it over MIDI would brick the device)\n" % page)
            continue
        data = ih.tobinarray(start=page * PAGE_SIZE, size=PAGE_SIZE)
        outbuf.write(build_sysex(page, data))

    outbuf.write(bytes([SYSEX_START, MFR_ID, CMD_EXECUTE, SYSEX_END]))


def process(in_file, outbuf):
    emit(IntelHex(in_file), outbuf)


def main():

    if len(sys.argv) < 2:
        print("usage: hex2sysex firmware.hex")
        return

    process(sys.argv[1], sys.stdout.buffer)

if __name__ == "__main__":
    main()
