#!/usr/bin/env python3
"""
SuperOS-303: pattern SysEx must not trigger the MIDI bootloader flash path.

bootload.c process_sysex() requires:
  - data[0] == 0x7D
  - data[1] == 0x01 AND len > 9 AND packed_len + 8 <= len (flash page write)
  - OR data[1] == 0x02 (jump app)

Pattern protocol uses cmd in {0x10, 0x11, 0x12, 0x13, 0x14} — never 0x01/0x02.
This script asserts representative messages are rejected by the same gate logic.
"""


def bootloader_would_attempt_flash(data):
    """Mirror bootload.c branches that touch flash (cmd 0x01 path)."""
    if len(data) < 2:
        return False
    if data[0] != 0x7D:
        return False
    cmd = data[1]
    if cmd == 0x02:
        return False  # jump only
    if cmd == 0x01 and len(data) > 9:
        packed_len = (data[4] << 7) | data[5]
        if packed_len + 8 > len(data):
            return False
        return True  # would call decode_7bit + flash_write_page
    return False


def test_pattern_cmds_safe():
    # Inner body (no F0/F7), as in firmware after stripping sysex bounds
    samples = [
        bytes([0x7D, 0x10, 0]),
        bytes([0x7D, 0x10, 15]),
        bytes([0x7D, 0x13]),
        bytes([0x7D, 0x14, 0]),
        bytes([0x7D, 0x12, 0, 0, 0]) + bytes(150),  # short inner, still cmd != 01
    ]
    for s in samples:
        assert not bootloader_would_attempt_flash(s), f"false positive: {s[:8]!r}..."


def test_real_bootloader_shape():
    # Minimal valid-looking 0x01 header (checksum will fail in real bootloader)
    page = 0
    packed_len = 8
    inner = bytes([
        0x7D,
        0x01,
        (page >> 7) & 0x7F,
        page & 0x7F,
        (packed_len >> 7) & 0x7F,
        packed_len & 0x7F,
        0,
        0,
    ]) + bytes(8)
    assert bootloader_would_attempt_flash(inner)


if __name__ == "__main__":
    test_pattern_cmds_safe()
    test_real_bootloader_shape()
    print("verify_bootloader_safe: OK")
