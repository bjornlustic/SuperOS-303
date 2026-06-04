# Flash-as-EEPROM Emulation - Design Notes

Goal: stop being limited by the 4 KB hardware EEPROM. Use the AT90USB1286's
internal flash as a much larger emulated EEPROM so we can store many more
patterns / banks / tracks.

## Current state (measured on this branch)

- HW EEPROM: 4096 bytes, and the v1 2-bit layout already fills all of it
  (settings 128 + pitch 2048 + time 512 + meta 512 + tracks 832 + aux 64 = 4096).
  There is zero headroom left in real EEPROM.
- App flash: ~42 KB used (`.text`+`.data` end near 0x0A4A6).
- Bootloader: linked at 0x1F000, only 854 bytes used of the 4 KB boot section.
- Free flash between app end and bootloader: ~85 KB (roughly 0x0B000 .. 0x1F000).
- RAM: ~3000 / 8192 B used, ~5 KB free.
- Page size (SPM_PAGESIZE): 256 bytes. Erase/write granularity is one 256 B page.
- All persistence goes through a single object `EEPROMClass storage`
  (`src/persistent_settings.h`), and only four methods are ever called:
  `read`, `update`, `get`, `put`. That is the entire surface to reimplement.

## Hard AVR constraints (the stuff that makes this non-trivial)

1. SPM (the flash self-write instruction) can ONLY execute from the boot
   section (0x1F000..0x1FFFF). The app section physically cannot write flash.
   => app-time flash writes MUST call a routine that lives in the bootloader.
   Good news: the bootloader ALREADY has a correct `flash_write_page()` we can
   reuse verbatim. We just expose it at a fixed address as a callable service.

2. Flash endurance is ~10,000 erase cycles per page, vs ~100,000 for EEPROM.
   A naive "rewrite a page on every byte update" scheme would wear a page out.
   => we need wear-leveling (spread writes across many pages, erase rarely).

3. Flash is page-erase, not byte-write. You cannot flip a single byte; you
   erase a whole 256 B page (all bits -> 1) then write it once. You CAN write
   0->bits within an already-erased page, but the safe model is "append into
   an erased region, never rewrite in place."

4. Writing a flash page halts/stalls the CPU for the duration (a few ms).
   This is the same hazard the current code already manages: it defers EEPROM
   writes to idle (clock stopped) so MIDI RX is not corrupted. We flush flash
   at the same safe points.

5. No fuse changes needed. SPM-from-boot is already enabled (the bootloader
   uses it today). The emulated arena lives in RWW, far from the boot section.

## The core tension

We have ~5 KB free RAM. To grow logical storage BEYOND 4 KB we cannot keep a
full RAM mirror of the emulated EEPROM (that is what classic AVR101-style
emulation does, and it caps logical size at what fits in RAM). To actually get
"more space" the backend has to read through from flash with only a small RAM
index, not a full shadow.

Helpful fact: the data is already block-structured. `WritePatternFlat(seq,
flat_idx)` / `ReadPatternFlat(...)` are per-pattern (0..63). Tracks are
per-track. Settings are one block. So natural "block IDs" already exist.

## Recommended architecture: wear-leveled block-record store

Reserve a flash arena (e.g. 32-64 KB) in RWW below the bootloader. Treat it as
an append-only log of block records:

    record = [ block_id (1-2 B) | seq# (2 B) | payload (fixed/var) | crc (1-2 B) ]

- Write a block (pattern/track/settings): append a new record with seq#+1.
- Read a block: jump to the flash offset of its newest version (kept in a small
  RAM index) and copy it out. No scan at runtime.
- RAM index: newest-offset per block. ~73 blocks (64 patterns + 8 tracks + 1
  settings) x 2 B = ~150 B. Trivial.
- Boot: scan the arena once, build the index (newest seq# wins per block).
- Garbage collect: when the arena runs low on erased space, copy each block's
  newest version into a fresh erase region, then erase the old region. Erases
  happen only at GC time, spread across the whole arena => wear-leveled.

Why this one:
- Achieves the actual goal (more space): want 256 patterns? more block IDs +
  bigger arena. 85 KB of free flash holds thousands of pattern versions.
- Small, fixed RAM cost regardless of logical size.
- Endurance: erases only on GC, distributed across all pages.
- Contained refactor: only `persistent_settings.h` changes. `engine.h` keeps
  calling `WritePatternFlat/ReadPatternFlat` and the track/settings helpers; we
  reimplement those on top of the block store. The handful of per-byte settings
  `storage.update(...)` calls collapse into a settings-block read/modify/write.

Power-loss safety: a record is only "live" once its crc and seq# are fully
written; a torn write is ignored on the next boot scan (older version still
wins). GC writes the new region fully before erasing the old one.

## Alternative considered (and why not, for now)

Full RAM-shadow + dirty-page flush. Simplest to reason about and keeps the flat
byte API untouched, BUT it needs the whole logical image in RAM. With ~5 KB free
that caps us near the current 4 KB, so it does NOT deliver more space. Useful
only as a quick proof-of-life, not the real backend.

## Getting the SPM service in WITHOUT an ISP programmer (chosen path)

Confirmed (Microchip AVR109 + avr-libc): SPM can ONLY execute from the boot
section. The app section physically cannot self-program. So the SPM routine has
to live in the boot section. We have no ISP, so we cannot reflash the bootloader
the normal way. The trick:

- The running bootloader is only 854 bytes = boot pages 0x1F0..0x1F3 (496..499).
- The upper 12 boot pages 0x1F4..0x1FF (500..511, ~3 KB) are empty (0xFF) today.
- The EXISTING SysEx updater (`cmd 0x01` in bootload.c) writes any page it is
  told to, with no range check.

So we install a self-contained SPM service into the empty upper boot pages
(entry at 0x1FE00, currently a single page 0x1FE = 178 bytes) via the existing
updater, and NEVER touch pages 0x1F0..0x1F3. Erasing/writing an empty upper boot page halts the CPU only for
that op and resumes in the untouched lower pages, so the running bootloader is
never at risk. If an install is interrupted, the core bootloader still works, so
it is retryable. No ISP, low brick risk.

Install UX = same as a firmware update: enter bootloader (hold TAP at power-on),
send a small .syx that writes only the service pages. We can also bundle the
service pages into the normal app-update .syx so a single update both updates the
app and drops in the service (the .syx must include ONLY RWW app pages + upper
boot pages, never 0x1F0..0x1F3).

Brick-safety guard: the SPM service itself range-checks every requested page and
refuses anything outside the emulated arena (rejects all boot pages + app code).
So normal pattern saving can never brick the device. The only remaining brick
vector is a corrupt manual .syx that targets 0x1F0..0x1F3 - which is a
preexisting property of the current updater, not something this change adds. Our
tooling simply never emits those pages.

## Lock bits

App calling into the boot section is instruction execution, not LPM/SPM across
the boundary, so it is allowed regardless of BLB bits. Default lock bits (0xFF,
unlocked - this project never sets them) impose no restriction. Verify lockbits
are unprogrammed before relying on this.

## Decisions (locked in)

1. No ISP. Use the upper-empty-boot-pages install path above.
2. Backend = wear-leveled block-record store.
3. Migration = start clean. New firmware initializes an empty flash arena; old
   HW-EEPROM patterns are NOT carried over. (HW EEPROM code can stay compiled out
   or behind a flag for fallback, but is not the live store.)

## Proposed phases

- Phase 0 - prove flash writes from the running app  [IMPLEMENTED - see below]
  - Self-contained SPM service (`src/flash_service/flash_service.c`), entry
    trampoline pinned at 0x1FE00, with an arena page range-guard.
  - `[env:flash-service]` builds it and emits `service-install.syx` (writes only
    the service pages; hard-refuses anything below page 0x1F4).
  - App side: `src/flash_store.h` - `flash_write_page(page, buf)` /
    `flash_read(addr)`; `flash_write_page` no-ops with FLASH_ERR_NO_SERVICE if
    the service is not installed, so it never ICALLs into garbage.
  - Arena overlap guard: `makesyx.py` fails the app build if app end >= 0x10000.
  - Self-test: `[env:app-flashtest]` runs `flash_selftest()` at boot (write +
    read-back the last arena page) and shows PASS/FAIL on the keypad LEDs.

- Phase 1 - block-record store behind the existing API  [IMPLEMENTED + host-tested]
  - `src/flash_eeprom.h`: two-bank, append-only log of 256-byte page records;
    RAM index (newest page per block); CRC16 per record; generation-stamped so
    interrupted-GC leftovers are ignored; power-safe GC (spare bank fully written
    + headered before the old bank is retired). Host-tested in
    `tools/flash_eeprom_hosttest.cpp` (write/read/overwrite/persist, GC, torn
    write, crash-before-GC-header, crash-after-GC-header, variable sizes).
  - `src/flash_persist.h`: device page hooks (SPM service + far reads), the
    single `g_flash` instance, block-id map (0..63 patterns, 64..71 tracks,
    72 settings).
  - `persistent_settings.h` / `engine.h`: `WritePatternFlat/ReadPatternFlat`,
    `LoadTrack/SaveTrack`, settings, and the track-format check now go through
    `g_flash`. `main.cpp` mounts it (`flash_persist_begin()`) before
    `engine.Load()`. The hardware EEPROM path is fully removed.
  - Blocks: pattern = 48 B, track = 104 B, settings = 22 B. Each = one page.
    Bank = 112 pages (1 header + 111 records); ~73 live blocks today, so ~38
    writes between GCs. NEEDS ON-HARDWARE VALIDATION (see below).
  - Wear note: `Engine::Save(-1)` rewrites all 16 patterns of the group on each
    stop, so ~16 page-appends/save, GC every ~7 saves. Spread over 224 pages x
    ~10k erase ~= 140k saves of lifetime (comparable to the old per-byte EEPROM).
    Per-pattern dirty tracking would cut this; deferred to a later pass.

## Phase 1 on-hardware test

The SPM service must already be installed (Phase 0). Flash the normal app
(`pio run -e app` -> app-update.syx) and:
1. Edit/create a pattern, then STOP the clock (triggers the flash save).
2. Power-cycle. The pattern should still be there (loaded from flash).
3. Repeat across group/bank switches and tracks.
A debug build (`pio run -e app-debug`, USB serial) prints
`flash mount=1 bank=.. gen=.. append=..` at boot to confirm the mount.
If flash is unavailable (service missing), the app still runs but does not
persist; nothing hangs.

- Phase 2 - grow capacity
  - Add banks/patterns/tracks now that storage is no longer 4 KB-bound; bump the
    SysEx + web-editor pattern index range to match.

## Phase 0 build + on-hardware test

    pio run -e flash-service     # -> service-install.syx (writes page 0x1FE only)
    pio run -e app-flashtest     # -> app-update.syx (self-test build)

On hardware (no ISP needed, all over MIDI SysEx):
1. Power on holding TAP_NEXT -> enters the existing bootloader.
2. Send `service-install.syx` (SysEx Librarian / MIDI-OX, throttled). This drops
   the SPM service into page 0x1FE. The running bootloader is never written.
3. Send `app-update.syx` (the app-flashtest build).
4. Power-cycle normally. The self-test runs at boot:
   - All chromatic key LEDs solid  = PASS (write + read-back verified).
   - ACCENT+SLIDE blink, UP lit     = service present but write/verify FAILED.
   - ACCENT+SLIDE blink, DOWN lit   = SPM service NOT installed (redo step 2).
   With a USB-serial (debug) build it also prints the result.

Note: any `app*` build overwrites `app-update.syx`. Rebuild `-e app` to restore
the normal (non-self-test) update file.

## Key implementation facts (verified)

- avr:51 function pointers are WORD addresses used verbatim (no shift by the
  compiler). To call byte 0x1FE00 the app uses pointer value 0xFF00. The service
  entry is a 2-word JMP whose first opcode word is 0x940C - used as the
  "service present" probe via `pgm_read_word_far`.
- `boot.h` picks the EXTENDED (32-bit, RAMPZ-setting) page macros because
  FLASHEND (0x1FFFF) > USHRT_MAX, so writes to the >64 KB arena are addressed
  correctly. The service restores RAMPZ=0 before returning.
- PlatformIO's atmelavr builder forwards `build_flags` only to compilation, not
  the link. `-nostartfiles`/`-nostdlib` are therefore applied via a pre-script
  (`tools/flash_service_link.py`) so no crt0 / `main` is pulled in.
- Latent pre-existing bug (NOT touched here): `main.cpp` `jumptoboot()` casts
  `0x1F000` straight to a function pointer, which avr-gcc treats as a WORD
  address -> it actually ICALLs byte 0x1E000, not the bootloader at 0x1F000. It
  appears to "work" only by sliding through erased flash. Correct value would be
  0xF800. Worth fixing separately.

## Arena sizing (initial)

- App currently ends ~0x0A4A6. Reserve app headroom to 0x10000 (64 KB) and put
  the arena at 0x10000..0x1DFFF = 56 KB (224 pages), all in RWW (below the
  0x1E000 NRWW edge so the boot service keeps running during arena writes).
- 56 KB / 4 KB EEPROM = 14x the old capacity, before any wear-leveling overhead.
  Tune base/size once Phase 0 confirms the mechanism.
