# Flash-as-EEPROM - Implementation Reference

This document describes the flash-based EEPROM replacement for SuperOS-303 as it
is actually built and running. It is written as a handoff for an implementer
porting the same scheme into another codebase. It covers what the subsystem
adds, what it changes in the surrounding firmware, how each piece works, and how
to install it on hardware that has no ISP programmer.

Target MCU: AT90USB1286 (Teensy++ 2.0), 128 KB flash, 8 KB SRAM, 256-byte flash
page, ~10k erase cycles/page. Bootloader already present at 0x1F000.

---

## 1. Why

The 4 KB hardware EEPROM was full and capped how many patterns/banks/tracks
could persist. This subsystem moves all persistence into a 64 KB arena carved
out of internal flash, with wear-leveling and power-loss safety. Logical storage
is no longer bound by RAM size: only a small RAM index is kept; block payloads
are read straight from flash.

The hardware EEPROM path is fully removed. There is no migration of old EEPROM
contents - first boot formats a clean arena.

---

## 2. What was added

New files:

| File | Role |
|---|---|
| `src/flash_service/flash_service.c` | SPM flash-write service. Tiny freestanding routine that lives in the boot section and erases+writes one 256-byte page. The only code that can physically write flash. |
| `src/flash_store.h` | App-side caller for the SPM service + arena bounds + service-present probe. `flash_write_page()`, `flash_read()`. |
| `src/flash_eeprom.h` | Wear-leveled, two-bank, append-only block-record store. Platform-independent (depends only on `<stdint.h>`/`<string.h>`), so it host-unit-tests. |
| `src/flash_persist.h` | Device glue: page program/read hooks, the single `g_flash` instance, the logical block-id map and serialized block sizes. |
| `tools/flash_service_link.py` | PlatformIO pre-link script: applies `-nostartfiles -nostdlib -fno-lto` to the service build (the builder otherwise only forwards build_flags to compilation). |
| `tools/make_service_syx.py` | PlatformIO post-build script: turns the service `.hex` into `service-install.syx`, with a hard brick-safety bound (refuses any page below 0x1F4). |

New build environment: `[env:flash-service]` in `platformio.ini`.

---

## 3. What was changed

| File | Change |
|---|---|
| `src/persistent_settings.h` | `EEPROMClass storage` and all hardware-EEPROM access removed. `WritePatternFlat/ReadPatternFlat`, `PersistentSettings::Load/Save`, and the track-format check now go through `g_flash`. Settings serialize into a 22-byte block. |
| `src/engine.h` | `Engine::Load/Save`, `LoadTrack/SaveTrack`, and the track-format wipe call `g_flash.read/write` instead of EEPROM. `static_assert(TRACK_BYTES == FB_TRACK_LEN)` ties the track block size to the engine layout. |
| `src/main.cpp` | `flash_persist_begin()` is called in `setup()` before `engine.Load()`. Pattern save fires on a transport-stop edge from any clock source (internal RUN, MIDI Stop, DIN sync stop), not only `RUN.falling() && !midi_clk`. |
| `src/midi.cpp` | SysEx pattern writes update RAM only (never inline flash); they mark a per-pattern dirty bitmap that is flushed incrementally when idle. |
| `tools/makesyx.py` | App build fails if the app image reaches the arena base (0xE000); an app update could otherwise overwrite saved patterns. |
| `platformio.ini` | Added `[env:flash-service]`; `[env:app]` arena guard via `makesyx.py`. |

---

## 4. How it works

Three layers, bottom to top:

```
flash_service.c   (boot section)   erase+write one page, arena-guarded
      ^ ICALL
flash_store.h     (app)            flash_write_page / flash_read / probe
      ^
flash_eeprom.h    (app)            block-record store: index, append, GC
      ^
flash_persist.h + persistent_settings.h + engine.h   block-id map, serialize
```

### 4.1 SPM flash-write service (`flash_service.c`)

The SPM instruction (flash self-write) can ONLY execute from the boot section;
app code physically cannot write flash. There is no ISP, so the running
bootloader cannot be reflashed. The service is therefore installed into the
empty top of the boot section, above the running bootloader, using the
bootloader's own SysEx page-writer.

- Entry trampoline pinned at byte `0x1FE00` (`.service_entry` section). It is a
  single `JMP flash_service_impl`, leaving the argument registers untouched so
  they pass straight through. The implementation is linked just after it
  (`.text` at `0x1FE40`). The whole image occupies page `0x1FE` (~180 bytes) and
  is always kept above page `0x1F3` (the running bootloader is pages
  `0x1F0..0x1F3`).
- ABI (avr-gcc default): `page` in r24:r25, `buf` pointer in r22:r23, status in
  r24 (0 = ok). avr:51 function pointers are WORD addresses used verbatim, so to
  call byte `0x1FE00` the app uses pointer value `0xFF00`.
- `flash_service_impl(page, buf)`:
  1. Range-check: reject any `page` outside `[0xE0, 0x1DF]` (return 1). This is
     the hardware brick-safety guard - the service will never erase app code,
     the boot section, or anything outside the arena, no matter what the app
     asks.
  2. `cli()` (interrupt vectors live in RWW and are unreadable mid-write),
     `eeprom_busy_wait()`.
  3. `boot_page_erase` -> `boot_page_fill` (word at a time) -> `boot_page_write`
     -> `boot_spm_busy_wait` -> `boot_rww_enable`.
  4. `RAMPZ = 0` (the extended `boot_page_*` macros leave RAMPZ set; restore it
     for plain LPM), restore SREG, return 0.

`boot.h` picks the EXTENDED (32-bit, RAMPZ-setting) page macros automatically
because `FLASHEND (0x1FFFF) > USHRT_MAX`, so writes to the >64 KB arena are
addressed correctly.

The arena is entirely in RWW (Read-While-Write, below the `0x1E000` NRWW edge),
so erasing/writing an arena page does not stall instruction fetch from the boot
section: the service keeps running during the write.

### 4.2 App-side access (`flash_store.h`)

- `flash_service_present()` probes the entry: `pgm_read_word_far(0x1FE00) ==
  0x940C` (the first opcode word of `JMP`). If the service is not installed this
  is false.
- `flash_write_page(page, buf)`: returns `FLASH_ERR_NO_SERVICE (0xFE)` if absent
  (never ICALLs into garbage), else ICALLs the service via the `0xFF00` word
  pointer. Halts the CPU a few ms; call only when the clock is stopped.
- `flash_read(addr)`: `pgm_read_byte_far(addr)` (far read, addresses > 64 KB).

Arena constants here mirror the service and the store:
`FLASH_ARENA_BASE = 0xE000`, `FLASH_ARENA_END = 0x1E000`, pages `0xE0..0x1DF`.

### 4.3 Block-record store (`flash_eeprom.h`)

A `FlashEeprom` object presents `read(id, dst, cap)` / `write(id, src, len)` over
fixed logical block IDs. It is wired to the device by `begin(programFn, readFn)`
(the two hooks from `flash_persist.h`).

Arena geometry (one page = 256 bytes):

| Constant | Value | Meaning |
|---|---|---|
| `FE_ARENA_FIRST_PAGE` | `0xE0` | first arena page |
| `FE_NUM_BANKS` | 2 | two equal banks |
| `FE_BANK_PAGES` | 128 | pages per bank (256 total = 64 KB) |
| `FE_RECORD_PAGES` | 127 | usable record pages per bank (page 0 is the bank header) |
| `FE_PAGE` | 256 | page size |
| `FE_MAX_BLOCKS` | 96 | logical block-id space |
| `FE_REC_HDR` | 13 | record header bytes |
| `FE_MAX_PAYLOAD` | 243 | `FE_PAGE - FE_REC_HDR` |
| `FE_REC_TAG` | `0xA5` | record tag |

One record per page (a logical block uses exactly one page regardless of payload
size). Two banks: at any time one is active (live data), the other is spare (GC
target).

**Record page layout** (page 0 of a bank is a header, pages 1..127 are records):

```
[0]      tag = 0xA5
[1]      block_id
[2..5]   seq     (u32, little-endian)   monotonic, assigned per append
[6]      len     (payload byte count)
[7..10]  gen     (u32, little-endian)   generation of the bank it belongs to
[11..12] crc16                          CRC-16/CCITT over bytes [1..10] + payload
[13..]   payload (len bytes)
```

**Bank header page layout** (relative page 0 of each bank):

```
[0..3]   'F','E','0','1'
[4..7]   gen (u32, little-endian)
[8..9]   crc16 over bytes [0..7]
```

**RAM state:** `active_bank_`, `active_gen_`, `seq_` (highest seq seen/written),
`append_` (next free record page), `index_[FE_MAX_BLOCKS]` (newest record page
per block, 0 = none), and a single `scratch_[256]` page buffer. Total RAM cost
is ~96 + 256 bytes - independent of how much logical data is stored.

**mount()** (run once at boot):
1. Read both bank headers. If neither validates, `format()`.
2. If both validate (an interrupted GC left two headers), pick the higher `gen`
   as active and invalidate the other bank's header.
3. Scan the active bank's record pages (1..127). For each page passing
   `parse_record` against the active gen, keep the newest `seq` per block in
   `index_`. Track the highest used page; `append_ = highest + 1`.

`parse_record` rejects a page unless: tag == `0xA5`, `len <= FE_MAX_PAYLOAD`,
`gen` matches the active generation, and the stored CRC matches. This is what
makes torn writes and stale (previous-generation) records invisible.

**write(id, src, len):**
1. If `append_ > FE_RECORD_PAGES` (active bank full), run `gc()`.
2. `append_record`: assign `seq_+1`, build the record page in `scratch_`,
   program it, then read it back and re-parse. If the read-back fails (torn or
   bad page), advance past it and retry once. On success update `index_[id]` and
   `append_`.

**read(id, dst, cap):** if `index_[id]` is 0, return 0 (never written). Else read
that page, parse it against the active gen, `memcpy` up to `cap` bytes, return the
stored length.

**gc()** (compaction, power-safe):
1. Snapshot the live index.
2. Copy each live block's newest record into the spare bank with `gen+1` and a
   fresh `seq`. (If the live block count would overflow `FE_RECORD_PAGES`, fail.)
3. Write the spare bank header with `gen+1`. **This header write is the commit
   point** - it is written only after every live record is safely in the spare
   bank.
4. Invalidate the old bank's header, then switch `active_bank_`/`active_gen_`/
   `index_`/`append_` to the spare.

If power is lost before step 3, the old bank is still intact and wins on the next
mount. If lost between step 3 and step 4, mount sees two valid headers and picks
the higher gen (the new bank), then cleans up the stale old header.

**format():** write bank 0 header with gen 1, invalidate bank 1, clear the index.

**Wear-leveling:** erases happen only during GC, and GC alternates between the
two banks, so erase load is spread across all 256 pages rather than hammering a
single page on every save.

### 4.4 Block-id map and serialized sizes (`flash_persist.h`)

```
0..63   patterns   (flat index = bank*16 + pat, banks 0..3 x 16 patterns)
64..71  tracks 0..7
72      settings
```

`FE_MAX_BLOCKS = 96` leaves headroom over the 73 live blocks in use today.

| Block | Constant | Bytes | Composition |
|---|---|---|---|
| Pattern | `FB_PATTERN_LEN` | 92 | `pitch[64]` + `time_data[16]` + `METADATA_SIZE (12)` |
| Track | `FB_TRACK_LEN` | 104 | `p_chain_packed[32]` + `t_chain_last[8]` + `t_chain_transpose[64]` |
| Settings | `FB_SETTINGS_LEN` | 22 | signature[16] + midi_channel + flags + direction + thru + led_brightness + track_format |

`FB_PATTERN_LEN` is derived from `MAX_STEPS (64)`:
`MAX_STEPS + MAX_STEPS/4 + METADATA_SIZE = 64 + 16 + 12 = 92`. `FB_TRACK_LEN` is
asserted equal to the engine's `TRACK_BYTES` so the two never drift.

The device hooks:

```c
fe_dev_program(abs_page, buf) -> flash_write_page(abs_page, buf)
fe_dev_read(abs_page, buf)    -> pgm_read_byte_far((abs_page<<8)+i) for i in 0..255
```

`flash_persist_begin()` calls `g_flash.begin(fe_dev_program, fe_dev_read)`. It
returns false if flash is unavailable (service missing); the app then runs
without persistence and nothing hangs.

### 4.5 Persistence layer (`persistent_settings.h`, `engine.h`)

- `PersistentSettings::Load/Save` read/write block 72. On a fresh arena `read`
  returns 0; `Load` zeroes the signature so `Validate()` fails and the engine
  runs its clean-init path (clears all patterns, writes defaults to all banks,
  re-initializes tracks).
- `WritePatternFlat/ReadPatternFlat(seq, flat_idx)` serialize a `Sequence` to/from
  block `0 + flat_idx`. Missing block -> in-RAM pattern is blanked with
  `length = 0`, which `Engine::Load` promotes to `SetLength(8)`.
- `Engine::LoadTrack/SaveTrack` map to blocks `64 + track`. A 0xFF-filled block
  (written by the track-format wipe) is treated as "fresh" and initializes the
  chain to empty + transpose to no-op.
- Track-format versioning: `PersistentSettings::track_format` vs
  `kTrackFormatVersion`. On mismatch, all 8 track blocks are overwritten with
  0xFF and the version is bumped. Patterns are unaffected.

### 4.6 Write discipline (when flash is actually written)

Flash page writes halt the CPU for a few ms, which would corrupt MIDI RX and
glitch audio mid-playback. So writes are deferred to safe points:

- **Hardware edits:** mark RAM dirty during editing; `Engine::Save()` /
  `SaveTrack()` run on the transport-stop edge (RUN button, MIDI Stop, or DIN
  sync stop - any source) and on the relevant dial-mode change.
- **Web/SysEx edits:** SysEx handlers (`0x12/0x16/0x18/0x1B`) update RAM only and
  set a per-pattern dirty bit; `midi_flush_pending_pattern_saves()` writes at
  most one pattern per call, only while the clock is stopped and only after a 2 s
  quiet window so a burst of edits coalesces into one write per pattern.
  (`0x19` step-lock is RAM-only and never marks dirty.)

---

## 5. Per-pattern dirty tracking (write-amplification reduction)

> This section describes work in progress being added now. It refines the save
> path; the rest of the subsystem above is stable.

### Problem

`Engine::Save(-1)` currently rewrites **all 16 patterns** of the active group on
every transport stop, even if only one pattern changed. At ~16 page appends per
save, the active bank (127 record pages) fills in ~7 saves, forcing a GC every
~7 saves. That is heavy write amplification and burns erase cycles fast.

Note there are already two independent dirty flags in the codebase that this
change consolidates:
- `Engine::stale` - a single coarse "something in the group changed" bool.
- `midi.cpp s_pat_dirty_mask` (`uint16_t`) - a per-pattern bitmap, but only for
  web/SysEx edits, flushed by `midi_flush_pending_pattern_saves()`.

### Design

Add one authoritative per-pattern dirty bitmap on the Engine and route every
pattern edit - hardware and web - through it.

1. **Engine state.** Add `uint16_t pat_dirty_ = 0;` (bit i = pattern i in the
   active group needs writing). Add a helper:

   ```cpp
   void MarkPatternDirty(uint8_t idx) { pat_dirty_ |= uint16_t(1u << (idx & 0x0F)); }
   ```

   Keep `stale` as a derived convenience (`stale = pat_dirty_ != 0`) or drop it
   in favor of `pat_dirty_ != 0` at the call sites that test it.

2. **Mark the right pattern.** Replace each `stale = true;` in `engine.h` with
   `MarkPatternDirty(p_select)` for the edit ops that operate on the active
   pattern (`get_sequence()` == `pattern[p_select]`). The few cross-pattern ops
   mark their explicit target instead:
   - `ClearPattern(idx)` -> `MarkPatternDirty(idx)`.
   - `import_pattern_blob(idx, ...)` (clipboard paste, web set-pattern) ->
     `MarkPatternDirty(idx)`.
   - Whole-group operations that touch every pattern -> set `pat_dirty_ = 0xFFFF`.

3. **Save only what changed.**

   ```cpp
   void Save(int pidx = -1) {
     if (pidx >= 0) {
       if (pat_dirty_ & (1u << (pidx & 0x0F))) {
         WritePattern(pattern[pidx], pidx, group_);
         pat_dirty_ &= ~uint16_t(1u << (pidx & 0x0F));
       }
       return;
     }
     for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
       if (pat_dirty_ & (1u << i)) {
         WritePattern(pattern[i], i, group_);
         pat_dirty_ &= ~uint16_t(1u << i);
       }
   }
   ```

4. **Group switch.** `apply_pending_group()` reloads all 16 patterns from flash
   into RAM, so any unsaved edits to the outgoing group are discarded by design.
   Clear the mask there: `pat_dirty_ = 0;` after the reload. (This preserves the
   current "edits not stopped-through are lost on group switch" behavior.)

5. **Merge the MIDI mask.** Have the SysEx handlers call
   `g_eng->MarkPatternDirty(pat)` instead of the private `s_pat_dirty_mask`, and
   have `midi_flush_pending_pattern_saves()` consume `engine.pat_dirty_` (still
   one pattern per tick, clock-stopped, 2 s quiet window). Web edits and hardware
   edits then share one source of truth, and the active-group RAM that both
   mutate is persisted exactly once per changed pattern. (Web edits always
   address patterns 0..15 within the active group, the same `engine.pattern[]`
   RAM hardware edits use, so a single mask is correct.)

### Effect

A save now writes only the patterns that actually changed (often 1), cutting
appends-per-save from 16 to N-changed and pushing GC frequency down by the same
factor - directly extending flash lifetime.

---

## 6. Build environments (`platformio.ini`)

| Env | Target | Output | Purpose |
|---|---|---|---|
| `app` | Teensy++ 2.0 | `app-update.syx` | Main firmware. `makesyx.py` fails the build if the app image reaches `0xE000` (would clobber the arena). |
| `app-debug` | Teensy++ 2.0 | - | `-DUSB_SERIAL -DDEBUG=1`; prints `flash mount=...` over USB serial at boot. |
| `flash-service` | AT90USB1286 | `service-install.syx` | The SPM service. Freestanding (no crt0/main); linked at `0x1FE00`. `make_service_syx.py` refuses any page below `0x1F4`. |
| `bootloader` | AT90USB1286 | - | Pre-existing SysEx bootloader at `0x1F000` (unchanged; flashed via ISP only). |

Linker flags for `[env:flash-service]`:
```
-Wl,--entry=flash_service_entry
-Wl,--section-start=.service_entry=0x1FE00
-Wl,--section-start=.text=0x1FE40
```
plus `-nostartfiles -nostdlib -fno-lto` applied via `tools/flash_service_link.py`
(PlatformIO's atmelavr builder forwards `build_flags` only to compilation, not
the link; LTO is off so the trampoline's inline `jmp` resolves to a plain symbol).

Build commands:
```bash
pio run -e flash-service     # -> service-install.syx (writes page 0x1FE only)
pio run -e app               # -> app-update.syx (normal firmware)
```

---

## 7. Installation (no ISP needed - all over MIDI SysEx)

The SPM service must be installed once before flashed firmware can persist. Both
files go in through the existing bootloader's SysEx page-writer.

**One-time service install:**
1. Build it: `pio run -e flash-service` -> `service-install.syx`.
2. Power on the unit holding **TAP_NEXT** -> enters the existing bootloader.
3. Send `service-install.syx` via a throttled SysEx sender (SysEx Librarian,
   MIDI-OX). This writes only the service page(s) (0x1FE) and never touches the
   running bootloader (pages 0x1F0..0x1F3). An interrupted install is harmless
   and retryable - the core bootloader still works.

**Firmware install / update (each time):**
4. Build it: `pio run -e app` -> `app-update.syx`.
5. Power on holding **TAP_NEXT** -> bootloader.
6. Send `app-update.syx` (throttled).
7. Power-cycle normally.

**Verify persistence:** create/edit a pattern, STOP the clock (this is the save
trigger), power-cycle, confirm it is still there. An `app-debug` build prints
`flash mount=1 bank=.. gen=.. append=..` over USB serial at boot.

If the service is missing, the app still boots and runs but does not persist
(no hang). Re-do the service install.

---

## 8. Brick-safety (three independent guards)

1. **Service range-check (hardware-level).** `flash_service_impl` refuses any
   page outside `[0xE0, 0x1DF]`. Even a buggy app can never erase app code, the
   boot section, or anything outside the arena.
2. **App-build arena guard.** `makesyx.py` fails `-e app` if the app image
   reaches `0xE000`, so a too-large firmware can never overlap saved patterns.
3. **Service-install bound.** `make_service_syx.py` refuses to emit a `.syx`
   that targets any page below `0x1F4`, so the install file can never overwrite
   the running bootloader.

The only residual brick vector is hand-crafting a malicious `.syx` that targets
pages 0x1F0..0x1F3 - a pre-existing property of the bootloader's updater, not
introduced here. The project tooling never emits those pages.

Lock bits: app calling into the boot section is instruction execution (not
LPM/SPM across the boundary), allowed regardless of BLB bits. Default lock bits
(0xFF, unlocked) impose no restriction; the project never sets them. No fuse
changes are needed - SPM-from-boot is already enabled (the bootloader uses it).

---

## 9. Porting checklist

To bring this into another firmware:

1. Confirm the MCU's flash is page-erased, has spare flash below the bootloader,
   and that SPM is restricted to the boot section. Adjust page size and arena
   page range to fit.
2. Drop in `flash_eeprom.h` unchanged (it is platform-independent) and unit-test
   it on the host with stub program/read functions backed by a RAM array. Cover:
   write/read/overwrite, GC, torn write (program returns ok but read-back
   corrupt), crash-before-GC-header, crash-after-GC-header.
3. Install an SPM service in the spare boot pages (`flash_service.c` model) and
   write the two hooks in `flash_persist.h`.
4. Define your block-id map and per-block serialized sizes; keep each block <=
   `FE_MAX_PAYLOAD` (243) so one block == one page.
5. Route all persistence through `g_flash.read/write` and defer writes to a safe
   idle point (clock stopped). Add per-pattern (per-block) dirty tracking from
   the start to keep write amplification down (Section 5).
6. Reproduce the three brick-safety guards (service range-check, app-build arena
   guard, service-install page bound).

---

## 10. Constants quick reference

```
Flash page size              256 bytes
Arena                        0xE000 .. 0x1DFFF   (pages 0xE0..0x1DF, 64 KB, RWW)
Banks                        2 x 128 pages       (page 0 = header, 1..127 = records)
SPM service entry (byte)     0x1FE00             (word pointer 0xFF00)
SPM service probe word       0x940C              (first opcode of JMP)
Running bootloader pages     0x1F0 .. 0x1F3      (never written by tooling)
Service-install lower bound  page 0x1F4
NRWW edge                    0x1E000             (arena stays below it)

Record header                13 bytes (tag, id, seq u32, len, gen u32, crc16)
Bank header                  'FE01' + gen u32 + crc16
FE_MAX_BLOCKS                96
FE_MAX_PAYLOAD               243

Block ids                    0..63 patterns, 64..71 tracks, 72 settings
Pattern block                92 bytes (pitch[64] + time[16] + meta[12])
Track block                  104 bytes (== TRACK_BYTES)
Settings block               22 bytes
```
