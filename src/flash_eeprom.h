// flash_eeprom.h -- wear-leveled block-record store over internal flash.
//
// Backend for flash-as-EEPROM. Stores fixed logical "blocks" (one per pattern /
// track / settings) one-per-page in a SINGLE arena, with a small RAM index,
// rotating allocation for wear spreading, and power-loss-safe write-then-erase
// updates. No garbage collection: a superseded record's page is erased by the
// write that replaced it. See FLASH_EEPROM_DESIGN.md.
//
// Depends only on <stdint.h>/<string.h> so it can be unit-tested on the host.
// The platform provides page program/read via function pointers (begin()).
#pragma once
#include <stdint.h>
#include <string.h>

// --- Arena geometry (absolute flash page numbers; one page = 256 bytes) ------
// All arena pages are in RWW (below the 0x1E000 NRWW edge). Page 0 is the arena
// header; every other page holds exactly one record.
//
// Combined build: the SuperOS+D650C image is ~92 KB, so the arena starts at
// 0x17200 (pages 0x172..0x1DF, 110 pages -> 109 record pages, 108 distinct
// blocks after the update reserve -- the worst-case live set of 107 records
// plus the reserve fits EXACTLY, see the static_assert in flash_persist.h;
// the arena cannot move up again without shrinking that set). Only a few
// hundred bytes of app headroom are left below it, so watch makesyx's ceiling
// check when adding code. The SPM service accepts pages
// 0x100..0x1DF, so no service reflash is needed. Keep FLASH_ARENA_BASE in
// tools/makesyx.py in sync. First boot of a different layout finds no valid
// header at its base and formats (stored patterns are wiped on a layout change).
// History: 0x15800 -> 0x16000 -> 0x18000 -> 0x17000 -> 0x17200. The arena CANNOT grow downward past
// the app: 0x1E000 is the NRWW boundary and the SPM service refuses pages above
// 0x1DF, so every KB of app code is paid for out of the record budget.
#ifdef SUPEROS_COMBINED
static constexpr uint16_t FE_ARENA_FIRST_PAGE = 0x172;
static constexpr uint16_t FE_BANK_PAGES       = 55;           // half the arena (kept for sizing)
#else
// SuperOS-only build: moved from 0x10000 (112/bank) to 0x12000 for the same
// reason as the combined image above -- the pattern-write spec pushed the app
// past 64 KB. 95 records per bank still comfortably exceeds what one device
// writes in practice.
static constexpr uint16_t FE_ARENA_FIRST_PAGE = 0x120;
static constexpr uint16_t FE_BANK_PAGES       = 96;           // pages per bank (2 banks)
#endif
// SINGLE arena: every page below page 0 (the header) holds one live record.
// The old layout split this into two banks and kept one of them empty purely
// as a landing zone for whole-bank GC, which cost half the capacity. AVR erases
// ONE page at a time (see flash_service.c), so a superseded record's page can
// be reclaimed on its own and the spare bank is unnecessary. Capacity roughly
// doubles: 110 live records combined, 190 SuperOS-only.
static constexpr uint16_t FE_ARENA_PAGES      = 2 * FE_BANK_PAGES;
static constexpr uint16_t FE_RECORD_PAGES     = FE_ARENA_PAGES - 1; // page 0 = header
static constexpr uint16_t FE_PAGE             = 256;

// Logical block-id space, for NUM_SLOTS = 64 (4 groups x 16) at MAX_STEPS = 32,
// with patterns and probability stored trimmed several-per-page: 48 pattern
// groups + 192 dense pattern overflow + 32 poly groups + 64 dense poly overflow
// + 4 track pairs + 1 settings + 22 probability groups + 64 dense probability
// overflow -> ids run 0..426, so the index spans 427. (The 16-bit record id
// below is kept: the FRAM build is slated for 7 groups / 112 slots, where ids
// run past 255, and a uniform record format means the two builds stay
// wire-compatible.)
//
// At 32 steps the dense overflow ids are UNREACHABLE (a full pattern always
// fits its sparse region), so the worst-case live set is 48 + 32 + 22 + 4 + 1
// = 107 records -- guaranteed to fit the 110-block combined arena with every
// slot fully armed. The overflow id ranges are kept only so the map layout
// stays uniform.
static constexpr uint16_t FE_MAX_BLOCKS = 427;

// The record header carries a 16-bit id (see above: the FRAM build needs ids
// past 255). This costs one payload byte; the largest payload is the 227-byte
// poly blob, so there is still ample room.
static constexpr uint8_t  FE_ID_BYTES  = 2;
static constexpr uint8_t  FE_REC_HDR   = 14;                  // record header bytes
static constexpr uint8_t  FE_MAX_PAYLOAD = FE_PAGE - FE_REC_HDR; // 242
static constexpr uint8_t  FE_REC_TAG   = 0xA5;

// Record page layout:
//   [0] tag(0xA5) [1..2] block_id(u16 LE) [3..6] seq(u32) [7] len
//   [8..11] gen(u32) [12..13] crc16 [14..] payload
// Arena header page layout (page 0):
//   [0..3] 'F','E','0','2' [4..7] gen(u32) [8..9] crc16

inline uint16_t fe_crc16(uint16_t crc, const uint8_t *p, uint16_t n) {
  for (uint16_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)p[i] << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
inline uint32_t fe_rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline void fe_wr32(uint8_t *p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

class FlashEeprom {
public:
  typedef uint8_t (*ProgramFn)(uint16_t abs_page, const uint8_t *buf256); // 0 = ok
  typedef void    (*ReadFn)(uint16_t abs_page, uint8_t *buf256);

  // Mount the arena (formats it if blank/corrupt). Returns true on success.
  bool begin(ProgramFn prog, ReadFn rd) {
    program_ = prog; read_ = rd;
    return mount();
  }

  // Read the newest record for `id` into dst (up to dst_cap bytes). Returns the
  // stored payload length, or 0 if the block has never been written.
  uint8_t read(uint16_t id, uint8_t *dst, uint8_t dst_cap) {
    if (id >= FE_MAX_BLOCKS || index_[id] == 0) return 0;
    read_(page_of(index_[id]), scratch_);
    uint8_t len;
    if (!parse_record(scratch_, id, active_gen_, &len)) return 0;
    if (len > dst_cap) len = dst_cap;
    memcpy(dst, scratch_ + FE_REC_HDR, len);
    return len;
  }

  // Store a new version of block `id`. Returns true on success.
  //
  // Write-then-erase, so a power cut can never lose the stored value: the new
  // page is programmed and read back VERIFIED before the old page is erased.
  // A cut in between leaves two valid records for the id; mount() keeps the
  // higher seq and erases the loser.
  bool write(uint16_t id, const uint8_t *src, uint8_t len) {
    if (id >= FE_MAX_BLOCKS || len > FE_MAX_PAYLOAD) return false;
    const uint8_t old_page = index_[id];
    // One page is held in reserve so an EXISTING block can always be updated:
    // the update frees its old page the moment the new one verifies. Without
    // this, a full arena would refuse to let you edit a pattern you already
    // saved. Only brand-new blocks are turned away.
    if (old_page == 0 && free_pages() <= 1) return false;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
      const uint8_t page = alloc_page();
      if (page == 0) return false;                    // arena full
      mark_used(page, true);                          // claimed either way
      build_record(id, src, len, active_gen_, ++seq_);
      if (program_(page_of(page), scratch_) != 0) continue;   // bad page, quarantined
      read_(page_of(page), scratch_);                 // verify it actually took
      uint8_t vlen;
      if (!parse_record(scratch_, id, active_gen_, &vlen) || vlen != len) continue;
      index_[id] = page;
      if (old_page != 0 && old_page != page) erase_page(old_page);
      return true;
    }
    return false;
  }

  // Wipe and re-initialize the arena (start clean).
  bool format() {
    // Pick a generation strictly greater than anything already on flash.
    // format() does NOT erase the record pages (that would be ~1 s of CPU-
    // halting SPM), so stale records survive; a fresh generation makes
    // mount()/read() reject them and their pages read as free. Reusing a
    // generation would let old records with higher seq numbers win on the next
    // mount, resurrecting wiped data. mount() sets active_gen_ to the highest
    // generation present on flash -- even when the header is unreadable -- so
    // +1 here is strictly newer than anything stored.
    const uint32_t new_gen = active_gen_ + 1;
    active_gen_  = new_gen;
    seq_         = 0;
    next_alloc_  = 1;
    used_count_  = 0;
    memset(index_, 0, sizeof(index_));
    memset(used_,  0, sizeof(used_));
    return write_arena_header(new_gen);
  }

private:
  ProgramFn program_ = nullptr;
  ReadFn    read_    = nullptr;
  uint32_t  active_gen_  = 0;
  uint32_t  seq_         = 0;     // highest record seq seen/written
  uint8_t   next_alloc_  = 1;     // rotating allocation cursor (wear spreading)
  uint8_t   used_count_  = 0;     // live record pages, so free_pages() is O(1)
  uint8_t   index_[FE_MAX_BLOCKS];
  uint8_t   used_[(FE_ARENA_PAGES + 7) / 8];  // page occupancy bitmap
  uint8_t   scratch_[FE_PAGE];

  static uint16_t page_of(uint16_t rel_page) {
    return FE_ARENA_FIRST_PAGE + rel_page;
  }

  bool is_used(uint8_t p) const { return (used_[p >> 3] >> (p & 7)) & 1; }
  void mark_used(uint8_t p, bool on) {
    const uint8_t m = uint8_t(1u << (p & 7));
    if (is_used(p) == on) return;
    if (on) { used_[p >> 3] |= m; ++used_count_; }
    else    { used_[p >> 3] &= uint8_t(~m); --used_count_; }
  }

  // Next free record page, scanning forward from the rotating cursor so
  // erase/program wear spreads evenly instead of hammering low pages.
  // Returns 0 when the arena is full.
  uint8_t alloc_page() {
    for (uint8_t n = 0; n < FE_RECORD_PAGES; ++n) {
      const uint8_t p = uint8_t(1 + (next_alloc_ - 1 + n) % FE_RECORD_PAGES);
      if (!is_used(p)) {
        next_alloc_ = uint8_t(1 + (p % FE_RECORD_PAGES));
        return p;
      }
    }
    return 0;
  }

  // Return a page to the pool. Programming it all-0xFF erases it, so a stale
  // record can never be resurrected by a later mount.
  void erase_page(uint8_t p) {
    memset(scratch_, 0xFF, FE_PAGE);
    program_(page_of(p), scratch_);
    mark_used(p, false);
  }

  // Record field offsets (16-bit block id, see the layout comment above).
  static constexpr uint8_t FE_OFF_ID  = 1;   // u16 LE
  static constexpr uint8_t FE_OFF_SEQ = 3;   // u32
  static constexpr uint8_t FE_OFF_LEN = 7;
  static constexpr uint8_t FE_OFF_GEN = 8;   // u32
  static constexpr uint8_t FE_OFF_CRC = 12;  // u16 LE
  static constexpr uint8_t FE_CRC_SPAN = FE_OFF_CRC - FE_OFF_ID; // bytes covered before payload
  static constexpr uint16_t FE_ID_ANY = 0xFFFF;                  // "match any id" sentinel

  static uint16_t fe_rec_id(const uint8_t *page) {
    return uint16_t(page[FE_OFF_ID]) | (uint16_t(page[FE_OFF_ID + 1]) << 8);
  }

  // Returns true if `page` holds a valid record for `id` in generation `gen`.
  static bool parse_record(const uint8_t *page, uint16_t want_id, uint32_t gen, uint8_t *out_len) {
    if (page[0] != FE_REC_TAG) return false;
    uint8_t len = page[FE_OFF_LEN];
    if (len > FE_MAX_PAYLOAD) return false;
    if (fe_rd32(page + FE_OFF_GEN) != gen) return false;
    uint16_t crc = fe_crc16(0xFFFF, page + FE_OFF_ID, FE_CRC_SPAN);
    crc = fe_crc16(crc, page + FE_REC_HDR, len);
    uint16_t stored = (uint16_t)page[FE_OFF_CRC] | ((uint16_t)page[FE_OFF_CRC + 1] << 8);
    if (crc != stored) return false;
    if (want_id != FE_ID_ANY && fe_rec_id(page) != want_id) return false;
    if (out_len) *out_len = len;
    return true;
  }

  // Fill a record page in scratch_ (everything but the program call).
  void build_record(uint16_t id, const uint8_t *payload, uint8_t len,
                    uint32_t gen, uint32_t s) {
    memset(scratch_, 0xFF, FE_PAGE);
    scratch_[0] = FE_REC_TAG;
    scratch_[FE_OFF_ID]     = uint8_t(id);
    scratch_[FE_OFF_ID + 1] = uint8_t(id >> 8);
    fe_wr32(scratch_ + FE_OFF_SEQ, s);
    scratch_[FE_OFF_LEN] = len;
    fe_wr32(scratch_ + FE_OFF_GEN, gen);
    memcpy(scratch_ + FE_REC_HDR, payload, len);
    uint16_t crc = fe_crc16(0xFFFF, scratch_ + FE_OFF_ID, FE_CRC_SPAN);
    crc = fe_crc16(crc, scratch_ + FE_REC_HDR, len);
    scratch_[FE_OFF_CRC]     = uint8_t(crc);
    scratch_[FE_OFF_CRC + 1] = uint8_t(crc >> 8);
  }

  bool write_arena_header(uint32_t gen) {
    memset(scratch_, 0xFF, FE_PAGE);
    // 'FE02' = single-arena layout. A two-bank 'FE01' arena fails this check
    // and is formatted, so the layout change can never be half-read.
    scratch_[0] = 'F'; scratch_[1] = 'E'; scratch_[2] = '0'; scratch_[3] = '2';
    fe_wr32(scratch_ + 4, gen);
    uint16_t crc = fe_crc16(0xFFFF, scratch_, 8);
    scratch_[8] = crc; scratch_[9] = crc >> 8;
    return program_(page_of(0), scratch_) == 0;
  }
  bool read_arena_header(uint32_t *out_gen) {
    read_(page_of(0), scratch_);
    if (scratch_[0] != 'F' || scratch_[1] != 'E' || scratch_[2] != '0' || scratch_[3] != '2')
      return false;
    uint16_t crc = fe_crc16(0xFFFF, scratch_, 8);
    uint16_t stored = (uint16_t)scratch_[8] | ((uint16_t)scratch_[9] << 8);
    if (crc != stored) return false;
    *out_gen = fe_rd32(scratch_ + 4);
    return true;
  }

  // Highest generation stamped on any parseable record page. Used when the
  // header is missing/corrupt so format() can still pick a strictly newer
  // generation and never resurrect stale records.
  uint32_t scan_max_gen() {
    uint32_t best = 0;
    for (uint16_t p = 1; p <= FE_RECORD_PAGES; ++p) {
      read_(page_of(p), scratch_);
      if (scratch_[0] != FE_REC_TAG) continue;
      const uint8_t len = scratch_[FE_OFF_LEN];
      if (len > FE_MAX_PAYLOAD) continue;
      const uint32_t g = fe_rd32(scratch_ + FE_OFF_GEN);
      // Only trust the generation if the record checksums out under it.
      if (parse_record(scratch_, FE_ID_ANY, g, nullptr) && g > best) best = g;
    }
    return best;
  }

  // Sequence number of the record on `page`, or 0 if it is not a live record.
  uint32_t seq_at(uint8_t page) {
    read_(page_of(page), scratch_);
    if (!parse_record(scratch_, FE_ID_ANY, active_gen_, nullptr)) return 0;
    return fe_rd32(scratch_ + FE_OFF_SEQ);
  }

  bool mount() {
    uint32_t gen = 0;
    if (!read_arena_header(&gen)) {
      active_gen_ = scan_max_gen();     // strictly-newer basis for format()
      return format();
    }
    active_gen_ = gen;
    seq_        = 0;
    next_alloc_ = 1;
    used_count_ = 0;
    memset(index_, 0, sizeof(index_));
    memset(used_,  0, sizeof(used_));

    // Claim one page per block id. Allocation ROTATES, so page order is not
    // seq order and duplicates must be resolved by comparing seq: a power cut
    // between "new page verified" and "old page erased" leaves two valid
    // records for one id. Keep the higher seq and erase the loser so the
    // ambiguity does not survive to the next mount.
    for (uint16_t p = 1; p <= FE_RECORD_PAGES; ++p) {
      read_(page_of(p), scratch_);
      uint8_t len;
      if (!parse_record(scratch_, FE_ID_ANY, active_gen_, &len)) continue;
      const uint16_t id = fe_rec_id(scratch_);
      if (id >= FE_MAX_BLOCKS) continue;             // unknown id: leave page free
      const uint32_t s = fe_rd32(scratch_ + FE_OFF_SEQ);
      if (s > seq_) seq_ = s;
      const uint8_t prev = index_[id];
      if (prev == 0) {
        index_[id] = (uint8_t)p;
        mark_used((uint8_t)p, true);
        continue;
      }
      if (s > seq_at(prev)) {                        // this page is newer
        erase_page(prev);
        index_[id] = (uint8_t)p;
        mark_used((uint8_t)p, true);
      } else {
        erase_page((uint8_t)p);
      }
    }
    return true;
  }

public:
  // Test/diagnostic accessors.
  uint32_t active_gen() const { return active_gen_; }
  uint8_t alloc_cursor() const { return next_alloc_; }

  // Free record pages left in the arena (0 = the next write fails).
  uint8_t free_pages() const { return (uint8_t)(FE_RECORD_PAGES - used_count_); }
  // No-op kept for API compatibility. There is nothing to compact: a
  // superseded record's page is erased by the write that replaced it, so
  // free_pages() is always exact and no caller ever has to pay a GC stall.
  bool maintain(uint8_t) { return true; }
};
