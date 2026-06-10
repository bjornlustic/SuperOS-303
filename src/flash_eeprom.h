// flash_eeprom.h -- wear-leveled block-record store over internal flash.
//
// Phase 1 backend for flash-as-EEPROM. Stores fixed logical "blocks" (one per
// pattern / track / settings) in a two-bank, append-only log of page records,
// with a small RAM index and power-loss-safe garbage collection. See
// FLASH_EEPROM_DESIGN.md.
//
// Depends only on <stdint.h>/<string.h> so it can be unit-tested on the host.
// The platform provides page program/read via function pointers (begin()).
#pragma once
#include <stdint.h>
#include <string.h>

// --- Arena geometry (absolute flash page numbers; one page = 256 bytes) ------
// Arena = 0xE000..0x1DFFF = pages 0xE0..0x1DF (256 pages = 64 KB), all in RWW
// (below the 0x1E000 NRWW edge). Split into two equal banks; page 0 of each
// bank is its header, the rest hold one record each.
static constexpr uint16_t FE_ARENA_FIRST_PAGE = 0xE0;
static constexpr uint16_t FE_BANK_PAGES       = 128;          // pages per bank
static constexpr uint16_t FE_NUM_BANKS        = 2;
static constexpr uint16_t FE_RECORD_PAGES     = FE_BANK_PAGES - 1; // 127 usable
static constexpr uint16_t FE_PAGE             = 256;

// Logical block-id space: 64 pair blocks + 32 mono-var3 blocks + 64 poly-var3
// blocks + 8 tracks + 1 settings -> ids run 0..168, so the index must span 169.
// LIVE records still must stay <= FE_RECORD_PAGES (127) for GC to fit: a fully-used
// device is ~105 live when var3 is mono (fits), rising toward 137 as poly slots
// accumulate. Past 127 live, GC fails gracefully (write returns false), no corruption.
static constexpr uint8_t  FE_MAX_BLOCKS = 170;

static constexpr uint8_t  FE_REC_HDR   = 13;                  // record header bytes
static constexpr uint8_t  FE_MAX_PAYLOAD = FE_PAGE - FE_REC_HDR; // 243
static constexpr uint8_t  FE_REC_TAG   = 0xA5;

// Record page layout:
//   [0] tag(0xA5) [1] block_id [2..5] seq(u32) [6] len
//   [7..10] gen(u32) [11..12] crc16 [13..] payload
// Bank header page layout:
//   [0..3] 'F','E','0','1' [4..7] gen(u32) [8..9] crc16

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
  uint8_t read(uint8_t id, uint8_t *dst, uint8_t dst_cap) {
    if (id >= FE_MAX_BLOCKS || index_[id] == 0) return 0;
    read_(page_of(active_bank_, index_[id]), scratch_);
    uint8_t len;
    if (!parse_record(scratch_, id, active_gen_, &len)) return 0;
    if (len > dst_cap) len = dst_cap;
    memcpy(dst, scratch_ + FE_REC_HDR, len);
    return len;
  }

  // Append a new version of block `id`. Returns true on success.
  bool write(uint8_t id, const uint8_t *src, uint8_t len) {
    if (id >= FE_MAX_BLOCKS || len > FE_MAX_PAYLOAD) return false;
    if (append_ > FE_RECORD_PAGES) {            // active bank full
      if (!gc()) return false;
    }
    return append_record(id, src, len, active_gen_, /*advance_seq=*/true);
  }

  // Wipe and re-initialize the arena (start clean).
  bool format() {
    // Pick a generation strictly greater than anything already on flash. format()
    // does NOT erase the record pages, so stale records from a previous epoch
    // survive; a fresh generation makes mount()/read() reject them by gen. Reusing
    // gen 1 would let old gen-1 records with higher seq numbers win on the next
    // mount, resurrecting wiped data (settings/patterns) on every power cycle.
    // mount() always runs before format() and sets active_gen_ to the existing
    // generation (0 only for a truly blank arena), so +1 is strictly newer.
    const uint32_t new_gen = active_gen_ + 1;
    active_bank_ = 0;
    active_gen_  = new_gen;
    seq_         = 0;
    append_      = 1;
    memset(index_, 0, sizeof(index_));
    if (!write_bank_header(0, new_gen)) return false;
    invalidate_bank_header(1);                  // ensure bank 1 not chosen
    return true;
  }

private:
  ProgramFn program_ = nullptr;
  ReadFn    read_    = nullptr;
  uint8_t   active_bank_ = 0;
  uint32_t  active_gen_  = 0;
  uint32_t  seq_         = 0;     // highest record seq seen/written
  uint8_t   append_      = 1;     // next free record page in active bank
  uint8_t   index_[FE_MAX_BLOCKS];
  uint8_t   scratch_[FE_PAGE];

  static uint16_t page_of(uint8_t bank, uint16_t rel_page) {
    return FE_ARENA_FIRST_PAGE + bank * FE_BANK_PAGES + rel_page;
  }

  // Returns true if `page` holds a valid record for `id` in generation `gen`.
  static bool parse_record(const uint8_t *page, uint8_t want_id, uint32_t gen, uint8_t *out_len) {
    if (page[0] != FE_REC_TAG) return false;
    uint8_t len = page[6];
    if (len > FE_MAX_PAYLOAD) return false;
    if (fe_rd32(page + 7) != gen) return false;
    uint16_t crc = fe_crc16(0xFFFF, page + 1, 10);
    crc = fe_crc16(crc, page + FE_REC_HDR, len);
    uint16_t stored = (uint16_t)page[11] | ((uint16_t)page[12] << 8);
    if (crc != stored) return false;
    if (want_id != 0xFF && page[1] != want_id) return false;
    if (out_len) *out_len = len;
    return true;
  }

  bool write_bank_header(uint8_t bank, uint32_t gen) {
    memset(scratch_, 0xFF, FE_PAGE);
    scratch_[0] = 'F'; scratch_[1] = 'E'; scratch_[2] = '0'; scratch_[3] = '1';
    fe_wr32(scratch_ + 4, gen);
    uint16_t crc = fe_crc16(0xFFFF, scratch_, 8);
    scratch_[8] = crc; scratch_[9] = crc >> 8;
    return program_(page_of(bank, 0), scratch_) == 0;
  }
  void invalidate_bank_header(uint8_t bank) {
    memset(scratch_, 0, FE_PAGE);                // zeroed magic -> invalid
    program_(page_of(bank, 0), scratch_);
  }
  bool read_bank_header(uint8_t bank, uint32_t *out_gen) {
    read_(page_of(bank, 0), scratch_);
    if (scratch_[0] != 'F' || scratch_[1] != 'E' || scratch_[2] != '0' || scratch_[3] != '1')
      return false;
    uint16_t crc = fe_crc16(0xFFFF, scratch_, 8);
    uint16_t stored = (uint16_t)scratch_[8] | ((uint16_t)scratch_[9] << 8);
    if (crc != stored) return false;
    *out_gen = fe_rd32(scratch_ + 4);
    return true;
  }

  // Build+program one record page. advance_seq=false reuses seq_ as-is + 1 only
  // when bumping; here we always assign a fresh increasing seq.
  bool append_record(uint8_t id, const uint8_t *payload, uint8_t len, uint32_t gen, bool advance_seq) {
    (void)advance_seq;
    for (uint8_t attempt = 0; attempt < 2 && append_ <= FE_RECORD_PAGES; ++attempt) {
      uint32_t s = ++seq_;
      memset(scratch_, 0xFF, FE_PAGE);
      scratch_[0] = FE_REC_TAG;
      scratch_[1] = id;
      fe_wr32(scratch_ + 2, s);
      scratch_[6] = len;
      fe_wr32(scratch_ + 7, gen);
      memcpy(scratch_ + FE_REC_HDR, payload, len);
      uint16_t crc = fe_crc16(0xFFFF, scratch_ + 1, 10);
      crc = fe_crc16(crc, scratch_ + FE_REC_HDR, len);
      scratch_[11] = crc; scratch_[12] = crc >> 8;

      uint8_t page = append_;
      if (program_(page_of(active_bank_, page), scratch_) != 0) { ++append_; continue; }
      // Verify the write took (reject torn/failed pages, advance past them).
      read_(page_of(active_bank_, page), scratch_);
      uint8_t vlen;
      if (!parse_record(scratch_, id, gen, &vlen) || vlen != len) { ++append_; continue; }
      index_[id] = page;
      append_ = page + 1;
      return true;
    }
    return false;
  }

  bool mount() {
    uint32_t g0 = 0, g1 = 0;
    bool v0 = read_bank_header(0, &g0);
    bool v1 = read_bank_header(1, &g1);

    if (!v0 && !v1) return format();

    if (v0 && v1) {
      active_bank_ = (g0 >= g1) ? 0 : 1;
      active_gen_  = (g0 >= g1) ? g0 : g1;
      invalidate_bank_header(active_bank_ ^ 1); // clean up an interrupted GC
    } else {
      active_bank_ = v0 ? 0 : 1;
      active_gen_  = v0 ? g0 : g1;
    }

    // Scan active bank: newest valid record (this generation) wins per block.
    memset(index_, 0, sizeof(index_));
    seq_ = 0;
    uint8_t highest_used = 0;
    uint32_t best_seq[FE_MAX_BLOCKS];
    memset(best_seq, 0, sizeof(best_seq));
    for (uint16_t p = 1; p <= FE_RECORD_PAGES; ++p) {
      read_(page_of(active_bank_, p), scratch_);
      uint8_t len;
      if (!parse_record(scratch_, 0xFF, active_gen_, &len)) continue;
      uint8_t id = scratch_[1];
      if (id >= FE_MAX_BLOCKS) continue;
      uint32_t s = fe_rd32(scratch_ + 2);
      if (s > seq_) seq_ = s;
      if (index_[id] == 0 || s > best_seq[id]) { index_[id] = (uint8_t)p; best_seq[id] = s; }
      highest_used = (uint8_t)p;
    }
    append_ = highest_used + 1;
    return true;
  }

  // Compact live records into the spare bank, then switch to it. Power-safe:
  // the active bank is never erased until the spare is fully written + headered.
  bool gc() {
    uint8_t spare = active_bank_ ^ 1;
    uint32_t new_gen = active_gen_ + 1;

    // Snapshot the live index (read() below would clobber scratch_/index_).
    uint8_t live[FE_MAX_BLOCKS];
    memcpy(live, index_, sizeof(live));

    uint8_t spare_page = 1;
    uint8_t new_index[FE_MAX_BLOCKS];
    memset(new_index, 0, sizeof(new_index));

    for (uint8_t id = 0; id < FE_MAX_BLOCKS; ++id) {
      if (live[id] == 0) continue;
      if (spare_page > FE_RECORD_PAGES) return false;   // would overflow (too many live blocks)
      read_(page_of(active_bank_, live[id]), scratch_);
      uint8_t len;
      if (!parse_record(scratch_, id, active_gen_, &len)) continue; // skip unreadable
      uint8_t payload[FE_MAX_PAYLOAD];
      memcpy(payload, scratch_ + FE_REC_HDR, len);
      // Write into spare with the NEW generation.
      uint32_t s = ++seq_;
      memset(scratch_, 0xFF, FE_PAGE);
      scratch_[0] = FE_REC_TAG; scratch_[1] = id;
      fe_wr32(scratch_ + 2, s); scratch_[6] = len; fe_wr32(scratch_ + 7, new_gen);
      memcpy(scratch_ + FE_REC_HDR, payload, len);
      uint16_t crc = fe_crc16(0xFFFF, scratch_ + 1, 10);
      crc = fe_crc16(crc, scratch_ + FE_REC_HDR, len);
      scratch_[11] = crc; scratch_[12] = crc >> 8;
      if (program_(page_of(spare, spare_page), scratch_) != 0) return false;
      new_index[id] = spare_page++;
    }

    if (!write_bank_header(spare, new_gen)) return false;  // commit point (written last)
    invalidate_bank_header(active_bank_);                  // retire the old bank

    active_bank_ = spare;
    active_gen_  = new_gen;
    memcpy(index_, new_index, sizeof(index_));
    append_ = spare_page;
    return true;
  }

public:
  // Test/diagnostic accessors.
  uint8_t active_bank() const { return active_bank_; }
  uint32_t active_gen() const { return active_gen_; }
  uint8_t append_page() const { return append_; }
};
