// Host test for src/flash_eeprom.h (the single-arena block store) against a
// RAM-backed fake flash. Covers the 16-bit block id, rotating page allocation,
// eager reclaim of superseded pages, the write-then-erase power-cut guarantee,
// and duplicate resolution at mount.
//
// Build/run:
//   c++ -std=c++17 -I src -I tools/host_stubs tools/flash_hosttest.cpp \
//       -o /tmp/flashtest && /tmp/flashtest
#include "../src/sequence.h"
#include "../src/flash_eeprom.h"
#include <cstdio>
#include <cassert>
#include <cstring>
#include <vector>

static constexpr uint16_t kTotalPages = FE_ARENA_FIRST_PAGE + 2 * FE_BANK_PAGES + 4;
static std::vector<uint8_t> g_flashmem(size_t(kTotalPages) * FE_PAGE, 0xFF);
static bool     g_fail_program = false;   // simulate a dead page programmer
static unsigned g_program_calls = 0;

static uint8_t fake_program(uint16_t page, const uint8_t *buf) {
  ++g_program_calls;
  if (g_fail_program) return 1;
  assert(page < kTotalPages);
  memcpy(&g_flashmem[size_t(page) * FE_PAGE], buf, FE_PAGE);
  return 0;
}
static void fake_read(uint16_t page, uint8_t *buf) {
  assert(page < kTotalPages);
  memcpy(buf, &g_flashmem[size_t(page) * FE_PAGE], FE_PAGE);
}

static void wipe() { std::fill(g_flashmem.begin(), g_flashmem.end(), 0xFF); }

// Payload whose contents are derived from (id, tag) so a mix-up is detectable.
static void fill_payload(uint8_t *p, uint8_t len, uint16_t id, uint8_t tag) {
  for (uint8_t i = 0; i < len; ++i)
    p[i] = uint8_t(id * 7 + tag * 13 + i);
}
static bool payload_matches(const uint8_t *p, uint8_t len, uint16_t id, uint8_t tag) {
  for (uint8_t i = 0; i < len; ++i)
    if (p[i] != uint8_t(id * 7 + tag * 13 + i)) return false;
  return true;
}

int main() {
  static FlashEeprom fe;
  uint8_t buf[FE_MAX_PAYLOAD];
  uint8_t got[FE_MAX_PAYLOAD];

  // 1) Round-trip across the whole id space, including ids above 255 -- the
  //    reason the record header carries a 16-bit id at all.
  {
    wipe();
    assert(fe.begin(fake_program, fake_read));
    const uint16_t ids[] = {0, 1, 63, 111, 190, 200, 254, FE_MAX_BLOCKS - 1};
    for (uint16_t id : ids) {
      fill_payload(buf, 92, id, 1);
      assert(fe.write(id, buf, 92));
    }
    for (uint16_t id : ids) {
      assert(fe.read(id, got, sizeof(got)) == 92);
      assert(payload_matches(got, 92, id, 1));
    }
    // Out-of-range ids are rejected, not wrapped into a valid block.
    assert(!fe.write(FE_MAX_BLOCKS, buf, 92));
    assert(fe.read(FE_MAX_BLOCKS, got, sizeof(got)) == 0);
    assert(fe.read(7, got, sizeof(got)) == 0);   // never written
    printf("1) 16-bit block ids round-trip: ok\n");
  }

  // 2) Rewrites supersede: the newest record for an id wins, and it survives a
  //    remount (this is what the page-order scan has to get right).
  {
    for (uint8_t tag = 2; tag <= 6; ++tag) {
      fill_payload(buf, 92, 200, tag);
      assert(fe.write(200, buf, 92));
    }
    assert(fe.read(200, got, sizeof(got)) == 92);
    assert(payload_matches(got, 92, 200, 6));

    static FlashEeprom fe2;                     // fresh instance, same flash
    assert(fe2.begin(fake_program, fake_read));
    assert(fe2.read(200, got, sizeof(got)) == 92);
    assert(payload_matches(got, 92, 200, 6));   // newest, not the first
    assert(fe2.read(111, got, sizeof(got)) == 92);
    assert(payload_matches(got, 92, 111, 1));
    printf("2) newest-record-wins across remount: ok\n");
  }

  // 3) Eager reclaim: rewriting one id forever must NOT consume the arena --
  //    each write erases the page the previous version occupied.
  {
    wipe();
    static FlashEeprom fe3;
    assert(fe3.begin(fake_program, fake_read));
    const uint16_t live_ids[] = {0, 5, 130, 200, 254};
    for (uint16_t id : live_ids) {
      fill_payload(buf, 100, id, 1);
      assert(fe3.write(id, buf, 100));
    }
    const uint8_t free_after_setup = fe3.free_pages();
    for (uint16_t n = 0; n < FE_RECORD_PAGES * 4; ++n) {
      fill_payload(buf, 100, 5, uint8_t(n & 0x7F));
      assert(fe3.write(5, buf, 100));             // never runs out
    }
    assert(fe3.free_pages() == free_after_setup); // steady state, no leak
    for (uint16_t id : live_ids) {
      if (id == 5) continue;
      assert(fe3.read(id, got, sizeof(got)) == 100);
      assert(payload_matches(got, 100, id, 1));
    }
    const uint8_t last = uint8_t((FE_RECORD_PAGES * 4 - 1) & 0x7F);
    assert(fe3.read(5, got, sizeof(got)) == 100);
    assert(payload_matches(got, 100, 5, last));

    static FlashEeprom fe4;                       // survives a remount
    assert(fe4.begin(fake_program, fake_read));
    assert(fe4.read(254, got, sizeof(got)) == 100);
    assert(payload_matches(got, 100, 254, 1));
    assert(fe4.read(5, got, sizeof(got)) == 100);
    assert(payload_matches(got, 100, 5, last));
    printf("3) rewrites reclaim their old page (no leak): ok\n");
  }

  // 4) Wear spreading: consecutive writes must land on DIFFERENT pages rather
  //    than erasing and reprogramming one hot page over and over.
  {
    wipe();
    static FlashEeprom fe7;
    assert(fe7.begin(fake_program, fake_read));
    uint8_t prev = 0; int distinct = 0;
    for (uint16_t n = 0; n < 16; ++n) {
      fill_payload(buf, 64, 12, uint8_t(n));
      assert(fe7.write(12, buf, 64));
      const uint8_t cur = fe7.alloc_cursor();
      if (cur != prev) ++distinct;
      prev = cur;
    }
    assert(distinct >= 12);                       // cursor keeps moving
    printf("4) allocation rotates for wear: ok\n");
  }

  // 5) Store full: writes fail cleanly and everything already stored survives.
  {
    wipe();
    static FlashEeprom fe5;
    assert(fe5.begin(fake_program, fake_read));
    uint16_t stored = 0;
    for (uint16_t id = 0; id < FE_MAX_BLOCKS; ++id) {
      fill_payload(buf, 200, id, 1);
      if (!fe5.write(id, buf, 200)) break;
      ++stored;
    }
    assert(stored > 0);
    assert(stored <= FE_RECORD_PAGES);
    for (uint16_t id = 0; id < stored; ++id) {
      assert(fe5.read(id, got, sizeof(got)) == 200);
      assert(payload_matches(got, 200, id, 1));
    }
    // A full store must still accept a REWRITE of an existing block: the old
    // page is freed by that same write, so the value can always be updated.
    fill_payload(buf, 200, 0, 9);
    assert(fe5.write(0, buf, 200));
    assert(fe5.read(0, got, sizeof(got)) == 200);
    assert(payload_matches(got, 200, 0, 9));
    printf("5) full store: new blocks refused, rewrites still work (%u): ok\n", stored);
  }

  // 6) Power cut mid-write: the programmer dies while storing a new version.
  //    The PREVIOUS value must still be there -- write-then-erase never
  //    destroys the old page before the new one verifies.
  {
    wipe();
    static FlashEeprom fe8;
    assert(fe8.begin(fake_program, fake_read));
    fill_payload(buf, 150, 77, 1);
    assert(fe8.write(77, buf, 150));
    g_fail_program = true;
    fill_payload(buf, 150, 77, 2);
    assert(!fe8.write(77, buf, 150));             // the doomed update
    g_fail_program = false;
    assert(fe8.read(77, got, sizeof(got)) == 150);
    assert(payload_matches(got, 150, 77, 1));     // old value intact
    static FlashEeprom fe9;
    assert(fe9.begin(fake_program, fake_read));
    assert(fe9.read(77, got, sizeof(got)) == 150);
    assert(payload_matches(got, 150, 77, 1));     // and after a remount
    printf("6) failed write keeps the previous value: ok\n");
  }

  // 7) Power cut BETWEEN the new page verifying and the old page erasing: two
  //    valid records for one id. Mount must keep the higher seq, erase the
  //    loser, and leave the store with no lingering duplicate.
  {
    wipe();
    static FlashEeprom feA;
    assert(feA.begin(fake_program, fake_read));
    fill_payload(buf, 90, 21, 1);
    assert(feA.write(21, buf, 90));
    // Snapshot the arena, write the newer version, then splice the OLD page
    // back in -- exactly the state a cut before the erase would leave.
    std::vector<uint8_t> before = g_flashmem;
    fill_payload(buf, 90, 21, 2);
    assert(feA.write(21, buf, 90));
    for (uint16_t p = 1; p <= FE_RECORD_PAGES; ++p) {
      const size_t off = size_t(FE_ARENA_FIRST_PAGE + p) * FE_PAGE;
      const bool was_rec = before[off] == FE_REC_TAG;
      const bool is_rec  = g_flashmem[off] == FE_REC_TAG;
      if (was_rec && !is_rec)                      // the page that got erased
        memcpy(&g_flashmem[off], &before[off], FE_PAGE);
    }
    static FlashEeprom feB;
    assert(feB.begin(fake_program, fake_read));
    assert(feB.read(21, got, sizeof(got)) == 90);
    assert(payload_matches(got, 90, 21, 2));       // newer wins
    const uint8_t free_1 = feB.free_pages();
    static FlashEeprom feC;                        // duplicate must be gone
    assert(feC.begin(fake_program, fake_read));
    assert(feC.read(21, got, sizeof(got)) == 90);
    assert(payload_matches(got, 90, 21, 2));
    assert(feC.free_pages() == free_1);            // stable, not leaking
    printf("7) interrupted erase resolves to the newer record: ok\n");
  }

  // 8) A foreign/old arena (two-bank 'FE01' header, or garbage) must format
  //    rather than misread, and must not resurrect stale records afterwards.
  {
    wipe();
    static FlashEeprom feD;
    assert(feD.begin(fake_program, fake_read));
    fill_payload(buf, 80, 50, 1);
    assert(feD.write(50, buf, 80));
    const uint32_t gen_before = feD.active_gen();
    // Corrupt the header only; the records stay on flash.
    memset(&g_flashmem[size_t(FE_ARENA_FIRST_PAGE) * FE_PAGE], 0x5A, 16);
    static FlashEeprom feE;
    assert(feE.begin(fake_program, fake_read));
    assert(feE.active_gen() > gen_before);         // strictly newer generation
    assert(feE.read(50, got, sizeof(got)) == 0);   // stale record NOT resurrected
    fill_payload(buf, 80, 50, 3);
    assert(feE.write(50, buf, 80));
    assert(feE.read(50, got, sizeof(got)) == 80);
    assert(payload_matches(got, 80, 50, 3));
    printf("8) corrupt header reformats without resurrecting data: ok\n");
  }

  printf("ALL FLASH STORE HOST TESTS PASSED\n");
  return 0;
}
