// Host unit test for src/flash_eeprom.h. Models the arena as RAM and injects
// power-loss (torn writes, crash mid-GC) to verify the FS is robust.
//
//   c++ -std=c++14 -I ../src flash_eeprom_hosttest.cpp -o /tmp/fetest && /tmp/fetest
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csetjmp>
#include "flash_eeprom.h"

static const int FE_TOTAL_PAGES = FE_NUM_BANKS * FE_BANK_PAGES;     // 224
static uint8_t g_mem[FE_TOTAL_PAGES][FE_PAGE];

// Power-loss injection.
static int      g_prog_count = 0;
static int      g_crash_at   = -1;   // crash when prog_count reaches this (1-based)
static bool     g_torn       = false;
static jmp_buf  g_crash_jmp;

static void flash_erase_all() { memset(g_mem, 0xFF, sizeof(g_mem)); }

static uint8_t host_program(uint16_t abs_page, const uint8_t *buf) {
  int rel = (int)abs_page - FE_ARENA_FIRST_PAGE;
  if (rel < 0 || rel >= FE_TOTAL_PAGES) { fprintf(stderr, "OOB program %u\n", abs_page); abort(); }
  ++g_prog_count;
  if (g_crash_at >= 0 && g_prog_count == g_crash_at) {
    if (g_torn) {                       // most of the page landed, but the write
      memcpy(g_mem[rel], buf, FE_PAGE); // did not finish cleanly: one byte is
      g_mem[rel][FE_REC_HDR + 4] ^= 0xFF; // wrong, so CRC must reject the page
    }
    longjmp(g_crash_jmp, 1);
  }
  memcpy(g_mem[rel], buf, FE_PAGE);     // erase+write = full overwrite
  return 0;
}
static void host_read(uint16_t abs_page, uint8_t *buf) {
  int rel = (int)abs_page - FE_ARENA_FIRST_PAGE;
  if (rel < 0 || rel >= FE_TOTAL_PAGES) { fprintf(stderr, "OOB read %u\n", abs_page); abort(); }
  memcpy(buf, g_mem[rel], FE_PAGE);
}

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } } while (0)

static void fill(uint8_t *b, uint8_t n, uint8_t seed) { for (uint8_t i = 0; i < n; ++i) b[i] = uint8_t(seed + i * 7); }
static bool same(const uint8_t *a, const uint8_t *b, uint8_t n) { return memcmp(a, b, n) == 0; }

// ---- scenarios -------------------------------------------------------------

static void test_basic() {
  printf("test_basic (write/read/overwrite/persist)\n");
  flash_erase_all();
  FlashEeprom fe; fe.begin(host_program, host_read);

  uint8_t a[48], b[48], out[48];
  fill(a, 48, 10); fill(b, 48, 200);
  CHECK(fe.write(5, a, 48), "write blk5");
  CHECK(fe.read(5, out, 48) == 48 && same(out, a, 48), "read blk5 == a");
  CHECK(fe.read(6, out, 48) == 0, "blk6 absent");

  CHECK(fe.write(5, b, 48), "overwrite blk5");
  CHECK(fe.read(5, out, 48) == 48 && same(out, b, 48), "read blk5 == b (newest)");

  // Remount: index rebuilt from flash, newest wins.
  FlashEeprom fe2; fe2.begin(host_program, host_read);
  CHECK(fe2.read(5, out, 48) == 48 && same(out, b, 48), "persist blk5 == b after remount");
  CHECK(fe2.read(6, out, 48) == 0, "blk6 still absent after remount");
}

static void test_gc() {
  printf("test_gc (fill past one bank, all live blocks survive)\n");
  flash_erase_all();
  FlashEeprom fe; fe.begin(host_program, host_read);

  // 20 distinct blocks, each rewritten many times -> forces GC (>111 writes).
  uint8_t expect[20][48];
  for (int round = 0; round < 12; ++round)
    for (int id = 0; id < 20; ++id) {
      fill(expect[id], 48, uint8_t(id * 3 + round * 11));
      CHECK(fe.write(id, expect[id], 48), "write during churn");
    }
  CHECK(fe.active_gen() >= 2, "GC happened (gen advanced)");

  uint8_t out[48];
  for (int id = 0; id < 20; ++id)
    CHECK(fe.read(id, out, 48) == 48 && same(out, expect[id], 48), "live block survived GC");

  // Remount after GC: everything still correct.
  FlashEeprom fe2; fe2.begin(host_program, host_read);
  for (int id = 0; id < 20; ++id)
    CHECK(fe2.read(id, out, 48) == 48 && same(out, expect[id], 48), "survives remount after GC");
}

static void test_torn_write() {
  printf("test_torn_write (power loss mid record write -> previous value kept)\n");
  flash_erase_all();
  { FlashEeprom fe; fe.begin(host_program, host_read);
    uint8_t a[48]; fill(a, 48, 77);
    fe.write(9, a, 48);                       // good record
  }
  // Now crash on the NEXT program (a torn overwrite of blk9).
  g_prog_count = 0; g_crash_at = 1; g_torn = true;
  if (setjmp(g_crash_jmp) == 0) {
    FlashEeprom fe; fe.begin(host_program, host_read);
    uint8_t b[48]; fill(b, 48, 123);
    fe.write(9, b, 48);                       // dies mid-write
    CHECK(false, "should have crashed");
  }
  g_crash_at = -1; g_torn = false;

  // Reboot: blk9 must still read its last GOOD value (a), torn page ignored.
  FlashEeprom fe; fe.begin(host_program, host_read);
  uint8_t a[48], out[48]; fill(a, 48, 77);
  CHECK(fe.read(9, out, 48) == 48 && same(out, a, 48), "blk9 == last good value after torn write");
  // And we can still write after recovering.
  uint8_t c[48]; fill(c, 48, 5);
  CHECK(fe.write(9, c, 48), "write works after torn recovery");
  CHECK(fe.read(9, out, 48) == 48 && same(out, c, 48), "new value after recovery");
}

// Fill bank to the brim (append == FE_RECORD_PAGES+1) without triggering GC,
// recording the final value of each of 20 blocks. The NEXT write will GC.
static void fill_to_brim(uint8_t expect[20][48]) {
  flash_erase_all();
  FlashEeprom fe; fe.begin(host_program, host_read);
  int id = 0;
  while (fe.append_page() <= FE_RECORD_PAGES) {
    fill(expect[id], 48, uint8_t(id * 5 + fe.append_page()));
    fe.write(id, expect[id], 48);
    id = (id + 1) % 20;
  }
}

static void test_crash_mid_gc() {
  printf("test_crash_mid_gc (crash before spare header -> old bank unchanged)\n");
  uint8_t expect[20][48];
  fill_to_brim(expect);
  uint32_t gen_before;
  { FlashEeprom fe; fe.begin(host_program, host_read); gen_before = fe.active_gen(); }

  // Reboot; first write triggers GC. Crash at the 4th program = mid copy (the
  // 20 record copies are programs 1..20, spare header is program 21).
  g_prog_count = 0; g_crash_at = 4; g_torn = false;
  if (setjmp(g_crash_jmp) == 0) {
    FlashEeprom fe; fe.begin(host_program, host_read);
    uint8_t tmp[48]; fill(tmp, 48, 99);
    fe.write(0, tmp, 48);
    CHECK(false, "should have crashed mid-GC");
  }
  g_crash_at = -1;

  // Reboot: GC never committed -> old bank still active, gen unchanged, all
  // committed values intact.
  FlashEeprom fe; fe.begin(host_program, host_read);
  CHECK(fe.active_gen() == gen_before, "gen unchanged after crash mid-GC");
  uint8_t out[48];
  for (int id = 0; id < 20; ++id)
    CHECK(fe.read(id, out, 48) == 48 && same(out, expect[id], 48), "old value intact after crash mid-GC");
  uint8_t z[48]; fill(z, 48, 7);
  CHECK(fe.write(3, z, 48), "writable after crash mid-GC");
  CHECK(fe.read(3, out, 48) == 48 && same(out, z, 48), "consistent after recovery");
}

static void test_crash_after_gc_header() {
  printf("test_crash_after_gc_header (crash after spare header -> new bank wins)\n");
  uint8_t expect[20][48];
  fill_to_brim(expect);
  uint32_t gen_before;
  { FlashEeprom fe; fe.begin(host_program, host_read); gen_before = fe.active_gen(); }

  // Crash at program 22: 20 record copies (1..20) + spare header (21) done,
  // crash on invalidate-old (22). Both bank headers valid afterward.
  g_prog_count = 0; g_crash_at = 22; g_torn = false;
  if (setjmp(g_crash_jmp) == 0) {
    FlashEeprom fe; fe.begin(host_program, host_read);
    uint8_t tmp[48]; fill(tmp, 48, 55);
    fe.write(0, tmp, 48);
    CHECK(false, "should have crashed after header");
  }
  g_crash_at = -1;

  // Reboot: higher-gen (new) bank wins; values are the GC'd newest = expect[].
  FlashEeprom fe; fe.begin(host_program, host_read);
  CHECK(fe.active_gen() == gen_before + 1, "gen advanced (new bank chosen)");
  uint8_t out[48];
  for (int id = 0; id < 20; ++id)
    CHECK(fe.read(id, out, 48) == 48 && same(out, expect[id], 48), "values intact on new bank");
  uint8_t z[48]; fill(z, 48, 8);
  CHECK(fe.write(5, z, 48), "writable after committed-GC recovery");
  CHECK(fe.read(5, out, 48) == 48 && same(out, z, 48), "consistent after recovery");
}

static void test_variable_sizes() {
  printf("test_variable_sizes (patterns 48B, tracks 104B, settings 32B)\n");
  flash_erase_all();
  FlashEeprom fe; fe.begin(host_program, host_read);
  uint8_t pat[48], trk[104], cfg[32], out[104];
  fill(pat, 48, 1); fill(trk, 104, 2); fill(cfg, 32, 3);
  CHECK(fe.write(0, pat, 48), "write pattern");
  CHECK(fe.write(64, trk, 104), "write track");
  CHECK(fe.write(72, cfg, 32), "write settings");
  CHECK(fe.read(0, out, 104) == 48 && same(out, pat, 48), "pattern back");
  CHECK(fe.read(64, out, 104) == 104 && same(out, trk, 104), "track back");
  CHECK(fe.read(72, out, 104) == 32 && same(out, cfg, 32), "settings back");
}

int main() {
  test_basic();
  test_gc();
  test_torn_write();
  test_crash_mid_gc();
  test_crash_after_gc_header();
  test_variable_sizes();
  if (g_fail == 0) printf("\nALL TESTS PASSED\n");
  else printf("\n%d CHECK(S) FAILED\n", g_fail);
  return g_fail ? 1 : 0;
}
