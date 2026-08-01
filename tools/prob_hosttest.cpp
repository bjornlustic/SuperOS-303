// Host test for src/prob_codec.h. Round-trips step-probability tables through
// the sparse storage codec against
// the real FlashEeprom, including the dense-overflow fallback and the sharing
// of one record by PROB_SLOTS_PER_BLOCK slots.
#include "../src/sequence.h"
#include "../src/flash_eeprom.h"
#include <cstdio>
#include <cassert>
#include <cstring>
#include <vector>

static constexpr uint16_t kTotalPages = FE_ARENA_FIRST_PAGE + FE_ARENA_PAGES + 4;
static std::vector<uint8_t> mem(size_t(kTotalPages) * FE_PAGE, 0xFF);
static uint8_t prog(uint16_t p, const uint8_t *b) { memcpy(&mem[size_t(p)*FE_PAGE], b, FE_PAGE); return 0; }
static void rd(uint16_t p, uint8_t *b) { memcpy(b, &mem[size_t(p)*FE_PAGE], FE_PAGE); }
static FlashEeprom g_flash;

// Mirror of flash_persist.h's map (that header needs Arduino.h).
static constexpr uint16_t FB_MONOVAR2_BASE = NUM_SLOTS;
static constexpr uint16_t FB_POLY_BASE  = FB_MONOVAR2_BASE + NUM_SLOTS / 2;
static constexpr uint16_t FB_TRACK_BASE = FB_POLY_BASE + NUM_SLOTS;
static constexpr uint16_t FB_SETTINGS   = FB_TRACK_BASE + 8;
static constexpr uint8_t  PROB_SLOTS_PER_BLOCK = 3;
static constexpr uint8_t  PROB_REGION          = 80;
static constexpr uint8_t  PROB_SPARSE_MAX      = 31;
static constexpr uint8_t  FB_PROB_BLK_LEN      = PROB_SLOTS_PER_BLOCK * PROB_REGION;
static constexpr uint16_t FB_PROB_BLOCKS       = (NUM_SLOTS + PROB_SLOTS_PER_BLOCK - 1) / PROB_SLOTS_PER_BLOCK;
static constexpr uint16_t FB_PROB_BASE  = FB_SETTINGS + 1;
static constexpr uint16_t FB_PROBX_BASE = FB_PROB_BASE + FB_PROB_BLOCKS;
static constexpr uint8_t  FB_PROB_LEN   = 3 * MAX_STEPS;
#include "../src/prob_codec.h"

static void make_table(uint8_t *t, int n_armed, unsigned salt) {
  memset(t, 0, FB_PROB_LEN);
  int placed = 0;
  for (uint8_t s = 0; s < MAX_STEPS && placed < n_armed; ++s) {
    if ((s * 7 + salt) % MAX_STEPS >= (unsigned)(MAX_STEPS - n_armed)) {
      const uint8_t k = s * 3;
      t[k]     = uint8_t(1 + (s + salt) % 13) | uint8_t((1 + (s * 3 + salt) % 13) << 4);
      t[k + 1] = uint8_t(1 + (s * 5 + salt) % 13) | uint8_t((1 + (s + 2 * salt) % 13) << 4);
      t[k + 2] = ((s + salt) & 1);
      ++placed;
    }
  }
}

int main() {
  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  uint8_t in[FB_PROB_LEN], out[FB_PROB_LEN];

  // Every density, including both sides of the sparse/dense boundary.
  for (int n : {0, 1, 2, 8, 16, 30, 31, 32, 40, 64}) {
    make_table(in, n, unsigned(n));
    WriteProbAt(5, in);
    ReadProbAt(5, out);
    if (memcmp(in, out, FB_PROB_LEN) != 0) {
      printf("MISMATCH at n=%d\n", n);
      return 1;
    }
  }
  printf("1) round-trip at every density (0..64 armed): ok\n");

  // Three slots sharing one record must not disturb each other.
  uint8_t a[FB_PROB_LEN], b[FB_PROB_LEN], c[FB_PROB_LEN];
  make_table(a, 6, 1); make_table(b, 40, 2); make_table(c, 12, 3);
  WriteProbAt(0, a); WriteProbAt(1, b); WriteProbAt(2, c);
  ReadProbAt(0, out); assert(memcmp(a, out, FB_PROB_LEN) == 0);
  ReadProbAt(1, out); assert(memcmp(b, out, FB_PROB_LEN) == 0);
  ReadProbAt(2, out); assert(memcmp(c, out, FB_PROB_LEN) == 0);
  // Rewrite the middle one; neighbours must survive.
  make_table(b, 3, 9); WriteProbAt(1, b);
  ReadProbAt(0, out); assert(memcmp(a, out, FB_PROB_LEN) == 0);
  ReadProbAt(1, out); assert(memcmp(b, out, FB_PROB_LEN) == 0);
  ReadProbAt(2, out); assert(memcmp(c, out, FB_PROB_LEN) == 0);
  printf("2) slots sharing a record stay independent: ok\n");

  // An unarmed slot with no group record must consume nothing.
  const uint8_t free_before = g_flash.free_pages();
  memset(in, 0, FB_PROB_LEN);
  WriteProbAt(60, in);
  assert(g_flash.free_pages() == free_before);
  ReadProbAt(60, out);
  assert(memcmp(in, out, FB_PROB_LEN) == 0);
  printf("3) unarmed slot consumes no page: ok\n");

  // Clearing a slot inside an existing group must actually clear it.
  WriteProbAt(0, in);                    // in is all-zero here
  ReadProbAt(0, out); assert(memcmp(in, out, FB_PROB_LEN) == 0);
  ReadProbAt(2, out); assert(memcmp(c, out, FB_PROB_LEN) == 0);
  printf("4) clearing one slot leaves its neighbours: ok\n");

  // Pages consumed by N armed slots, sparse vs the old one-page-per-slot.
  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  const uint8_t start = g_flash.free_pages();
  make_table(in, 8, 4);
  for (uint8_t s = 0; s < 60; ++s) WriteProbAt(s, in);
  printf("5) 60 armed slots (8 steps each): %u pages, was 60: ok\n",
         unsigned(start - g_flash.free_pages()));
  return 0;
}
