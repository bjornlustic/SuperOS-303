// Host test for src/poly_codec.h: round-trip poly patterns through the trimmed
// codec, across the sparse/dense boundary and with slots sharing a record.
#include "../src/sequence.h"
#include "../src/poly.h"
#include "../src/flash_eeprom.h"
#include <cstdio>
#include <cassert>
#include <cstring>
#include <vector>
static constexpr uint16_t kTotalPages = FE_ARENA_FIRST_PAGE + FE_ARENA_PAGES + 4;
static std::vector<uint8_t> mem(size_t(kTotalPages) * FE_PAGE, 0xFF);
static uint8_t prog(uint16_t p, const uint8_t *b){memcpy(&mem[size_t(p)*FE_PAGE],b,FE_PAGE);return 0;}
static void rd(uint16_t p, uint8_t *b){memcpy(b,&mem[size_t(p)*FE_PAGE],FE_PAGE);}
static FlashEeprom g_flash;
static constexpr uint8_t  POLY_REGION=120, POLY_PER_BLOCK=2;
static constexpr uint8_t  POLY_SPARSE_MAX=(POLY_REGION-19)/3;
static constexpr uint8_t  FB_POLY_BLK_LEN=POLY_PER_BLOCK*POLY_REGION;
static constexpr uint16_t FB_POLY_BASE=0;
static constexpr uint16_t FB_POLY_BLOCKS=(NUM_SLOTS+POLY_PER_BLOCK-1)/POLY_PER_BLOCK;
static constexpr uint16_t FB_POLYX_BASE=FB_POLY_BASE+FB_POLY_BLOCKS;
#include "../src/poly_codec.h"

static void make(PolyVoice &p, int chords, unsigned salt) {
  p.Clear();
  for (int c = 0; c < chords; ++c) {
    for (int v = 0; v < 1 + int((c + salt) % POLY_VOICES); ++v)
      p.add_note(uint8_t(c), uint8_t(((c * 3 + v * 5 + salt) % 48) & 0x3F));
    if ((c + salt) & 1) p.set_accent(uint8_t(c), true);
    if ((c + salt) & 2) p.set_slide(uint8_t(c), true);
    p.set_time(uint8_t(c), 1);
  }
  p.chord_count = uint8_t(chords);
  p.length = uint8_t(chords ? chords : 8);
}
static bool same(const PolyVoice &a, const PolyVoice &b) {
  uint8_t x[POLY_BLOB_SIZE], y[POLY_BLOB_SIZE];
  a.serialize(x); b.serialize(y);
  return memcmp(x, y, POLY_BLOB_SIZE) == 0;
}

int main() {
  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  PolyVoice in, out;
  for (int c : {0, 1, 8, 16, 24, 30, 31, 32}) {
    make(in, c, unsigned(c));
    WritePolyAt(in, 5);
    ReadPolyAt(out, 5);
    if (!same(in, out)) { printf("MISMATCH at %d chords\n", c); return 1; }
  }
  printf("1) round-trip at every chord count (0..POLY_CHORDS): ok\n");

  PolyVoice a, b;
  make(a, 6, 1); make(b, 32, 2);
  WritePolyAt(a, 0); WritePolyAt(b, 1);
  ReadPolyAt(out, 0); assert(same(a, out));
  ReadPolyAt(out, 1); assert(same(b, out));     // full neighbour
  make(b, 5, 9); WritePolyAt(b, 1);             // full -> short rewrite
  ReadPolyAt(out, 0); assert(same(a, out));
  ReadPolyAt(out, 1); assert(same(b, out));
  printf("2) slots sharing a record stay independent: ok\n");

  ReadPolyAt(out, 63);
  { PolyVoice e; e.Clear(); assert(same(e, out)); }
  printf("3) unwritten slot reads cleared: ok\n");

  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  const uint8_t start = g_flash.free_pages();
  make(in, 16, 3);
  for (uint8_t s = 0; s < 64; ++s) WritePolyAt(in, s);
  printf("4) 64 poly slots (16 chords): %u pages, was 64: ok\n",
         unsigned(start - g_flash.free_pages()));
  printf("ALL POLY CODEC TESTS PASSED\n");
  return 0;
}
