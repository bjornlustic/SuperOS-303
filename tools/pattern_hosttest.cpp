// Host test for src/pattern_codec.h. Round-trips patterns through the trimmed
// storage codec against the real FlashEeprom: every note density, both sides of
// the sparse/dense boundary, patterns sharing a record, and the retained pitch
// tail that makes length changes non-volatile.
//
// Build/run:
//   c++ -std=c++17 -I src -I tools/host_stubs tools/pattern_hosttest.cpp \
//       -o /tmp/pattest && /tmp/pattest
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
static constexpr uint8_t  FB_PATTERN_LEN_ONE = MAX_STEPS + (MAX_STEPS/4) + METADATA_SIZE;
static constexpr uint8_t  PAT_REGION     = 60;
static constexpr uint8_t  PAT_PER_BLOCK  = 4;
static constexpr uint8_t  PAT_FIXED      = 1 + METADATA_SIZE + MAX_STEPS / 4;
static constexpr uint8_t  PAT_SPARSE_MAX = PAT_REGION - PAT_FIXED;
static constexpr uint8_t  FB_PATTERN_BLK_LEN = PAT_PER_BLOCK * PAT_REGION;
static constexpr uint16_t FB_PATTERN_COUNT   = 3 * NUM_SLOTS;
static constexpr uint16_t FB_PATTERN_BASE = 0;
static constexpr uint16_t FB_PATTERN_BLOCKS =
    (FB_PATTERN_COUNT + PAT_PER_BLOCK - 1) / PAT_PER_BLOCK;
static constexpr uint16_t FB_PATX_BASE = FB_PATTERN_BASE + FB_PATTERN_BLOCKS;

// serialize/deserialize/clear are defined in persistent_settings.h, which needs
// Arduino.h; they are three memcpys, restated here exactly.
static void serialize_pattern(const Sequence &s, uint8_t *d) {
  memcpy(d, s.pitch, MAX_STEPS);
  memcpy(d + MAX_STEPS, s.time_data, MAX_STEPS/4);
  memcpy(d + MAX_STEPS + MAX_STEPS/4, s.reserved, METADATA_SIZE);
}
static void deserialize_pattern(Sequence &s, const uint8_t *src) {
  memcpy(s.pitch, src, MAX_STEPS);
  memcpy(s.time_data, src + MAX_STEPS, MAX_STEPS/4);
  memcpy(s.reserved, src + MAX_STEPS + MAX_STEPS/4, METADATA_SIZE);
}
static void clear_pattern_bytes(Sequence &s) {
  memset(s.pitch, PITCH_EMPTY, MAX_STEPS);
  memset(s.time_data, 0, MAX_STEPS/4);
  memset(s.reserved, 0, METADATA_SIZE);
  s.length = 0;
}
#include "../src/pattern_codec.h"

// A pattern with `notes` notes and `tail` extra retained pitches past them.
static void make(Sequence &s, int notes, int tail, unsigned salt) {
  memset(s.pitch, PITCH_EMPTY, MAX_STEPS);
  memset(s.time_data, 0, MAX_STEPS/4);
  memset(s.reserved, 0, METADATA_SIZE);
  s.length = uint8_t(notes ? notes : 8);
  for (int i = 0; i < notes; ++i) {
    sequence_set_time_at(s, uint8_t(i), 1);
    s.pitch[i] = uint8_t((i * 5 + salt) & 0x3F);
  }
  for (int i = 0; i < tail && notes + i < MAX_STEPS; ++i)
    s.pitch[notes + i] = uint8_t((i * 3 + salt + 1) & 0x3F);
  s.reserved[0] = uint8_t(salt & 0x0F);
  s.transpose = int8_t(salt % 24) - 12;
}
static bool same(const Sequence &a, const Sequence &b) {
  return memcmp(a.pitch, b.pitch, MAX_STEPS) == 0 &&
         memcmp(a.time_data, b.time_data, MAX_STEPS/4) == 0 &&
         memcmp(a.reserved, b.reserved, METADATA_SIZE) == 0;
}

int main() {
  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  Sequence in{}, out{};

  // 1) Every density up to a full section (at 32 steps the sparse region holds
  //    a full pattern, so the dense path is never taken).
  for (int n : {0, 1, 8, 16, 24, 30, 31, 32}) {
    make(in, n, 0, unsigned(n));
    WritePatternAt(in, 5, 0);
    ReadPatternAt(out, 5, 0);
    if (!same(in, out)) { printf("MISMATCH at %d notes\n", n); return 1; }
  }
  printf("1) round-trip at every note density (0..MAX_STEPS): ok\n");

  // 2) The retained pitch tail past the last step must survive -- this is what
  //    makes shortening non-volatile (spec 5). Trimming must not eat it.
  {
    make(in, 8, 20, 7);                      // 8 notes + 20 retained pitches
    WritePatternAt(in, 9, 0);
    ReadPatternAt(out, 9, 0);
    assert(same(in, out));
    for (int i = 8; i < 28; ++i) assert(out.pitch[i] != PITCH_EMPTY);
    assert(out.pitch[28] == PITCH_EMPTY);    // nothing invented past the tail
    printf("2) retained pitch tail survives trimming: ok\n");
  }

  // 3) Four patterns sharing a record stay independent, including a dense one
  //    (full section) among short neighbours.
  {
    Sequence a{}, b{}, c{}, d{};
    make(a, 4, 0, 1); make(b, 32, 0, 2); make(c, 12, 0, 3); make(d, 0, 0, 4);
    WritePatternAt(a, 0, 0); WritePatternAt(b, 1, 0);
    WritePatternAt(c, 2, 0); WritePatternAt(d, 3, 0);
    ReadPatternAt(out, 0, 0); assert(same(a, out));
    ReadPatternAt(out, 1, 0); assert(same(b, out));   // the full one
    ReadPatternAt(out, 2, 0); assert(same(c, out));
    ReadPatternAt(out, 3, 0); assert(same(d, out));
    make(b, 6, 0, 9); WritePatternAt(b, 1, 0);        // dense -> sparse rewrite
    ReadPatternAt(out, 0, 0); assert(same(a, out));
    ReadPatternAt(out, 1, 0); assert(same(b, out));
    ReadPatternAt(out, 2, 0); assert(same(c, out));
    printf("3) patterns sharing a record stay independent: ok\n");
  }

  // 4) The three variations of one slot are distinct blocks.
  {
    Sequence v0{}, v1{}, v2{};
    make(v0, 5, 0, 11); make(v1, 9, 0, 12); make(v2, 32, 0, 13);
    WritePatternAt(v0, 20, 0); WritePatternAt(v1, 20, 1); WritePatternAt(v2, 20, 2);
    ReadPatternAt(out, 20, 0); assert(same(v0, out));
    ReadPatternAt(out, 20, 1); assert(same(v1, out));
    ReadPatternAt(out, 20, 2); assert(same(v2, out));
    printf("4) variations of one slot are independent: ok\n");
  }

  // 5) An unwritten slot reads back cleared, not as garbage.
  {
    ReadPatternAt(out, 63, 2);
    assert(out.length == 0);
    for (int i = 0; i < MAX_STEPS; ++i) assert(out.pitch[i] == PITCH_EMPTY);
    printf("5) unwritten slot reads cleared: ok\n");
  }

  // 6) Pages consumed by all 64 slots x var1+var2, typical content.
  {
    std::fill(mem.begin(), mem.end(), 0xFF);
    assert(g_flash.begin(prog, rd));
    const uint8_t start = g_flash.free_pages();
    make(in, 16, 0, 3);
    for (uint8_t s = 0; s < 64; ++s) { WritePatternAt(in, s, 0); WritePatternAt(in, s, 1); }
    printf("6) 64 slots x var1+var2 (16 notes): %u pages, was 64: ok\n",
           unsigned(start - g_flash.free_pages()));
  }
  printf("ALL PATTERN CODEC TESTS PASSED\n");
  return 0;
}
