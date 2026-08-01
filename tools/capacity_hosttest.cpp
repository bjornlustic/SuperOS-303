// Worst-case capacity proof for the 32-step combined-build layout: every slot
// fully armed -- all 192 patterns written full (32 notes), poly on all 64
// slots (32 chords), probability on all 64 slots (every step armed), all 8
// tracks and settings -- must fit the combined arena with the update-reserve
// page still free. This is the guarantee the MAX_STEPS = 32 layout exists for.
//
// Build/run:
//   c++ -std=c++17 -DSUPEROS_COMBINED -I src -I tools/host_stubs \
//       tools/capacity_hosttest.cpp -o /tmp/captest && /tmp/captest
#include "../src/sequence.h"
#include "../src/poly.h"
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
static constexpr uint8_t  POLY_REGION = 120, POLY_PER_BLOCK = 2;
static constexpr uint8_t  POLY_SPARSE_MAX = (POLY_REGION - 19) / 3;
static constexpr uint8_t  FB_POLY_BLK_LEN = POLY_PER_BLOCK * POLY_REGION;
static constexpr uint16_t FB_POLY_BASE  = FB_PATX_BASE + FB_PATTERN_COUNT;
static constexpr uint16_t FB_POLY_BLOCKS = (NUM_SLOTS + POLY_PER_BLOCK - 1) / POLY_PER_BLOCK;
static constexpr uint16_t FB_POLYX_BASE = FB_POLY_BASE + FB_POLY_BLOCKS;
static constexpr uint8_t  TRACK_PER_BLOCK = 2;
static constexpr uint16_t FB_TRACK_BASE = FB_POLYX_BASE + NUM_SLOTS;
static constexpr uint16_t FB_TRACK_BLOCKS = (8 + TRACK_PER_BLOCK - 1) / TRACK_PER_BLOCK;
static constexpr uint16_t FB_SETTINGS   = FB_TRACK_BASE + FB_TRACK_BLOCKS;
static constexpr uint8_t  PROB_SLOTS_PER_BLOCK = 3;
static constexpr uint8_t  PROB_REGION = 80;
static constexpr uint8_t  PROB_SPARSE_MAX = MAX_STEPS;
static constexpr uint8_t  FB_PROB_BLK_LEN = PROB_SLOTS_PER_BLOCK * PROB_REGION;
static constexpr uint16_t FB_PROB_BLOCKS =
    (NUM_SLOTS + PROB_SLOTS_PER_BLOCK - 1) / PROB_SLOTS_PER_BLOCK;
static constexpr uint16_t FB_PROB_BASE  = FB_SETTINGS + 1;
static constexpr uint16_t FB_PROBX_BASE = FB_PROB_BASE + FB_PROB_BLOCKS;
static constexpr uint8_t  FB_TRACK_LEN = 104;
static constexpr uint8_t  FB_PROB_LEN  = 3 * MAX_STEPS;

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
#include "../src/poly_codec.h"

// prob_codec.h needs the same includes; ReadProbAt/WriteProbAt only.
#include "../src/prob_codec.h"

static void make_pat(Sequence &s, unsigned salt) {   // full section: 32 notes
  clear_pattern_bytes(s);
  s.length = MAX_STEPS;
  for (int i = 0; i < MAX_STEPS; ++i) {
    sequence_set_time_at(s, uint8_t(i), 1);
    s.pitch[i] = uint8_t((i * 5 + salt) & 0x3F);
  }
  s.reserved[0] = uint8_t(salt & 0x07);
  s.transpose = int8_t(salt % 24) - 12;
}
static bool same_pat(const Sequence &a, const Sequence &b) {
  return memcmp(a.pitch, b.pitch, MAX_STEPS) == 0 &&
         memcmp(a.time_data, b.time_data, MAX_STEPS/4) == 0 &&
         memcmp(a.reserved, b.reserved, METADATA_SIZE) == 0;
}
static void make_poly(PolyVoice &p, unsigned salt) { // full: 32 chords, 4 voices
  p.Clear();
  for (int c = 0; c < POLY_CHORDS; ++c) {
    for (int v = 0; v < POLY_VOICES; ++v)
      p.add_note(uint8_t(c), uint8_t(((c * 3 + v * 5 + salt) % 48) & 0x3F));
    p.set_time(uint8_t(c), 1);
    if ((c + salt) & 1) p.set_accent(uint8_t(c), true);
  }
  p.chord_count = POLY_CHORDS;
  p.length = POLY_CHORDS;
}

int main() {
  std::fill(mem.begin(), mem.end(), 0xFF);
  assert(g_flash.begin(prog, rd));
  const uint16_t start = g_flash.free_pages();

  Sequence s{}, out{};
  for (uint8_t v = 0; v < 3; ++v)
    for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) {
      make_pat(s, unsigned(v) * 64u + sl);
      WritePatternAt(s, sl, v);
    }
  PolyVoice pv, pout;
  for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) { make_poly(pv, sl); WritePolyAt(pv, sl); }
  uint8_t prob[FB_PROB_LEN];
  for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) {
    for (int st = 0; st < MAX_STEPS; ++st) {       // every step armed
      prob[st*3]   = uint8_t(0x11 + (st & 3));
      prob[st*3+1] = uint8_t(0x22 + (sl & 3));
      prob[st*3+2] = uint8_t(st & 1);
    }
    WriteProbAt(sl, prob);
  }
  uint8_t trk[TRACK_PER_BLOCK * FB_TRACK_LEN];
  for (uint8_t t = 0; t < 8; t += TRACK_PER_BLOCK) {
    memset(trk, uint8_t(t + 1), sizeof(trk));
    assert(g_flash.write(uint16_t(FB_TRACK_BASE + t / TRACK_PER_BLOCK), trk, sizeof(trk)));
  }
  uint8_t cfg[32]; memset(cfg, 0x5A, sizeof(cfg));
  assert(g_flash.write(FB_SETTINGS, cfg, sizeof(cfg)));

  const uint16_t used = uint16_t(start - g_flash.free_pages());
  printf("1) everything armed: %u pages used of %u records: %s\n",
         used, unsigned(FE_RECORD_PAGES),
         used <= FE_RECORD_PAGES - 1 ? "ok" : "OVER BUDGET");
  assert(used <= FE_RECORD_PAGES - 1);          // update reserve still free
  assert(g_flash.free_pages() >= 1);

  // Everything reads back intact.
  for (uint8_t v = 0; v < 3; ++v)
    for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) {
      make_pat(s, unsigned(v) * 64u + sl);
      ReadPatternAt(out, sl, v);
      assert(same_pat(s, out));
    }
  for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) {
    make_poly(pv, sl);
    ReadPolyAt(pout, sl);
    uint8_t x[POLY_BLOB_SIZE], y[POLY_BLOB_SIZE];
    pv.serialize(x); pout.serialize(y);
    assert(memcmp(x, y, POLY_BLOB_SIZE) == 0);
  }
  uint8_t pr[FB_PROB_LEN];
  for (uint8_t sl = 0; sl < NUM_SLOTS; ++sl) {
    for (int st = 0; st < MAX_STEPS; ++st) {
      prob[st*3]   = uint8_t(0x11 + (st & 3));
      prob[st*3+1] = uint8_t(0x22 + (sl & 3));
      prob[st*3+2] = uint8_t(st & 1);
    }
    ReadProbAt(sl, pr);
    assert(memcmp(prob, pr, FB_PROB_LEN) == 0);
  }
  printf("2) full worst case reads back intact: ok\n");

  // The reserve does its job: an existing block can still be updated at the
  // worst-case fill.
  make_pat(s, 999);
  WritePatternAt(s, 0, 0);
  ReadPatternAt(out, 0, 0);
  assert(same_pat(s, out));
  printf("3) update of an existing block still works at full fill: ok\n");
  printf("ALL CAPACITY TESTS PASSED (%u of %u record pages)\n",
         used, unsigned(FE_RECORD_PAGES));
  return 0;
}
