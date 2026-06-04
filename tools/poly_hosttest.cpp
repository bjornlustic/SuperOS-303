// Host test for src/poly.h (PolyVoice).
// Build/run: c++ -std=c++14 -I src tools/poly_hosttest.cpp -o /tmp/polytest && /tmp/polytest
#include "../src/poly.h"
#include <cstdio>
#include <cassert>
#include <cstring>

static inline uint8_t pk(uint8_t semi, uint8_t oct) { return uint8_t(semi | (oct << 4)); }

int main() {
  // ---- add / duplicate / full ----
  PolyVoice p; p.Clear();
  assert(p.note_count(0) == 0);
  assert(p.add_note(0, pk(0, 1)));   // C  oct1
  assert(p.add_note(0, pk(4, 1)));   // E  oct1
  assert(p.add_note(0, pk(7, 1)));   // G  oct1
  assert(p.add_note(0, pk(12, 2)));  // high-C oct2
  assert(p.note_count(0) == 4);
  assert(!p.add_note(0, pk(2, 1)));  // step full -> rejected
  assert(!p.add_note(0, pk(4, 1)));  // duplicate -> rejected
  assert(p.has_note(0, pk(7, 1)));
  assert(!p.has_note(0, pk(7, 0)));

  // remove frees a slot
  assert(p.remove_note(0, pk(7, 1)));
  assert(p.note_count(0) == 3);
  assert(p.add_note(0, pk(2, 1)));   // now fits
  assert(p.note_count(0) == 4);

  // ---- octave move: range + collision ----
  p.Clear();
  assert(p.add_note(3, pk(9, 1)));   // A oct1
  assert(p.move_note_octave(3, pk(9, 1), +1)); // A1 -> A2
  assert(p.has_note(3, pk(9, 2)) && !p.has_note(3, pk(9, 1)));
  assert(p.move_note_octave(3, pk(9, 2), +1));  // A2 -> A3 (oct3 in range)
  assert(p.has_note(3, pk(9, 3)));
  assert(!p.move_note_octave(3, pk(9, 3), +1)); // A3 -> A4 out of range -> blocked
  // re-add A1 and A0, then a down-move that would collide
  p.Clear();
  assert(p.add_note(3, pk(9, 0)));   // A oct0
  assert(!p.move_note_octave(3, pk(9, 0), -1)); // oct -1 out of range -> blocked
  assert(p.add_note(3, pk(9, 1)));   // A oct1
  assert(!p.move_note_octave(3, pk(9, 1), -1)); // A1 -> A0 but A0 present -> blocked
  assert(p.remove_note(3, pk(9, 0)));
  assert(p.move_note_octave(3, pk(9, 1), -1)); // now free -> ok
  assert(p.has_note(3, pk(9, 0)));
  assert(!p.move_note_octave(3, pk(9, 9), -1)); // moving a note that isn't there -> false

  // ---- time / accent / slide are step-level ----
  p.set_time(5, 2);  assert(p.time(5) == 2);
  p.set_time(5, 1);  assert(p.time(5) == 1);
  p.set_accent(5, true); assert(p.accent(5) && !p.accent(6));
  p.toggle_slide(5); assert(p.slide(5));
  p.toggle_slide(5); assert(!p.slide(5));

  // ---- serialize round-trip ----
  PolyVoice q; q.Clear();
  assert(q.add_note(0, pk(1, 0)));
  assert(q.add_note(0, pk(11, 3)));
  assert(q.add_note(0, pk(0, 2)));
  assert(q.add_note(10, pk(6, 2)));
  q.set_time(0, 1); q.set_time(10, 2);
  q.set_accent(10, true); q.set_slide(0, true);
  q.length = 33; q.direction = 2;

  uint8_t blob[POLY_BLOB_SIZE];
  q.serialize(blob);
  PolyVoice r; r.deserialize(blob);
  assert(memcmp(q.notes, r.notes, sizeof(q.notes)) == 0);
  assert(memcmp(q.time_data, r.time_data, sizeof(q.time_data)) == 0);
  assert(memcmp(q.acc_bits, r.acc_bits, sizeof(q.acc_bits)) == 0);
  assert(memcmp(q.sld_bits, r.sld_bits, sizeof(q.sld_bits)) == 0);
  assert(r.length == 33 && r.direction == 2);
  assert(r.accent(10) && r.slide(0) && r.time(10) == 2);
  assert(r.note_count(0) == 3 && r.has_note(0, pk(11, 3)) && r.has_note(0, pk(6, 2)) == false);
  assert(r.has_note(10, pk(6, 2)));

  // exhaustive per-step 4-voice pack/unpack across every 6-bit value
  PolyVoice g; g.Clear();
  for (uint8_t s = 0; s < POLY_STEPS; ++s) {
    uint8_t *v = g.step(s);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      v[i] = uint8_t((s * 4 + i) & 0x3F); // spread all 6-bit patterns
  }
  uint8_t gb[POLY_BLOB_SIZE];
  g.serialize(gb);
  PolyVoice h; h.deserialize(gb);
  assert(memcmp(g.notes, h.notes, sizeof(g.notes)) == 0);

  printf("POLY_BLOB_SIZE=%d (one record holds <=243)  all poly tests passed\n", POLY_BLOB_SIZE);
  return 0;
}
