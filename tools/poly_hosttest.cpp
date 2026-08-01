// Host test for src/poly.h (PolyVoice, chord-list / two-stream model).
// Build/run: c++ -std=c++14 -I src tools/poly_hosttest.cpp -o /tmp/polytest && /tmp/polytest
#include "../src/poly.h"
#include <cstdio>
#include <cassert>
#include <cstring>

static inline uint8_t pk(uint8_t semi, uint8_t oct) { return uint8_t(semi | (oct << 4)); }

int main() {
  // ---- chord voices: add / duplicate / full / remove (indexed by CHORD) ----
  PolyVoice p; p.Clear();
  assert(p.voice_count(0) == 0);
  assert(p.add_note(0, pk(0, 1)));   // C  oct1
  assert(p.add_note(0, pk(4, 1)));   // E  oct1
  assert(p.add_note(0, pk(7, 1)));   // G  oct1
  assert(p.add_note(0, pk(12, 2)));  // high-C oct2
  assert(p.voice_count(0) == 4);
  assert(!p.add_note(0, pk(2, 1)));  // chord full -> rejected
  assert(!p.add_note(0, pk(4, 1)));  // duplicate -> rejected
  assert(p.has_note(0, pk(7, 1)));
  assert(!p.has_note(0, pk(7, 0)));
  assert(p.remove_note(0, pk(7, 1)));
  assert(p.voice_count(0) == 3);
  assert(p.add_note(0, pk(2, 1)));   // now fits
  assert(p.voice_count(0) == 4);

  // ---- octave move: range + collision (within a chord) ----
  p.Clear();
  assert(p.add_note(3, pk(9, 1)));               // A oct1
  assert(p.move_note_octave(3, pk(9, 1), +1));   // A1 -> A2
  assert(p.has_note(3, pk(9, 2)) && !p.has_note(3, pk(9, 1)));
  assert(p.move_note_octave(3, pk(9, 2), +1));   // A2 -> A3
  assert(p.has_note(3, pk(9, 3)));
  assert(!p.move_note_octave(3, pk(9, 3), +1));  // A3 -> A4 out of range
  p.Clear();
  assert(p.add_note(3, pk(9, 0)));
  assert(!p.move_note_octave(3, pk(9, 0), -1));   // oct -1 out of range
  assert(p.add_note(3, pk(9, 1)));
  assert(!p.move_note_octave(3, pk(9, 1), -1));   // A1 -> A0 but A0 present -> blocked
  assert(p.remove_note(3, pk(9, 0)));
  assert(p.move_note_octave(3, pk(9, 1), -1));    // now free -> ok
  assert(p.has_note(3, pk(9, 0)));
  assert(!p.move_note_octave(3, pk(9, 9), -1));   // moving a note that isn't there -> false

  // ---- two-stream mapping: time is per step, chord index = NOTE-events-before ----
  PolyVoice m; m.Clear();
  m.length = 8;
  m.set_time(0, 1);          // NOTE  -> chord 0
  m.set_time(1, 0);          // REST
  m.set_time(2, 1);          // NOTE  -> chord 1
  m.set_time(3, 2);          // TIE   (holds chord 1)
  m.set_time(4, 1);          // NOTE  -> chord 2
  assert(m.note_count() == 3);
  assert(m.chord_index_for_step(0) == 0);
  assert(m.chord_index_for_step(2) == 1);
  // count_notes_to is the raw NOTE-count-before; a TIE step is never queried for a
  // new index at playback (the engine holds the prior chord across a TIE).
  assert(m.chord_index_for_step(3) == 2);
  assert(m.chord_index_for_step(4) == 2);
  // ensure grows the chord stream to cover all NOTEs, with default chords
  assert(m.get_chord_count() == 0);
  m.ensure_chords_for_notes();
  assert(m.get_chord_count() == 3);
  assert(m.has_note(0, POLY_DEFAULT) && m.has_note(2, POLY_DEFAULT));
  // inserting a NOTE shifts the mapping; ensure appends a new default at the end
  m.set_time(1, 1);          // step1 REST -> NOTE
  assert(m.note_count() == 4);
  assert(m.chord_index_for_step(1) == 1); // step1 now pulls chord 1
  assert(m.chord_index_for_step(2) == 2); // old chord-1 step now pulls chord 2
  m.ensure_chords_for_notes();
  assert(m.get_chord_count() == 4);
  // removing a NOTE keeps the chord queued (count does not shrink)
  m.set_time(1, 0);
  m.ensure_chords_for_notes();
  assert(m.get_chord_count() == 4 && m.note_count() == 3);

  // ---- accent / slide are per CHORD ----
  PolyVoice a; a.Clear();
  a.set_accent(5, true); assert(a.accent(5) && !a.accent(6));
  a.toggle_slide(5); assert(a.slide(5));
  a.toggle_slide(5); assert(!a.slide(5));

  // ---- serialize round-trip ----
  PolyVoice q; q.Clear();
  assert(q.add_note(0, pk(1, 0)));
  assert(q.add_note(0, pk(11, 3)));
  assert(q.add_note(0, pk(0, 2)));
  assert(q.add_note(10, pk(6, 2)));
  q.set_time(0, 1); q.set_time(10, 2);
  q.set_accent(10, true); q.set_slide(0, true);
  q.set_chord_count(11);
  q.length = 33; q.direction = 2;

  uint8_t blob[POLY_BLOB_SIZE];
  q.serialize(blob);
  PolyVoice r; r.deserialize(blob);
  assert(memcmp(q.notes, r.notes, sizeof(q.notes)) == 0);
  assert(memcmp(q.time_data, r.time_data, sizeof(q.time_data)) == 0);
  assert(memcmp(q.acc_bits, r.acc_bits, sizeof(q.acc_bits)) == 0);
  assert(memcmp(q.sld_bits, r.sld_bits, sizeof(q.sld_bits)) == 0);
  assert(r.length == 33 && r.direction == 2 && r.get_chord_count() == 11);
  assert(r.accent(10) && r.slide(0) && r.time(10) == 2);
  assert(r.voice_count(0) == 3 && r.has_note(0, pk(11, 3)) && r.has_note(0, pk(6, 2)) == false);
  assert(r.has_note(10, pk(6, 2)));

  // exhaustive per-chord 4-voice pack/unpack across every 6-bit value
  PolyVoice g; g.Clear();
  for (uint8_t i = 0; i < POLY_CHORDS; ++i) {
    uint8_t *v = g.chord(i);
    for (uint8_t k = 0; k < POLY_VOICES; ++k)
      v[k] = uint8_t((i * 4 + k) & 0x3F);
  }
  uint8_t gb[POLY_BLOB_SIZE];
  g.serialize(gb);
  PolyVoice h; h.deserialize(gb);
  assert(memcmp(g.notes, h.notes, sizeof(g.notes)) == 0);

  printf("POLY_BLOB_SIZE=%d (one record holds <=243)  all poly chord-list tests passed\n", POLY_BLOB_SIZE);
  return 0;
}
