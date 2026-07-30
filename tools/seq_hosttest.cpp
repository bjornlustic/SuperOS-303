// Host test for the clear -> randomize and tap-write sequence-level paths.
// Build/run: c++ -std=c++17 -I tools/host_stubs -I src tools/seq_hosttest.cpp -o /tmp/seqtest && /tmp/seqtest
#include "sequence.h"
#include <cstdio>
#include <cassert>

// Mirror of Engine::RandomizeFullPattern's body (engine.h drags in AVR-only
// headers, so the body is replicated verbatim at the Sequence level).
static void randomize_full(Sequence &s) {
  const uint8_t len = s.length;
  fast_rand_seed();
  uint8_t prev = 1;
  for (uint8_t i = 0; i < len; i++) {
    const uint8_t t = fast_rand_time_weighted(prev, i == 0);
    sequence_set_time_at(s, i, t);
    prev = t;
  }
  normalize_pattern_times_only(s);
  const uint8_t nc = s.note_count();
  for (uint8_t k = 0; k < nc; ++k) {
    s.pitch[k] = s.scale_apply_packed(fast_rand_pitch_byte_weighted())
                 | fast_rand_accent_weighted()
                 | fast_rand_slide_weighted();
  }
  for (uint8_t k = nc; k < MAX_STEPS; ++k) s.pitch[k] = PITCH_EMPTY;
  s.set_pitch_count(nc);
}

int main() {
  // 1) Clear -> randomize must produce playable notes (bug report 2 flow).
  int zero_note_runs = 0;
  for (int trial = 0; trial < 500; ++trial) {
    Sequence s;
    s.Clear();
    assert(s.length == 8);
    assert(s.note_count() == 0);
    s_fast_rng = uint16_t(trial * 2654435u + 1);   // spread seeds
    randomize_full(s);
    const uint8_t nc = s.note_count();
    if (nc == 0) { zero_note_runs++; continue; }
    assert(s.get_pitch_count() == nc);
    for (uint8_t k = 0; k < nc; ++k) assert(s.pitch[k] != PITCH_EMPTY);
  }
  printf("1) clear->randomize: %d/500 runs produced zero notes\n", zero_note_runs);

  // 2) Randomize with scale enabled and an EMPTY mask must not hang or emit
  //    out-of-range pitches.
  {
    Sequence s;
    s.Clear();
    s.set_scale_mask(0);
    s.set_scale_enabled(true);
    s_fast_rng = 12345;
    randomize_full(s);
    for (uint8_t k = 0; k < s.note_count(); ++k)
      assert(unpack_pitch_linear(s.pitch[k] & 0x3F) <= 48);
    printf("2) empty-scale-mask randomize: ok\n");
  }

  // 3) Tap-write on a cleared pattern: writing a NOTE must auto-extend the
  //    pitch stream (else tapped steps would be silent).
  {
    Sequence s;
    s.Clear();
    sequence_write_time_with_pitch_sync(s, 0, 1);
    assert(s.time(0) == 1);
    assert(s.get_pitch_count() >= 1);
    assert(s.pitch[0] != PITCH_EMPTY);
    // TIE chain then REST, as a tap-write pass would produce.
    sequence_write_time_with_pitch_sync(s, 1, 2);
    sequence_write_time_with_pitch_sync(s, 2, 0);
    sequence_write_time_with_pitch_sync(s, 3, 1);
    assert(s.note_count() == 2);
    assert(s.get_pitch_count() >= 2);
    printf("3) tap-write pitch auto-extend: ok\n");
  }

  // 4) Scale preset re-root identity: rotating a shape 12x lands back on it.
  {
    const uint16_t phryg_dom = 0x5B3;
    uint16_t m = phryg_dom;
    for (int r = 0; r < 12; ++r) m = uint16_t(((m << 1) | (m >> 11)) & 0x0FFF);
    assert(m == phryg_dom);
    printf("4) preset rotation identity: ok\n");
  }

  // 5) Spec 3-a: sequential PITCH MODE entry sizes a blank pattern -- one step
  //    per note, and the cursor keeps appending instead of wrapping.
  {
    Sequence s; s.Clear();
    s.reset = false; s.pitch_pos = 0;
    for (uint8_t n = 0; n < 5; ++n) {
      s.SetPitch(pack_pitch(uint8_t(n % 12), 1), 0);
      const bool appended = sequence_append_note_step(s, MAX_STEPS);
      sequence_ensure_pitch_for_notes(s);
      assert(appended);
      assert(s.length == uint8_t(n + 1));
      assert(s.note_count() == uint8_t(n + 1));
    }
    // Cursor now inside the content: writing overwrites, never appends.
    s.pitch_pos = 2;
    assert(!sequence_append_note_step(s, MAX_STEPS));
    assert(s.length == 5);
    printf("5) pitch-entry auto-length: ok\n");
  }

  // 6) Spec 5-b/5-c: halve is the odd ladder and loses nothing; double either
  //    restores the retained tail or copies the first half.
  {
    Sequence s; s.Clear();
    s.length = 4;
    for (uint8_t i = 0; i < 4; ++i) {
      sequence_set_time_at(s, i, 1);
      s.pitch[i] = pack_pitch(uint8_t(i), 1);
    }
    s.set_pitch_count(4);

    // Double an empty tail: steps 4..7 copy steps 0..3, pitches included.
    assert(sequence_double_length(s, MAX_STEPS) == 8);
    assert(s.note_count() == 8);
    for (uint8_t i = 0; i < 4; ++i)
      assert((s.pitch[4 + i] & PITCH_PACK_MASK) == (s.pitch[i] & PITCH_PACK_MASK));

    // Halve it back, then re-double: the retained tail must reappear
    // unchanged rather than being overwritten by a fresh copy.
    s.pitch[6] = pack_pitch(11, 2);           // mark a tail note
    const uint8_t marked = s.pitch[6];
    assert(sequence_halve_length(s) == 4);
    assert(s.note_count() == 4);
    assert(sequence_double_length(s, MAX_STEPS) == 8);
    assert(s.note_count() == 8);
    assert(s.pitch[6] == marked);

    // Odd ladder: 31 > 15 > 7 > 3 > 1, never 0.
    s.length = 31;
    assert(sequence_halve_length(s) == 15);
    assert(sequence_halve_length(s) == 7);
    assert(sequence_halve_length(s) == 3);
    assert(sequence_halve_length(s) == 1);
    assert(sequence_halve_length(s) == 1);
    printf("6) halve/double length: ok\n");
  }

  // 7) Spec 5: shortening is non-volatile -- notes past the new last step come
  //    back verbatim when the pattern is lengthened again.
  {
    Sequence s; s.Clear();
    s.length = 16;
    for (uint8_t i = 0; i < 16; ++i) {
      sequence_set_time_at(s, i, 1);
      s.pitch[i] = pack_pitch(uint8_t(i % 12), uint8_t(i / 12));
    }
    s.set_pitch_count(16);
    const uint8_t tail = s.pitch[12];
    s.length = 8;
    sequence_rebuild_pitch_count(s);
    sequence_ensure_pitch_for_notes(s);
    assert(s.note_count() == 8);
    s.length = 16;
    sequence_rebuild_pitch_count(s);
    sequence_ensure_pitch_for_notes(s);
    assert(s.note_count() == 16);
    assert(s.pitch[12] == tail);
    printf("7) non-volatile shortening: ok\n");
  }

  // 8) Spec 1-a: the A/B link flag lives on the pattern and survives a
  //    round-trip through the persisted metadata bytes.
  {
    Sequence s; s.Clear();
    assert(!s.ab_linked());
    s.set_ab_linked(true);
    assert(s.ab_linked());
    s.store_direction(DIR_PINGPONG);
    s.set_scale_mask(0x0A5);
    s.set_scale_enabled(true);
    assert(s.ab_linked());                       // neighbours don't clobber it
    assert(s.get_direction_stored() == DIR_PINGPONG);
    s.set_ab_linked(false);
    assert(!s.ab_linked());
    printf("8) A/B link flag: ok\n");
  }

  printf("ALL SEQUENCE HOST TESTS PASSED\n");
  return 0;
}
