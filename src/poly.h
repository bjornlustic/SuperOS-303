// poly.h -- variation 3 polyphonic voice (chord-list / two-stream model).
//
// Variation 3 in poly mode is a two-stream voice, exactly like the mono voice
// (variations 1/2): a CHORD STREAM (a flat list of chords, each up to POLY_VOICES
// notes plus its own accent + slide) and an independent TIME STREAM (rest/note/
// tie per step). The K-th NOTE event in time order plays chords[K]; a TIE holds
// the previous chord; a REST is silent. Inserting a rest/tie/note shifts which
// chord each NOTE pulls -- chords travel WITH the list, not the step position,
// matching how pitches behave in variations 1/2.
//
// Accent and slide are per-CHORD (they ride with the chord in the list), like the
// accent/slide bits ride on the mono pitch byte. Pitches use a 6-bit packed form
// (bits[3:0]=semi 0..12, bits[5:4]=oct 0..3); POLY_EMPTY marks an unused voice.
//
// Pure (stdint/string only) so it host-tests without the Arduino platform.
#pragma once
#include <stdint.h>
#include <string.h>

#ifndef POLY_STEPS
#define POLY_STEPS 32   // matches MAX_STEPS (32 steps per section)
#endif

static constexpr uint8_t POLY_VOICES  = 4;
static constexpr uint8_t POLY_EMPTY   = 0x3F;  // unused 6-bit value -> empty voice slot
static constexpr uint8_t POLY_DEFAULT = 0x10;  // semi=0, oct=1 (centre C), no flags
static constexpr uint8_t POLY_CHORDS  = POLY_STEPS; // max chords = max NOTE events in length

// Flash blob layout. Chord notes pack 4x6 bits = exactly 3 bytes per chord (no
// ragged boundary across chords). time stays per-step; accent/slide are per-chord.
static constexpr int POLY_NOTES_BYTES = POLY_CHORDS * 3;            // 192
static constexpr int POLY_TIME_OFF    = POLY_NOTES_BYTES;          // 192
static constexpr int POLY_ACC_OFF     = POLY_TIME_OFF + POLY_STEPS / 4;  // 208
static constexpr int POLY_SLD_OFF     = POLY_ACC_OFF + POLY_CHORDS / 8;  // 216
static constexpr int POLY_CNT_OFF     = POLY_SLD_OFF + POLY_CHORDS / 8;  // 224 (chord_count)
static constexpr int POLY_DIR_OFF     = POLY_CNT_OFF + 1;          // 225
static constexpr int POLY_LEN_OFF     = POLY_DIR_OFF + 1;          // 226
static constexpr int POLY_BLOB_SIZE   = POLY_LEN_OFF + 1;          // 227 (<= 243 = one record)

static_assert(POLY_VOICES == 4, "the 3-byte-per-chord note packing assumes 4 voices");

struct PolyVoice {
  uint8_t notes[POLY_CHORDS * POLY_VOICES]; // 6-bit packed pitch or POLY_EMPTY, flat per CHORD
  uint8_t time_data[POLY_STEPS / 4];         // 2 bits/step: 0=rest 1=note 2=tie
  uint8_t acc_bits[POLY_CHORDS / 8];         // 1 bit per CHORD (accent)
  uint8_t sld_bits[POLY_CHORDS / 8];         // 1 bit per CHORD (slide)
  uint8_t chord_count = 0;                   // size of the chord stream
  uint8_t direction   = 0;
  uint8_t length      = 16;

  void Clear() {
    memset(notes, POLY_EMPTY, sizeof(notes));
    memset(time_data, 0, sizeof(time_data));
    memset(acc_bits, 0, sizeof(acc_bits));
    memset(sld_bits, 0, sizeof(sld_bits));
    chord_count = 0;
    direction   = 0;
    length      = 8;
  }

  // ---- time stream (rest/note/tie), indexed by STEP ----
  uint8_t time(uint8_t s) const { return (time_data[s >> 2] >> (2 * (s & 3))) & 3; }
  void set_time(uint8_t s, uint8_t t) {
    const uint8_t sh = 2 * (s & 3);
    time_data[s >> 2] = uint8_t((time_data[s >> 2] & ~(3u << sh)) | ((t & 3u) << sh));
  }
  // Number of NOTE events strictly before step `time_idx` (within length). This is
  // the chord-stream index the step pulls from -- the two-stream mapping.
  uint8_t count_notes_to(uint8_t time_idx) const {
    uint8_t cnt = 0;
    const uint8_t lim = (time_idx < length) ? time_idx : length;
    for (uint8_t i = 0; i < lim; ++i)
      if (time(i) == 1) ++cnt;
    return cnt;
  }
  uint8_t chord_index_for_step(uint8_t time_idx) const { return count_notes_to(time_idx); }
  uint8_t note_count() const {
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < length; ++i)
      if (time(i) == 1) ++cnt;
    return cnt;
  }

  // ---- chord stream size ----
  uint8_t get_chord_count() const { return chord_count; }
  // Grow the chord stream so every NOTE event has a chord; new chords default to a
  // single centre-C note (audible) with no accent/slide. Never shrinks (a removed
  // NOTE keeps its chord queued so re-adding replays it -- matches the mono streams).
  void ensure_chords_for_notes() {
    const uint8_t nc = note_count();
    uint8_t cc = chord_count;
    while (cc < nc && cc < POLY_CHORDS) {
      uint8_t *v = chord(cc);
      v[0] = POLY_DEFAULT; v[1] = POLY_EMPTY; v[2] = POLY_EMPTY; v[3] = POLY_EMPTY;
      set_accent(cc, false);
      set_slide(cc, false);
      ++cc;
    }
    if (cc != chord_count) chord_count = cc;
  }

  // Apply a step-indexed edit (the wire form the web editor / SysEx 0x27 uses):
  // set the step's time, grow the chord stream to cover all NOTEs, and -- only for a
  // NOTE step -- write the chord pulled at that step's stream index (with its accent/
  // slide). REST/TIE leave the chord contents untouched; only the time mapping moves.
  void set_step(uint8_t step, uint8_t n0, uint8_t n1, uint8_t n2, uint8_t n3,
                uint8_t t, bool acc, bool sld) {
    set_time(step, t & 3);
    ensure_chords_for_notes();
    if ((t & 3) == 1) {
      const uint8_t ci = chord_index_for_step(step);
      if (ci < POLY_CHORDS) {
        uint8_t *v = chord(ci);
        v[0] = n0 & 0x3F; v[1] = n1 & 0x3F; v[2] = n2 & 0x3F; v[3] = n3 & 0x3F;
        set_accent(ci, acc);
        set_slide(ci, sld);
      }
    }
  }

  // ---- accent / slide, indexed by CHORD ----
  bool accent(uint8_t i) const { return (acc_bits[i >> 3] >> (i & 7)) & 1; }
  bool slide(uint8_t i)  const { return (sld_bits[i >> 3] >> (i & 7)) & 1; }
  void set_accent(uint8_t i, bool on) {
    const uint8_t m = uint8_t(1u << (i & 7));
    if (on) acc_bits[i >> 3] |= m; else acc_bits[i >> 3] &= uint8_t(~m);
  }
  void set_slide(uint8_t i, bool on) {
    const uint8_t m = uint8_t(1u << (i & 7));
    if (on) sld_bits[i >> 3] |= m; else sld_bits[i >> 3] &= uint8_t(~m);
  }
  void toggle_accent(uint8_t i) { set_accent(i, !accent(i)); }
  void toggle_slide(uint8_t i)  { set_slide(i, !slide(i)); }

  // ---- chord voices (a chord holds an unordered set of up to POLY_VOICES pitches) ----
  const uint8_t *chord(uint8_t i) const { return &notes[i * POLY_VOICES]; }
  uint8_t       *chord(uint8_t i)       { return &notes[i * POLY_VOICES]; }

  uint8_t voice_count(uint8_t i) const {
    const uint8_t *v = chord(i);
    uint8_t n = 0;
    for (uint8_t k = 0; k < POLY_VOICES; ++k)
      if (v[k] != POLY_EMPTY) ++n;
    return n;
  }
  bool has_note(uint8_t i, uint8_t packed) const {
    const uint8_t *v = chord(i);
    for (uint8_t k = 0; k < POLY_VOICES; ++k)
      if (v[k] == (packed & 0x3F)) return true;
    return false;
  }
  // Add packed (semi|oct<<4) to chord i. No-op (false) if it duplicates an existing
  // voice or the chord already holds POLY_VOICES notes; true if added.
  bool add_note(uint8_t i, uint8_t packed) {
    packed &= 0x3F;
    if (has_note(i, packed)) return false;
    uint8_t *v = chord(i);
    for (uint8_t k = 0; k < POLY_VOICES; ++k)
      if (v[k] == POLY_EMPTY) { v[k] = packed; return true; }
    return false; // full
  }
  bool remove_note(uint8_t i, uint8_t packed) {
    packed &= 0x3F;
    uint8_t *v = chord(i);
    for (uint8_t k = 0; k < POLY_VOICES; ++k)
      if (v[k] == packed) { v[k] = POLY_EMPTY; return true; }
    return false;
  }
  // ---- flash blob (POLY_BLOB_SIZE bytes, fits one record) ----
  void serialize(uint8_t *b) const {
    for (uint8_t i = 0; i < POLY_CHORDS; ++i) {
      const uint8_t *v = chord(i);
      uint8_t *o = b + i * 3;
      o[0] = uint8_t(v[0] | (v[1] << 6));
      o[1] = uint8_t((v[1] >> 2) | (v[2] << 4));
      o[2] = uint8_t((v[2] >> 4) | (v[3] << 2));
    }
    memcpy(b + POLY_TIME_OFF, time_data, sizeof(time_data));
    memcpy(b + POLY_ACC_OFF,  acc_bits,  sizeof(acc_bits));
    memcpy(b + POLY_SLD_OFF,  sld_bits,  sizeof(sld_bits));
    b[POLY_CNT_OFF] = chord_count;
    b[POLY_DIR_OFF] = direction;
    b[POLY_LEN_OFF] = length;
  }
  void deserialize(const uint8_t *b) {
    for (uint8_t i = 0; i < POLY_CHORDS; ++i) {
      const uint8_t *in = b + i * 3;
      uint8_t *v = chord(i);
      v[0] = in[0] & 0x3F;
      v[1] = uint8_t((in[0] >> 6) | (in[1] << 2)) & 0x3F;
      v[2] = uint8_t((in[1] >> 4) | (in[2] << 4)) & 0x3F;
      v[3] = uint8_t(in[2] >> 2) & 0x3F;
    }
    memcpy(time_data, b + POLY_TIME_OFF, sizeof(time_data));
    memcpy(acc_bits,  b + POLY_ACC_OFF,  sizeof(acc_bits));
    memcpy(sld_bits,  b + POLY_SLD_OFF,  sizeof(sld_bits));
    chord_count = b[POLY_CNT_OFF];
    direction   = b[POLY_DIR_OFF];
    length      = b[POLY_LEN_OFF];
  }
};
