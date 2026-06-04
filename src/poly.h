// poly.h -- variation 3 polyphonic voice.
//
// When variation 3 is in poly mode it holds, per step, up to POLY_VOICES notes
// plus a step-level time cell (rest/note/tie), accent, and slide. Accent and
// slide are per step (not per note): the whole chord accents or slides as one,
// matching the mono voice behavior. Pitches use the same 6-bit packed form as
// the mono voice (bits[3:0]=semi 0..12, bits[5:4]=oct 0..3) but are stored flat
// per step, NOT in the mono two-stream NOTE-event order.
//
// Pure (stdint/string only) so it host-tests without the Arduino platform.
#pragma once
#include <stdint.h>
#include <string.h>

#ifndef POLY_STEPS
#define POLY_STEPS 64
#endif

static constexpr uint8_t POLY_VOICES = 4;
static constexpr uint8_t POLY_EMPTY  = 0x3F;   // unused 6-bit value -> empty voice slot

// Flash blob layout. Notes pack 4x6 bits = exactly 3 bytes per step (no ragged
// boundary across steps). time/accent/slide stay byte-aligned bitmaps.
static constexpr int POLY_NOTES_BYTES = POLY_STEPS * 3;            // 192
static constexpr int POLY_TIME_OFF    = POLY_NOTES_BYTES;          // 192
static constexpr int POLY_ACC_OFF     = POLY_TIME_OFF + POLY_STEPS / 4; // 208
static constexpr int POLY_SLD_OFF     = POLY_ACC_OFF + POLY_STEPS / 8;  // 216
static constexpr int POLY_DIR_OFF     = POLY_SLD_OFF + POLY_STEPS / 8;  // 224
static constexpr int POLY_LEN_OFF     = POLY_DIR_OFF + 1;          // 225
static constexpr int POLY_BLOB_SIZE   = POLY_LEN_OFF + 1;          // 226 (<= 243 = one record)

static_assert(POLY_VOICES == 4, "the 3-byte-per-step note packing assumes 4 voices");

struct PolyVoice {
  uint8_t notes[POLY_STEPS * POLY_VOICES]; // 6-bit packed pitch or POLY_EMPTY, flat per step
  uint8_t time_data[POLY_STEPS / 4];        // 2 bits/step: 0=rest 1=note 2=tie
  uint8_t acc_bits[POLY_STEPS / 8];         // 1 bit/step (step-level accent)
  uint8_t sld_bits[POLY_STEPS / 8];         // 1 bit/step (step-level slide)
  uint8_t direction = 0;
  uint8_t length    = 16;

  void Clear() {
    memset(notes, POLY_EMPTY, sizeof(notes));
    memset(time_data, 0, sizeof(time_data));
    memset(acc_bits, 0, sizeof(acc_bits));
    memset(sld_bits, 0, sizeof(sld_bits));
    direction = 0;
    length = 8;
  }

  // ---- time (rest/note/tie) ----
  uint8_t time(uint8_t s) const { return (time_data[s >> 2] >> (2 * (s & 3))) & 3; }
  void set_time(uint8_t s, uint8_t t) {
    const uint8_t sh = 2 * (s & 3);
    time_data[s >> 2] = uint8_t((time_data[s >> 2] & ~(3u << sh)) | ((t & 3u) << sh));
  }

  // ---- step-level accent / slide ----
  bool accent(uint8_t s) const { return (acc_bits[s >> 3] >> (s & 7)) & 1; }
  bool slide(uint8_t s)  const { return (sld_bits[s >> 3] >> (s & 7)) & 1; }
  void set_accent(uint8_t s, bool on) {
    const uint8_t m = uint8_t(1u << (s & 7));
    if (on) acc_bits[s >> 3] |= m; else acc_bits[s >> 3] &= uint8_t(~m);
  }
  void set_slide(uint8_t s, bool on) {
    const uint8_t m = uint8_t(1u << (s & 7));
    if (on) sld_bits[s >> 3] |= m; else sld_bits[s >> 3] &= uint8_t(~m);
  }
  void toggle_accent(uint8_t s) { set_accent(s, !accent(s)); }
  void toggle_slide(uint8_t s)  { set_slide(s, !slide(s)); }

  // ---- notes (a step holds an unordered set of up to POLY_VOICES unique pitches) ----
  const uint8_t *step(uint8_t s) const { return &notes[s * POLY_VOICES]; }
  uint8_t       *step(uint8_t s)       { return &notes[s * POLY_VOICES]; }

  uint8_t note_count(uint8_t s) const {
    const uint8_t *v = step(s);
    uint8_t n = 0;
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] != POLY_EMPTY) ++n;
    return n;
  }
  bool has_note(uint8_t s, uint8_t packed) const {
    const uint8_t *v = step(s);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] == (packed & 0x3F)) return true;
    return false;
  }
  // Add packed (semi|oct<<4) to the step. No-op (false) if it duplicates an
  // existing note or the step already has POLY_VOICES notes; true if added.
  bool add_note(uint8_t s, uint8_t packed) {
    packed &= 0x3F;
    if (has_note(s, packed)) return false;
    uint8_t *v = step(s);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] == POLY_EMPTY) { v[i] = packed; return true; }
    return false; // full
  }
  bool remove_note(uint8_t s, uint8_t packed) {
    packed &= 0x3F;
    uint8_t *v = step(s);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] == packed) { v[i] = POLY_EMPTY; return true; }
    return false;
  }
  // Move one note's octave by d (-1/+1), keeping its semitone. Blocked (false)
  // if the new octave is out of range [0..3] or the destination pitch already
  // exists at this step (no two voices on the same pitch).
  bool move_note_octave(uint8_t s, uint8_t packed, int8_t d) {
    packed &= 0x3F;
    uint8_t *v = step(s);
    for (uint8_t i = 0; i < POLY_VOICES; ++i) {
      if (v[i] != packed) continue;
      const int oct = int((packed >> 4) & 0x03) + d;
      if (oct < 0 || oct > 3) return false;
      const uint8_t dst = uint8_t((packed & 0x0F) | (uint8_t(oct) << 4));
      if (has_note(s, dst)) return false;
      v[i] = dst;
      return true;
    }
    return false;
  }

  // ---- flash blob (POLY_BLOB_SIZE bytes, fits one record) ----
  void serialize(uint8_t *b) const {
    for (uint8_t s = 0; s < POLY_STEPS; ++s) {
      const uint8_t *v = step(s);
      uint8_t *o = b + s * 3;
      o[0] = uint8_t(v[0] | (v[1] << 6));
      o[1] = uint8_t((v[1] >> 2) | (v[2] << 4));
      o[2] = uint8_t((v[2] >> 4) | (v[3] << 2));
    }
    memcpy(b + POLY_TIME_OFF, time_data, sizeof(time_data));
    memcpy(b + POLY_ACC_OFF,  acc_bits,  sizeof(acc_bits));
    memcpy(b + POLY_SLD_OFF,  sld_bits,  sizeof(sld_bits));
    b[POLY_DIR_OFF] = direction;
    b[POLY_LEN_OFF] = length;
  }
  void deserialize(const uint8_t *b) {
    for (uint8_t s = 0; s < POLY_STEPS; ++s) {
      const uint8_t *in = b + s * 3;
      uint8_t *v = step(s);
      v[0] = in[0] & 0x3F;
      v[1] = uint8_t((in[0] >> 6) | (in[1] << 2)) & 0x3F;
      v[2] = uint8_t((in[1] >> 4) | (in[2] << 4)) & 0x3F;
      v[3] = uint8_t(in[2] >> 2) & 0x3F;
    }
    memcpy(time_data, b + POLY_TIME_OFF, sizeof(time_data));
    memcpy(acc_bits,  b + POLY_ACC_OFF,  sizeof(acc_bits));
    memcpy(sld_bits,  b + POLY_SLD_OFF,  sizeof(sld_bits));
    direction = b[POLY_DIR_OFF];
    length    = b[POLY_LEN_OFF];
  }
};
