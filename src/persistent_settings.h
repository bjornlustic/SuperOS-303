// Copyright (c) 2026, Nicholas J. Michalek
//
// persistent_settings.h -- settings + pattern persistence over flash-as-EEPROM.
// Patterns, tracks, and settings are stored as logical blocks in the flash
// block store (flash_eeprom.h / flash_persist.h), not the hardware EEPROM.
// Depends on sequence.h for the Sequence struct.
#pragma once
#include <Arduino.h>
#include "sequence.h"
#include "flash_persist.h"
#include "poly.h"

// SysEx / flash pattern blob = 92 raw bytes (pitch[64] + time_data[16] + 12 metadata).
static constexpr int PATTERN_SIZE = MAX_STEPS + (MAX_STEPS / 4) + METADATA_SIZE;

static_assert(POLY_STEPS == MAX_STEPS, "poly step count must match MAX_STEPS");
static_assert(POLY_BLOB_SIZE <= FE_MAX_PAYLOAD, "poly blob must fit one flash record");
static_assert(FB_SETTINGS_LEN <= FE_MAX_PAYLOAD, "settings block must fit one record");

// Sig is prefix-matched: anything starting with sig_compat_prefix passes.
// Prefix bumped to "superOS-pol3" because the variation-3 poly blob format changed
// from flat-per-step chords to a chord-list (two-stream) layout; wipe to relayout.
const char *const sig_pew = "superOS-pol3-v1";
const char *const sig_compat_prefix = "superOS-pol3";
static constexpr int kSigCompatPrefixLen = 12;
static constexpr int kSigEepromLen = 16;

struct PersistentSettings {
  char signature[16];
  /// MIDI input channel: 0 = omni, 1-16 = listen on that channel only.
  uint8_t midi_channel = 1;
  /// When false, MIDI Clock / Start / Stop are ignored (internal + DIN CLOCK jack only).
  bool midi_clock_receive = true;
  /// Sequence playback direction (SequenceDirection enum).
  uint8_t sequence_direction = 0; // DIR_FORWARD
  /// When true, MIDI IN messages are forwarded to MIDI OUT (software MIDI thru).
  bool midi_thru = false;
  /// LED brightness 1..8 (8 = full).
  uint8_t led_brightness = 8;
  /// Track-storage layout version; bump invalidates stored track blocks.
  uint8_t track_format = 0;
  /// MIDI output channels for multitimbral variations 2 and 3 (1..16).
  /// Variation 1 uses midi_channel; these drive the shadow voices.
  uint8_t var2_channel = 2;
  uint8_t var3_channel = 3;
  /// Per-slot bitmap: bit set = variation 3 of that slot is polyphonic.
  uint8_t var3_poly[NUM_SLOTS / 8] = {0};

  static constexpr uint8_t kTrackFormatVersion = 4;

  bool var3_is_poly(uint8_t slot) const {
    slot &= uint8_t(NUM_SLOTS - 1);
    return (var3_poly[slot >> 3] >> (slot & 7)) & 1;
  }
  void set_var3_poly(uint8_t slot, bool on) {
    slot &= uint8_t(NUM_SLOTS - 1);
    const uint8_t m = uint8_t(1u << (slot & 7));
    if (on) var3_poly[slot >> 3] |= m; else var3_poly[slot >> 3] &= uint8_t(~m);
  }

  // Settings block byte layout (FB_SETTINGS_LEN = 32):
  //   [0..15] signature  [16] midi_channel  [17] flags(bit0=clock_rx)
  //   [18] direction  [19] thru  [20] led_brightness  [21] track_format
  //   [22] var2_channel  [23] var3_channel  [24..31] var3 poly bitmap
  void serialize(uint8_t *b) const {
    memcpy(b, signature, 16);
    b[16] = midi_channel;
    b[17] = midi_clock_receive ? 1 : 0;
    b[18] = sequence_direction;
    b[19] = midi_thru ? 1 : 0;
    b[20] = led_brightness;
    b[21] = track_format;
    b[22] = var2_channel;
    b[23] = var3_channel;
    memcpy(b + 24, var3_poly, sizeof(var3_poly));
  }
  void deserialize(const uint8_t *b) {
    memcpy(signature, b, 16);
    midi_channel       = (b[16] <= 16) ? b[16] : 1;
    midi_clock_receive = (b[17] <= 1)  ? (b[17] != 0) : true;
    sequence_direction = (b[18] < uint8_t(DIR_COUNT)) ? b[18] : 0;
    midi_thru          = (b[19] == 1);
    led_brightness     = (b[20] >= 1 && b[20] <= 8) ? b[20] : 8;
    track_format       = b[21];
    var2_channel       = (b[22] >= 1 && b[22] <= 16) ? b[22] : 2;
    var3_channel       = (b[23] >= 1 && b[23] <= 16) ? b[23] : 3;
    memcpy(var3_poly, b + 24, sizeof(var3_poly));
  }

  // Load the settings block. If absent (fresh flash), zero the signature so
  // Validate() fails and the engine runs its clean-init path. Tolerates an old
  // 22-byte record (pre var2/var3 channels): the buffer is preset with the
  // channel defaults so a short read keeps them and does NOT trigger a wipe.
  void Load() {
    uint8_t b[FB_SETTINGS_LEN];
    b[22] = 2; b[23] = 3;
    memset(b + 24, 0, FB_SETTINGS_LEN - 24); // default var3 poly bitmap = all mono
    const int got = g_flash.read(FB_SETTINGS, b, FB_SETTINGS_LEN);
    if (got >= 22)
      deserialize(b);
    else
      memset(signature, 0, sizeof(signature));
  }

  void Save() {
    uint8_t b[FB_SETTINGS_LEN];
    serialize(b);
    g_flash.write(FB_SETTINGS, b, FB_SETTINGS_LEN);
  }

  bool Validate() const {
    return strncmp(signature, sig_compat_prefix, kSigCompatPrefixLen) == 0;
  }

  // MIDI fields are loaded/clamped in Load() and persisted by Save(); these
  // shims keep the existing call sites working.
  void load_midi_from_storage() {}
  void save_midi_to_storage() { Save(); }

  uint8_t get_track_format() const { return track_format; }
  void set_track_format(uint8_t v) { track_format = v; Save(); }
};

extern PersistentSettings GlobalSettings;

// -----------------------------------------------------------------------------
// Pattern serialization (one pattern = FB_PATTERN_LEN_ONE = 92 bytes) and
// two-per-page super-blocks. Super-block s holds flat patterns 2s and 2s+1.
// -----------------------------------------------------------------------------
inline void serialize_pattern(const Sequence &seq, uint8_t *dst) {
  memcpy(dst, seq.pitch, MAX_STEPS);
  memcpy(dst + MAX_STEPS, seq.time_data, MAX_STEPS / 4);
  memcpy(dst + MAX_STEPS + (MAX_STEPS / 4), seq.reserved, METADATA_SIZE); // reserved[]+transpose+engine+length
}
inline void deserialize_pattern(Sequence &seq, const uint8_t *src) {
  memcpy(seq.pitch, src, MAX_STEPS);
  memcpy(seq.time_data, src + MAX_STEPS, MAX_STEPS / 4);
  memcpy(seq.reserved, src + MAX_STEPS + (MAX_STEPS / 4), METADATA_SIZE);
}
inline void clear_pattern_bytes(Sequence &seq) {
  memset(seq.pitch, PITCH_EMPTY, MAX_STEPS);
  memset(seq.time_data, 0, MAX_STEPS / 4);
  memset(seq.reserved, 0, METADATA_SIZE);
  seq.length = 0; // Load() promotes 0 -> SetLength(8)
}

// One mono pattern at (abs_slot, var), all packed two-per-page. var0+var1 share
// block = slot (half = var). Variation 3 in MONO form packs two slots per block:
// block = FB_MONOVAR2_BASE + slot/2, half = slot&1. Each write is a read-modify-
// write so the neighbouring half is preserved. (Poly var3 uses ReadPolyAt/WritePolyAt.)
inline void mono_block_of(uint8_t abs_slot, uint8_t var, uint8_t &block, uint8_t &half) {
  if (var >= 2) { block = uint8_t(FB_MONOVAR2_BASE + (abs_slot >> 1)); half = abs_slot & 1; }
  else          { block = uint8_t(FB_PATTERN_BASE + abs_slot);         half = var & 1; }
}
inline void ReadPatternAt(Sequence &seq, uint8_t abs_slot, uint8_t var) {
  uint8_t block, half;
  mono_block_of(abs_slot, var, block, half);
  uint8_t buf[FB_PATTERN_LEN];
  if (g_flash.read(block, buf, FB_PATTERN_LEN) == FB_PATTERN_LEN)
    deserialize_pattern(seq, buf + half * FB_PATTERN_LEN_ONE);
  else
    clear_pattern_bytes(seq);
}
inline void WritePatternAt(const Sequence &seq, uint8_t abs_slot, uint8_t var) {
  uint8_t block, half;
  mono_block_of(abs_slot, var, block, half);
  uint8_t buf[FB_PATTERN_LEN];
  if (g_flash.read(block, buf, FB_PATTERN_LEN) != FB_PATTERN_LEN)
    memset(buf, 0, FB_PATTERN_LEN);        // unwritten neighbour half -> empty (length 0)
  serialize_pattern(seq, buf + half * FB_PATTERN_LEN_ONE);
  g_flash.write(block, buf, FB_PATTERN_LEN);
}

// Per-slot step-probability tables: one block per slot, [0..63] var1,
// [64..127] var2, [128..191] var3 mono. Missing record = all zeros (unarmed).
inline void ReadProbAt(uint8_t abs_slot, uint8_t *dst192) {
  memset(dst192, 0, FB_PROB_LEN);
  g_flash.read(uint8_t(FB_PROB_BASE + abs_slot), dst192, FB_PROB_LEN);
}
inline void WriteProbAt(uint8_t abs_slot, const uint8_t *src192) {
  // All-zero table with no existing record: skip the write so unarmed slots
  // never consume a live flash record.
  bool any = false;
  for (uint8_t i = 0; i < FB_PROB_LEN; ++i)
    if (src192[i]) { any = true; break; }
  if (!any) {
    uint8_t probe;
    if (g_flash.read(uint8_t(FB_PROB_BASE + abs_slot), &probe, 1) == 0) return;
  }
  g_flash.write(uint8_t(FB_PROB_BASE + abs_slot), src192, FB_PROB_LEN);
}

// Variation 3 in poly form: its own dedicated block (poly slots only).
inline void ReadPolyAt(PolyVoice &pv, uint8_t abs_slot) {
  uint8_t buf[POLY_BLOB_SIZE];
  if (g_flash.read(uint8_t(FB_POLY_BASE + abs_slot), buf, POLY_BLOB_SIZE) == POLY_BLOB_SIZE)
    pv.deserialize(buf);
  else
    pv.Clear();
}
inline void WritePolyAt(const PolyVoice &pv, uint8_t abs_slot) {
  uint8_t buf[POLY_BLOB_SIZE];
  pv.serialize(buf);
  g_flash.write(uint8_t(FB_POLY_BASE + abs_slot), buf, POLY_BLOB_SIZE);
}
