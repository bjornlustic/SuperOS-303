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

// SysEx pattern blob = 48 raw bytes (pitch[32] + time_data[8] + 8 metadata).
static constexpr int PATTERN_SIZE = MAX_STEPS + (MAX_STEPS / 4) + METADATA_SIZE;

// Sig is prefix-matched: anything starting with sig_compat_prefix passes.
// Prefix bumped from "superOS-2bit" because the flash layout changed (patterns
// are now stored two-per-page); old arenas must wipe to a clean packed format.
const char *const sig_pew = "superOS-pack-v1";
const char *const sig_compat_prefix = "superOS-pack";
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

  static constexpr uint8_t kTrackFormatVersion = 4;

  // Settings block byte layout (FB_SETTINGS_LEN = 22):
  //   [0..15] signature  [16] midi_channel  [17] flags(bit0=clock_rx)
  //   [18] direction  [19] thru  [20] led_brightness  [21] track_format
  void serialize(uint8_t *b) const {
    memcpy(b, signature, 16);
    b[16] = midi_channel;
    b[17] = midi_clock_receive ? 1 : 0;
    b[18] = sequence_direction;
    b[19] = midi_thru ? 1 : 0;
    b[20] = led_brightness;
    b[21] = track_format;
  }
  void deserialize(const uint8_t *b) {
    memcpy(signature, b, 16);
    midi_channel       = (b[16] <= 16) ? b[16] : 1;
    midi_clock_receive = (b[17] <= 1)  ? (b[17] != 0) : true;
    sequence_direction = (b[18] < uint8_t(DIR_COUNT)) ? b[18] : 0;
    midi_thru          = (b[19] == 1);
    led_brightness     = (b[20] >= 1 && b[20] <= 8) ? b[20] : 8;
    track_format       = b[21];
  }

  // Load the settings block. If absent (fresh flash), zero the signature so
  // Validate() fails and the engine runs its clean-init path.
  void Load() {
    uint8_t b[FB_SETTINGS_LEN];
    if (g_flash.read(FB_SETTINGS, b, FB_SETTINGS_LEN) == FB_SETTINGS_LEN)
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
    if (strncmp(signature, sig_compat_prefix, kSigCompatPrefixLen) == 0) return true;
    memcpy((char *)signature, sig_pew, kSigEepromLen);
    return false;
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

// Write/read one super-block (the pattern pair 2*super, 2*super+1).
inline void WritePatternPair(const Sequence &a, const Sequence &b, uint8_t super) {
  uint8_t buf[FB_PATTERN_LEN];
  serialize_pattern(a, buf);
  serialize_pattern(b, buf + FB_PATTERN_LEN_ONE);
  g_flash.write(uint8_t(FB_PATTERN_BASE + super), buf, FB_PATTERN_LEN);
}
inline void ReadPatternPair(Sequence &a, Sequence &b, uint8_t super) {
  uint8_t buf[FB_PATTERN_LEN];
  if (g_flash.read(uint8_t(FB_PATTERN_BASE + super), buf, FB_PATTERN_LEN) == FB_PATTERN_LEN) {
    deserialize_pattern(a, buf);
    deserialize_pattern(b, buf + FB_PATTERN_LEN_ONE);
  } else {
    clear_pattern_bytes(a);
    clear_pattern_bytes(b);
  }
}

// One pattern at (slot, variation). flat = slot*NUM_VARIATIONS + var; the pattern
// is one half of super-block flat/2. Writes are read-modify-write so the other
// half (a different slot/variation) is preserved.
inline void ReadPatternAt(Sequence &seq, uint8_t abs_slot, uint8_t var) {
  uint16_t flat = uint16_t(abs_slot) * NUM_VARIATIONS + var;
  uint8_t super = uint8_t(flat >> 1), half = uint8_t(flat & 1);
  uint8_t buf[FB_PATTERN_LEN];
  if (g_flash.read(uint8_t(FB_PATTERN_BASE + super), buf, FB_PATTERN_LEN) == FB_PATTERN_LEN)
    deserialize_pattern(seq, buf + half * FB_PATTERN_LEN_ONE);
  else
    clear_pattern_bytes(seq);
}
inline void WritePatternAt(const Sequence &seq, uint8_t abs_slot, uint8_t var) {
  uint16_t flat = uint16_t(abs_slot) * NUM_VARIATIONS + var;
  uint8_t super = uint8_t(flat >> 1), half = uint8_t(flat & 1);
  uint8_t buf[FB_PATTERN_LEN];
  if (g_flash.read(uint8_t(FB_PATTERN_BASE + super), buf, FB_PATTERN_LEN) != FB_PATTERN_LEN)
    memset(buf, 0, FB_PATTERN_LEN);        // unwritten neighbour half -> empty (length 0)
  serialize_pattern(seq, buf + half * FB_PATTERN_LEN_ONE);
  g_flash.write(uint8_t(FB_PATTERN_BASE + super), buf, FB_PATTERN_LEN);
}

// Per-slot CV variation (which of the 3 variations the CV/gate plays), 64 bytes.
inline void ReadCvVarConfig(uint8_t *dst /*[NUM_SLOTS]*/) {
  if (g_flash.read(FB_CVVAR, dst, FB_CVVAR_LEN) != FB_CVVAR_LEN) {
    memset(dst, 0, FB_CVVAR_LEN);          // default: variation 1 for all slots
  } else {
    for (uint8_t i = 0; i < FB_CVVAR_LEN; ++i)
      if (dst[i] >= NUM_VARIATIONS) dst[i] = 0;
  }
}
inline void WriteCvVarConfig(const uint8_t *src /*[NUM_SLOTS]*/) {
  g_flash.write(FB_CVVAR, src, FB_CVVAR_LEN);
}
