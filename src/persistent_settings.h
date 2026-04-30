// Copyright (c) 2026, Nicholas J. Michalek
//
// persistent_settings.h -- EEPROM layout (OS-303 v0.6 compatible) +
// pattern read/write. Depends on sequence.h for Sequence struct.
//
// EEPROM layout (4096 bytes total, byte-for-byte compatible with OS-303 v0.6):
//   0    .. 127    SETTINGS     (128 B)  signature[16] + flags + reserved
//   128  .. 2175   PITCH_DATA   (2048 B) 64 patterns x 32 bytes (pitch[32])
//   2176 .. 3199   TIME_DATA    (1024 B) 64 patterns x 16 bytes (time_data[16])
//   3200 .. 3711   PATTERN_META (512 B)  64 patterns x 8 bytes (reserved[5] + transpose + engine_select + length)
//   3712 .. 3967   TRACK_DATA   (256 B)  8 tracks x 32 bytes (p_chain[16] + t_chain[16])
//   3968 .. 4095   AUX_DATA     (128 B)  free
//
// Patterns are addressed by a flat index 0..63: bank * 16 + pattern_in_bank.

#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "sequence.h"

static constexpr int SETTINGS_SIZE       = 128;
static constexpr int SETTINGS_OFFSET     = 0;
static constexpr int PITCH_DATA_SIZE     = 64 * MAX_STEPS;        // 2048
static constexpr int PITCH_DATA_OFFSET   = SETTINGS_OFFSET + SETTINGS_SIZE; // 128
static constexpr int TIME_DATA_SIZE      = 64 * (MAX_STEPS / 2);  // 1024
static constexpr int TIME_DATA_OFFSET    = PITCH_DATA_OFFSET + PITCH_DATA_SIZE; // 2176
static constexpr int PATTERN_META_SIZE   = 64 * METADATA_SIZE;    // 512
static constexpr int PATTERN_META_OFFSET = TIME_DATA_OFFSET + TIME_DATA_SIZE; // 3200
static constexpr int TRACK_DATA_SIZE     = 8 * 32;                // 256
static constexpr int TRACK_DATA_OFFSET   = PATTERN_META_OFFSET + PATTERN_META_SIZE; // 3712
static constexpr int AUX_DATA_OFFSET     = TRACK_DATA_OFFSET + TRACK_DATA_SIZE;     // 3968
static constexpr int AUX_DATA_SIZE       = 4096 - AUX_DATA_OFFSET;                  // 128

// SysEx pattern blob = 56 raw bytes (pitch[32] + time_data[16] + 8 metadata).
static constexpr int PATTERN_SIZE = MAX_STEPS + (MAX_STEPS / 2) + METADATA_SIZE;

// Sig is prefix-matched: anything starting with sig_compat_prefix passes.
// OS-303's own firmware uses sig_compat_prefix as its full sig, so EEPROM
// written by either firmware is readable by the other.
//
// signature[] is 16 bytes (locked by OS-303's settings layout). The full sig
// "OS-303-v0.6+superOS" is 19 chars + null = 20 bytes, so it doesn't fit
// strcpy'd. We write via memcpy with fixed 16-byte length, so EEPROM bytes
// 0..15 hold the first 16 chars ("OS-303-v0.6+supe"). Prefix match (11 chars)
// works in both directions.
const char *const sig_pew = "OS-303-v0.6+superOS";
const char *const sig_compat_prefix = "OS-303-v0.6";
static constexpr int kSigCompatPrefixLen = 11;
static constexpr int kSigEepromLen = 16;

extern EEPROMClass storage;

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

  static constexpr int kEepromMidiChannel   = 16;
  static constexpr int kEepromMidiFlags     = 17;
  static constexpr int kEepromDirection     = 18;
  static constexpr int kEepromMidiThru      = 19;
  static constexpr int kEepromLedBrightness = 20;

  void Load() { storage.get(0, signature); }

  void Save() { storage.put(0, signature); }

  bool Validate() const {
    if (strncmp(signature, sig_compat_prefix, kSigCompatPrefixLen) == 0) return true;
    memcpy((char *)signature, sig_pew, kSigEepromLen);
    return false;
  }

  void load_midi_from_storage() {
    const uint8_t ch   = storage.read(kEepromMidiChannel);
    const uint8_t fl   = storage.read(kEepromMidiFlags);
    const uint8_t dir  = storage.read(kEepromDirection);
    const uint8_t thru = storage.read(kEepromMidiThru);
    midi_channel        = (ch <= 16) ? ch : 1;
    midi_clock_receive  = (fl <= 1)  ? (fl != 0) : true;
    sequence_direction  = (dir < uint8_t(DIR_COUNT)) ? dir : 0;
    midi_thru           = (thru == 1);
    const uint8_t br    = storage.read(kEepromLedBrightness);
    led_brightness      = (br >= 1 && br <= 8) ? br : 8;
  }

  void save_midi_to_storage() {
    storage.update(kEepromMidiChannel, midi_channel);
    storage.update(kEepromMidiFlags, midi_clock_receive ? 1 : 0);
    storage.update(kEepromDirection, sequence_direction);
    storage.update(kEepromMidiThru, midi_thru ? 1 : 0);
    storage.update(kEepromLedBrightness, led_brightness);
  }
};

extern PersistentSettings GlobalSettings;

// -----------------------------------------------------------------------------
// Pattern read/write -- 4-region OS-303 layout, flat index 0..63 = bank*16 + pat.
// -----------------------------------------------------------------------------
inline void WritePatternFlat(Sequence &seq, uint8_t flat_idx) {
  flat_idx &= 0x3F;
  storage.put(PITCH_DATA_OFFSET + (int(flat_idx) * MAX_STEPS), seq.pitch);
  storage.put(TIME_DATA_OFFSET  + (int(flat_idx) * (MAX_STEPS / 2)), seq.time_data);
  // Metadata block: reserved[0..4], transpose, engine_select, length (8 bytes,
  // contiguous in struct memory because all uint8_t).
  uint8_t *src = seq.reserved;
  for (uint8_t i = 0; i < METADATA_SIZE; ++i) {
    storage.update(PATTERN_META_OFFSET + (int(flat_idx) * METADATA_SIZE) + i, src[i]);
  }
}
inline void ReadPatternFlat(Sequence &seq, uint8_t flat_idx) {
  flat_idx &= 0x3F;
  storage.get(PITCH_DATA_OFFSET + (int(flat_idx) * MAX_STEPS), seq.pitch);
  storage.get(TIME_DATA_OFFSET  + (int(flat_idx) * (MAX_STEPS / 2)), seq.time_data);
  uint8_t *dst = seq.reserved;
  for (uint8_t i = 0; i < METADATA_SIZE; ++i) {
    dst[i] = storage.read(PATTERN_META_OFFSET + (int(flat_idx) * METADATA_SIZE) + i);
  }
}

// Back-compat shim for engine.h's (idx, group) call style.
inline void WritePattern(Sequence &seq, int idx, int bank) {
  WritePatternFlat(seq, uint8_t(bank * NUM_PATTERNS + (idx & 0x0F)));
}
inline void ReadPattern(Sequence &seq, int idx, int bank) {
  ReadPatternFlat(seq, uint8_t(bank * NUM_PATTERNS + (idx & 0x0F)));
}
