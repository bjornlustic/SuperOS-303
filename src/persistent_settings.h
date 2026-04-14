// Copyright (c) 2026, Nicholas J. Michalek
//
// persistent_settings.h -- EEPROM layout + pattern read/write.
// Depends on sequence.h for Sequence struct and MAX_STEPS etc.
//
// EEPROM signature versions:
//   "PewPewPew!!2" -- original: pitch slots packed sequentially by NOTE event
//   "PewPewPew!!3" -- 1:1 pitch/time model, 64-step, 16 patterns
//   "PewPewPew!!4" -- 1:1 pitch/time model, 16-step max, 4 groups x 16 patterns
//   "PewPewPew!!5" -- groups 0-2: 16-step compact; group 3: 64-step full blob

#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "sequence.h"

static constexpr int SETTINGS_SIZE = 128;
static constexpr int PATTERN_SIZE = MAX_STEPS * 2; // in-RAM / SysEx blob size (128 bytes)

const char *const sig_pew = "PewPewPew!!5"; // v5: groups 0-2=16-step, group 3=64-step

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
  /// LED brightness 1..8 (8 = full). PWM via Leds::Send tick counter.
  uint8_t led_brightness = 8;

  static constexpr int kEepromMidiChannel   = 16;
  static constexpr int kEepromMidiFlags     = 17;
  static constexpr int kEepromDirection     = 18;
  static constexpr int kEepromMidiThru      = 19;
  static constexpr int kEepromLedBrightness = 20;

  void Load() { storage.get(0, signature); }

  void Save() { storage.put(0, signature); }

  bool Validate() const {
    if (strncmp(signature, sig_pew, 12) == 0) return true;
    strcpy((char *)signature, sig_pew);
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

// Compact EEPROM format (32 bytes):
//   [0..15]  pitch[0..15]
//   [16..23] time_data[0..7]  (covers 16 steps)
//   [24..25] step_lock[0..1]
//   [26..29] ratchets for steps 0-15 (reserved[1..4])
//   [30]     bits[2:0]=direction, bits[7:3]=stash_count
//   [31]     length
inline void WritePattern(Sequence &seq, int idx, int group) {
  if (group == 3) {
    // Group 3: full 128-byte blob directly (supports 64 steps)
    const uint8_t *blob = seq.pitch; // pitch[] is the start of the 128-byte in-RAM layout
    int off = G4_EEPROM_BASE + idx * EEPROM_G4_PATTERN_SIZE;
    for (uint8_t i = 0; i < EEPROM_G4_PATTERN_SIZE; ++i)
      storage.update(off + i, blob[i]);
    return;
  }
  uint8_t compact[EEPROM_PATTERN_SIZE];
  memcpy(compact,       seq.pitch,        16);
  memcpy(compact + 16,  seq.time_data,     8);
  memcpy(compact + 24,  seq.step_lock,     2);
  memcpy(compact + 26,  seq.reserved + 1,  4); // ratchets for steps 0-15
  uint8_t stash = (seq.reserved[21] < MAX_USER_STEPS) ? seq.reserved[21] : 0;
  compact[30] = (seq.reserved[0] & 0x07) | uint8_t(stash << 3);
  compact[31] = seq.length;
  int off = SETTINGS_SIZE + (group * NUM_PATTERNS + idx) * EEPROM_PATTERN_SIZE;
  for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
    storage.update(off + i, compact[i]);
}
inline void ReadPattern(Sequence &seq, int idx, int group) {
  if (group == 3) {
    // Group 3: full 128-byte blob directly (supports 64 steps)
    int off = G4_EEPROM_BASE + idx * EEPROM_G4_PATTERN_SIZE;
    uint8_t *blob = seq.pitch;
    for (uint8_t i = 0; i < EEPROM_G4_PATTERN_SIZE; ++i)
      blob[i] = storage.read(off + i);
    return;
  }
  int off = SETTINGS_SIZE + (group * NUM_PATTERNS + idx) * EEPROM_PATTERN_SIZE;
  uint8_t compact[EEPROM_PATTERN_SIZE];
  for (uint8_t i = 0; i < EEPROM_PATTERN_SIZE; ++i)
    compact[i] = storage.read(off + i);
  memset(seq.pitch,     PITCH_EMPTY, MAX_STEPS);
  memset(seq.time_data, 0,           sizeof(seq.time_data));
  memset(seq.step_lock, 0,           sizeof(seq.step_lock));
  memset(seq.reserved,  0,           sizeof(seq.reserved));
  memcpy(seq.pitch,        compact,      16);
  memcpy(seq.time_data,    compact + 16,  8);
  memcpy(seq.step_lock,    compact + 24,  2);
  memcpy(seq.reserved + 1, compact + 26,  4); // ratchets
  seq.reserved[0]  = compact[30] & 0x07;        // direction
  seq.reserved[21] = (compact[30] >> 3) & 0x1F; // stash_count
  seq.length = compact[31];
}
