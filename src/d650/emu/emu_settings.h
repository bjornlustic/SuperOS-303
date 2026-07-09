// emu_settings.h — emulator settings (clock source, MIDI channel, ...), mirroring
// the SuperOS PersistentSettings pattern. Portable; serialize()/deserialize() to a
// flash block. See emu_avr.cpp for the flash + menu wiring.
#pragma once
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Clock source. Mirrors the SuperOS "MIDI vs DIN sync" toggle, plus INTERNAL.
enum { EMU_CLK_INTERNAL = 0, EMU_CLK_MIDI = 1, EMU_CLK_DIN = 2, EMU_CLK_COUNT = 3 };

#define EMU_SETTINGS_LEN 16
#define EMU_SIG "D650emu1"          // 8 bytes

typedef struct {
  char     sig[8];
  uint8_t  midi_channel;   // 0 = omni, 1..16 = listen/send on that channel
  uint8_t  clock_source;   // EMU_CLK_*
  uint8_t  midi_out;       // 1 = send sequencer notes to MIDI OUT
  uint8_t  midi_thru;      // 1 = forward MIDI IN -> MIDI OUT
  uint8_t  led_brightness; // 1..8
  uint8_t  pitch_base;     // MIDI note number for 303 pitch value 0 (default 36 = C2)
  uint16_t internal_bpm;   // internal-tempo BPM when clock_source == INTERNAL
} EmuSettings;

static inline void emu_settings_defaults(EmuSettings *s) {
  memcpy(s->sig, EMU_SIG, 8);
  s->midi_channel   = 1;
  s->clock_source   = EMU_CLK_INTERNAL;
  s->midi_out       = 1;
  s->midi_thru      = 0;
  s->led_brightness = 8;
#ifdef SUPEROS_COMBINED
  s->pitch_base     = 25;      // SuperOS MIDI parity: key C (code 23) = note 48
#else
  s->pitch_base     = 36;      // C2
#endif
  s->internal_bpm   = 120;
}

// 16-byte block layout: [0..7]sig [8]chan [9]clk_src [10]midi_out [11]thru
//                       [12]brightness [13]pitch_base [14..15]bpm(LE)
static inline void emu_settings_serialize(const EmuSettings *s, uint8_t *b) {
  memcpy(b, s->sig, 8);
  b[8]=s->midi_channel; b[9]=s->clock_source; b[10]=s->midi_out; b[11]=s->midi_thru;
  b[12]=s->led_brightness; b[13]=s->pitch_base;
  b[14]=(uint8_t)(s->internal_bpm & 0xff); b[15]=(uint8_t)(s->internal_bpm >> 8);
}
// Returns 1 if the block is valid (signature matches), else fills defaults.
static inline int emu_settings_deserialize(EmuSettings *s, const uint8_t *b) {
  if (memcmp(b, EMU_SIG, 8) != 0) { emu_settings_defaults(s); return 0; }
  memcpy(s->sig, b, 8);
  s->midi_channel   = b[8]  <= 16 ? b[8]  : 1;
  s->clock_source   = b[9]  < EMU_CLK_COUNT ? b[9] : EMU_CLK_INTERNAL;
  s->midi_out       = b[10] ? 1 : 0;
  s->midi_thru      = b[11] ? 1 : 0;
  s->led_brightness = (b[12] >= 1 && b[12] <= 8) ? b[12] : 8;
  s->pitch_base     = b[13];
  s->internal_bpm   = (uint16_t)(b[14] | (b[15] << 8));
  if (s->internal_bpm < 20 || s->internal_bpm > 300) s->internal_bpm = 120;
  return 1;
}

#ifdef __cplusplus
}
#endif
