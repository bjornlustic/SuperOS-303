// emu_notes.h — translate between the emulated 303's CV/gate and MIDI notes.
// The 303 pitch value is a 6-bit semitone index (pins.h), so MIDI note =
// pitch_base + pitch. Portable + testable.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- MIDI OUT: sequencer gate/pitch -> Note On/Off -------------------------
typedef struct {
  void (*note_on)(void *u, uint8_t note, uint8_t vel);
  void (*note_off)(void *u, uint8_t note);
  void   *u;
  uint8_t base;         // MIDI note for 303 pitch value 0
  // internal
  uint8_t active;       // a note is currently sounding
  uint8_t note;         // its MIDI number
} EmuNotesOut;

static inline void emu_notes_out_init(EmuNotesOut *o, uint8_t base) {
  o->active = 0; o->note = 0; o->base = base;
}
// Call whenever the decoded CV state changes (e.g. from the CV hook). Emits
// Note On on gate rising and on legato pitch change; Note Off on gate fall.
// slide != 0 makes pitch changes legato (new note on before old note off).
void emu_notes_out_update(EmuNotesOut *o, uint8_t pitch, uint8_t gate,
                          uint8_t accent, uint8_t slide);
void emu_notes_out_all_off(EmuNotesOut *o);   // transport stop / channel change

// --- MIDI IN (live play): Note On/Off -> CV/gate ---------------------------
typedef struct {
  uint8_t active;   // a live note is held
  uint8_t note;     // its MIDI number
  uint8_t accent;   // velocity >= 100
  uint8_t slide;    // legato (note-on while another was held)
  uint8_t base;     // MIDI note for pitch 0
} EmuLive;

static inline void emu_live_init(EmuLive *l, uint8_t base) {
  l->active = l->note = l->accent = l->slide = 0; l->base = base;
}
void emu_live_note_on(EmuLive *l, uint8_t note, uint8_t vel);
void emu_live_note_off(EmuLive *l, uint8_t note);
// Resolve to 303 CV outputs. Returns 1 if a note is held (caller drives DAC).
int  emu_live_resolve(const EmuLive *l, uint8_t *pitch, uint8_t *gate,
                      uint8_t *accent, uint8_t *slide);

#ifdef __cplusplus
}
#endif
