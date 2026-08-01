// emu_notes.c — see emu_notes.h.
#include "emu_notes.h"

static uint8_t clamp7(int v) { return v < 0 ? 0 : (v > 127 ? 127 : (uint8_t)v); }

void emu_notes_out_update(EmuNotesOut *o, uint8_t pitch, uint8_t gate,
                          uint8_t accent, uint8_t slide) {
  if (!o->note_on || !o->note_off) return;
  const uint8_t vel = accent ? 127 : 80;
  const uint8_t note = clamp7((int)o->base + (int)(pitch & 0x3f));

  if (gate) {
    if (!o->active) {                    // gate rising -> new note
      o->note = note; o->active = 1;
      o->note_on(o->u, note, vel);
    } else if (note != o->note) {        // pitch changed while held
      if (slide) {                       // legato: new on before old off
        o->note_on(o->u, note, vel);
        o->note_off(o->u, o->note);
      } else {                           // retrigger
        o->note_off(o->u, o->note);
        o->note_on(o->u, note, vel);
      }
      o->note = note;
    }
  } else if (o->active) {                // gate falling
    o->note_off(o->u, o->note);
    o->active = 0;
  }
}

void emu_notes_out_all_off(EmuNotesOut *o) {
  if (o->active && o->note_off) o->note_off(o->u, o->note);
  o->active = 0;
}

void emu_live_note_on(EmuLive *l, uint8_t note, uint8_t vel) {
  l->slide  = l->active ? 1 : 0;         // legato if a note was already held
  l->active = 1;
  l->note   = note;
  l->accent = (vel >= 100) ? 1 : 0;
}

void emu_live_note_off(EmuLive *l, uint8_t note) {
  if (l->active && note == l->note) { l->active = 0; l->slide = 0; }
}

int emu_live_resolve(const EmuLive *l, uint8_t *pitch, uint8_t *gate,
                     uint8_t *accent, uint8_t *slide) {
  if (!l->active) { *gate = 0; return 0; }
  int p = (int)l->note - (int)l->base;
  if (p < 0) p = 0; if (p > 63) p = 63;
  *pitch = (uint8_t)p; *gate = 1; *accent = l->accent; *slide = l->slide;
  return 1;
}
