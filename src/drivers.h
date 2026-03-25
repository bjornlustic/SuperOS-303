// Copyright (c) 2026, Nicholas J. Michalek

#pragma once
#include "pins.h"

static constexpr uint16_t SWITCH_DELAY = 15; // microseconds

static constexpr uint32_t SLIDE_LATCH_US   = 44; // latch pulse width for new notes
static constexpr uint32_t SLIDE_RELATCH_US = 9;  // re-latch pulse during multi-note slides

//
// --- 303 CPU driver functions
//

namespace DAC {
  static uint8_t pitch_ = 0;
  static uint8_t slide_ = false;
  static uint8_t accent_ = false;
  static uint8_t gate_ = false;

  // Set by Engine when a new note step begins (triggers 44μs latch pulse).
  static bool new_note_ = false;
  // Set by Engine when pitch changes mid-slide (triggers 8.75μs re-latch pulse).
  static bool slide_relatch_ = false;

  inline void Send() {
    // set 6-bit pitch for CV Out
    PORTC = pitch_;

    if (new_note_) {
      new_note_ = false;
      // Pulse slide line high for 44μs to latch the new pitch and accent into the
      // R2R DAC buffer (IC13 flip-flop), even on non-slide notes (per Sonic Potions paper §3).
      PORTE = 0;
      PORTE = (gate_ << 1) | (accent_ << 6) | 0x1; // slide high
      delayMicroseconds(SLIDE_LATCH_US);
      if (!slide_) PORTE ^= 0x1; // pull slide back low for non-slide notes
    } else if (slide_relatch_) {
      slide_relatch_ = false;
      // During a multi-note slide, briefly pull slide low to re-latch the new pitch
      // value into the DAC, then restore slide high (per Sonic Potions paper §6).
      PORTE &= ~0x1; // slide low
      delayMicroseconds(SLIDE_RELATCH_US);
      PORTE |= 0x1;  // slide high again
    } else {
      PORTE = 0; // disable latch
      PORTE = (gate_ << 1) | (accent_ << 6) | 0x1;
      if (!slide_) PORTE ^= 0x1;
    }
  }

  inline void SetPitch(uint8_t p) {
    if (p > 63) gate_ = false;
    pitch_ = p;
  }
  inline void SetGate(bool on) {
    gate_ = on;
  }
  inline void SetAccent(bool on) {
    accent_ = on;
  }
  inline void SetSlide(bool on) {
    slide_ = on;
  }
  // Call once per new note step (gate rising edge) to trigger the 44μs latch pulse.
  inline void NotifyNewNote() {
    new_note_ = true;
  }
  // Call when pitch changes during an active slide to trigger the 8.75μs re-latch.
  inline void NotifySlideRelatch() {
    slide_relatch_ = true;
  }
} // namespace DAC

namespace Leds {
  // like a framebuffer, each bit corresponds to an entry in the switched_leds table
  static uint8_t ledstate[3];

  // set a bit in the framebuffer
  void Set(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    ledstate[row] = (ledstate[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  // directly set hardware
  void Set(const MatrixPin pins, bool enable = true) {
    if (enable && pins.select) {
      PORTF = 0x0f;
      digitalWriteFast(pins.select, LOW);
    }
    digitalWriteFast(pins.led, enable ? HIGH : LOW);
    //if (enable && pins.select) digitalWriteFast(pins.select, HIGH);
  }

  // helper function
  void SetLedSelection(uint8_t select_pin, uint8_t enable_mask) {
    const uint8_t switched_pins[4] = {
      PG0_PIN, PG1_PIN, PG2_PIN, PG3_PIN,
    };

    PORTF = 0x0f;
    delayMicroseconds(SWITCH_DELAY);
    digitalWriteFast(select_pin, LOW);
    for (uint8_t i = 0; i < 4; ++i) {
      digitalWriteFast(switched_pins[i], (enable_mask & (1 << i))?HIGH:LOW);
    }
  }

  // hardware output, framebuffer reset
  void Send(const uint8_t tick, const bool clear = true) {
    //const uint8_t cycle = (tick >> 2) & 0x3; // scanner for select pins, bits 0-3

    // switched LEDs
    // which row depends on tick
    uint8_t mask = ledstate[(tick >> 1) & 1] >> (4 * ((tick >> 0) & 1));
    SetLedSelection(switched_leds[(tick & 0x3) << 2].select, mask);

    // direct LEDs
    for (uint8_t i = 16; i < 20; ++i) {
      digitalWriteFast(switched_leds[i].led, (ledstate[2] & (1 << (i-16))) ? HIGH : LOW);
    }

    if (clear) {
      // blank slate for next time
      ledstate[0] = 0;
      ledstate[1] = 0;
      ledstate[2] = 0;
    }
  }

} // namespace Leds

void PollInputs(PinState *inputs) {
  // Raise all select pins and drive all LED pins LOW before reading.
  // This clears any residual LED drive state from the previous Leds::Send() call,
  // which would otherwise cause matrix crosstalk and ghost button reads.
  PORTF = 0x0f;
  digitalWriteFast(PG0_PIN, LOW);
  digitalWriteFast(PG1_PIN, LOW);
  digitalWriteFast(PG2_PIN, LOW);
  digitalWriteFast(PG3_PIN, LOW);
  delayMicroseconds(SWITCH_DELAY);

  // read PA and PB pins while select pins are high
  for (uint8_t i = 0; i < 4; ++i) {
    inputs[EXTRA_PIN_OFFSET + i].push(digitalReadFast(status_pins[i])); // PAx
  }

  // open each switched channel with select pin
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWriteFast(select_pin[i], LOW); // PHx
    delayMicroseconds(SWITCH_DELAY);
    for (uint8_t j = 0; j < 4; ++j) {
      // read pins
      inputs[ 0 + i*4 + j].push(digitalReadFast(button_pins[j])); // PBx
      inputs[16 + i*4 + j].push(digitalReadFast(status_pins[j])); // PAx
    }
    digitalWriteFast(select_pin[i], HIGH); // PHx
  }
}
