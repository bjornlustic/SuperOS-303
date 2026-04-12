// Copyright (c) 2026, Nicholas J. Michalek
//
// drivers.h — CV/gate/accent/slide output and LED matrix multiplex + input polling
//
// CV out: same PORTE wiring as reference/OS-303 (gate bit 1, accent bit 6, slide/latch bit 0).

#pragma once
#include "pins.h"

static constexpr uint16_t SWITCH_DELAY = 15; // microseconds

//
// --- 303 CPU driver functions
//
namespace DAC {
  static uint8_t pitch_ = 0;
  static uint8_t slide_ = false;
  static uint8_t accent_ = false;
  static uint8_t gate_ = false;

  // Last values actually pushed to hardware. Send() is called every loop
  // iteration; without this cache PORTC/PORTE would be re-thrashed (and the
  // latch re-pulsed) thousands of times per second even when nothing changed,
  // bleeding switching noise into the analog supply.
  static uint8_t last_pitch_  = 0xFF;
  static uint8_t last_slide_  = 0xFF;
  static uint8_t last_accent_ = 0xFF;
  static uint8_t last_gate_   = 0xFF;

  inline void Send() {
    if (pitch_ == last_pitch_ && slide_ == last_slide_
        && accent_ == last_accent_ && gate_ == last_gate_) {
      return;
    }
    last_pitch_  = pitch_;
    last_slide_  = slide_;
    last_accent_ = accent_;
    last_gate_   = gate_;

    // set 6-bit pitch for CV Out
    PORTC = pitch_; // & 0x3f;

    PORTE = 0; // disable latch
    // set gate and accent pins, enable latch/slide
    PORTE = (gate_ << 1) | (accent_ << 6) | 0x1;
    delayMicroseconds(10); // make sure the latch stays on long enough
    if (!slide_) // turn slide bit back off
      PORTE ^= 0x1;
  }

  inline void SetPitch(uint8_t p) {
    // 6-bit CV: clamp — values >63 used to clear gate and leave stale pitch (bad preview)
    pitch_ = (p > 63) ? 63 : p;
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
} // namespace DAC

// =============================================================================
// Leds namespace — 3-byte framebuffer + time-multiplexed matrix output
// =============================================================================
namespace Leds {
  // like a framebuffer, each bit corresponds to an entry in the switched_leds table
  static uint8_t ledstate[3];
  // Second framebuffer: bits set here render at half the current brightness.
  // A bit must NOT be set in both ledstate[] and dimstate[] simultaneously.
  static uint8_t dimstate[3];
  // Global brightness 1..8 (8 = full). PWM-via-tick: on each Send() call,
  // a small counter advances; only the first `brightness` slots out of every 8
  // actually drive the matrix. The remaining slots blank the row.
  static uint8_t brightness = 8;
  static uint8_t pwm_phase  = 0;

  // set a bit in the framebuffer
  void Set(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    ledstate[row] = (ledstate[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  // set a bit in the dim framebuffer (half brightness)
  void SetDim(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    dimstate[row] = (dimstate[row] & ~(1 << bit_idx)) | (enable << bit_idx);
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
    // PWM dim: advance phase once per full 4-row matrix sweep so every row
    // is drawn (or skipped) uniformly. `lit` gates this entire sweep.
    if ((tick & 0x3) == 0) pwm_phase = (pwm_phase + 1) & 0x7;
    const bool lit     = (pwm_phase < brightness);
    const uint8_t half = 1; // minimal dim: 1 out of 8 PWM phases
    const bool dim_lit = (pwm_phase < half);

    // switched LEDs
    // which row depends on tick
    const uint8_t ri = (tick >> 1) & 1;
    const uint8_t sh = 4 * ((tick >> 0) & 1);
    uint8_t mask = 0;
    if (lit)     mask |= (ledstate[ri] >> sh);
    if (dim_lit) mask |= (dimstate[ri] >> sh);
    SetLedSelection(switched_leds[(tick & 0x3) << 2].select, mask & 0x0F);

    // direct LEDs
    for (uint8_t i = 16; i < 20; ++i) {
      const uint8_t b = 1 << (i - 16);
      const bool on = (lit && (ledstate[2] & b)) || (dim_lit && (dimstate[2] & b));
      digitalWriteFast(switched_leds[i].led, on ? HIGH : LOW);
    }

    if (clear) {
      // blank slate for next time
      ledstate[0] = 0;
      ledstate[1] = 0;
      ledstate[2] = 0;
      dimstate[0] = 0;
      dimstate[1] = 0;
      dimstate[2] = 0;
    }
  }

} // namespace Leds

// =============================================================================
// PollInputs — read full button matrix; clears mux ghosting before sampling
// =============================================================================
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
