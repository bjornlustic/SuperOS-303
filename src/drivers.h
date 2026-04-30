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
  // Double-buffered framebuffer: main loop writes to front[], ISR reads from
  // back[]. Swap() publishes a new frame. ISR-driven refresh eliminates
  // brightness flicker caused by variable main-loop timing (MIDI TX, DAC, etc.).
  static volatile uint8_t back[3];     // ISR reads this
  static volatile uint8_t back_dim[3]; // ISR reads this (dim layer)
  static uint8_t front[3];             // main loop writes via Set()
  static uint8_t front_dim[3];         // main loop writes via SetDim()

  static volatile uint8_t brightness = 8;
  static uint8_t pwm_phase  = 0;
  static volatile uint8_t isr_tick = 0;

  void Set(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    front[row] = (front[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  void SetDim(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    front_dim[row] = (front_dim[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  // directly set hardware
  void Set(const MatrixPin pins, bool enable = true) {
    if (enable && pins.select) {
      PORTF = 0x0f;
      digitalWriteFast(pins.select, LOW);
    }
    digitalWriteFast(pins.led, enable ? HIGH : LOW);
  }

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

  // Publish current front buffer to ISR. Call once per main-loop iteration
  // after all Set()/SetDim() calls, then clear front buffers for next frame.
  void Swap() {
    uint8_t sreg = SREG;
    cli();
    back[0] = front[0]; back[1] = front[1]; back[2] = front[2];
    back_dim[0] = front_dim[0]; back_dim[1] = front_dim[1]; back_dim[2] = front_dim[2];
    SREG = sreg;
    front[0] = 0; front[1] = 0; front[2] = 0;
    front_dim[0] = 0; front_dim[1] = 0; front_dim[2] = 0;
  }

  // Called from Timer3 ISR at a fixed rate. Drives one matrix row per call.
  void SendISR() {
    const uint8_t tick = isr_tick++;
    if ((tick & 0x3) == 0) pwm_phase = (pwm_phase + 1) & 0x7;
    const bool lit     = (pwm_phase < brightness);
    const bool dim_lit = (pwm_phase < 1);

    const uint8_t ri = (tick >> 1) & 1;
    const uint8_t sh = 4 * ((tick >> 0) & 1);
    uint8_t mask = 0;
    if (lit)     mask |= (back[ri] >> sh);
    if (dim_lit) mask |= (back_dim[ri] >> sh);
    SetLedSelection(switched_leds[(tick & 0x3) << 2].select, mask & 0x0F);

    for (uint8_t i = 16; i < 20; ++i) {
      const uint8_t b = 1 << (i - 16);
      const bool on = (lit && (back[2] & b)) || (dim_lit && (back_dim[2] & b));
      digitalWriteFast(switched_leds[i].led, on ? HIGH : LOW);
    }
  }

  // Start Timer3-driven LED refresh. Call once from setup().
  void BeginRefresh() {
    // Timer3 CTC mode, prescaler 64: 16MHz/64 = 250kHz tick.
    // Earlier OCR3A = 15 → ~15.6kHz ISR rate. With each ISR taking 15-25us
    // for the matrix multiplex + PWM bookkeeping, that ate ~25-30% CPU and
    // starved the main loop, missing CLOCK rising edges at high BPMs.
    // OCR3A = 63 → 3.9kHz ISR (~6% CPU), 977 Hz frame, 122 Hz PWM cycle.
    // 122 Hz PWM is still well above visible flicker (~60 Hz) so no LED change.
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30);
    OCR3A  = 63;
    TIMSK3 = (1 << OCIE3A);
  }

  // Suppress ISR during PollInputs (shared GPIO ports).
  void PauseRefresh() { TIMSK3 &= ~(1 << OCIE3A); }
  void ResumeRefresh() { TIMSK3 |= (1 << OCIE3A); }

} // namespace Leds

ISR(TIMER3_COMPA_vect) {
  Leds::SendISR();
}

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
