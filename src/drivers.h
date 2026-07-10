// Copyright (c) 2026, Nicholas J. Michalek
//
// drivers.h — CV/gate/accent/slide output and LED matrix multiplex + input polling
//
// CV out: same PORTE wiring as reference/OS-303 (gate bit 1, accent bit 6, slide/latch bit 0).

#pragma once
#include "pins.h"

// State and functions are C++17 `inline`: the combined build compiles this
// header into more than one TU (main.cpp and d650/emu_avr.cpp) and both must
// see the SAME driver state (the Timer3 ISR snapshot in particular).

static constexpr uint16_t SWITCH_DELAY = 15; // microseconds

//
// --- 303 CPU driver functions
//
namespace DAC {
  inline uint8_t pitch_ = 0;
  inline uint8_t slide_ = false;
  inline uint8_t accent_ = false;
  inline uint8_t gate_ = false;

  // Last values actually pushed to hardware. Send() is called every loop
  // iteration; without this cache PORTC/PORTE would be re-thrashed (and the
  // latch re-pulsed) thousands of times per second even when nothing changed,
  // bleeding switching noise into the analog supply.
  inline uint8_t last_pitch_  = 0xFF;
  inline uint8_t last_slide_  = 0xFF;
  inline uint8_t last_accent_ = 0xFF;
  inline uint8_t last_gate_   = 0xFF;

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

    // 4013 needs data stable before the latch clock edge (CD4013/HD14013)
    PORTE = 0;
    PORTE = (gate_ << 1) | (accent_ << 6);
    PORTE |= 0x1;
    if (!slide_)
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
  inline volatile uint8_t back[3];     // ISR reads this
  inline volatile uint8_t back_dim[3]; // ISR reads this (dim layer)
  inline uint8_t front[3];             // main loop writes via Set()
  inline uint8_t front_dim[3];         // main loop writes via SetDim()

  inline volatile uint8_t brightness = 8;
  inline uint8_t pwm_phase  = 0;
  inline volatile uint8_t isr_tick = 0;

  inline void Set(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    front[row] = (front[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  inline void SetDim(OutputIndex ledidx, bool enable = true) {
    const uint8_t bit_idx = ledidx & 0x7;
    const uint8_t row = ledidx >> 3;
    front_dim[row] = (front_dim[row] & ~(1 << bit_idx)) | (enable << bit_idx);
  }
  // Status-line snapshot (PINB bits 0-3 = PA0-3: RUN/TAP/-/CLOCK), captured
  // at the end of the all-selects-high settle below. A dedicated status
  // sampler would have to recreate exactly this electrical state with its
  // own PORTF write + busy-wait; consumers read a ~2 kHz sample for free.
  inline volatile uint8_t last_status_pinb = 0;

  // Last matrix / mode-LED port images actually driven by SendISR. Redraw()
  // re-asserts them after an input scan blanks the matrix.
  inline uint8_t last_portf    = 0x0f;
  inline uint8_t last_portd_hi = 0;

  inline void SetLedSelection(uint8_t select_pin, uint8_t enable_mask) {
    const uint8_t switched_pins[4] = {
      PG0_PIN, PG1_PIN, PG2_PIN, PG3_PIN,
    };
    PORTF = 0x0f;
    delayMicroseconds(SWITCH_DELAY);
    last_status_pinb = PINB;
    digitalWriteFast(select_pin, LOW);
    for (uint8_t i = 0; i < 4; ++i) {
      digitalWriteFast(switched_pins[i], (enable_mask & (1 << i))?HIGH:LOW);
    }
  }

  // Publish current front buffer to ISR. Call once per main-loop iteration
  // after all Set()/SetDim() calls, then clear front buffers for next frame.
  inline void Swap() {
    uint8_t sreg = SREG;
    cli();
    back[0] = front[0]; back[1] = front[1]; back[2] = front[2];
    back_dim[0] = front_dim[0]; back_dim[1] = front_dim[1]; back_dim[2] = front_dim[2];
    SREG = sreg;
    front[0] = 0; front[1] = 0; front[2] = 0;
    front_dim[0] = 0; front_dim[1] = 0; front_dim[2] = 0;
  }

  // Called from Timer3 ISR at a fixed rate. Drives one matrix row per call.
  //
  // Direct port writes: all 8 matrix-drive pins live on PORTF (PH0-3 selects
  // = PF0-3, PG0-3 column drives = PF4-7) and the 4 mode LEDs on PD4-7, so
  // the whole refresh is two port writes plus the status settle. The old
  // per-pin digitalWriteFast calls all had variable pin arguments, i.e. the
  // slow digitalWrite fallback: ~10-20 us of ISR body per tick, and the
  // matrix stayed dark while the column writes staggered in. Electrical
  // sequence is unchanged: deselect all rows, settle, snapshot the status
  // lines, then select row + columns in one atomic write.
  inline void SendISR() {
    const uint8_t tick = isr_tick++;
    // Advance the PWM phase every 2 ticks (was 4): keeps the dim-PWM cycle
    // above flicker (~61 Hz) when the ISR runs at the emulator's halved
    // 977 Hz rate; at stock rates it just doubles the PWM frequency.
    if ((tick & 0x1) == 0) pwm_phase = (pwm_phase + 1) & 0x7;
    const bool lit     = (pwm_phase < brightness);
    // Dim window = phases 0..1 = 4 CONSECUTIVE ticks, so it covers every
    // matrix row (tick&3) exactly once per PWM cycle. With the old 1-phase
    // window (2 ticks) only two of the four rows ever coincided with it --
    // which two depended on drifting tick/phase alignment -- so dim LEDs on
    // the other rows were invisible or flickered. Duty is 4/16 (was 4/32);
    // dim reads a bit brighter but stays clearly below full.
    const bool dim_lit = (pwm_phase < 2);

    const uint8_t ri = (tick >> 1) & 1;
    const uint8_t sh = 4 * ((tick >> 0) & 1);
    uint8_t mask = 0;
    if (lit)     mask |= (back[ri] >> sh);
    if (dim_lit) mask |= (back_dim[ri] >> sh);

    PORTF = 0x0f;                          // all rows deselected, columns off
    delayMicroseconds(SWITCH_DELAY);
    last_status_pinb = PINB;               // status snapshot (see above)
    const uint8_t pf = (uint8_t)((0x0f & ~(1 << (tick & 0x3))) | ((mask & 0x0f) << 4));
    PORTF = pf;

    uint8_t direct = 0;                    // mode LEDs 16-19 on PD4-7
    if (lit)     direct |= back[2];
    if (dim_lit) direct |= back_dim[2];
    const uint8_t pd_hi = (uint8_t)((direct & 0x0f) << 4);
    PORTD = (uint8_t)((PORTD & 0x0f) | pd_hi);

    last_portf = pf; last_portd_hi = pd_hi;   // for Redraw() after an input scan
  }

  // Re-assert the last frame SendISR drove, filling the matrix-dark gap left by
  // an input scan (PollInputs). It does NOT advance the PWM/row counters and
  // does NOT touch Timer3, so the free-running refresh cadence stays uniform and
  // independent of the scan cadence. The previous fix reset TCNT3 and forced a
  // counter-advancing SendISR on every scan, which pinned the LED-refresh phase
  // to the scan; a note (SuperOS) or clock step (d650c) that lengthened a loop
  // pass delayed the scan and thus re-phased the whole matrix -- a brightness
  // blip locked to the note/clock rate. Holding the last frame instead removes
  // that coupling while still killing the dark-tail shimmer the scan would leave.
  inline void Redraw() {
    PORTF = 0x0f;
    delayMicroseconds(SWITCH_DELAY);
    last_status_pinb = PINB;
    PORTF = last_portf;
    PORTD = (uint8_t)((PORTD & 0x0f) | last_portd_hi);
  }

  // Start Timer3-driven LED refresh. Call once from setup().
  inline void BeginRefresh() {
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
  inline void PauseRefresh() { TIMSK3 &= ~(1 << OCIE3A); }
  inline void ResumeRefresh() { TIMSK3 |= (1 << OCIE3A); }

} // namespace Leds

// Exactly one TU may own the vector. In the combined build d650/emu_avr.cpp
// defines DRIVERS_NO_ISR before including this header; main.cpp keeps it.
#ifndef DRIVERS_NO_ISR
ISR(TIMER3_COMPA_vect) {
  Leds::SendISR();
}
#endif

// =============================================================================
// PollInputs — read full button matrix; clears mux ghosting before sampling
// =============================================================================
inline void PollInputs(PinState *inputs) {
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
