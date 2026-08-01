// emu_sync.h — clock/transport router for the D650C emulator.
//
// The ROM's interrupt handler (0x3C) is a fixed ~555 Hz sampler (TM2): each /INT
// it reads PORTA status and edge-detects the CLOCK line (PA3) to advance the
// sequencer — RISING edges only (it keeps the previous status nibble in RAM $1B
// and sets flag F0 when the stored bit is 0 and the line is 1). So tempo does
// NOT go on /INT — it goes on the CLOCK input. This router produces both:
//   * a periodic /INT rising edge at ~555 Hz (int_pending), and
//   * a CLOCK line (clk_level) whose RISING edges run at 24 ppqn, from the
//     selected source.
// Transport (MIDI Start/Stop, DIN run) gates whether CLOCK advances; the panel
// RUN button remains the firmware's own play latch, so the two are decoupled.
#pragma once
#include <stdint.h>
#include "emu_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

// Roland Sync24 = 24 rising edges per quarter note on the CLOCK line; MIDI
// clock (0xF8) is 24 ppqn too, so one incoming pulse = one CLOCK rising edge.
#define EMU_CLK_PPQN 24
#define EMU_INT_PERIOD_US 1800u   // TM2 ~= 1.8 ms => ~555 Hz /INT sampler
// External pulses are stretched to this high time so the 555 Hz sampler is
// guaranteed to see the level (>= 2 samples high, low again well before the
// next 24 ppqn pulse even at 300 BPM: 8.3 ms period).
#define EMU_CLK_PULSE_US (2u * EMU_INT_PERIOD_US)
// INTERNAL tempo comes from the board's own tempo oscillator (the panel TEMPO
// knob) on the CLOCK status line (PA3), sampled as 24 ppqn pulses exactly like
// DIN sync — this is how the real 303 and SuperOS-303 work. If those pulses
// stop for this long (knob line dead / bench with no analog board), fall back
// to the stored digital BPM square wave so the sequencer still runs. 300 ms
// covers the slowest musical tempo gap (24 ppqn at ~8 BPM).
#define EMU_CLK_KNOB_TIMEOUT_US 300000u

typedef struct {
  uint8_t  source;         // EMU_CLK_*
  uint16_t bpm;            // internal tempo (digital fallback when no knob pulses)
  uint8_t  run;            // transport running (gates CLOCK advance)
  uint8_t  clk_level;      // current CLOCK line level (host -> H.in[CLOCK])
  uint8_t  int_pending;    // a /INT rising edge is due (host -> d650_clock pulse)
  uint8_t  ext_pending;    // external pulses waiting to be issued as CLOCK highs
  uint8_t  clk_knob;       // INTERNAL: tempo-knob oscillator pulsing PA3 (else fallback)

  uint32_t int_next_us;
  uint32_t clk_next_us;    // internal-tempo next toggle
  uint32_t clk_half_us;    // internal-tempo half period
  uint32_t clk_fall_us;    // when a stretched external pulse drops low
  uint32_t clk_last_pulse_us; // host timestamp of the last sampled CLOCK rising edge

  // telemetry / test: rising CLOCK edges (= 24 ppqn pulses seen by the ROM)
  uint32_t int_edges, clk_edges;
} EmuSync;

void emu_sync_init(EmuSync *s, const EmuSettings *cfg, uint32_t now_us);
void emu_sync_set_bpm(EmuSync *s, uint16_t bpm);
void emu_sync_set_source(EmuSync *s, uint8_t source);

// Advance time. Sets int_pending when a /INT edge is due. CLOCK (clk_level) is
// driven by sampled 24 ppqn hardware pulses when available — an external source
// whose transport is running, or INTERNAL while the tempo-knob oscillator is
// pulsing PA3 (clk_knob, refreshed by the host's CLOCK sampler). Otherwise it
// free-runs as a square wave at the stored BPM (INTERNAL fallback, or an
// external source whose transport is stopped) so the panel RUN/STOP can still
// start the sequencer. Call every host loop.
void emu_sync_service(EmuSync *s, uint32_t now_us);

// External clock: call once per received MIDI Clock (0xF8) or per DIN sync
// rising edge. Queues one CLOCK rising edge (issued by emu_sync_service) when
// it matches the selected source and transport is running.
void emu_sync_ext_pulse(EmuSync *s, uint8_t from_source);

// Transport. start!=0 => running; 0 => stopped (queue cleared, CLOCK reverts
// to the internal fallback wave). Ignored for INTERNAL source (run stays 1).
void emu_sync_transport(EmuSync *s, int start);

#ifdef __cplusplus
}
#endif
