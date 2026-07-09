// emu_sync.c — see emu_sync.h.
#include "emu_sync.h"

static void recalc_internal(EmuSync *s) {
  // 24 ppqn rising edges: edges/sec = bpm/60 * PPQN, square wave toggles at
  // twice that -> half period (us) = 60e6 / (bpm * PPQN * 2)
  uint32_t denom = (uint32_t)s->bpm * EMU_CLK_PPQN * 2u;
  s->clk_half_us = denom ? (60000000u / denom) : 10417u;
}

void emu_sync_init(EmuSync *s, const EmuSettings *cfg, uint32_t now_us) {
  s->source = cfg->clock_source;
  s->bpm = cfg->internal_bpm ? cfg->internal_bpm : 120;
  // internal free-runs; external waits for Start (otherwise the first Start
  // after boot is not a run transition and transport coupling never engages)
  s->run = (cfg->clock_source == EMU_CLK_INTERNAL) ? 1 : 0;
  s->clk_level = 0;
  s->int_pending = 0;
  s->ext_pending = 0;
  s->int_next_us = now_us + EMU_INT_PERIOD_US;
  recalc_internal(s);
  s->clk_next_us = now_us + s->clk_half_us;
  s->clk_fall_us = now_us;
  s->clk_last_pulse_us = now_us;
  s->clk_knob = 0;
  s->int_edges = s->clk_edges = 0;
}

void emu_sync_set_bpm(EmuSync *s, uint16_t bpm) { s->bpm = bpm ? bpm : 120; recalc_internal(s); }
void emu_sync_set_source(EmuSync *s, uint8_t source) {
  s->source = source;
  s->ext_pending = 0;
  s->clk_knob = 0;   // start INTERNAL on the digital fallback until a knob pulse arrives
  // internal has no external transport; external waits for Start
  s->run = (source == EMU_CLK_INTERNAL) ? 1 : 0;
}

void emu_sync_service(EmuSync *s, uint32_t now_us) {
  // /INT fixed sampler (rising edges at ~555 Hz)
  while ((int32_t)(now_us - s->int_next_us) >= 0) {
    s->int_pending = 1;
    s->int_edges++;
    s->int_next_us += EMU_INT_PERIOD_US;
  }
  // Is the CLOCK line fed by sampled 24 ppqn hardware pulses right now?
  //   * external source (MIDI/DIN): while its transport runs
  //   * INTERNAL source: while the panel tempo-knob oscillator pulses PA3
  int passthru;
  if (s->source == EMU_CLK_INTERNAL)
    passthru = s->clk_knob
            && (int32_t)(now_us - s->clk_last_pulse_us) < (int32_t)EMU_CLK_KNOB_TIMEOUT_US;
  else
    passthru = s->run;

  if (!passthru) {
    // Free-running square wave at the stored BPM. INTERNAL with no tempo-knob
    // oscillator (bench / dead line), or an external source whose transport is
    // stopped: the CLOCK line always ticks so the panel RUN/STOP can start the
    // sequencer (the ROM's own play latch gates actual playback).
    if (s->source == EMU_CLK_INTERNAL) s->clk_knob = 0;   // knob pulses lapsed
    if ((int32_t)(now_us - s->clk_next_us) >= 0) {
      s->clk_level ^= 1;
      if (s->clk_level) s->clk_edges++;
      s->clk_next_us += s->clk_half_us;
      // After a long host stall (flash page program, USB blocking) a backlog
      // of toggles is due. Bursting them through here would collapse into at
      // most one visible level change at the ROM's sampler anyway, so the
      // pulses are lost either way; drop the backlog and resume on schedule
      // so the line comes back clean instead of at a random phase.
      if ((int32_t)(now_us - s->clk_next_us) >= (int32_t)s->clk_half_us)
        s->clk_next_us = now_us + s->clk_half_us;
    }
  } else {
    // Stretch queued hardware pulses into sampler-visible highs.
    if (s->clk_level && (int32_t)(now_us - s->clk_fall_us) >= 0)
      s->clk_level = 0;
    if (s->ext_pending && !s->clk_level) {
      s->ext_pending--;
      s->clk_level = 1;
      s->clk_edges++;
      s->clk_fall_us = now_us + EMU_CLK_PULSE_US;
    }
    // keep the free-run schedule fresh so the fallback resumes without a burst
    if ((int32_t)(now_us - s->clk_next_us) >= 0) s->clk_next_us = now_us + s->clk_half_us;
  }
}

void emu_sync_ext_pulse(EmuSync *s, uint8_t from_source) {
  if (s->source == from_source && s->run && s->ext_pending < 2)
    s->ext_pending++;
}

void emu_sync_transport(EmuSync *s, int start) {
  // Internal source has no external transport: a stray MIDI Stop must not
  // gate off the free-running clock (it froze the sequencer and the
  // note-out path until the next Start).
  if (s->source == EMU_CLK_INTERNAL) { s->run = 1; return; }
  s->run = start ? 1 : 0;
  if (!s->run) s->ext_pending = 0;
}
