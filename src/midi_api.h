// Public MIDI API (implementation in midi.cpp)
// Arduino MIDI library header resolves correctly on case-insensitive filesystems.
#pragma once

#include <Arduino.h>  

struct Engine;

void midi_init(Engine *engine);
/// Apply after EEPROM load / config menu: `ch` 0 = omni, 1–16 = listen; `clock_rx` = use MIDI transport clock.
void midi_apply_settings(uint8_t midi_in_channel_0_omni_16, bool midi_clock_receive);
/// Channel 1–16 for sequencer Note On/Off (omni listen → 1).
uint8_t midi_sequencer_out_channel();
/// Sets `midi_clock_pulse` true when a MIDI Clock byte was received (24 ppqn).
void midi_poll(Engine &engine, bool clk_run, bool &midi_clk, bool &midi_clock_pulse);
/// Call once per 16th when `engine.Clock()` returned true while transport running.
void midi_after_clock(Engine &engine, uint8_t transpose);
/// DIN MIDI leader: clock pulses on `clocked` when transport runs and we are not synced to
/// incoming MIDI Clock; optional Start/Stop with RUN edges (same conditions as engine reset).
void midi_leader_transport(bool clocked, bool clk_run, bool midi_transport_slave,
                           bool run_rising, bool run_falling);
/// True when the last live (non-sequencer) MIDI Note On had velocity >= 100 (accent).
/// Valid only while clock is stopped; used by main.cpp to drive DAC accent CV.
bool midi_live_accent();
/// True while a live MIDI Note On is held (clock stopped); used by main.cpp to open gate.
bool midi_live_gate();
