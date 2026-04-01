// Public MIDI API (implementation in midi.cpp)
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
/// incoming MIDI Clock; optional Start/Stop with RUN edges.
void midi_leader_transport(bool clocked, bool clk_run, bool midi_transport_slave,
                           bool run_rising, bool run_falling);
/// True when the last live (non-sequencer) MIDI Note On had velocity >= 100 (accent).
bool midi_live_accent();
/// True while a live MIDI Note On is held (clock stopped); driven by most-recent note only.
bool midi_live_gate();
/// True while the destination note of a live legato slide is active (clock stopped).
bool midi_live_slide();

/// Audition a single note during pitch write/edit (not sequencer output).
/// Tracks the last-sent audition note; sends Note Off for previous if pitch changes.
void midi_audition_note_on(uint8_t note, uint8_t vel);
/// Send Note Off for the currently-open audition note (TAP_NEXT / BACK_KEY falling).
void midi_audition_note_off();

/// Broadcast current sequencer position to host (SysEx 0x15).
/// Called once per 16th-note advance while transport is running.
void midi_send_step_position(uint8_t pat, uint8_t step);

/// Broadcast the full pattern blob to host (SysEx 0x11).
/// Call after any operation that rewrites the whole pattern (e.g. Clear).
void midi_send_pattern_update(uint8_t pat);

/// Broadcast a single step edit to host (SysEx 0x16).
/// Called after every pitch or time write in pattern-write mode so the
/// web editor can reflect 303 hardware edits in real time.
/// Format: F0 7D 16 <pat:0-15> <step:0-63> <pitch_lo7> <pitch_hi1> <time_nibble> F7
void midi_send_step_update(uint8_t pat, uint8_t step, uint8_t pitch_byte, uint8_t time_nibble);

/// Broadcast pattern length change to host (SysEx 0x18).
/// Format: F0 7D 18 <pat:0-15> <length:1-64> F7
void midi_send_length_update(uint8_t pat, uint8_t len);

/// Broadcast sequencer direction change to host (SysEx 0x17).
/// Format: F0 7D 17 <direction:0-4> F7
void midi_send_direction_update(uint8_t direction);

/// Broadcast a single step-lock toggle to host (SysEx 0x19).
/// Format: F0 7D 19 <pat:0-15> <step:0-63> <locked:0|1> F7
void midi_send_step_lock_update(uint8_t pat, uint8_t step, bool locked);

/// Play a metronome tick note via MIDI (E3 on first beat of pattern, E4 otherwise).
void midi_metronome_tick(bool first_beat);
/// Stop the open metronome note (on mode exit / clock stop).
void midi_metronome_stop();
