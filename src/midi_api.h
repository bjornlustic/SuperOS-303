// Public MIDI API (implementation in midi.cpp)
#pragma once

#include <Arduino.h>

struct Engine;

void midi_init(Engine *engine);
/// Apply after EEPROM load / config menu: `ch` 0 = omni, 1–16 = listen; `clock_rx` = use MIDI transport clock; `thru` = forward MIDI IN to MIDI OUT.
void midi_apply_settings(uint8_t midi_in_channel_0_omni_16, bool midi_clock_receive, bool midi_thru);
/// Channel 1–16 for sequencer Note On/Off (omni listen → 1).
uint8_t midi_sequencer_out_channel();
/// Increments `midi_clock_pulses` for each MIDI Clock byte received (24 ppqn).
/// Using a counter instead of a boolean ensures no clock ticks are lost when
/// multiple clocks arrive during a single poll (e.g. while parsing a long SysEx).
void midi_poll(Engine &engine, bool clk_run, bool &midi_clk, uint8_t &midi_clock_pulses);
/// Call once per 16th when `engine.Clock()` returned true while transport running.
void midi_after_clock(Engine &engine, uint8_t transpose);
/// Mute the sequencer's MIDI Note On for a specific step index. -1 = no muted step.
/// Used by the step-select detail editor so the edited step doesn't repeat audibly.
void midi_set_silence_step(int step);
/// Call when `engine.is_ratchet_retrigger()` returns true (sub-tick within a ratcheted step).
/// Sends Note Off + Note On for the current note without advancing the step.
void midi_ratchet_retrigger(Engine &engine, uint8_t transpose);
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

/// Lightweight pattern broadcast: sends per-step 0x16 messages instead of
/// the heavy 0x11 packed blob. No memcpy/xor/pack_7bit -- just ring buffer
/// writes. Use when the clock is running to avoid delaying gate timing.
void midi_send_pattern_steps(uint8_t pat, const struct Sequence &seq, uint8_t len);

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

/// Broadcast pattern group change to host (SysEx 0x1C).
/// Format: F0 7D 1C <group:0-3> F7
void midi_send_group_update(uint8_t group);

/// Broadcast active pattern selection while stopped (SysEx 0x1E).
/// Used so the web editor follows hardware pat-key presses without showing
/// the "playing" indicator that 0x15 would imply. Includes the current group
/// so the web can resync even if its hwGroup state is stale.
/// Format: F0 7D 1E <pat:0-15> <group:0-3> F7
void midi_send_active_pattern(uint8_t pat);

/// Broadcast a single step-lock toggle to host (SysEx 0x19).
/// Format: F0 7D 19 <pat:0-15> <step:0-63> <locked:0|1> F7
void midi_send_step_lock_update(uint8_t pat, uint8_t step, bool locked);

/// Broadcast a single ratchet value change to host (SysEx 0x1B).
/// Format: F0 7D 1B <pat:0-15> <step:0-63> <val:0-2> F7
/// Targeted replacement for midi_send_pattern_update() on single-step ratchet edits.
void midi_send_ratchet_update(uint8_t pat, uint8_t step, uint8_t val);

/// Play a metronome tick note via MIDI (E3 on first beat of pattern, E4 otherwise).
void midi_metronome_tick(bool first_beat);
/// Stop the open metronome note (on mode exit / clock stop).
void midi_metronome_stop();

/// Broadcast current chain state to host (SysEx 0x1A).
/// active_len=0 means no chain active. Patterns at indices >= their respective lengths are ignored.
/// Format: F0 7D 1A <active_len:0-4> <a0> <a1> <a2> <a3> <queued_len:0-4> <q0> <q1> <q2> <q3> F7
void midi_send_chain_state(uint8_t active_len, const uint8_t *active_pats,
                            uint8_t queued_len, const uint8_t *queued_pats);

/// Poll for chain state received from the host via SysEx 0x1A.
/// Returns true and fills out parameters if a new chain state arrived since last call.
bool midi_get_received_chain(uint8_t *out_active_len, uint8_t out_active_pats[4],
                              uint8_t *out_queued_len, uint8_t out_queued_pats[4]);

/// Flush EEPROM writes deferred by SysEx handlers (0x22 config).
/// Call from a main-loop save point; blocking EEPROM writes inside the SysEx
/// fast path would overflow the UART RX buffer and drop subsequent messages.
void midi_flush_pending_saves();
