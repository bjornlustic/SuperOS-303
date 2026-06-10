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
/// Main voice (variation 1) MIDI: call EVERY clock tick while running. Drives Note
/// On/Off from engine.get_gate() so MIDI note length tracks the analog gate exactly.
void midi_seq_gate_tick(Engine &engine, uint8_t transpose);
/// Mute the sequencer's MIDI Note On for a specific step index. -1 = no muted step.
/// Used by the step-select detail editor so the edited step doesn't repeat audibly.
void midi_set_silence_step(int step);
/// Shadow voices (variations 2/3) MIDI: call EVERY clock tick while running, after
/// Engine::AdvanceShadows() ran at the 16th boundary. Gate-follows like the main voice.
void midi_shadows_gate_tick(Engine &engine, uint8_t transpose);
/// Flush note-offs queued by the gate-ticks this tick. Call once per clock tick AFTER
/// midi_seq_gate_tick + midi_shadows_gate_tick so all voices' note-ONs cluster first.
void midi_flush_note_offs();
/// Send Note Off for any open shadow-voice notes (transport stop / pattern switch).
void midi_shadows_all_notes_off(Engine &engine);
/// Set MIDI output channels (1-16) for multitimbral variations 2 and 3.
void midi_set_var_channels(uint8_t v2, uint8_t v3);
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
/// Audition a full variation-3 poly chord during edit: Note On per voice on the
/// var3 channel (MIDI only, no 303 CV). Closes any open audition chord first.
void midi_audition_chord_on(const uint8_t *voices, bool accent, uint8_t transpose);
/// Close the currently-open audition chord (poly TAP_NEXT / BACK_KEY navigation).
void midi_audition_chord_off();

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
/// Format: F0 7D 16 <pat:0-15> <step:0-63> <pitch_lo7> <pitch_hi1> <time_nibble> <var:0-2> F7
void midi_send_step_update(uint8_t pat, uint8_t step, uint8_t pitch_byte, uint8_t time_nibble,
                           uint8_t var = 0);

/// Broadcast pattern length change to host (SysEx 0x18).
/// Format: F0 7D 18 <pat:0-15> <length:1-64> <var:0-2> F7
void midi_send_length_update(uint8_t pat, uint8_t len, uint8_t var = 0);

/// Broadcast sequencer direction change to host (SysEx 0x17).
/// Format: F0 7D 17 <direction:0-5> <var:0-2> F7
void midi_send_direction_update(uint8_t direction, uint8_t var = 0);

/// Broadcast pattern group change to host (SysEx 0x1C).
/// Format: F0 7D 1C <group:0-3> F7
void midi_send_group_update(uint8_t group);

/// Broadcast / receive which variation the hardware is editing (SysEx 0x1F).
/// Bidirectional: host sends the same format to set the edit variation.
/// Format: F0 7D 1F <pat:0-15> <var:0-2> F7
void midi_send_edit_variation(uint8_t pat, uint8_t var);

/// Broadcast variation-3 poly/mono flag for a slot (SysEx 0x29).
/// Bidirectional: host sends the same format to set the flag.
/// Format: F0 7D 29 <pat:0-15> <flag:0|1> F7
void midi_send_poly_flag(uint8_t pat, uint8_t flag);
/// Broadcast one variation-3 poly step (device->host, SysEx 0x27) after a hardware
/// chord edit so the web editor mirrors panel edits live.
void midi_send_poly_step(uint8_t pat, uint8_t step);

/// Broadcast active pattern selection while stopped (SysEx 0x1E).
/// Used so the web editor follows hardware pat-key presses without showing
/// the "playing" indicator that 0x15 would imply. Includes the current group
/// so the web can resync even if its hwGroup state is stale.
/// Format: F0 7D 1E <pat:0-15> <group:0-3> F7
void midi_send_active_pattern(uint8_t pat);

/// Broadcast a single step-lock toggle to host (SysEx 0x19).
/// Format: F0 7D 19 <pat:0-15> <step:0-63> <locked:0|1> F7
void midi_send_step_lock_update(uint8_t pat, uint8_t step, bool locked);

/// Play a metronome tick note via MIDI (E3 on first beat of pattern, E4 otherwise).
void midi_metronome_tick(bool first_beat);
/// Stop the open metronome note (on mode exit / clock stop).
void midi_metronome_stop();

/// Broadcast Track Write / Track Play state to host (SysEx 0x23).
/// Format: F0 7D 23 <dial_mode:0-3> <track_idx:0-7> <track_active:0|1> <clk_run:0|1>
///                 <chain_len:0-16> <chain_pos:0-15> <patsel:0-15> <group:0-3>
///                 <p_chain[16]> <t_chain_last_flag_bits_lo7> <t_chain_last_flag_bits_hi7>
/// p_chain: low nibble = pattern in group, high nibble = repeats (0-15).
/// last-flag bits: bit i of (lo|hi<<7) is 1 if t_chain[i] has the last-step flag.
void midi_send_track_state(uint8_t dial_mode, uint8_t track_idx, bool track_active,
                            bool clk_run, struct Engine &engine);

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

/// Incrementally persist web-edited patterns to EEPROM. Writes at most one
/// pattern per call, and only after a quiet period since the last SysEx edit
/// so bursts of web edits coalesce. Call from the main loop.
void midi_flush_pending_pattern_saves(Engine &engine);
