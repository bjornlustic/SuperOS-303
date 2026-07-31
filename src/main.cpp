// Copyright (c) 2026, Nicholas J. Michalek
//
// main.cpp — setup/loop, MIDI & DIN clock, UI modes, Engine → DAC output
//

#ifndef DEBUG
#define DEBUG 0
#endif

#include <Arduino.h>
#include "pins.h"
#include "drivers.h"
#include "engine.h"
#include "midi_api.h"
#include "flash_store.h"

#ifdef SUPEROS_COMBINED
// Combined build: combined.cpp owns the real setup/loop and dispatches here
// or to the d650c emulator per the EEPROM firmware-select byte.
#include "combined.h"
// avr-gcc ships no <new>; placement new for the g_fw_arena Engine.
inline void *operator new(size_t, void *p) noexcept { return p; }
#define setup superos_setup
#define loop  superos_loop
#endif

FlashEeprom g_flash;
PersistentSettings GlobalSettings;

// =============================================================================
// Globals — timing, debounced inputs, engine, UI timers
// =============================================================================
static uint8_t clk_count = 0;
static uint8_t transpose = 12; // global performance transpose, 0..47 (12 = no transpose)
// Effective transpose = global performance + per-pattern transpose (-24..+24) + track
// step. Signed so per-pattern down-transpose can pull the note below the baseline.
static int16_t total_transpose = 12;
// Clamp a transposed 6-bit CV value to the DAC range so a down-transpose floors at 0
// instead of wrapping to the top of the range.
//
// Factory pitch standard (TB-303 Service Notes p.6, CV adjustment): the
// untransposed low C key must emit 1.000 V at the CV jack = DAC code 23
// (transfer V = (code-11)/12), which is what the original D650C mask ROM
// emits (measured: key C = 23). SuperOS's linear+transpose baseline lands one
// code above that, so every DAC value is shifted down one here. MIDI mapping
// is untouched (key C = note 48 both firmwares, both directions).
static inline uint8_t clamp_cv(int v) { --v; return uint8_t(v < 0 ? 0 : (v > 63 ? 63 : v)); }

#ifdef SUPEROS_COMBINED
static Engine &engine = *(Engine *)g_fw_arena;  // overlays the d650 machine;
                                                // placement-new'ed in setup
#endif

// Non-static: midi.cpp's 0x34 panel diagnostic reads the debounced states.
PinState inputs[INPUT_COUNT];

static uint8_t s_prev_tracknum = 0xff; // 0xff = not yet initialized
static uint8_t s_display_group = 0;    // group shown by dial (may differ from playing group when running)
static uint8_t s_group_debounce_val   = 0xff; // pending new group value
static uint8_t s_group_debounce_count = 0;    // consecutive frames seen
static constexpr uint8_t GROUP_DEBOUNCE_FRAMES = 5;
static bool midi_clk = false;
static uint8_t s_time_edit_steps = 0; // counts writes in the current TIME_MODE edit session

/// Stopped-clock CV preview: audition paths set these; unified DAC block applies them.
static bool s_tap_pitch_preview_gate = false;
static uint8_t s_tap_pitch_preview_cv = 0;
static bool s_tap_pitch_preview_accent = false;
// Slide intent captured at audition-arm time. Stored-note audition (TAP /
// BACK / step-select) captures the stored bit; new-note write captures
// SLIDE_KEY.held(). DAC reads this rather than seq.get_slide() so the bit
// at the post-advance pitch_pos cannot bleed into a fresh write's audition.
static bool s_tap_pitch_preview_slide = false;
static uint8_t s_tap_pitch_preview_retrig = 0; // ticks to force gate low for envelope retrigger
static bool s_back_pitch_preview_gate = false;
static uint8_t s_back_pitch_preview_cv = 0;

static elapsedMillis pattern_cleared_flash_timer;
static constexpr uint16_t PATTERN_CLEARED_FLASH_MS = 400;
// Holds the flash LEDs lit until CLEAR is released after a pattern-clear action,
// so the user sees confirmation for as long as they keep CLEAR held.
static bool s_pat_cleared_hold = false;

// Metronome tap-write state (CLEAR+write+clk_run in NORMAL_MODE).
//
// The recorder is a port of the REAL TB-303 ROM's tap time-write, measured
// cycle-exactly against the mask ROM on the desktop emulator core
// (tools-side tapmode_sweep.c). The ROM's laws:
//   * The session is ONE BAR from a bar reset; afterwards the pattern plays.
//   * Taps are accepted on clock ticks 2..5 of each 6-tick step (1/3 in and
//     later). An accept requires the key STILL HELD; accept at tick 2 writes
//     the CURRENT step, ticks 3..5 write the NEXT step (so up to 2/3 step
//     early aims at the next beat; more than that and the press is stale by
//     the next decision tick and DROPS -- "don't tap between measures").
//   * Each step's fate is decided at ITS tick 2: fresh accept = NOTE, key
//     still held from an earlier note = TIE, released = REST; holding any
//     SELECTOR key (pattern keys 1-8) turns would-be RESTs into TIEs once a
//     note exists (the manual's SUSTAIN).
//   * The monitor voice gates from the accept tick until physical release at
//     the note's pitch; metronome clicks (low at bar start, high on other
//     8ths, 2 ticks long) are suppressed while it sounds; the engine's own
//     gate stays silent for the whole session.
static bool s_metronome_active                  = false;
static bool s_metro_press_pending               = false;  // TAP press awaiting an accept tick
static bool s_metro_note_active                 = false;  // tapped note's finger still down (tie source)
static bool s_metro_any_note                    = false;  // sustain tie-fill arms after the first note
static bool s_metro_pass_accept                 = false;  // a tap was HELD at its accept tick this pass
static bool s_metro_bar_started                 = false;  // bar reset reached step 0; writes enabled
static bool s_metro_step_prewritten             = false;  // upcoming step already holds a next-step NOTE
// Two per-pattern phases while the session runs. RECORD = the ROM's measure
// (clicks on, engine voice silent, monitor under the finger), looping until
// the bar wraps with content. OVERDUB = the pattern has been input: clicks
// stop, the pattern plays clean, taps still record on top.
// Session lifecycle: ONE guided pass through the whole unit (every chain
// member's bar, linked halves counted), then the session AUTO-EXITS at the
// pass-completing wrap IF anything was recorded. An all-empty pass loops the
// metronome (the ROM's endless empty measure) until the first real take or
// a manual exit (transport stop / dial off Pattern Write).
static bool     s_metro_record_phase            = true;
static uint16_t s_metro_recorded_mask           = 0;      // patterns that finished a pass with notes
static uint16_t s_metro_unit_mask               = 0;      // patterns wiped at entry = the session's unit
static uint8_t  s_metro_pass_bars               = 1;      // bars in one full pass of the unit
static uint8_t  s_metro_bar_count               = 0;      // bars completed since the pass began
static bool s_metro_gate_pulse                  = false;
static uint8_t s_metro_pitch_cv                 = 63;     // final DAC pitch for metronome click
static uint8_t s_metro_gate_ticks               = 0;      // click length in clock ticks (tempo-scaled)
static uint8_t s_metro_prev_pat                 = 0;      // pattern playing during the last step window
static bool    s_metro_monitor_gate             = false;  // monitor voice sounding (accept -> release)
static uint8_t s_metro_tap_monitor_cv           = 63;
static bool    s_metro_tap_monitor_accent       = false;
static uint8_t s_metro_tail_cv                  = 63;     // pitch held through decay tails (click or tapped note)
// TAP ownership guard: while tap-write runs (and until the finger comes UP
// after it exits), TAP belongs to tap-write ONLY. Without this, the session
// auto-exiting at the wrap with TAP still held dropped straight into the
// edit-variation picker (TAP_NEXT held in Pattern Write/normal = picker).
static bool    s_metro_tap_swallow              = false;
static uint8_t s_metro_step_tick                = 0;      // clock ticks since the current window began
static elapsedMillis s_metro_gate_timer;

// Direction mode (FN + TIME_KEY)
static bool s_dir_mode = false;

// Keyboard play mode (FN + PITCH_KEY toggle while dial is in Pattern Play).
// Pitched keys play the 303 voice live (DAC override) without modifying the
// pattern. Mirrors stopped-clock PITCH_MODE audition behavior but persists
// indefinitely and works whether the sequencer is running or stopped.
static bool s_keyboard_mode = false;
// Press-order stack of pitched-key slots held during keyboard mode. Used to
// drive legato slide-on-overlap (no gate retrigger) and slide-back when the
// top note is released while older notes are still held.
static uint8_t s_kb_stack_key[8];
static uint8_t s_kb_stack_cv[8];
static uint8_t s_kb_stack_note[8]; // MIDI note sent for this key (for its Note Off)
static uint8_t s_kb_stack_depth = 0;
// Octave latch: TIME_KEY toggles, TIME_MODE_LED shows state. Latched (LED on):
// tap DOWN/UP to step the octave register 0..3 and it holds. Off: DOWN/UP must
// be held while pressing keys (stock behavior; octave 3 = DOWN+UP together).
static bool    s_kb_oct_latch = true;
static uint8_t s_kb_oct       = 1;

static void kb_stack_clear() { s_kb_stack_depth = 0; }
static uint8_t kb_stack_note_of(uint8_t key) {
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i)
    if (s_kb_stack_key[i] == key) return s_kb_stack_note[i];
  return 0xFF;
}
static void kb_stack_remove(uint8_t key) {
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i) {
    if (s_kb_stack_key[i] == key) {
      for (uint8_t j = i; j < s_kb_stack_depth - 1; ++j) {
        s_kb_stack_key[j]  = s_kb_stack_key[j + 1];
        s_kb_stack_cv[j]   = s_kb_stack_cv[j + 1];
        s_kb_stack_note[j] = s_kb_stack_note[j + 1];
      }
      --s_kb_stack_depth;
      return;
    }
  }
}
static bool kb_stack_push(uint8_t key, uint8_t cv, uint8_t note) {
  kb_stack_remove(key);
  if (s_kb_stack_depth >= 8) return false;
  s_kb_stack_key[s_kb_stack_depth]  = key;
  s_kb_stack_cv[s_kb_stack_depth]   = cv;
  s_kb_stack_note[s_kb_stack_depth] = note;
  ++s_kb_stack_depth;
  return true;
}

// Track Write: CLEAR ("bar reset") arms "next TAP_NEXT writes the last step".
// The TAP_NEXT after CLEAR writes at the current cursor, marks it as the last
// chain step, and resets the cursor so the next session starts at step 0.
static bool s_track_arm_last = false;


// Step-select mode (FN + PITCH_KEY held): pick one step via black-key bank + white key.
// Chase LED lights only when playhead is within the active bank. -1 = no selection.
// `s_step_sel_edit` = entered the per-step detail editor via ACCENT_KEY rising.
static int     s_step_sel      = -1;
static uint8_t s_step_sel_base = 0; // 0,8,16,24 (+32 in extended half)
static bool    s_step_sel_ext  = false; // A# toggle: reach steps 32..63
static bool    s_step_sel_edit = false;
static bool    s_step_sel_time = false; // true = time sub-mode, false = pitch sub-mode
static bool    s_step_sel_mode = false; // toggled: FN+PITCH enters, FN exits
// Chain slot being viewed in step-select. 0..3 = DOWN/UP/ACCENT/SLIDE. Only
// meaningful when s_chain_active && s_chain_len >= 2. Reset to 0 on each entry.
static uint8_t s_step_sel_chain_view = 0;

// Incremental pattern sync state (drains 2 steps/loop while running)
static uint8_t s_pat_sync_pat = 0;
static uint8_t s_pat_sync_pos = 0;
static uint8_t s_pat_sync_len = 0;

// FN+write length entry state
static bool    s_len_extended     = false;
static uint8_t s_len_black_base   = 0;
static bool    s_len_black_pressed = false;

#ifndef SUPEROS_COMBINED
static Engine engine;
#endif

// Pattern chain: while stopped, hold anchor key + tap adjacent keys to build a chain
// (same bank, consecutive, max 4).  While playing, hold any chain key to loop that pattern.
static uint8_t s_chain_pats[4]    = {0, 0, 0, 0};
static uint8_t s_chain_len        = 0;
static uint8_t s_chain_pos        = 0;
static bool    s_chain_active     = false;
static uint8_t s_chain_anchor_key = 0xff; // key index 0-7; 0xff = not building
static uint8_t s_chain_bank       = 0;    // bank (0=A, 1=B) of the chain being built
static bool     s_chain_hold_loop       = false; // true this frame: loop when target reached
static uint8_t  s_chain_hold_target_pat = 0xff;  // actual pattern to loop (0xff = any/none)
static uint8_t  s_chain_queued[4]    = {0, 0, 0, 0};
static uint8_t  s_chain_queue_len    = 0;     // ≥1 = pattern(s) waiting to activate

// Wrap-intent tracking for the chain advance (see the advance block in loop()):
// what we queued for the next wrap, and whether it was the end-of-chain handoff
// to s_chain_queued. Also the chainless linked-pair defer slot.
static uint8_t s_chain_prev_tp        = 0xFF;
static uint8_t s_chain_expect         = 0xFF; // slot we queued (0xFF = none)
static uint8_t s_chain_expect_pos     = 0;    // cursor value once it lands
static bool    s_chain_expect_handoff = false; // expect = queued-chain handoff
static uint8_t s_pair_defer           = 0xFF; // switch waiting for the B half to finish
static uint8_t s_pair_defer_pair      = 0xFF; // pair (slot & 7) the defer was parked on
// Forget queued-wrap intent. Must be called whenever chain contents or position
// are set from OUTSIDE the advance block (arm, build commit, web apply, run
// start): a stale expect from the previous run/chain would otherwise satisfy
// the first wrap and snap the cursor to a stale index.
static void chain_intent_reset() {
  s_chain_prev_tp        = 0xFF;
  s_chain_expect         = 0xFF;
  s_chain_expect_handoff = false;
  s_pair_defer           = 0xFF;
  s_pair_defer_pair      = 0xFF;
}

// Persistent chains: a chain saved with A+B held + TAP lives in its first
// pattern's reserved[4..6] (len, then 4-bit slot entries), so it persists with
// the pattern and re-arms whenever that pattern is selected while stopped (and
// at boot). Clearing the pattern clears the stored chain with it.
static void emit_chain_state();
// Sticky panel A/B MODE. ON = every selected pattern (and every member of a
// chain) plays A then B; OFF = patterns play their selected single section.
// Entered by pressing ACCENT+SLIDE together, or automatically by selecting a
// pattern whose SAVED A/B flag is set (or arming a stored chain whose first
// member's flag is set); exited by pressing ACCENT or SLIDE alone. The
// per-pattern link flag (reserved[3] bit0 on the A section) is only the saved
// MEMORY: selection imports it into the mode, the save gestures (chain keys +
// FN, A+B held + TAP) write it, and FN + a held pattern key clears it together
// with the stored chain. Playback never reads the flag directly.
static bool s_ab_mode = false;
// An A/B pair is always ENTERED at its A section (it then plays A-then-B
// on its own). Every fresh entry point -- taps, chain commits, queue writes,
// stored-chain arming, transport start -- routes through this, so playback can
// never start on the B half. The only paths that select B directly are the
// explicit SLIDE section button and the intra-pair wrap hand-off. Selecting a
// pattern with saved A/B memory re-enters the mode here.
static uint8_t chain_entry_start(uint8_t pat) {
  if (engine.pair_linked(pat)) s_ab_mode = true;
  return s_ab_mode ? Engine::section_a_of(pat) : pat;
}
// Chain-wide A/B: with a chain live the panel mode makes every member play A
// then B (the A+B LEDs are a mode indicator, not a per-member property).
static bool chain_ab_mode() {
  return s_chain_active && s_chain_len > 1 && s_ab_mode;
}
// Section-bank browse view while a chain is running: 0/1 = pattern keys and
// A/B LEDs show that bank without touching the playing chain; 0xFF = follow
// the playing pattern. Only meaningful while a chain is live (see
// ProcessDefault); reset whenever the chain ends or the transport stops.
static uint8_t s_section_view = 0xFF;
static void store_chain_on(uint8_t pat, const uint8_t *pats, uint8_t len) {
  Sequence &s = engine.pattern[pat & 0x0F];
  s.reserved[4] = uint8_t(len & 0x07);
  s.reserved[5] = uint8_t((pats[0] & 0x0F) | ((len > 1 ? pats[1] & 0x0F : 0) << 4));
  s.reserved[6] = uint8_t((len > 2 ? pats[2] & 0x0F : 0) | ((len > 3 ? pats[3] & 0x0F : 0) << 4));
  engine.stale = true;
  midi_send_pattern_update(pat & 0x0F);
}
static bool arm_stored_chain(uint8_t pat, bool set_first) {
  const Sequence &s = engine.pattern[pat & 0x0F];
  const uint8_t len = uint8_t(s.reserved[4] & 0x07);
  if (len < 2 || len > 4) return false;
  s_chain_pats[0] = uint8_t(s.reserved[5] & 0x0F);
  s_chain_pats[1] = uint8_t((s.reserved[5] >> 4) & 0x0F);
  s_chain_pats[2] = uint8_t(s.reserved[6] & 0x0F);
  s_chain_pats[3] = uint8_t((s.reserved[6] >> 4) & 0x0F);
  s_chain_len       = len;
  s_chain_pos       = 0;
  s_chain_active    = true;
  s_chain_queue_len = 0;
  s_chain_bank      = uint8_t((s_chain_pats[0] >> 3) & 1);
  chain_intent_reset();
  // Saved A/B memory on the chain's first member re-enters the mode.
  if (engine.pair_linked(s_chain_pats[0])) s_ab_mode = true;
  if (set_first) {
    uint8_t first = s_chain_pats[0];
    if (s_ab_mode) first = Engine::section_a_of(first);
    engine.SetPattern(first, true);
  }
  emit_chain_state();
  return true;
}

// A+B (ACCENT+SLIDE) held chain builder: tap pattern keys in ANY order (repeats
// allowed) while both section buttons are down; releasing commits the chain.
// Unlike the hold+tap builder this is not limited to an ascending run.
static uint8_t s_ab_chain_pats[4] = {0, 0, 0, 0};
static uint8_t s_ab_chain_len     = 0;     // 0 = no A+B build in progress
static bool    s_ab_prev_link     = false; // pair-link state before the A+B press
static uint8_t s_ab_link_pat      = 0xff;  // pattern the A+B press linked (0xff = none)
// FN pressed while chain-build keys were held = "save this chain"; the same
// FN hold must not also drive the length editor. Cleared on FN release.
static bool    s_fn_chain_saved   = false;
static uint8_t  s_chain_hold_key     = 0xff;  // key being tracked for tap/hold
static uint32_t s_chain_hold_ms      = 0;     // millis() when hold key was pressed
static bool     s_chain_hold_crossed = false; // hold threshold crossed
static const uint16_t CHAIN_HOLD_MS  = 300;   // tap vs. hold threshold (ms)

// Broadcast current chain state to web editor (SysEx 0x1A).
static void emit_chain_state() {
  midi_send_chain_state(
    s_chain_active ? s_chain_len : 0, s_chain_pats,
    s_chain_queue_len, s_chain_queued);
}

// (Re)start a tap-write session. ROM entry semantics: clear the time data,
// BAR RESET (session starts at step 0, like the d650c's CLEAR-while-running),
// start the metronome. Pitch streams are preserved so tapped NOTEs consume
// the user's pitches in stream order.
// The session's UNIT is everything the playhead will traverse: the entry
// section, the other half of a linked pair, and every member of an active
// chain (plus their linked halves). The WHOLE unit is wiped to RECORD --
// otherwise members with content join in OVERDUB, the clicks cut off there,
// and after one recorded pass no member ever re-enters RECORD again. A chain
// session restarts at member 0 (and a linked pair at its A section -- same
// normalization as transport start), so which members get the metronome does
// not depend on which slot happened to be playing at the gesture. Patterns
// pulled in AFTER entry (new chain builds, queued taps) keep their content
// and join in OVERDUB.
// Callable MID-SESSION (CLEAR+TIME while active = restart from the top), so
// it also silences any sounding click / monitor voice before resetting.
static void metro_session_begin() {
  midi_metronome_stop();
  midi_audition_note_off();
  // The gesture also fires from an accidentally opened TIME_MODE (see the
  // simultaneous-press note at the gesture); the session runs in NORMAL.
  engine.SetMode(NORMAL_MODE, false);
  const bool chain_session = s_chain_active && s_chain_len > 1;
  if (chain_session) {
    s_chain_pos       = 0;
    s_chain_queue_len = 0;
    chain_intent_reset();
    engine.SetPattern(chain_entry_start(s_chain_pats[0]), true);
    emit_chain_state();
  } else {
    // Pin the engine to the session start: a pattern switch queued before
    // the gesture (quick tap / parked pair defer) must not yank the session
    // to an un-wiped pattern at the bar-1 wrap.
    chain_intent_reset();
    engine.SetPattern(s_ab_mode
                          ? Engine::section_a_of(engine.get_patsel())
                          : engine.get_patsel(),
                      true);
  }
  uint16_t wipe = 0;
  uint8_t  pass_bars = 0;   // bars in one full traversal (linked halves count)
  if (chain_session) {
    for (uint8_t ci = 0; ci < s_chain_len; ++ci) {
      const uint8_t m = uint8_t(s_chain_pats[ci] & 0x0F);
      wipe |= uint16_t(1u << m);
      if (s_ab_mode) {
        wipe |= uint16_t((1u << Engine::section_a_of(m)) |
                         (1u << Engine::section_b_of(m)));
        pass_bars = uint8_t(pass_bars + 2);
      } else {
        pass_bars = uint8_t(pass_bars + 1);
      }
    }
  } else {
    const uint8_t m = engine.get_patsel();
    wipe |= uint16_t(1u << m);
    pass_bars = 1;
    if (s_ab_mode) {
      wipe |= uint16_t((1u << Engine::section_a_of(m)) |
                       (1u << Engine::section_b_of(m)));
      pass_bars = 2;
    }
  }
  for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
    if (!(wipe & uint16_t(1u << p))) continue;
    Sequence &ws = engine.pattern[p];
    for (uint8_t i = 0; i < ws.length; ++i) sequence_set_time_at(ws, i, 0);
    midi_send_pattern_update(p);
  }
  s_metro_unit_mask = wipe;
  s_metro_pass_bars = pass_bars ? pass_bars : 1;
  s_metro_bar_count = 0;
  engine.Reset();                       // bar reset: next tick = step 0
  s_metro_prev_pat        = engine.get_patsel();
  s_metro_tail_cv         = 63;   // idle tail starts at the click pitch
  s_metro_step_tick       = 0;
  s_metro_press_pending   = false;
  s_metro_pass_accept     = false;
  s_metro_note_active     = false;
  s_metro_any_note        = false;
  s_metro_bar_started     = false;
  s_metro_step_prewritten = false;
  s_metro_monitor_gate    = false;
  s_metro_gate_pulse      = false;
  s_metro_gate_ticks      = 0;
  s_metro_record_phase    = true;
  s_metro_recorded_mask   = 0;
  engine.stale = true;   // wiped patterns re-sent in the wipe loop above
}

// Broadcast track-mode state (SysEx 0x23) for the web editor's Track view.
// Call at every event that changes track state: dial transition, chain edit,
// cursor change, RUN edges, chain wrap.
static void emit_track_state(DialMode dial, bool clk_run, uint8_t track_idx) {
  midi_send_track_state(uint8_t(dial), track_idx, engine.track_active, clk_run, engine);
}

// =============================================================================
// Setup menu: hold FUNCTION + press CLEAR (TAP not held) → main menu.
//   C → MIDI channel (C# + white keys 1–8, D# then 9–16; CLEAR → main).
//   D → MIDI clock: TIME_MODE_LED on = internal only; LED off = MIDI clock receive.
//     Press TIME_KEY to toggle; CLEAR → main.
//   E → MIDI thru: SLIDE_KEY_LED on = thru enabled; off = disabled.
//     Press SLIDE_KEY to toggle; CLEAR → main. CLEAR in main exits menu entirely.
//   EEPROM bytes 16–19 + midi_apply_settings() — survives power cycle.
// =============================================================================
static const InputIndex kCfgWhiteKeys[8] = {
    C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY, B_KEY, C_KEY2};

enum class CfgMenu : uint8_t { Off, Midi };
static CfgMenu s_cfg_menu = CfgMenu::Off;
static bool s_cfg_suppress_clear_exit = false;

static uint8_t cfg_display_channel() {
  uint8_t c = GlobalSettings.midi_channel;
  if (c == 0 || c > 16) c = 1;
  return c;
}

static void cfg_save_midi() {
  GlobalSettings.save_midi_to_storage();
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
  midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
}

// Combined MIDI config screen:
//   Pat keys 0-7 → set MIDI channel (1-8, or 9-16 if D# high-bank latched).
//   D# key      → toggle high-bank for channel selection (LED solid while latched).
//   Pat LED     → lit on the slot of the active channel within its bank.
//   TIME mode LED ON  = DIN sync (midi_clock_receive == false).
//   TIME mode LED OFF = MIDI sync (midi_clock_receive == true).
//   TIME_KEY rising → toggle clock source.
//   ACCENT LED ON  = MIDI OUT (midi_thru == false).
//   ACCENT LED OFF = MIDI THRU (midi_thru == true).
//   ACCENT_KEY rising → toggle thru.
//   CLEAR or FN rising → exit menu.

// Preset scale shapes as 12-bit class masks rooted at C (bit0). Order matches the
// web editor's SCALES list exactly so FN-cycling lands on the same scale.
static const uint16_t SCALE_PRESETS[] PROGMEM = {
  0xAB5, // Major
  0x5AD, // Minor
  0x6AD, // Dorian
  0x6B5, // Mixolydian
  0xAD5, // Lydian
  0x5AB, // Phrygian
  0x56B, // Locrian
  0x555, // Whole Tone
  0x6DB, // Half-Whole Dim.
  0xB6D, // Whole-Half Dim.
  0x4E9, // Minor Blues
  0x4A9, // Minor Pentatonic
  0x295, // Major Pentatonic
  0x9AD, // Harmonic Minor
  0x9B5, // Harmonic Major
  0x6CD, // Dorian #4
  0x5B3, // Phrygian Dominant
  0xAAD, // Melodic Minor
  0xB55, // Lydian Augmented
  0x6D5, // Lydian Dominant
  0x55B, // Super Locrian
  0x57B, // 8-Tone Spanish
  0x9B3, // Bhairav
  0x9CD, // Hungarian Minor
  0x18D, // Hirajoshi
  0x4A3, // In-Sen
  0x463, // Iwato
  0x28D, // Kumoi
  0x18B, // Pelog Selisir
  0x1A3, // Pelog Tembung
  0xDDD, // Messiaen 3
  0x9E7, // Messiaen 4
  0x8E3, // Messiaen 5
  0xD75, // Messiaen 6
  0xBEF, // Messiaen 7
};
static constexpr uint8_t NUM_SCALE_PRESETS = 35;
static bool    s_scale_mode       = false; // per-pattern scale editor active
static bool    s_scale_fn_entry   = false; // FN still held from the entry gesture
static uint8_t s_scale_cycle_root = 0xFF;  // last FN+note root this FN-hold (0xFF = none)
static uint8_t s_scale_cycle_idx  = 0;     // preset index within the cycle

// Per-step probability editor (FN + SLIDE in Pattern Write), variation 1 only.
static bool    s_prob_mode   = false;
static uint8_t s_prob_base   = 0;  // 0,8,16,24 (+32 in extended half)
static bool    s_prob_ext    = false;
static int8_t  s_prob_step   = -1; // selected step, -1 = none
static uint8_t s_prob_target = 0;  // %-edit target: 0 accent, 1 slide, 2 transpose

// Per-pattern scale editor. Entered by FN + ACCENT in Pattern Write; edits the
// active edit pattern's scale (stored in its reserved metadata, saved with the
// pattern). Non-destructive quantization applies live to playback.
//   Without FN: pitch keys C..B toggle individual classes; High C (pitch_leds[12])
//     toggles the scale on/off.
//   With FN held: pressing a note loads a preset scale rooted at that note --
//     first press = Major, pressing the SAME note again advances through the
//     preset list (Minor, Dorian, ...). A different note resets to Major at that
//     root. Releasing FN resets the cycle, so the next FN+note is Major.
//   FN tap (press and release without a preset note) exits the editor. The
//   FN release that ends the entry gesture (FN + ACCENT) is ignored.
void ProcessScaleMode() {
  Leds::Set(FUNCTION_MODE_LED, true);
  Sequence &s = engine.get_edit_sequence();

  if (inputs[FUNCTION_KEY].falling()) {
    if (s_scale_fn_entry)
      s_scale_fn_entry = false;         // entry gesture's release: stay
    else if (s_scale_cycle_root == 0xFF)
      s_scale_mode = false;             // FN tap with no preset: exit
    s_scale_cycle_root = 0xFF;
    s_scale_cycle_idx = 0;
  }

  for (uint8_t i = 0; i < 12; ++i)
    Leds::Set(pitch_leds[i], s.scale_allows(i));
  Leds::Set(pitch_leds[12], s.scale_enabled());

  bool changed = false;
  if (inputs[FUNCTION_KEY].held()) {
    for (uint8_t i = 0; i < 12; ++i) {
      if (!inputs[pitched_keys[i]].rising()) continue;
      if (i == s_scale_cycle_root)
        s_scale_cycle_idx = uint8_t((s_scale_cycle_idx + 1) % NUM_SCALE_PRESETS);
      else { s_scale_cycle_root = i; s_scale_cycle_idx = 0; }
      const uint16_t shape = pgm_read_word(&SCALE_PRESETS[s_scale_cycle_idx]);
      s.set_scale_mask(uint16_t(((shape << i) | (shape >> (12 - i))) & 0x0FFF));
      s.set_scale_enabled(true);
      changed = true;
      break;
    }
  } else {
    for (uint8_t i = 0; i < 12; ++i) {
      if (inputs[pitched_keys[i]].rising()) { s.toggle_scale_class(i); changed = true; }
    }
    if (inputs[pitched_keys[12]].rising()) {
      s.set_scale_enabled(!s.scale_enabled());
      changed = true;
    }
  }

  // Edits apply live from RAM; the pattern (with its scale) is saved lazily via
  // engine.stale, so there is no per-toggle flash write to stall playback. Each
  // change is broadcast to the web editor (SysEx 0x2A).
  if (changed) {
    engine.stale = true;
    midi_send_scale_update(engine.get_patsel(), s.scale_mask(), s.scale_enabled(),
                           engine.get_edit_var());
  }
}

// Per-step probability editor (variation 1 / CV voice). A step can arm accent,
// slide, a down transpose, AND an up transpose all at once, each with its own
// probability 1..13 (13 = 100%, the arming default). Rolled independently each
// step; when both down and up pass, the engine coin-flips which shift applies.
//   Picker layer (TAP_NEXT not held): black keys pick the 8-step bank (A# =
//     extended 32-63), white keys 1-8 select a step, BACK deselects. With a
//     step selected: ACCENT / SLIDE toggle those on/off; DOWN toggles octave
//     down (-12); UP cycles none -> up (+12) -> double-up (+24). Toggling a
//     characteristic on arms it at 100% and makes it the %-edit target.
//   Level layer (TAP_NEXT held, step selected): ACCENT/SLIDE/DOWN/UP pick which
//     characteristic's % to edit; note keys C..high C set its level 1..13, shown
//     as a bar on the pitch LEDs.
//   Randomize layer (CLEAR held): ACCENT/SLIDE/DOWN/UP scatter random
//     probabilities for that characteristic across the pattern; TAP_NEXT does
//     all four. ~1/3 of steps land unarmed, the rest at a random level.
//   FN exits (shared FN exit chain). Edits are RAM-only; flash is deferred.
static void ProcessProbabilityMode(bool clk_run, bool dial_pattern_write) {
  Leds::Set(FUNCTION_MODE_LED, true);
  const Sequence &seq = engine.get_sequence();   // probability rides variation 1
  const uint8_t blen = seq.length;
  uint8_t *table = engine.prob_var1_;            // 3 bytes/step
  // CLEAR held = randomize layer: ACCENT/SLIDE/DOWN/UP scatter random
  // probabilities for that one characteristic across the pattern; TAP_NEXT
  // randomizes all four. Takes priority over the picker/level layers.
  const bool clear_held = inputs[CLEAR_KEY].held() && dial_pattern_write;
  const bool level_layer = !clear_held && inputs[TAP_NEXT].held() && s_prob_step >= 0;
  const bool blink = bool((millis() >> 7) & 1);

  if (clear_held) {
    uint8_t kind = 0xFF;
    if (inputs[ACCENT_KEY].rising()) kind = 0;
    if (inputs[SLIDE_KEY].rising())  kind = 1;
    if (inputs[DOWN_KEY].rising())   kind = 2;
    if (inputs[UP_KEY].rising())     kind = 3;
    if (inputs[TAP_NEXT].rising())   kind = 4;
    if (kind != 0xFF) {
      engine.prob_randomize(kind);
      midi_send_prob_table(engine.get_patsel()); // one full-table push to the web
    }
  } else if (!level_layer) {
    // A# extended half only exists above 32 steps; at MAX_STEPS = 32 the four
    // black-key banks cover the whole section.
    if (MAX_STEPS > 32 && inputs[ASHARP_KEY].rising()) {
      s_prob_ext = !s_prob_ext;
      s_prob_base = uint8_t((s_prob_base & 31) + (s_prob_ext ? 32 : 0));
    }
    const uint8_t ext = s_prob_ext ? 32 : 0;
    if (inputs[CSHARP_KEY].rising()) s_prob_base = uint8_t(ext + 0);
    if (inputs[DSHARP_KEY].rising()) s_prob_base = uint8_t(ext + 8);
    if (inputs[FSHARP_KEY].rising()) s_prob_base = uint8_t(ext + 16);
    if (inputs[GSHARP_KEY].rising()) s_prob_base = uint8_t(ext + 24);
    // Any step is selectable: a characteristic on a REST/TIE step is legal
    // and latent (it applies if the step later becomes a NOTE).
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        const uint8_t cand = uint8_t(s_prob_base + wi);
        if (cand < blen) s_prob_step = int8_t(cand);
      }
    }
    if (inputs[BACK_KEY].rising()) s_prob_step = -1;

    if (dial_pattern_write && s_prob_step >= 0) {
      const uint8_t k = uint8_t(s_prob_step) * 3;
      uint8_t acc = prob_accent_level(table[k]), sld = prob_slide_level(table[k]);
      uint8_t dn = prob_down_level(table[k + 1]), up = prob_up_level(table[k + 1]);
      bool updbl = prob_up_double(table[k + 2]);
      bool changed = false;
      if (inputs[ACCENT_KEY].rising()) { acc = acc ? 0 : PROB_LEVEL_MAX; s_prob_target = 0; changed = true; }
      if (inputs[SLIDE_KEY].rising())  { sld = sld ? 0 : PROB_LEVEL_MAX; s_prob_target = 1; changed = true; }
      if (inputs[DOWN_KEY].rising())   { dn  = dn  ? 0 : PROB_LEVEL_MAX; s_prob_target = 2; changed = true; }
      if (inputs[UP_KEY].rising()) {
        // Cycle none -> up (+12) -> double-up (+24) -> none.
        if (!up)          { up = PROB_LEVEL_MAX; updbl = false; }
        else if (!updbl)  { updbl = true; }
        else              { up = 0; updbl = false; }
        s_prob_target = 3; changed = true;
      }
      if (changed) {
        const uint8_t b0 = prob_pack_ac_sl(acc, sld);
        const uint8_t b1 = prob_pack_du(dn, up);
        const uint8_t b2 = updbl ? 1 : 0;
        engine.prob_set_resident(uint8_t(s_prob_step), b0, b1, b2);
        midi_send_prob_step(engine.get_patsel(), uint8_t(s_prob_step), b0, b1, b2);
      }
    }
  } else if (dial_pattern_write) {
    // Level layer: pick the target characteristic, then note keys set its level.
    if (inputs[ACCENT_KEY].rising()) s_prob_target = 0;
    if (inputs[SLIDE_KEY].rising())  s_prob_target = 1;
    if (inputs[DOWN_KEY].rising())   s_prob_target = 2;
    if (inputs[UP_KEY].rising())     s_prob_target = 3;
    const uint8_t k = uint8_t(s_prob_step) * 3;
    uint8_t b0 = table[k], b1 = table[k + 1], b2 = table[k + 2];
    for (uint8_t i = 0; i <= 12; ++i) {
      if (!inputs[pitched_keys[i]].rising()) continue;
      const uint8_t lvl = uint8_t(i + 1);
      if (s_prob_target == 0 && prob_accent_level(b0))
        b0 = prob_pack_ac_sl(lvl, prob_slide_level(b0));
      else if (s_prob_target == 1 && prob_slide_level(b0))
        b0 = prob_pack_ac_sl(prob_accent_level(b0), lvl);
      else if (s_prob_target == 2 && prob_down_level(b1))
        b1 = prob_pack_du(lvl, prob_up_level(b1));
      else if (s_prob_target == 3 && prob_up_level(b1))
        b1 = prob_pack_du(prob_down_level(b1), lvl);
      else break; // target not armed -> nothing to set
      engine.prob_set_resident(uint8_t(s_prob_step), b0, b1, b2);
      midi_send_prob_step(engine.get_patsel(), uint8_t(s_prob_step), b0, b1, b2);
      break;
    }
  }

  // -- LEDs --
  // The step-grid / bank / selection LEDs live on the same note-key LEDs that
  // the level bar uses, so in the level layer draw ONLY the bar (below) -- the
  // step grid would otherwise ghost behind the percentage readout.
  const bool blinkb = bool((millis() >> 8) & 1);
  if (!level_layer) {
    // Step LEDs: armed = bright, unarmed NOTE step = dim (orientation).
    for (uint8_t wi = 0; wi < 8; ++wi) {
      const uint8_t idx = uint8_t(s_prob_base + wi);
      if (idx >= blen) break;
      if (table[idx * 3] || table[idx * 3 + 1]) Leds::Set(OutputIndex(wi), true);
      else if (seq.time(idx) == 1) Leds::SetDim(OutputIndex(wi), true);
    }
    // Playhead chase within the visible bank.
    if (clk_run) {
      const uint8_t tp = uint8_t(engine.get_time_pos() & (MAX_STEPS - 1));
      if ((tp & ~uint8_t(7)) == s_prob_base)
        Leds::Set(OutputIndex(tp & 0x7), bool(clk_count & 4));
    }
    // Bank cover LEDs (selected bank blinks) + A# extended state.
    const uint8_t ext = s_prob_ext ? 32 : 0;
    const uint8_t base_off = uint8_t(s_prob_base & 31);
    const OutputIndex sel_base_led =
        (base_off == 0)  ? CSHARP_KEY_LED :
        (base_off == 8)  ? DSHARP_KEY_LED :
        (base_off == 16) ? FSHARP_KEY_LED : GSHARP_KEY_LED;
    Leds::Set(CSHARP_KEY_LED, (blen > ext + 0)  && (sel_base_led == CSHARP_KEY_LED ? blinkb : true));
    Leds::Set(DSHARP_KEY_LED, (blen > ext + 8)  && (sel_base_led == DSHARP_KEY_LED ? blinkb : true));
    Leds::Set(FSHARP_KEY_LED, (blen > ext + 16) && (sel_base_led == FSHARP_KEY_LED ? blinkb : true));
    Leds::Set(GSHARP_KEY_LED, (blen > ext + 24) && (sel_base_led == GSHARP_KEY_LED ? blinkb : true));
    Leds::Set(ASHARP_KEY_LED, s_prob_ext ? true : (blen > 32 ? blinkb : false));
    // Selection flash.
    if (s_prob_step >= 0 && (uint8_t(s_prob_step) & ~uint8_t(7)) == s_prob_base)
      Leds::Set(OutputIndex(s_prob_step & 0x7), blink);
  }

  if (s_prob_step >= 0) {
    const uint8_t k = uint8_t(s_prob_step) * 3;
    const uint8_t b0 = table[k], b1 = table[k + 1], b2 = table[k + 2];
    const uint8_t acc = prob_accent_level(b0), sld = prob_slide_level(b0);
    const uint8_t dn = prob_down_level(b1), up = prob_up_level(b1);
    // Characteristic LEDs: solid = armed. In the level layer the %-edit target
    // blinks so you can see which characteristic the note keys will set. UP
    // blinks when it is double-up (+24), solid when single up (+12).
    Leds::Set(ACCENT_KEY_LED, (level_layer && s_prob_target == 0) ? blink : (acc != 0));
    Leds::Set(SLIDE_KEY_LED,  (level_layer && s_prob_target == 1) ? blink : (sld != 0));
    Leds::Set(DOWN_KEY_LED,   (level_layer && s_prob_target == 2) ? blink : (dn != 0));
    Leds::Set(UP_KEY_LED,     (level_layer && s_prob_target == 3) ? blink
                                                                  : (up && (!prob_up_double(b2) || blink)));
    // Level bar on the pitch LEDs for the current %-edit target.
    if (level_layer) {
      uint8_t lvl = 0;
      if (s_prob_target == 0) lvl = acc;
      else if (s_prob_target == 1) lvl = sld;
      else if (s_prob_target == 2) lvl = dn;
      else if (s_prob_target == 3) lvl = up;
      for (uint8_t i = 0; i < 13 && i < lvl; ++i) Leds::Set(pitch_leds[i], true);
    }
  }
}

static void process_config_menu() {
  if (s_cfg_menu == CfgMenu::Off) return;
  Leds::Set(FUNCTION_MODE_LED, true);

  static bool s_high_bank = false;
  if (inputs[DSHARP_KEY].rising()) s_high_bank = !s_high_bank;
  Leds::Set(DSHARP_KEY_LED, s_high_bank);

  // Channel-edit target: hold PITCH_KEY -> variation 2 channel, SLIDE_KEY ->
  // variation 3 channel, neither -> the main (variation 1 / device) channel.
  uint8_t *chan_target = &GlobalSettings.midi_channel;
  uint8_t  dc = cfg_display_channel();
  if (inputs[PITCH_KEY].held()) {
    chan_target = &GlobalSettings.var2_channel;
    dc = GlobalSettings.var2_channel;
    Leds::Set(PITCH_MODE_LED, true);
  } else if (inputs[SLIDE_KEY].held()) {
    chan_target = &GlobalSettings.var3_channel;
    dc = GlobalSettings.var3_channel;
    Leds::Set(SLIDE_KEY_LED, true);
  }
  if (s_high_bank) {
    if (dc >= 9 && dc <= 16) Leds::Set(OutputIndex((dc - 9) & 0x7), true);
  } else {
    if (dc >= 1 && dc <= 8)  Leds::Set(OutputIndex((dc - 1) & 0x7), true);
  }
  for (uint8_t i = 0; i < 8; ++i) {
    if (inputs[i].rising()) {
      *chan_target = uint8_t(i + 1 + (s_high_bank ? 8 : 0));
      cfg_save_midi();
      break;
    }
  }

  Leds::Set(TIME_MODE_LED, !GlobalSettings.midi_clock_receive);
  if (inputs[TIME_KEY].rising()) {
    GlobalSettings.midi_clock_receive = !GlobalSettings.midi_clock_receive;
    cfg_save_midi();
  }

  Leds::Set(ACCENT_KEY_LED, !GlobalSettings.midi_thru);
  if (inputs[ACCENT_KEY].rising()) {
    GlobalSettings.midi_thru = !GlobalSettings.midi_thru;
    cfg_save_midi();
  }

  // A# held + UP/DOWN: LED brightness 1..8
  if (inputs[ASHARP_KEY].held()) {
    Leds::Set(ASHARP_KEY_LED, true);
    if (inputs[UP_KEY].rising() && GlobalSettings.led_brightness < 8) {
      GlobalSettings.led_brightness++;
      Leds::brightness = GlobalSettings.led_brightness;
      cfg_save_midi();
    }
    if (inputs[DOWN_KEY].rising() && GlobalSettings.led_brightness > 1) {
      GlobalSettings.led_brightness--;
      Leds::brightness = GlobalSettings.led_brightness;
      cfg_save_midi();
    }
  }

#ifdef SUPEROS_COMBINED
  // G# = boot the D650C firmware (original mask-ROM 303). Solid G# = SuperOS
  // is the running firmware (the d650c menu blinks it). On press: flush
  // pending saves (same set as transport stop), then reboot.
  Leds::Set(GSHARP_KEY_LED, true);
  if (inputs[GSHARP_KEY].rising()) {
    // persist_shadows() FIRST: variation 2/3 and poly edits live in RAM and
    // Save() skips them when !stale, so without this a firmware switch threw
    // them away. Matches the SysEx switch path in midi.cpp.
    engine.persist_shadows();
    if (engine.stale || engine.aux_dirty()) engine.Save();
    if (engine.track_stale) engine.SaveTrack();
    midi_flush_pending_saves();
    midi_flush_pending_pattern_saves(engine);
    combined_switch_firmware(FW_D650);   // does not return
  }
#endif

  if (inputs[CLEAR_KEY].rising()) {
    if (s_cfg_suppress_clear_exit)
      s_cfg_suppress_clear_exit = false;
    else
      s_cfg_menu = CfgMenu::Off;
  }
}

// =============================================================================
// Write-mode input helpers — map matrix keys to Engine sequence edits
// =============================================================================

uint8_t check_pitch_inputs() {
  uint8_t notes = 0;
  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].held()) ++notes;
  }
  return notes;
}
bool check_time_inputs() {
  if (inputs[DOWN_KEY].held())   return true;
  if (inputs[UP_KEY].held())     return true;
  if (inputs[ACCENT_KEY].held()) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Octave resolution. OS-303 encoding has 4 octave registers (0..3) which we
// expose via the original 303's UI: DOWN-only = 0, neither = 1, UP-only = 2,
// DOWN+UP held simultaneously = 3 (the very-top register). Both bits stored
// directly in the pitch byte's oct field (bits[5:4]) so OS-303 round-trip
// preserves the user's choice.
// ---------------------------------------------------------------------------
static uint8_t resolve_octave() {
  const bool dn = inputs[DOWN_KEY].held();
  const bool up = inputs[UP_KEY].held();
  if (dn && up) return 3; // OCTAVE_DOUBLE_UP (top register)
  if (up)       return 2; // OCTAVE_UP
  if (dn)       return 0; // OCTAVE_DOWN
  return 1;               // OCTAVE_ZERO (centre)
}

// Returns the MIDI note number (36-108) written this call, or 0 if nothing was written.
// Helper: SysEx broadcast of the pitch byte for the current edit cursor (time_pos).
static void send_step_update_for_cursor(Sequence &s) {
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  uint8_t slot;
  uint8_t pb = PITCH_EMPTY;
  if (s.edit_slot_index(slot)) pb = s.pitch[slot];
  midi_send_step_update(engine.get_edit_patsel(), tp, pb, s.time(tp), engine.get_edit_var());
}

uint8_t input_pitch(bool mod = false, bool clk_run = false) {
  Sequence &s = engine.get_edit_sequence();
  if (clk_run && engine.is_step_locked()) return 0;
  if (mod) {
    if (!clk_run) s.ensure_pitch_edit_entry();
    bool flag_changed = false;
    if (inputs[ACCENT_KEY].rising()) { engine.ToggleAccent(); flag_changed = true; }
    if (inputs[SLIDE_KEY].rising())  { engine.ToggleSlide();  flag_changed = true; }
    if (inputs[UP_KEY].rising())     { engine.NudgeOctave(1); flag_changed = true; }
    if (inputs[DOWN_KEY].rising())   { engine.NudgeOctave(-1);flag_changed = true; }
    if (flag_changed) send_step_update_for_cursor(s);
  } else {
    s.ensure_pitch_edit_entry();
  }
  for (int pi = int(ARRAY_SIZE(pitched_keys)) - 1; pi >= 0; --pi) {
    const uint8_t i = uint8_t(pi);
    if (inputs[pitched_keys[i]].rising()) {
      if (mod) {
        engine.SetPitchSemitone(i);
        send_step_update_for_cursor(s);
        return uint8_t(engine.get_midi_note());
      } else {
        const uint8_t oct   = resolve_octave();
        const uint8_t flags = (inputs[ACCENT_KEY].held() << 6) |
                              (inputs[SLIDE_KEY].held()   << 7);
        engine.SetPitch(pack_pitch(i, oct), flags);
        const uint8_t written_note = uint8_t(36 + unpack_pitch_linear(pack_pitch(i, oct)));
        // Spec 3-a: appending past the last NOTE grows the pattern by a step,
        // so a blank pattern ends up exactly as long as the run of notes just
        // played in. Inside existing content this is a plain overwrite and the
        // cursor walks the stream as before (spec 4-d).
        const uint8_t cap = s.is_triplet_mode() ? uint8_t(TRIPLET_MAX_STEPS)
                                                : uint8_t(MAX_STEPS);
        if (sequence_append_note_step(s, cap)) {
          sequence_ensure_pitch_for_notes(s);
          send_step_update_for_cursor(s);
          midi_send_length_update(engine.get_patsel(), s.length, engine.get_edit_var());
          // Section full. Spec 3-b: with both sections selected, notes 65-128
          // carry on into the B section, so hand the cursor over instead of
          // stopping. Otherwise "if 64 NOTES are entered, you'll automatically
          // exit out of PITCH MODE" -- there is nowhere left to append.
          if (s.note_count() >= cap) {
            const uint8_t cur = engine.get_patsel();
            if (s_ab_mode && !Engine::is_section_b(cur)) {
              engine.SetPattern(Engine::section_b_of(cur), true);
              Sequence &nb = engine.get_edit_sequence();
              nb.reset     = false;
              nb.pitch_pos = int(nb.note_count());
              nb.sync_time_pos_to_pitch_pos();
            } else {
              engine.SetMode(NORMAL_MODE, true);
            }
          }
        } else {
          send_step_update_for_cursor(s);
          s.advance_pitch_to_next_note();
        }
        return written_note;
      }
    }
  }
  return 0;
}
void input_time(bool mod = false, bool clk_run = false) {
  if (clk_run && engine.is_step_locked()) return;
  uint8_t written_time = 0xFF;
  uint8_t new_t = 0;
  if (inputs[DOWN_KEY].rising())        { new_t = 1; written_time = 1; }
  else if (inputs[UP_KEY].rising())     { new_t = 2; written_time = 2; }
  else if (inputs[ACCENT_KEY].rising()) { new_t = 0; written_time = 0; }
  if (written_time == 0xFF) return;

  if (!mod) { engine.AdvanceEditCursor(); ++s_time_edit_steps; }
  Sequence &s = engine.get_edit_sequence();
  const uint8_t len = s.length;
  uint8_t before_pt[MAX_STEPS];
  sequence_pack_per_time(s, before_pt);
  engine.SetTime(new_t);
  uint8_t after_pt[MAX_STEPS];
  sequence_pack_per_time(s, after_pt);

  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  // Send tp FIRST, then the diff loop. The web editor's 0x16 handler computes
  // noteIdx(blob, step) against the blob's current time_data; if a later step's
  // pitch update arrives before tp's time change is applied, noteIdx counts
  // tp as a NOTE and writes the pitch into the wrong stream slot, corrupting
  // pitch[]. Sending tp first lets the editor update time_data at tp so every
  // subsequent step computes the same K-th-NOTE index the firmware did.
  const uint8_t pat = engine.get_edit_patsel();
  const uint8_t ev = engine.get_edit_var();
  midi_send_step_update(pat, tp, after_pt[tp], written_time, ev);
  for (uint8_t i = 0; i < len; ++i) {
    if (i != tp && before_pt[i] != after_pt[i])
      midi_send_step_update(pat, i, after_pt[i], s.time(i), ev);
  }
}


extern "C" {
  static void jumptoboot(void) {
    // avr-gcc function pointers are WORD addresses: bootloader at byte
    // 0x1F000 = word 0xF800. (A 0x1F000 literal truncates to word 0xF000 =
    // byte 0x1E000 and only reached the bootloader by sliding through the
    // erased flash in between.)
    ((void (*)(void))0xF800)();
  }
}

// =============================================================================
// setup — MIDI, GPIO, optional bootloader, EEPROM load
// =============================================================================
// Only compiled for non-USB-MIDI builds (e.g. Arduino IDE with a Serial USB
// type); the app env always defines SUPEROS_USB_MIDI and keeps USB alive.
#if !DEBUG && !defined(SUPEROS_USB_MIDI)
static void usb_shutdown_hw() {
  UDIEN = 0;
  UDCON = 1;
  USBCON = (1 << FRZCLK);
  PLLCSR = 0;
}
#endif

void setup() {
#ifdef SUPEROS_COMBINED
  // Run the constructor (NSDMI defaults) over the zeroed shared arena; net
  // state matches what `static Engine engine;` produced.
  new (g_fw_arena) Engine;
#endif
  // Keep USB alive when built for USB MIDI; otherwise tear it down as before
  // (the stock app left USB detached to avoid the idle PLL/noise).
#if !DEBUG && !defined(SUPEROS_USB_MIDI)
  usb_shutdown_hw();
#endif
  midi_init(&engine);

  for (uint8_t i = 0; i < ARRAY_SIZE(INPUTS); ++i)
    pinMode(INPUTS[i], INPUT);
  for (uint8_t i = 0; i < ARRAY_SIZE(OUTPUTS); ++i)
    pinMode(OUTPUTS[i], OUTPUT);
  for (uint8_t i = 0; i < 4; ++i)
    digitalWriteFast(select_pin[i], HIGH);

  PollInputs(inputs);
  if (inputs[TAP_NEXT].held()) jumptoboot();

#if DEBUG
  Serial.begin(9600);
#endif

  flash_persist_begin(); // mount flash-as-EEPROM (formats on first boot)
  engine.Load();
  // A stored chain on the boot pattern re-arms so a saved chain plays from
  // power-on without re-selecting the pattern.
  arm_stored_chain(engine.get_patsel(), false);
  // Saved A/B memory on the boot pattern re-enters the mode from power-on.
  if (engine.pair_linked(engine.get_patsel())) s_ab_mode = true;
  midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
  midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
  Leds::brightness = GlobalSettings.led_brightness;
  Leds::BeginRefresh();
  // Quarter the ISR rate (3.9 kHz -> 977 Hz), same as the d650c side. The panel
  // scan blanks the shared matrix pins and is gated on the ISR tick counter to
  // phase-lock it to the multiplex frame. At the fast 256 us tick the frame is
  // only 1024 us, so the main loop's ~150 us per-iteration granularity is a big
  // fraction of it: the scan phase can't lock tightly and the residual jitter
  // beats (visible flicker when stopped, rate tracking loop speed). At 1024 us
  // per tick the frame is 4096 us and the same jitter is a small fraction, so
  // the scan locks cleanly and the beat disappears (this is exactly why the
  // d650c LEDs are steady). Row multiplex 244 Hz, dim-PWM ~61 Hz (SendISR
  // advances the PWM phase every 2 ticks), both above visible flicker.
  OCR3A = 255;
}

// =============================================================================
// Edit-mode LED feedback — current step pitch / time / flags
// =============================================================================
// Light the note LED + octave LEDs for a packed pitch (bits[3:0]=semi,
// bits[5:4]=octave button: 0 = DOWN, 1 = neither, 2 = UP, 3 = both).
static void show_pitch_leds(uint8_t packed) {
  Leds::Set(pitch_leds[packed & 0x0F], true);
  const uint8_t oct = (packed >> 4) & 0x03;
  // Spec 4-a: a single transposition lights its LED solid, a DOUBLE one blinks
  // it. Octave 1 is centre, so 0 = one down, 2 = one up, 3 = two up.
  const bool dbl = bool((millis() >> 7) & 1);
  Leds::Set(DOWN_KEY_LED, oct == 0);
  Leds::Set(UP_KEY_LED,   oct == 2 || (oct == 3 && dbl));
}

// Spec 4: PITCH MODE step/page display. The 1-8 LEDs show the cursor's step
// within its 8-step page; the black keys show how many pages the pattern has,
// with the viewed page blinking (A# marks the extended half, pages 5-8).
static void PrintPitchStepPage() {
  const Sequence &s = engine.edit_seq_view();
  const uint8_t tp       = uint8_t(s.time_pos & (MAX_STEPS - 1));
  const uint8_t pages    = uint8_t((s.length + 7) / 8);   // 1..8
  const uint8_t cur_page = uint8_t(tp >> 3);
  const bool    ext      = cur_page >= 4;
  const bool    blink    = bool((millis() >> 7) & 1);
  static const OutputIndex kPageLeds[4] =
      {CSHARP_KEY_LED, DSHARP_KEY_LED, FSHARP_KEY_LED, GSHARP_KEY_LED};
  Leds::Set(OutputIndex(tp & 0x7), true);
  Leds::Set(ASHARP_KEY_LED, ext ? blink : (pages > 4));
  const uint8_t base = ext ? 4 : 0;
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t p = uint8_t(base + i);
    if (p >= pages) continue;
    Leds::Set(kPageLeds[i], (p == cur_page) ? blink : true);
  }
}

void PrintPitch() {
  const Sequence &s = engine.edit_seq_view();
  const uint8_t pc = s.get_pitch_count();
  if (pc == 0) return;
  // Prefer the edit cursor's slot when on a NOTE; fall back to pitch_pos
  // (the held NOTE) when on TIE / REST so display still shows something.
  uint8_t slot = 0;
  bool have_slot = false;
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  if (tp < s.length && s.time(tp) == 1) {
    slot = s.pitch_index_for_note(tp);
    have_slot = (slot < pc);
  }
  if (!have_slot) {
    if (s.pitch_pos < 0 || s.pitch_pos >= int(pc)) return;
    slot = uint8_t(s.pitch_pos);
  }
  const uint8_t pb = s.pitch[slot];
  if (pb == PITCH_EMPTY) return;
  show_pitch_leds(pb & 0x3f);
  if (s.get_time() == 1) {
    Leds::Set(ACCENT_KEY_LED, (pb & (1 << 6)) != 0);
    Leds::Set(SLIDE_KEY_LED,  (pb & (1 << 7)) != 0);
  }
}
void PrintTime() {
  const uint8_t t = engine.edit_seq_view().get_time();
  Leds::Set(DOWN_KEY_LED,   t == 1);
  Leds::Set(UP_KEY_LED,     t == 2);
  Leds::Set(ACCENT_KEY_LED, t == 0);
  // Step-lock UI removed: it's RAM-only since the OS-303 layout migration
  // (no EEPROM byte for it), and the SLIDE_KEY toggle here confused users.
  Leds::Set(SLIDE_KEY_LED, false);
}

// Light the pitch-key LED of the NOTE at the edit cursor (no octave/flag
// LEDs; in TIME_MODE those belong to PrintTime's note/tie/rest display).
static void PrintCursorNoteLed() {
  const Sequence &s = engine.edit_seq_view();
  const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
  if (tp >= s.length || s.time(tp) != 1) return;
  const uint8_t slot = s.pitch_index_for_note(tp);
  if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY)
    Leds::Set(pitch_leds[s.pitch[slot] & 0x0F], true);
}

// ---------------------------------------------------------------------------
// ProcessDirectionMode — FN + PITCH_KEY: select playback direction
// C=Forward D=Reverse E=PingPong F=Random G=HalfRand A=Brownian; FN to exit
// ---------------------------------------------------------------------------
static const InputIndex  kDirKeys[DIR_COUNT] = {C_KEY, D_KEY, E_KEY, F_KEY, G_KEY, A_KEY};
static const OutputIndex kDirLeds[DIR_COUNT] = {C_KEY_LED, D_KEY_LED, E_KEY_LED, F_KEY_LED, G_KEY_LED, A_KEY_LED};

// `persist` = Pattern Write: write reserved[0] so direction persists per-pattern.
// `persist` false = Pattern Play / Track Play: RAM-only, undo SetDirection's stale
// flag so the change does not get saved on the next flush. Boot reload of the
// per-pattern direction byte is a Phase 3 task; today this just keeps RAM and
// EEPROM consistent with the user's intent for the current session.
void ProcessDirectionMode(bool persist) {
  Leds::Set(TIME_MODE_LED, clk_count & 4);
  const uint8_t ev = engine.get_edit_var();
  // Show/edit the direction of the variation being edited. Variation 1 uses the
  // engine's live direction; variations 2/3 store it on the shadow (read each
  // tick by AdvanceShadows) and persist with the shadow.
  const uint8_t cur_dir = (ev == 0) ? uint8_t(engine.get_direction())
                                    : engine.edit_seq_view().get_direction_stored();
  for (uint8_t d = 0; d < DIR_COUNT; ++d) {
    const bool active = (cur_dir == d);
    Leds::Set(kDirLeds[d], active ? bool(clk_count & 4) : true);
    if (inputs[kDirKeys[d]].rising()) {
      if (ev == 0) {
        engine.SetDirection(SequenceDirection(d));
        if (persist) engine.get_sequence().store_direction(uint8_t(d));
        else engine.stale = false;
      } else {
        engine.get_edit_sequence().store_direction(uint8_t(d)); // var2/3 -> shadow
      }
      midi_send_direction_update(d, ev); // var-tagged so the editor follows
    }
  }
  // Exit handled in main loop FUNCTION_KEY.rising() block
}

// ---------------------------------------------------------------------------
// ProcessEdit — TAP_NEXT held: pitch/time edit UI
//
// BACK_KEY behaviour:
//   rising → step back one position (clamps at step 0, never wraps)
//   If write_mode held AND not clk_run AND PITCH_MODE: audition the note now on.
//   falling → gate/audition off (handled in main loop TAP_NEXT.falling section)
// ---------------------------------------------------------------------------
void ProcessEdit(const bool &write_mode, const bool clk_run) {
  switch (engine.get_mode()) {
  case PITCH_MODE: {
    if (write_mode) {
      const uint8_t updated_note = input_pitch(true, clk_run);
      // When user presses a pitch key while TAP is held, re-audition with the new pitch.
      if (!clk_run && updated_note) {
        uint16_t mn = uint16_t(updated_note) + total_transpose;
        if (mn > 127) mn = 127;
        const Sequence &es = engine.edit_seq_view();
        const bool acc = es.get_accent();
        const uint8_t vel = acc ? 127 : 80;
        s_tap_pitch_preview_cv = clamp_cv(int(es.get_pitch()) + total_transpose);
        s_tap_pitch_preview_accent = acc;
        s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
        midi_audition_note_on(uint8_t(mn), vel);
      }
    }
    PrintPitch();
    break;
  }
  case TIME_MODE:
    if (write_mode) {
      input_time(true, clk_run);
      PrintTime();
      PrintCursorNoteLed(); // keep the step's note visible while stepping
    }
    break;
  case NORMAL_MODE:
    break;
  }

  // BACK_KEY: step back one position
  if (inputs[BACK_KEY].rising()) {
    engine.StepBack();
    const Sequence &es = engine.edit_seq_view();
    if (!clk_run && engine.get_mode() == PITCH_MODE && es.get_time() != 0) {
      uint16_t mn = uint16_t(36 + es.get_pitch()) + total_transpose;
      if (mn > 127) mn = 127;
      const bool acc = es.get_accent();
      const uint8_t vel = acc ? 127 : 80;
      s_back_pitch_preview_cv = clamp_cv(int(es.get_pitch()) + total_transpose);
      s_tap_pitch_preview_accent = acc;
      s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
      s_back_pitch_preview_gate = true;
      midi_audition_note_on(uint8_t(mn), vel);
    }
  }
  // Always close audition on BACK falling — fixes the infinitely-held note
  // bug when TAP_NEXT and BACK are pressed/released together (TAP falling
  // closed only its own preview; the back-preview latch was orphaned).
  if (inputs[BACK_KEY].falling() && engine.get_mode() == PITCH_MODE) {
    s_back_pitch_preview_gate = false;
    midi_audition_note_off();
  }
}

// Edit-variation picker: hold TAP_NEXT in Pattern Write + normal mode and the
// three variation LEDs (C/D/E) show immediately -- current variation lit, the
// other two half-dim. Press C/D/E to choose which variation (1/2/3) all pattern
// edits apply to; the choice is broadcast to the web editor (SysEx 0x1F). The
// edit variation is latched until a new pattern is selected (SetPattern resets
// it to variation 1). Edit a different pattern's variations by selecting that
// pattern first (TAP released), then holding TAP again.
void ProcessEditVarPicker(bool clk_run) {
  const uint8_t pat = engine.get_patsel();
  if (inputs[C_KEY].rising() && engine.SetEditVar(0)) midi_send_edit_variation(pat, 0);
  if (inputs[D_KEY].rising() && engine.SetEditVar(1)) midi_send_edit_variation(pat, 1);
  if (inputs[E_KEY].rising() && engine.SetEditVar(2)) midi_send_edit_variation(pat, 2);
  const uint8_t ev = engine.get_edit_var();
  // Selected variation flashes; the other two show dim (selectable targets).
  // The blink mask works while stopped too: clk_count free-runs at 120 BPM.
  const bool blink = bool(clk_count & 4);
  Leds::Set(C_KEY_LED, ev == 0 && blink); Leds::SetDim(C_KEY_LED, ev != 0);
  Leds::Set(D_KEY_LED, ev == 1 && blink); Leds::SetDim(D_KEY_LED, ev != 1);
  Leds::Set(E_KEY_LED, ev == 2 && blink); Leds::SetDim(E_KEY_LED, ev != 2);

  // Variation 3 only: SLIDE_KEY toggles poly/mono for the active slot (stopped
  // only -- it does a settings write + shadow reload). SLIDE_KEY_LED lit =
  // poly, off = mono.
  const uint8_t slot = engine.abs_slot(pat);
  if (ev == 2 && !clk_run && inputs[SLIDE_KEY].rising()) {
    engine.persist_shadows();                                       // flush pending var2/3 edits
    GlobalSettings.set_var3_poly(slot, !GlobalSettings.var3_is_poly(slot));
    GlobalSettings.Save();
    engine.ReloadShadows();                                          // (de)activate poly_ for this slot
    midi_send_poly_flag(pat, GlobalSettings.var3_is_poly(slot));     // tell the web editor
  }
  Leds::Set(SLIDE_KEY_LED, ev == 2 && GlobalSettings.var3_is_poly(slot));
}

// ---------------------------------------------------------------------------
// ProcessPolyEdit — variation-3 polyphonic chord editor (PITCH_MODE, var3 poly).
// View octave = held DOWN/UP (0/1/2/3). A pitch key toggles that note at the view
// octave (up to POLY_VOICES per step); ACCENT/SLIDE toggle the step flags;
// TAP_NEXT / BACK move between steps. A step plays when it holds >=1 note.
// (Moving a note across octaves = toggle it off at one octave, on at another.)
// ---------------------------------------------------------------------------
static uint8_t s_poly_step = 0;
// Latched octave for entering chord notes: tap UP/DOWN to set it (tap UP twice for
// the double-up octave); it stays until changed -- no need to hold UP/DOWN.
static uint8_t s_poly_view_oct = 1; // 0=down 1=centre 2=up 3=double-up
void ProcessPolyEdit() {
  PolyVoice &p = engine.poly_;
  const uint8_t len = p.length ? p.length : 1;
  if (s_poly_step >= len) s_poly_step = 0;
  p.ensure_chords_for_notes();   // keep the chord stream covering every NOTE event

  // TAP_NEXT advances (wraps); BACK steps back but CLAMPS at step 0 so re-listening
  // never loops around to the end. Either one auditions the landing chord below.
  bool nav = false;
  if (inputs[TAP_NEXT].rising()) { s_poly_step = uint8_t((s_poly_step + 1) % len); nav = true; }
  if (inputs[BACK_KEY].rising())  { if (s_poly_step) --s_poly_step; nav = true; }

  // Latched octave: a single UP/DOWN tap sets the octave new notes land in (tap UP
  // twice for double-up). No holding required; it stays until changed.
  if (inputs[UP_KEY].rising()   && s_poly_view_oct < 3) ++s_poly_view_oct;
  if (inputs[DOWN_KEY].rising() && s_poly_view_oct > 0) --s_poly_view_oct;
  const uint8_t view_oct = s_poly_view_oct;
  bool changed = false;

  // Pitch keys edit the chord this NOTE step pulls from the stream. On a REST the
  // first pitch press promotes it to a NOTE (which inserts a chord into the list).
  for (uint8_t k = 0; k < ARRAY_SIZE(pitched_keys); ++k) {
    if (!inputs[pitched_keys[k]].rising()) continue;
    if (p.time(s_poly_step) == 0) {
      const uint8_t cc0 = p.get_chord_count();     // promote REST -> NOTE
      p.set_time(s_poly_step, 1);
      p.ensure_chords_for_notes();
      const uint8_t cin = p.chord_index_for_step(s_poly_step);
      if (cin == cc0) { uint8_t *vn = p.chord(cin); vn[0]=vn[1]=vn[2]=vn[3]=POLY_EMPTY; } // start the fresh chord empty so only pressed notes land (no phantom default C)
    }
    if (p.time(s_poly_step) != 1) { changed = true; continue; } // TIE: holds prior chord, not editable
    const uint8_t ci = p.chord_index_for_step(s_poly_step);
    const uint8_t packed = pack_pitch(k, view_oct);
    if (p.has_note(ci, packed)) p.remove_note(ci, packed);
    else                        p.add_note(ci, packed);
    changed = true;
  }
  const bool is_note = (p.time(s_poly_step) == 1);
  const uint8_t ci = p.chord_index_for_step(s_poly_step);
  if (is_note && inputs[ACCENT_KEY].rising()) { p.toggle_accent(ci); changed = true; }
  if (is_note && inputs[SLIDE_KEY].rising())  { p.toggle_slide(ci);  changed = true; }
  if (inputs[TIME_KEY].rising()) { // cycle this step's time NOTE -> TIE -> REST (shifts the list)
    const uint8_t prv = uint8_t((s_poly_step + len - 1) % len);
    const uint8_t cur = p.time(s_poly_step);
    uint8_t nt = (cur == 1) ? 2 : (cur == 2) ? 0 : 1; // note -> tie -> rest -> note
    if (nt == 2 && p.time(prv) == 0) nt = 0;          // no tie after a rest -> skip to rest
    p.set_time(s_poly_step, nt);
    p.ensure_chords_for_notes();
    changed = true;
  }
  if (changed) {
    engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
    midi_send_poly_step(engine.get_patsel(), s_poly_step); // mirror the panel chord edit to the web
  }

  // Audition the full chord (MIDI only, var3 channel) when navigating to a step;
  // close it on release. The 303 CV is suppressed for non-CV variations at the DAC.
  if (nav) {
    if (is_note) midi_audition_chord_on(p.chord(ci), p.accent(ci), total_transpose);
    else         midi_audition_chord_off();
  }
  if (inputs[TAP_NEXT].falling() || inputs[BACK_KEY].falling()) midi_audition_chord_off();

  // ---- LEDs ---- (show the chord only on NOTE steps; TIE holds, REST is silent)
  if (is_note) {
    const uint8_t *v = p.chord(ci);
    for (uint8_t i = 0; i < POLY_VOICES; ++i)
      if (v[i] != POLY_EMPTY && ((v[i] >> 4) & 0x03) == view_oct)
        Leds::Set(pitch_leds[v[i] & 0x0F], true);
  }
  Leds::Set(DOWN_KEY_LED, view_oct == 0 || view_oct == 3);
  Leds::Set(UP_KEY_LED,   view_oct == 2 || view_oct == 3);
  Leds::Set(ACCENT_KEY_LED, is_note && p.accent(ci));
  Leds::Set(SLIDE_KEY_LED,  is_note && p.slide(ci));
  Leds::Set(TIME_MODE_LED,  p.time(s_poly_step) == 2);                              // tie indicator
  Leds::Set(OutputIndex(s_poly_step & 0x07), bool((millis() >> 7) & 1));            // step pat-LED (blink)
  Leds::Set(OutputIndex(CSHARP_KEY_LED + ((s_poly_step & 31) >> 3)), true);          // 8-step bank
  if (s_poly_step >= 32) Leds::Set(ASHARP_KEY_LED, (millis() >> 7) & 1);
  Leds::Set(PITCH_MODE_LED, true);
}

// Default overlay: pattern select, bank A/B, mode LEDs, running step chase
void ProcessDefault(const bool &write_mode, const bool &clear_mod,
               const bool &clk_run, const bool &dial_pattern_write) {
  switch (engine.get_mode()) {
  case PITCH_MODE:
    // Spec 4: stopped, the step/page display shows where in the pattern the
    // cursor sits (TAP held swaps this for the note itself -- ProcessEdit owns
    // that frame). Running, "the chase light will turn off": only the note
    // being played is shown, no step or page LEDs.
    if (clk_run) PrintPitch();
    else         PrintPitchStepPage();
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case TIME_MODE:
    // Time state + note LED stay visible in TIME_MODE whether running or
    // stopped (stopped display previously went dark between TAP presses).
    PrintTime();
    PrintCursorNoteLed();
    if (clk_run) {
      const uint8_t tp = engine.get_edit_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      // Bank indicator disabled in live time edit (see PITCH_MODE above).
      // Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      // if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }
    if (!write_mode) engine.SetMode(NORMAL_MODE);
    break;

  case NORMAL_MODE: {
    // Pattern-key target section. A linked pair alternates patsel between its
    // A and B slots as it PLAYS, so the raw section bit follows playback, not
    // the user's selection: a key tapped while the B half happened to be
    // playing targeted the B-section pattern (press 2 during 1B -> 2B). A
    // linked pair's home is its A section, so taps, queues and chain builds
    // target section A while one is selected; an unlinked selection keeps its
    // real section.
    const bool chain_live = s_chain_active && s_chain_len > 1 && clk_run;
    if (!chain_live) s_section_view = 0xFF;   // browse view only exists mid-chain
    uint8_t bank = s_ab_mode ? 0 : uint8_t(engine.get_patsel() >> 3);
    if (chain_live && s_section_view != 0xFF) bank = s_section_view;
    const bool browsing_other_group = clk_run && (s_display_group != engine.get_group());

    // ── LEDs ──
    // Suppress pattern LEDs when browsing a different group while running.
    if (!browsing_other_group) {
      for (uint8_t ci = 0; ci < s_chain_queue_len; ++ci)
        Leds::Set(OutputIndex(s_chain_queued[ci] & 0x7), true);
      if (s_chain_active && s_chain_len > 1) {
        for (uint8_t ci = 0; ci < s_chain_len; ++ci)
          Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
      } else if (s_chain_anchor_key != 0xff) {
        for (uint8_t ci = 0; ci < s_chain_len; ++ci)
          Leds::Set(OutputIndex(s_chain_pats[ci] & 0x7), true);
      } else {
        if (engine.get_patsel() != engine.get_next())
          Leds::Set(OutputIndex(engine.get_next() & 0x7), true);
      }
      Leds::Set(OutputIndex(engine.get_patsel() & 0x7), clk_count < 12);
    }
    // Section indicator. Unlinked: the selected section's LED alone (the
    // BROWSED bank while section-browsing a live chain). Linked -- per
    // pattern or chain-wide (spec 1-a): BOTH lit, and the section currently
    // playing / being written to blinks, so during a performance you can see
    // which half of the pattern is running without counting steps.
    if (s_ab_mode) {
      const bool on_b  = Engine::is_section_b(engine.get_patsel());
      const bool blink = bool(clk_run ? (clk_count < 12) : ((millis() >> 8) & 1));
      Leds::Set(ACCENT_KEY_LED, on_b ? true  : blink);
      Leds::Set(SLIDE_KEY_LED,  on_b ? blink : true);
    } else {
      Leds::Set(ACCENT_KEY_LED, !bank); // A
      Leds::Set(SLIDE_KEY_LED,   bank); // B
    }

    // Step chase: Pattern Write only. Pattern Play hides the chase to keep the
    // performance display calm.
    if (clk_run && dial_pattern_write) {
      const uint8_t tp = engine.get_edit_time_pos();
      Leds::Set(OutputIndex(tp & 0x7), true);
      Leds::Set(OutputIndex(CSHARP_KEY_LED + ((tp & 31) >> 3)), true);
      if (tp >= 32) Leds::Set(ASHARP_KEY_LED, clk_count & 4);
    }

    // ── Pattern select inputs ──
    // A+B chain builder: while ACCENT and SLIDE are both held, pattern-key taps
    // append to a chain (any order, repeats allowed, cap 4, current bank).
    // Releasing either button commits it: stopped -> starts now, running ->
    // takes over at the next pattern wrap. The first tap converts the gesture
    // from "link the pair" to "build a chain", so the link side effect of the
    // A+B press is restored to what it was.
    const bool ab_hold = inputs[ACCENT_KEY].held() && inputs[SLIDE_KEY].held() &&
                         !clear_mod && !s_metronome_active;
    if (ab_hold) {
      // Ghost guard (same rule as keyboard play): with both modifiers down, a
      // real key press in the ACCENT/SLIDE columns can phantom its row
      // partner (keys 3/4 and 7/8). Both rising together cannot be told
      // apart -- drop the pair rather than chain a wrong pattern.
      bool dropped[8] = {false};
      if (inputs[2].rising() && inputs[3].rising()) dropped[2] = dropped[3] = true;
      if (inputs[6].rising() && inputs[7].rising()) dropped[6] = dropped[7] = true;
      for (uint8_t i = 0; i < 8; ++i) {
        if (!inputs[i].rising() || dropped[i]) continue;
        if (s_ab_chain_len == 0) {
          // First tap: this is a chain build, not an A/B-mode entry -- the
          // A+B press's mode flip is reverted to what it was.
          if (s_ab_link_pat != 0xff) {
            s_ab_mode     = s_ab_prev_link;
            s_ab_link_pat = 0xff;
          }
          s_chain_hold_key        = 0xff;  // cancel single-key tap/hold tracking
          s_chain_hold_crossed    = false;
          s_chain_hold_target_pat = 0xff;
          s_chain_anchor_key      = 0xff;
        }
        if (s_ab_chain_len < 4)
          s_ab_chain_pats[s_ab_chain_len++] = uint8_t(bank * 8 + i);
      }
      // A+B + TAP: persist the chain. Mid-build saves the build; otherwise the
      // active chain is saved onto its first pattern. With nothing to save,
      // it clears a stored chain from the current pattern. The mode LEDs
      // flash as the acknowledgement.
      if (inputs[TAP_NEXT].rising()) {
        if (s_ab_chain_len >= 2) {
          store_chain_on(s_ab_chain_pats[0], s_ab_chain_pats, s_ab_chain_len);
          engine.set_pair_linked(s_ab_chain_pats[0], s_ab_mode);
        } else if (s_chain_active && s_chain_len >= 2) {
          store_chain_on(s_chain_pats[0], s_chain_pats, s_chain_len);
          engine.set_pair_linked(s_chain_pats[0], s_ab_mode);
        } else {
          Sequence &cs = engine.pattern[engine.get_patsel() & 0x0F];
          cs.reserved[4] = 0; cs.reserved[5] = 0; cs.reserved[6] = 0;
          engine.stale = true;
          midi_send_pattern_update(engine.get_patsel());
        }
        pattern_cleared_flash_timer = 0;
        s_pat_cleared_hold = true;
      }
    } else if (s_ab_chain_len) {
      // A or B released: commit the build.
      if (s_ab_chain_len >= 2) {
        s_chain_bank = bank;
        s_chain_len  = s_ab_chain_len;
        for (uint8_t ci = 0; ci < s_ab_chain_len; ++ci)
          s_chain_pats[ci] = s_ab_chain_pats[ci];
        s_chain_active    = true;
        s_chain_queue_len = 0;
        s_chain_hold_loop = false;
        chain_intent_reset();
        // Saved A/B memory on the first member re-enters the mode.
        if (engine.pair_linked(s_chain_pats[0])) s_ab_mode = true;
        if (clk_run) {
          s_chain_pos = uint8_t(s_chain_len - 1); // wrap advances into pats[0]
        } else {
          s_chain_pos = 0;
          uint8_t first = s_chain_pats[0];
          if (s_ab_mode) first = Engine::section_a_of(first);
          engine.SetPattern(first, true);
          midi_send_active_pattern(engine.get_patsel());
        }
      } else {
        // Single tap: behave like a plain pattern tap.
        const uint8_t pat = chain_entry_start(s_ab_chain_pats[0]);
        if (clk_run && s_chain_active && s_chain_len > 1) {
          s_chain_queue_len = 1;
          s_chain_queued[0] = pat;
        } else {
          engine.SetPattern(pat, !clk_run);
          if (!clk_run) midi_send_active_pattern(engine.get_patsel());
        }
      }
      emit_chain_state();
      s_ab_chain_len = 0;
    }
    if (!ab_hold && !inputs[ACCENT_KEY].held() && !inputs[SLIDE_KEY].held())
      s_ab_link_pat = 0xff;  // gesture fully released with no taps: link stands

    if (s_metronome_active) {
      // Tap-write session: pattern keys 1-8 are the ROM's SUSTAIN modifier
      // (metro_sustain_held); no selection, chains, or bank switches here.
    } else if (clk_run && clear_mod) {
      // CLEAR held while running: pat keys reserved for global copy/paste.
    } else if (clk_run && browsing_other_group) {
      // Running in a different group: queue the group switch + pattern to start at next wrap
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          engine.QueueGroup(s_display_group);
          // A live chain would re-queue its own next pattern every loop pass
          // and stomp this queued switch, so a group change ends the chain.
          s_chain_active    = false;
          s_chain_len       = 0;
          s_chain_queue_len = 0;
          chain_intent_reset();
          engine.SetPattern(chain_entry_start(uint8_t(bank * 8 + i)), false);
          emit_chain_state();
          break;
        }
      }
    }
    if (clk_run && !browsing_other_group && !clear_mod && !ab_hold) {
      // Running: chain building always available (whether or not a chain is currently active).
      //   two keys pressed simultaneously / hold+tap → build or queue a chain
      //   single key tap (quick press+release)       → queue single pattern (or chain pattern)
      //   single key hold (> CHAIN_HOLD_MS)          → loop that pattern when chain reaches it
      const uint32_t now = (uint32_t)millis();

      // 1. Detect hold+new-key: any rising key with another key already held → build chain
      bool chain_built = false;
      for (uint8_t ni = 0; ni < 8 && !chain_built; ++ni) {
        if (!inputs[ni].rising()) continue;
        for (uint8_t hi2 = 0; hi2 < 8; ++hi2) {
          if (hi2 == ni || !inputs[hi2].read()) continue;
          const uint8_t lo2 = (hi2 < ni) ? hi2 : ni;
          uint8_t       ht  = (hi2 > ni) ? hi2 : ni;
          if (ht - lo2 > 3) ht = lo2 + 3;
          const uint8_t new_len = ht - lo2 + 1;
          s_chain_hold_key     = 0xff; // cancel any pending tap
          s_chain_hold_crossed = false;
          s_chain_hold_target_pat = 0xff;
          if (s_chain_active && s_chain_len > 1) {
            // Already in chain: queue the new chain; current chain must finish first
            if (s_chain_queue_len == 0) { // don't overwrite an existing queue
              s_chain_queue_len = new_len;
              for (uint8_t ci = 0; ci < new_len; ++ci)
                s_chain_queued[ci] = bank * 8 + lo2 + ci;
            }
          } else {
            // Not in chain: activate new chain immediately.
            // Set pos to len-1 so the chain advance queues pats[0] as next.
            s_chain_active = true;
            s_chain_len    = new_len;
            for (uint8_t ci = 0; ci < new_len; ++ci)
              s_chain_pats[ci] = bank * 8 + lo2 + ci;
            s_chain_pos       = new_len - 1;
            s_chain_queue_len = 0;
            chain_intent_reset();
            // Saved A/B memory on the first member re-enters the mode.
            if (engine.pair_linked(s_chain_pats[0])) s_ab_mode = true;
          }
          emit_chain_state();
          chain_built = true;
          break;
        }
      }

      // 2. Track single key for tap/hold (only when pressed alone)
      if (!chain_built) {
        if (s_chain_hold_key == 0xff) {
          for (uint8_t i = 0; i < 8; ++i) {
            if (!inputs[i].rising()) continue;
            bool other = false;
            for (uint8_t j = 0; j < 8; ++j)
              if (j != i && inputs[j].read()) { other = true; break; }
            if (!other) {
              s_chain_hold_key     = i;
              s_chain_hold_ms      = now;
              s_chain_hold_crossed = false;
            }
            break;
          }
        }

        // Update hold threshold
        if (s_chain_hold_key != 0xff && !s_chain_hold_crossed &&
            (now - s_chain_hold_ms) >= CHAIN_HOLD_MS) {
          s_chain_hold_crossed    = true;
          s_chain_hold_target_pat = bank * 8 + s_chain_hold_key; // loop this when reached
        }

        // Hold key released
        if (s_chain_hold_key != 0xff && inputs[s_chain_hold_key].falling()) {
          if (!s_chain_hold_crossed) {
            // Tap: queue single pattern -- or, when the target carries a
            // STORED chain (FN-saved), queue/arm that whole chain so "press 1
            // plays 1-2" works live as well as stopped.
            const uint8_t tgt = uint8_t(bank * 8 + s_chain_hold_key);
            const Sequence &ts = engine.pattern[tgt & 0x0F];
            const uint8_t  sl  = uint8_t(ts.reserved[4] & 0x07);
            if (s_chain_active && s_chain_len > 1) {
              if (sl >= 2 && sl <= 4) {
                // In chain: queue the target's stored chain (promotes on arrival)
                s_chain_queue_len = sl;
                s_chain_queued[0] = chain_entry_start(uint8_t(ts.reserved[5] & 0x0F));
                s_chain_queued[1] = uint8_t((ts.reserved[5] >> 4) & 0x0F);
                s_chain_queued[2] = uint8_t(ts.reserved[6] & 0x0F);
                s_chain_queued[3] = uint8_t((ts.reserved[6] >> 4) & 0x0F);
              } else {
                // In chain: queue as single → deactivates chain when reached
                s_chain_queue_len = 1;
                s_chain_queued[0] = chain_entry_start(tgt);
              }
            } else if (sl >= 2 && sl <= 4) {
              // Not in chain: the stored chain takes over at the next wrap
              // (same mechanics as committing an A+B build while running).
              arm_stored_chain(tgt, false);
              s_chain_pos = uint8_t(s_chain_len - 1); // wrap advances into pats[0]
            } else {
              // Not in chain: direct pattern switch
              engine.SetPattern(chain_entry_start(tgt), false);
            }
            emit_chain_state();
          }
          // else hold released: chain continues advancing (hold_loop cleared below)
          s_chain_hold_key        = 0xff;
          s_chain_hold_crossed    = false;
          s_chain_hold_target_pat = 0xff;
        }
      }

      // Hold-to-loop: only active while key still held past threshold
      s_chain_hold_loop = (s_chain_hold_key != 0xff && s_chain_hold_crossed);

    } else if (!clk_run && !ab_hold) {
      // Stopped: chain building.  When CLEAR is held, pat keys are reserved for
      // global copy/paste handlers below — do nothing here.
      s_chain_hold_key = 0xff; // clear stale running state
      if (clear_mod) {
        // fall through: copy/paste and other CLEAR combos own pat-key rising.
      } else if (s_chain_anchor_key == 0xff) {
        for (uint8_t i = 0; i < 8; ++i) {
          if (inputs[i].rising()) {
            s_chain_anchor_key = i;
            s_chain_bank       = bank;
            s_chain_pats[0]    = bank * 8 + i;
            s_chain_len        = 1;
            s_chain_active     = false;
            s_chain_hold_loop  = false;
            engine.SetPattern(chain_entry_start(uint8_t(bank * 8 + i)), true);
            // Stopped: notify web editor of new active pattern (no 0x15 stream while stopped).
            midi_send_active_pattern(engine.get_patsel());
            break;
          }
        }
      } else {
        for (uint8_t i = 0; i < 8; ++i) {
          if (i == s_chain_anchor_key) continue;
          if (inputs[i].rising() && s_chain_bank == bank) {
            uint8_t lo = (s_chain_anchor_key < i) ? s_chain_anchor_key : i;
            uint8_t hi = (s_chain_anchor_key > i) ? s_chain_anchor_key : i;
            if (hi - lo > 3) hi = lo + 3;
            s_chain_len = hi - lo + 1;
            for (uint8_t ci = 0; ci < s_chain_len; ++ci)
              s_chain_pats[ci] = bank * 8 + lo + ci;
          }
        }
        if (inputs[s_chain_anchor_key].falling()) {
          if (s_chain_len > 1) {
            s_chain_active = true;
            s_chain_pos    = 0;
            chain_intent_reset();
            engine.SetPattern(chain_entry_start(s_chain_pats[0]), true);
          } else {
            s_chain_active = false;
            // Plain select: a pattern with a stored chain re-arms it, so a
            // saved chain "always plays" when its pattern is chosen.
            arm_stored_chain(uint8_t(s_chain_bank * 8 + s_chain_anchor_key), false);
          }
          s_chain_anchor_key = 0xff;
          emit_chain_state();
        }
      }
    }

    // Section buttons. Skipped when CLEAR is held so CLEAR+ACCENT
    // (randomize) and CLEAR+SLIDE combos can take the edge, and during a
    // tap-write session (its keys belong to the recorder).
    // A+B together = enter the sticky A/B MODE (every selected pattern plays
    // A then B until the mode is left). One button alone = leave the mode and
    // select that section. Saved per-pattern A/B memory is NOT touched here:
    // the save gestures write it and FN + held pattern clears it.
    // LIVE chain: the buttons never kill or redirect the chain -- A+B turns
    // the mode on, a single press turns it off when on, else BROWSES that
    // bank (pattern keys + LEDs show it, playback untouched).
    const bool acc_edge = inputs[ACCENT_KEY].rising() && !clear_mod && !s_metronome_active;
    const bool sld_edge = inputs[SLIDE_KEY].rising()  && !clear_mod && !s_metronome_active;
    if (acc_edge || sld_edge) {
      const bool link = (acc_edge && inputs[SLIDE_KEY].held()) ||
                        (sld_edge && inputs[ACCENT_KEY].held());
      if (chain_live) {
        if (link) {
          // Builder-restore bookkeeping: a pattern-key tap while A+B stays
          // held becomes a chain build and reverts this mode flip.
          s_ab_link_pat  = uint8_t(s_chain_pats[0] & 0x0F);
          s_ab_prev_link = s_ab_mode;
          s_ab_mode      = true;
          s_section_view = 0xFF;
        } else if (s_ab_mode) {
          s_ab_mode      = false;
          s_section_view = 0xFF;
        } else {
          s_section_view = acc_edge ? 0 : 1;
        }
        emit_chain_state();
      } else {
        if (link) {
          // Remember the pre-press mode: a pattern-key tap while A+B stays
          // held turns this gesture into a chain build and restores this.
          s_ab_link_pat  = engine.get_patsel();
          s_ab_prev_link = s_ab_mode;
        }
        s_chain_active     = false; s_chain_len       = 0;
        s_chain_queue_len  = 0;     s_chain_anchor_key = 0xff;
        s_chain_hold_key   = 0xff;  s_chain_hold_target_pat = 0xff;
        chain_intent_reset();
        s_ab_mode = link;
        // A/B-mode entry always lands on A: notes fill the A section first.
        const uint8_t want = (link || acc_edge) ? Engine::section_a_of(engine.get_patsel())
                                                : Engine::section_b_of(engine.get_patsel());
        engine.SetPattern(want, !clk_run);
        if (!clk_run) midi_send_active_pattern(engine.get_patsel());
        emit_chain_state();
      }
    }
    break;
  }
  }

  const bool pat_clr_flash = (pattern_cleared_flash_timer < PATTERN_CLEARED_FLASH_MS)
                          || s_pat_cleared_hold;
  const bool in_time  = engine.get_mode() == TIME_MODE;
  const bool in_pitch = engine.get_mode() == PITCH_MODE;
  // A/B step-edit indicator: while a linked pair (or chain-wide A/B) plays,
  // the active submode LED shows which section the step edits land on --
  // solid = A, blinking = B (PITCH held + CLEAR cycles it).
  bool ab_led_on = true;
  if (clk_run && (in_pitch || in_time) && s_ab_mode &&
      Engine::is_section_b(engine.get_edit_patsel()))
    ab_led_on = clk_count < 12;
  // Tap-write session indicator: TIME LED blinks for as long as a session is
  // active, RECORD or OVERDUB. An all-OVERDUB pass is silent, so without
  // this the user cannot tell a session is still running.
  Leds::Set(TIME_MODE_LED,     (in_time && ab_led_on) || pat_clr_flash ||
                               (s_metronome_active && (clk_count & 4)));
  Leds::Set(PITCH_MODE_LED,    (in_pitch && ab_led_on) || pat_clr_flash);
  // FUNCTION_MODE_LED = "normal mode" indicator. Lit whenever the engine is not
  // in TIME/PITCH submode. Driven off NOT-in-submode so that mode-LED edge
  // cases (e.g. brief mode flicker) never leave the indicator dark.
  Leds::Set(FUNCTION_MODE_LED, !in_time && !in_pitch && !pat_clr_flash);
  if (pat_clr_flash) Leds::Set(ASHARP_KEY_LED, true);
}

// Tap-write monitor (ROM-exact): starts at the PRESS with the pitch the
// aimed-at NOTE will consume from the stream, sounds until physical release.
// Scale and transpose applied like playback; MIDI mirrors it via the
// audition channel.
static void metro_start_monitor(const Sequence &s, uint8_t step) {
  const uint8_t k  = s.pitch_index_for_note(step);
  const uint8_t pb = (k < s.get_pitch_count()) ? s.pitch[k] : PITCH_DEFAULT;
  const uint8_t lin = s.scale_quantize_linear(unpack_pitch_linear(pb));
  s_metro_tap_monitor_cv     = clamp_cv(int(lin) + total_transpose);
  s_metro_tap_monitor_accent = (pb & 0x40) != 0;
  s_metro_tail_cv            = s_metro_tap_monitor_cv;
  s_metro_monitor_gate       = true;
  uint16_t mn = uint16_t(36 + lin) + total_transpose;
  if (mn > 127) mn = 127;
  midi_audition_note_on(uint8_t(mn), s_metro_tap_monitor_accent ? 127 : 80);
}

// The ROM's SUSTAIN modifier: any SELECTOR (pattern key 1-8) held during the
// session turns would-be RESTs into TIEs once a note exists.
static bool metro_sustain_held() {
  for (uint8_t i = 0; i < 8; ++i)
    if (inputs[i].held()) return true;
  return false;
}

// PITCH modifier: live transpose root / octave for performance
void ProcessPitchMod() {
  // Solid FUNCTION_MODE_LED so the indicator is visible while stopped (clk_count blink mask
  // would otherwise sit at 0). On release the next-frame ProcessDefault redraws normal LEDs.
  Leds::Set(FUNCTION_MODE_LED, true);
  show_pitch_leds(pack_pitch(transpose % 12, transpose / 12));

  for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
    if (inputs[pitched_keys[i]].rising())
      transpose = (transpose / 12) * 12 + i;
  }
  if (inputs[DOWN_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 - 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
  if (inputs[UP_KEY].rising()) {
    uint8_t oct = constrain(int(transpose) / 12 + 1, 0, 3);
    transpose = (transpose % 12) + oct * 12;
  }
}

// =============================================================================
// Track Write / Track Play UI: chain write/review, per-step transpose, bank
// toggle. See the workflow comments inline.
// =============================================================================
static void ProcessTrackUI(DialMode dial, bool dial_track_write, bool clk_run,
                           uint8_t track_idx, bool fn_mod, bool clear_mod,
                           bool pitch_mod) {
  const uint8_t patsel    = engine.get_patsel();
  const uint8_t pat_bank  = (patsel >> 3) & 1;                     // 0=A, 1=B (within active group)
  // When PITCH is held in TrackWrite the user is reading/writing the
  // per-step transpose; suppress the pattern + bank LEDs so only the
  // transpose key and octave LEDs are visible.
  const bool tw_transpose_view = dial_track_write && pitch_mod;
  if (!tw_transpose_view) {
    Leds::Set(ACCENT_KEY_LED, pat_bank == 0);
    Leds::Set(SLIDE_KEY_LED,  pat_bank == 1);
    // Show the active pattern's LED both stopped and running (blink chase while running).
    Leds::Set(OutputIndex(patsel & 0x07), clk_run ? bool(clk_count < 12) : true);
  }
  // FN-mode LED stays solid in track mode so it's clearly the "normal" state.
  Leds::Set(FUNCTION_MODE_LED, true);

  // Bank toggle via ACCENT / SLIDE: switch which 8-pattern half pat-keys
  // address. Works in both TrackWrite and TrackPlay so the user can audition
  // patterns across A/B before writing. !edit_mode is intentionally NOT in
  // the gate -- edit_mode == TAP_NEXT.held, and the user may hold TAP_NEXT
  // while reaching for ACCENT/SLIDE in TrackWrite.
  if (!fn_mod && !clear_mod) {
    if (inputs[ACCENT_KEY].rising()) {
      engine.SetPattern((patsel & 0x07), true);
    } else if (inputs[SLIDE_KEY].rising()) {
      engine.SetPattern((patsel & 0x07) | 0x08, true);
    }
  }

  if (dial_track_write) {
    // Track Write workflow ("bar reset" = CLEAR):
    //   - Pat-key 0..7 rising  : switch the currently-playing pattern (same as
    //                            Pattern Play). Does NOT write to the chain.
    //   - TAP_NEXT rising      : write the currently-active pattern into
    //                            chain[cursor], advance cursor.
    //   - CLEAR rising         : "bar reset" -- arms the next TAP_NEXT to
    //                            write the LAST chain step. CLEAR does NOT
    //                            wipe the chain; writes always overwrite.
    //   - TAP_NEXT rising while armed : write at cursor, mark last, reset
    //                            cursor to 0, disarm.
    //   - RUN.falling (stop)   : saves the track to EEPROM.
    // NOTE: edit_mode == inputs[TAP_NEXT].held() globally; do NOT gate handlers
    // on !edit_mode here -- it would block the rising-edge frame for TAP_NEXT.
    if (!fn_mod && inputs[CLEAR_KEY].rising()) {
      if (clk_run) {
        // Running: CLEAR arms the next TAP_NEXT as the last chain step.
        s_track_arm_last = true;
      } else {
        // Stopped: CLEAR jumps the cursor back to step 0 and auditions
        // chain[0]'s pattern so the user can review / edit the track from
        // the beginning. Mirrors TrackPlay CLEAR semantics.
        engine.TrackResetCursor();
        if (engine.track_has_chain()) {
          engine.SetPattern(engine.TrackGetPattern(0), true);
        }
      }
      emit_track_state(dial, clk_run, track_idx);
    }
    if (!fn_mod && !clear_mod && pitch_mod) {
      // PITCH held in Track Write: pitch-keys / UP / DOWN edit the per-step
      // transpose at the current cursor. Encoding mirrors the global
      // performance transpose (semi + 12*oct, 0..47, 12 = no transpose).
      const uint8_t pos = engine.get_chain_pos();
      uint8_t cur = engine.TrackGetTranspose(pos);
      bool changed = false;
      for (uint8_t i = 0; i < ARRAY_SIZE(pitched_keys); ++i) {
        if (inputs[pitched_keys[i]].rising()) {
          cur = uint8_t((cur / 12) * 12 + i);
          changed = true;
          break;
        }
      }
      if (inputs[DOWN_KEY].rising()) {
        uint8_t oct = constrain(int(cur) / 12 - 1, 0, 3);
        cur = uint8_t((cur % 12) + oct * 12);
        changed = true;
      }
      if (inputs[UP_KEY].rising()) {
        uint8_t oct = constrain(int(cur) / 12 + 1, 0, 3);
        cur = uint8_t((cur % 12) + oct * 12);
        changed = true;
      }
      if (changed) {
        engine.TrackSetTranspose(pos, cur);
        emit_track_state(dial, clk_run, track_idx);
      }
      // LED feedback: show the step's transpose on the pitch-key + octave LEDs.
      show_pitch_leds(pack_pitch(cur % 12, cur / 12));
    } else if (!fn_mod && !clear_mod) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          const uint8_t pat_in_bank = uint8_t((pat_bank << 3) | i);
          engine.SetPattern(pat_in_bank, !clk_run);
          emit_track_state(dial, clk_run, track_idx);
          break;
        }
      }
      if (clk_run && inputs[TAP_NEXT].rising()) {
        engine.TrackWriteCurrentStep(engine.get_patsel() & 0x0F, /*repeats=*/0);
        if (s_track_arm_last) {
          engine.TrackMarkLastStep();
          engine.TrackResetCursor();
          s_track_arm_last = false;
        } else {
          engine.TrackAdvanceCursor();
        }
        emit_track_state(dial, clk_run, track_idx);
      }
      // Stopped + chain has steps: TAP_NEXT cycles the cursor through the
      // chain so the user can review what was written (pattern + transpose).
      // BACK_KEY steps backward. Non-destructive -- writes only happen while
      // the clock is running.
      if (!clk_run && engine.track_has_chain()) {
        const uint8_t clen = engine.get_chain_len();
        uint8_t pos = engine.get_chain_pos();
        bool moved = false;
        if (inputs[TAP_NEXT].rising()) {
          pos = uint8_t((pos + 1) % clen);
          moved = true;
        } else if (inputs[BACK_KEY].rising()) {
          pos = uint8_t((pos + clen - 1) % clen);
          moved = true;
        }
        if (moved) {
          engine.p_chain_pos = pos;
          engine.SetPattern(engine.TrackGetPattern(pos), true);
          emit_track_state(dial, clk_run, track_idx);
        }
      }
    }
  } else {
    // Track Play: CLEAR rising while stopped resets playhead to chain[0].
    // While running, CLEAR is ignored -- track plays until user stops it.
    if (!fn_mod && !clk_run && inputs[CLEAR_KEY].rising()) {
      engine.TrackResetCursor();
      if (engine.track_has_chain()) {
        engine.SetPattern(engine.TrackGetPattern(0), true);
      }
      emit_track_state(dial, clk_run, track_idx);
    }
  }
}

// =============================================================================
// Step-select mode (FN + PITCH toggle): two sub-modes, pitch (default) and
// time. PITCH_KEY switches to pitch sub-mode, TIME_KEY to time sub-mode.
// Pitch sub-mode: only NOTE steps selectable, TAP_NEXT enters detail editor.
// Time sub-mode: ALL steps selectable, no enter needed; just select and edit.
// =============================================================================
static void ProcessStepSelect(bool clk_run, bool dial_pattern_write) {
  Leds::Set(FUNCTION_MODE_LED, true);
  // Resolve the pattern being viewed. With an active chain of >= 2 patterns
  // we view chain[s_step_sel_chain_view]; otherwise we view the engine's
  // active pattern. View is independent of playback: the engine keeps
  // playing its own pattern regardless of what we view here.
  const bool chain_view_active = s_chain_active && s_chain_len >= 2;
  if (chain_view_active && s_step_sel_chain_view >= s_chain_len)
    s_step_sel_chain_view = 0;
  uint8_t view_pat_idx = chain_view_active
      ? s_chain_pats[s_step_sel_chain_view]
      : engine.get_patsel();
  // A/B view: with a linked pair (or chain-wide A/B) the view NEVER follows
  // playback's A/B alternation. It shows section A by default; the mode-key +
  // CLEAR combo (PITCH or TIME held + CLEAR, either order) pins the other
  // section via engine.ab_edit_pat_.
  if (engine.ab_edit_pat_ != 0xFF)
    view_pat_idx = uint8_t((view_pat_idx & 7) |
                           (Engine::is_section_b(engine.ab_edit_pat_) ? 8 : 0));
  else if (s_ab_mode)
    view_pat_idx = Engine::section_a_of(view_pat_idx);
  // Sub-mode LEDs double as the A/B view indicator in A/B mode: solid =
  // viewing section A, blinking = section B. clk_count free-runs at 120 BPM,
  // so the blink works while stopped too.
  const bool ab_view_b = s_ab_mode && Engine::is_section_b(view_pat_idx);
  const bool ab_led_on = !ab_view_b || clk_count < 12;
  Leds::Set(PITCH_MODE_LED, !s_step_sel_time && ab_led_on);
  Leds::Set(TIME_MODE_LED,   s_step_sel_time && ab_led_on);
  // Edit the current variation when viewing the active pattern (var2/3 live
  // in the shadow buffers); chained non-active patterns edit variation 1.
  Sequence &seq = (view_pat_idx == engine.get_patsel())
                      ? engine.get_edit_sequence()
                      : engine.pattern[view_pat_idx];
  const bool playing_matches_view = (engine.get_patsel() == view_pat_idx);
  // Variation 3 poly: this slot's var3 plays from the chord-list voice, not the
  // (silent) mono shadow `seq`. Read the picker's steps/length and edit chords
  // from poly_ so the display shows the real steps instead of stale shadow data.
  const bool poly_sel = playing_matches_view && engine.get_edit_var() == 2 && engine.poly_active_;
  PolyVoice &pv = engine.poly_;
  const uint8_t blen = poly_sel ? uint8_t(pv.length ? pv.length : 1) : seq.length;
  #define SEL_TIME(i) (poly_sel ? pv.time(uint8_t(i)) : seq.time(uint8_t(i)))
  // An A/B pin flip can land on a shorter section; drop a selection that no
  // longer exists so the editors cannot write past the viewed length.
  if (s_step_sel >= int(blen)) s_step_sel = -1;

  // Sub-mode switching (outside detail editor to avoid accidental mode changes).
  // !CLEAR held: a mode key rising with CLEAR down is the A/B pin gesture
  // (either press order), not a sub-mode switch.
  if (!s_step_sel_edit && !inputs[CLEAR_KEY].held()) {
    if (inputs[TIME_KEY].rising() && !s_step_sel_time) {
      s_step_sel_time = true;
      s_step_sel = -1; // clear selection on mode switch
    }
    if (inputs[PITCH_KEY].rising() && s_step_sel_time) {
      s_step_sel_time = false;
      s_step_sel = -1;
    }
  }

  // ── Picker (shared between pitch and time sub-modes) ──
  if (!s_step_sel_edit) {
    // A# toggles the extended half (steps 32..63); keep the same black-key
    // offset when flipping so the picker stays on the same column. Above 32
    // steps only: at MAX_STEPS = 32 the four banks cover the whole section.
    if (MAX_STEPS > 32 && inputs[ASHARP_KEY].rising()) {
      s_step_sel_ext = !s_step_sel_ext;
      s_step_sel_base = uint8_t((s_step_sel_base & 31) + (s_step_sel_ext ? 32 : 0));
    }
    const uint8_t ext = s_step_sel_ext ? 32 : 0;
    // Bank pick
    if (inputs[CSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 0);
    if (inputs[DSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 8);
    if (inputs[FSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 16);
    if (inputs[GSHARP_KEY].rising()) s_step_sel_base = uint8_t(ext + 24);
    // Step pick
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        const uint8_t cand = uint8_t(s_step_sel_base + wi);
        if (cand < blen) {
          // Pitch sub-mode: only NOTE steps. Time sub-mode: any step.
          if (s_step_sel_time || SEL_TIME(cand) == 1)
            s_step_sel = int(cand);
        }
      }
    }
    // Step LEDs: NOTE=bright, TIE=dim, REST=off
    for (uint8_t wi = 0; wi < 8; ++wi) {
      const uint8_t idx = uint8_t(s_step_sel_base + wi);
      if (idx >= blen) break;
      const uint8_t tn = SEL_TIME(idx);
      if (tn == 1) Leds::Set(OutputIndex(wi), true);
      else if (tn == 2) Leds::SetDim(OutputIndex(wi), true);
    }
    // Chase LED: only when the currently-playing pattern is the one
    // being viewed (so chain slots not currently playing stay quiet).
    // Read the playhead from the edited variation (seq), not variation 1,
    // so the chase follows the right length when editing variation 2/3.
    if (clk_run && playing_matches_view) {
      const uint8_t tp = poly_sel ? uint8_t(engine.poly_time_pos_ & (MAX_STEPS - 1))
                                   : uint8_t(seq.time_pos & (MAX_STEPS - 1));
      if ((tp & ~uint8_t(7)) == s_step_sel_base)
        Leds::Set(OutputIndex(tp & 0x7), bool(clk_count & 4));
    }
    // Cover-bank LEDs (within the visible half; A# shows extended state).
    const bool blinkb = bool((millis() >> 8) & 1);
    const uint8_t base_off = uint8_t(s_step_sel_base & 31);
    const OutputIndex sel_base_led =
        (base_off == 0)  ? CSHARP_KEY_LED :
        (base_off == 8)  ? DSHARP_KEY_LED :
        (base_off == 16) ? FSHARP_KEY_LED : GSHARP_KEY_LED;
    Leds::Set(CSHARP_KEY_LED, (blen > ext + 0)  && (sel_base_led == CSHARP_KEY_LED ? blinkb : true));
    Leds::Set(DSHARP_KEY_LED, (blen > ext + 8)  && (sel_base_led == DSHARP_KEY_LED ? blinkb : true));
    Leds::Set(FSHARP_KEY_LED, (blen > ext + 16) && (sel_base_led == FSHARP_KEY_LED ? blinkb : true));
    Leds::Set(GSHARP_KEY_LED, (blen > ext + 24) && (sel_base_led == GSHARP_KEY_LED ? blinkb : true));
    // A#: solid in the extended half, blink when extended steps exist.
    Leds::Set(ASHARP_KEY_LED, s_step_sel_ext ? true : (blen > 32 ? blinkb : false));
    // Selection flash
    if (s_step_sel >= 0 && (uint8_t(s_step_sel) & ~uint8_t(7)) == s_step_sel_base)
      Leds::Set(OutputIndex(s_step_sel & 0x7), bool((millis() >> 7) & 1));

    // Chain slot select + LEDs. Available in the pitch sub-mode always (its
    // picker leaves DOWN/UP/ACCENT/SLIDE free); in the time sub-mode only
    // while no step is selected (a selection hands those keys to the time
    // editor). Hidden entirely when the chain has < 2 patterns.
    if (chain_view_active && (!s_step_sel_time || s_step_sel < 0)) {
      bool switched = false;
      if (inputs[DOWN_KEY].rising()   && s_chain_len > 0) { s_step_sel_chain_view = 0; switched = true; }
      if (inputs[UP_KEY].rising()     && s_chain_len > 1) { s_step_sel_chain_view = 1; switched = true; }
      if (inputs[ACCENT_KEY].rising() && s_chain_len > 2) { s_step_sel_chain_view = 2; switched = true; }
      if (inputs[SLIDE_KEY].rising()  && s_chain_len > 3) { s_step_sel_chain_view = 3; switched = true; }
      if (switched) {
        // Selection belongs to the previous pattern; drop it with the view.
        s_step_sel = -1;
        s_step_sel_base = 0;
        s_step_sel_ext  = false;
      }
      // LEDs: viewed slot = solid full brightness; playing slot = chase blink
      // at full brightness; remaining chain slots = dim.
      static const OutputIndex kChainLeds[4] =
          {DOWN_KEY_LED, UP_KEY_LED, ACCENT_KEY_LED, SLIDE_KEY_LED};
      const bool blinkc = bool(clk_count & 4);
      for (uint8_t ci = 0; ci < 4 && ci < s_chain_len; ++ci) {
        const bool sel  = (s_step_sel_chain_view == ci);
        const bool play = clk_run && (s_chain_pos == ci);
        if (sel || (play && blinkc)) Leds::Set(kChainLeds[ci], true);
        else                         Leds::SetDim(kChainLeds[ci], true);
      }
    }

    if (s_step_sel_time) {
      // ── Time sub-mode: edit directly from picker, no enter needed ──
      // Edits gated to Pattern Write; in Pattern Play the picker stays
      // visible as a read-only viewer (LED feedback shows current state).
      if (s_step_sel >= 0) {
        const uint8_t si = uint8_t(s_step_sel);
        if (dial_pattern_write && poly_sel) {
          // Poly var3: edit the chord-list voice's time stream (shifts chords).
          bool tchanged = false;
          if (inputs[DOWN_KEY].rising()) { pv.set_time(si, 1); tchanged = true; }
          if (inputs[UP_KEY].rising()) {
            const uint8_t prev_t = (si > 0) ? pv.time(uint8_t(si - 1)) : 0;
            if (prev_t != 0) { pv.set_time(si, 2); tchanged = true; }
          }
          if (inputs[ACCENT_KEY].rising()) { pv.set_time(si, 0); tchanged = true; }
          if (tchanged) {
            pv.ensure_chords_for_notes();
            engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
            midi_send_poly_step(engine.get_patsel(), si); // mirror to the web
          }
        } else if (dial_pattern_write) {
          bool tchanged = false;
          uint8_t before_pt[MAX_STEPS];
          sequence_pack_per_time(seq, before_pt);
          if (inputs[DOWN_KEY].rising()) {
            sequence_write_time_with_pitch_sync(seq, si, 1);
            engine.stale = true; tchanged = true;
          }
          if (inputs[UP_KEY].rising()) {
            const uint8_t prev_t = (si > 0) ? seq.time(uint8_t(si - 1)) : 0;
            if (prev_t != 0) {
              sequence_write_time_with_pitch_sync(seq, si, 2);
              engine.stale = true; tchanged = true;
            }
          }
          if (inputs[ACCENT_KEY].rising()) {
            sequence_write_time_with_pitch_sync(seq, si, 0);
            engine.stale = true; tchanged = true;
          }
          // SLIDE_KEY step-lock toggle removed (RAM-only feature).
          if (tchanged) {
            uint8_t after_pt[MAX_STEPS];
            sequence_pack_per_time(seq, after_pt);
            const uint8_t plen = seq.length;
            const uint8_t pat = view_pat_idx;
            for (uint8_t i = 0; i < plen; ++i) {
              if (i == si || before_pt[i] != after_pt[i])
                midi_send_step_update(pat, i, after_pt[i], seq.time(i));
            }
          }
        }
        // Show time info for the selected step (always, even in Play)
        const uint8_t st = SEL_TIME(si);
        Leds::Set(DOWN_KEY_LED,   st == 1);
        Leds::Set(UP_KEY_LED,     st == 2);
        Leds::Set(ACCENT_KEY_LED, st == 0);
        Leds::Set(SLIDE_KEY_LED,  false);
      }
      if (inputs[BACK_KEY].rising()) s_step_sel = -1;
    } else {
      // ── Pitch sub-mode picker ──
      // Enter detail editor on TAP_NEXT rising with a valid selection.
      if (s_step_sel >= 0 && inputs[TAP_NEXT].rising()) {
        s_step_sel_edit = true;
        if (!clk_run) {
          const uint8_t slot = seq.pitch_index_for_note(uint8_t(s_step_sel));
          const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
          if (pb != PITCH_EMPTY) {
            const uint8_t linear = unpack_pitch_linear(pb & 0x3f);
            const bool acc = (pb & (1 << 6)) != 0;
            if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
            s_tap_pitch_preview_cv = clamp_cv(int(linear) + total_transpose);
            s_tap_pitch_preview_accent = acc;
            s_tap_pitch_preview_slide = false; // previews never slide
            s_tap_pitch_preview_gate = true;
          }
        }
      }
      // BACK clears selection.
      else if (inputs[BACK_KEY].rising()) s_step_sel = -1;
    }
  } else {
    // ── Pitch detail editor (only reachable from pitch sub-mode) ──
    // Edits gated to Pattern Write. In Pattern Play the editor still
    // shows LED feedback and audition but no pitch/flag writes occur.
    if (s_step_sel < 0) {
      s_step_sel_edit = false;
    } else if (poly_sel) {
      // ── Poly chord detail editor: edit the chord this NOTE step pulls ──
      const uint8_t ci = pv.chord_index_for_step(uint8_t(s_step_sel));
      // Latched octave (tap UP/DOWN; tap UP twice for double-up) -- no holding.
      if (inputs[UP_KEY].rising()   && s_poly_view_oct < 3) ++s_poly_view_oct;
      if (inputs[DOWN_KEY].rising() && s_poly_view_oct > 0) --s_poly_view_oct;
      const uint8_t view_oct = s_poly_view_oct;
      bool pchanged = false;
      if (dial_pattern_write) {
        for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
          if (!inputs[pitched_keys[pi]].rising()) continue;
          const uint8_t packed = pack_pitch(pi, view_oct);
          if (pv.has_note(ci, packed)) pv.remove_note(ci, packed);
          else                         pv.add_note(ci, packed);
          pchanged = true;
        }
        if (inputs[ACCENT_KEY].rising()) { pv.toggle_accent(ci); pchanged = true; }
        if (inputs[SLIDE_KEY].rising())  { pv.toggle_slide(ci);  pchanged = true; }
      }
      if (pchanged) {
        engine.poly_stale_ = true; engine.shadow_dirty_ms_ = millis();
        midi_send_poly_step(engine.get_patsel(), uint8_t(s_step_sel)); // mirror to the web
      }
      if (inputs[BACK_KEY].rising()) s_step_sel_edit = false;
      // LEDs: show the chord at the current view octave (like ProcessPolyEdit).
      const uint8_t *v = pv.chord(ci);
      for (uint8_t i = 0; i < POLY_VOICES; ++i)
        if (v[i] != POLY_EMPTY && ((v[i] >> 4) & 0x03) == view_oct)
          Leds::Set(pitch_leds[v[i] & 0x0F], true);
      Leds::Set(DOWN_KEY_LED, view_oct == 0 || view_oct == 3);
      Leds::Set(UP_KEY_LED,   view_oct == 2 || view_oct == 3);
      Leds::Set(ACCENT_KEY_LED, pv.accent(ci));
      Leds::Set(SLIDE_KEY_LED,  pv.slide(ci));
    } else {
      // Pin pitch_pos to the SELECTED step's pitch slot for the duration
      // of the edit so SetPitchSemitone / NudgeOctave / Toggle ops target
      // the user's pick, not the currently-playing playhead.
      const int     saved_pp = seq.pitch_pos;
      const uint8_t slot = seq.pitch_index_for_note(uint8_t(s_step_sel));
      seq.pitch_pos = int(slot);
      bool changed = false;
      bool audition = false;
      if (dial_pattern_write) {
        if (inputs[ACCENT_KEY].rising()) {
          if (slot < seq.get_pitch_count()) {
            if (seq.pitch[slot] == PITCH_EMPTY) seq.pitch[slot] = PITCH_DEFAULT;
            seq.pitch[slot] ^= (1 << 6);
            engine.stale = true; changed = true;
          }
          audition = true;
        }
        if (inputs[SLIDE_KEY].rising()) {
          if (slot < seq.get_pitch_count()) {
            if (seq.pitch[slot] == PITCH_EMPTY) seq.pitch[slot] = PITCH_DEFAULT;
            seq.pitch[slot] ^= (1 << 7);
            engine.stale = true; changed = true;
          }
        }
        if (inputs[UP_KEY].rising())     { seq.nudge_octave_buttons(+1); engine.stale = true; changed = true; audition = true; }
        if (inputs[DOWN_KEY].rising())   { seq.nudge_octave_buttons(-1); engine.stale = true; changed = true; audition = true; }
        for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
          if (inputs[pitched_keys[pi]].rising()) {
            seq.SetPitchSemitone(pi);
            engine.stale = true; changed = true; audition = true;
            break;
          }
        }
      }
      seq.pitch_pos = saved_pp;
      if (changed) {
        const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
        midi_send_step_update(view_pat_idx, uint8_t(s_step_sel),
            pb, seq.time(uint8_t(s_step_sel)));
      }
      if (inputs[BACK_KEY].rising()) s_step_sel_edit = false;
      if (!clk_run && (inputs[TAP_NEXT].rising() || audition)) {
        const uint8_t ab = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
        if (ab != PITCH_EMPTY) {
          const uint8_t lin = unpack_pitch_linear(ab & 0x3f);
          const bool acc = (ab & (1 << 6)) != 0;
          if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
          s_tap_pitch_preview_cv = clamp_cv(int(lin) + total_transpose);
          s_tap_pitch_preview_accent = acc;
          s_tap_pitch_preview_slide = false; // previews never slide
          s_tap_pitch_preview_gate = true;
        }
      }

      const uint8_t pb = (slot < seq.get_pitch_count()) ? seq.pitch[slot] : PITCH_EMPTY;
      if (pb != PITCH_EMPTY) {
        show_pitch_leds(pb & 0x3f);
        Leds::Set(ACCENT_KEY_LED, (pb & (1 << 6)) != 0);
        Leds::Set(SLIDE_KEY_LED,  (pb & (1 << 7)) != 0);
      }
    }
  }
  // Close step-select audition gate when all relevant keys are released.
  if (!clk_run && s_tap_pitch_preview_gate &&
      !check_pitch_inputs() &&
      !inputs[UP_KEY].held() && !inputs[DOWN_KEY].held() &&
      !inputs[ACCENT_KEY].held() && !inputs[TAP_NEXT].held()) {
    s_tap_pitch_preview_gate = false;
  }
  #undef SEL_TIME
}

// =============================================================================
// Keyboard play: pitched keys play notes via the audition CV path; no pattern
// writes. Octave: latched (TIME toggles, TIME_MODE_LED lit) = tap DOWN/UP to
// step register 0..3 and it holds; unlatched = hold DOWN/UP while playing.
// Press-order stack drives legato slide: holding one key and pressing another
// keeps the gate high and forces slide CV high so the 303 portamentos between
// notes. Releasing the top note slides back to the next-most-recent held note.
// Releasing all notes drops the gate. MIDI out is per-key: every held key
// keeps its own Note On open until released, so overlaps come out as legato
// (mono synths slide) and a DAW records true note lengths.
// =============================================================================

// The button matrix has no diodes: DOWN/UP/ACCENT/SLIDE sit on scan row 2 at
// columns 0..3, sharing columns with the note rows. When two of them are held
// and a note key in a matching column is pressed, the scan reads back a
// phantom note in the same row at the other matching column (three corners of
// a rectangle conduct the fourth). The phantom rises in the same frame as the
// real key and is electrically indistinguishable from it.
static bool kb_mod_col_held(uint8_t col) {
  switch (col) {
    case 0: return inputs[DOWN_KEY].held();
    case 1: return inputs[UP_KEY].held();
    case 2: return inputs[ACCENT_KEY].held();
    default: return inputs[SLIDE_KEY].held();
  }
}
// True when a rising DOWN/UP read at matrix column `col` is a likely phantom:
// a note row has a pressed key at `col` plus a pressed key at another column
// whose row-2 modifier is held.
static bool kb_oct_tap_ghosted(uint8_t col) {
  for (uint8_t r = 0; r < 4; ++r) {
    if (r == 2) continue; // the modifier row itself
    if (!inputs[InputIndex(r * 4 + col)].held()) continue;
    for (uint8_t c = 0; c < 4; ++c)
      if (c != col && kb_mod_col_held(c) && inputs[InputIndex(r * 4 + c)].held())
        return true;
  }
  return false;
}

static void ProcessKeyboardPlay() {
  Leds::Set(PITCH_MODE_LED, true);
  Leds::Set(FUNCTION_MODE_LED, true);
  Leds::Set(TIME_MODE_LED, s_kb_oct_latch);

  // TIME toggles the octave latch.
  if (inputs[TIME_KEY].rising()) s_kb_oct_latch = !s_kb_oct_latch;

  if (s_kb_oct_latch) {
    // Tap DOWN/UP to step the latched octave; ghost-guarded so a phantom
    // octave read (accent/slide held plus overlapping notes) cannot move it.
    if (inputs[UP_KEY].rising() && !kb_oct_tap_ghosted(1) && s_kb_oct < 3) ++s_kb_oct;
    if (inputs[DOWN_KEY].rising() && !kb_oct_tap_ghosted(0) && s_kb_oct > 0) --s_kb_oct;
    Leds::Set(DOWN_KEY_LED, s_kb_oct == 0 || s_kb_oct == 3);
    Leds::Set(UP_KEY_LED,   s_kb_oct == 2 || s_kb_oct == 3);
  }

  // Falling edges: remove released keys from stack, close their MIDI notes;
  // slide back if the top changed and other keys remain.
  bool top_changed = false;
  for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
    if (!inputs[pitched_keys[pi]].falling()) continue;
    const bool was_top = (s_kb_stack_depth > 0 &&
                          s_kb_stack_key[s_kb_stack_depth - 1] == pi);
    const uint8_t off_note = kb_stack_note_of(pi);
    kb_stack_remove(pi);
    if (off_note != 0xFF) midi_kb_note_off(off_note);
    if (was_top) top_changed = true;
  }
  if (top_changed && s_kb_stack_depth > 0) {
    // CV slides back to the next-most-recent held key. Its MIDI note is
    // still open (per-key notes), so nothing to send.
    s_tap_pitch_preview_cv    = s_kb_stack_cv[s_kb_stack_depth - 1];
    s_tap_pitch_preview_slide = true;
  }

  // Ghost guard: two rising note keys in the same matrix row whose columns
  // both match held modifiers are a real+phantom pair that cannot be told
  // apart; drop both rather than play a possibly-wrong note. (With the
  // octave latch there is no need to hold DOWN/UP, so this never triggers
  // in normal latched play.)
  bool ghosted[ARRAY_SIZE(pitched_keys)] = {false};
  for (uint8_t a = 0; a < ARRAY_SIZE(pitched_keys); ++a) {
    const uint8_t ia = pitched_keys[a];
    if (ia >= 16 || !inputs[ia].rising() || !kb_mod_col_held(ia & 3)) continue;
    for (uint8_t b = uint8_t(a + 1); b < ARRAY_SIZE(pitched_keys); ++b) {
      const uint8_t ib = pitched_keys[b];
      if (ib >= 16 || !inputs[ib].rising() || !kb_mod_col_held(ib & 3)) continue;
      if ((ia >> 2) == (ib >> 2)) { ghosted[a] = true; ghosted[b] = true; }
    }
  }

  // Rising edges: legato if stack already non-empty, else fresh trigger.
  for (uint8_t pi = 0; pi < ARRAY_SIZE(pitched_keys); ++pi) {
    if (!inputs[pitched_keys[pi]].rising() || ghosted[pi]) continue;
    const uint8_t oct    = s_kb_oct_latch ? s_kb_oct : resolve_octave();
    const uint8_t packed = pack_pitch(pi, oct);
    const uint8_t linear = unpack_pitch_linear(packed);
    const uint8_t cv     = clamp_cv(int(linear) + total_transpose);
    const bool    legato = (s_kb_stack_depth > 0);
    const bool    acc    = inputs[ACCENT_KEY].held();
    const bool    sld    = inputs[SLIDE_KEY].held() || legato;
    uint16_t mn = uint16_t(36 + linear) + total_transpose;
    if (mn > 127) mn = 127;
    const uint8_t vel = acc ? 127 : 80;
    // Fresh trigger: retrig gate so envelope opens cleanly. Legato: leave
    // gate continuously high so the 303 envelope does not retrigger.
    if (!legato && s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
    s_tap_pitch_preview_cv     = cv;
    s_tap_pitch_preview_accent = acc;
    s_tap_pitch_preview_slide  = sld;
    s_tap_pitch_preview_gate   = true;
    if (kb_stack_push(pi, cv, uint8_t(mn)))
      midi_kb_note_on(uint8_t(mn), vel);
    break; // only one new note per loop iteration
  }

  // Light exactly the keys in the stack (raw held reads include phantoms and
  // are suppressed from the global pressed-button echo in keyboard mode).
  for (uint8_t i = 0; i < s_kb_stack_depth; ++i)
    Leds::Set(pitch_leds[s_kb_stack_key[i]], true);

  // Close gate when stack drains (per-key MIDI offs already went out).
  if (s_kb_stack_depth == 0 && s_tap_pitch_preview_gate) {
    s_tap_pitch_preview_gate  = false;
    s_tap_pitch_preview_slide = false;
  }
}

// =============================================================================
// FN held: pattern length editor + length LED display (Pattern Write only).
// =============================================================================
// =============================================================================
// A/B pair length helpers (spec 5-b..5-e). A linked pair is one pattern whose
// steps run 1..64 in the A section and 65..128 in the B section, so these walk
// the two Sequences as a single stream.
// =============================================================================
static uint8_t pair_time_at(const Sequence &a, const Sequence &b, uint8_t i) {
  return (i < MAX_STEPS) ? a.time(i) : b.time(uint8_t(i - MAX_STEPS));
}

// Pitch of the NOTE event at pair step `i`, or PITCH_EMPTY if that step is not
// a NOTE. Each section owns its own NOTE-indexed pitch stream.
static uint8_t pair_pitch_at(const Sequence &a, const Sequence &b, uint8_t i) {
  const Sequence &s = (i < MAX_STEPS) ? a : b;
  const uint8_t k = (i < MAX_STEPS) ? i : uint8_t(i - MAX_STEPS);
  if (s.time(k) != 1) return PITCH_EMPTY;
  const uint8_t slot = s.pitch_index_for_note(k);
  return (slot < MAX_STEPS) ? s.pitch[slot] : PITCH_EMPTY;
}

// Total last step across the pair: A alone, or A + B when linked.
static uint8_t pair_total_length(uint8_t patsel) {
  const Sequence &a = engine.pattern[Engine::section_a_of(patsel)];
  if (!s_ab_mode) return a.length;
  const uint16_t t = uint16_t(a.length) + engine.pattern[Engine::section_b_of(patsel)].length;
  return uint8_t(t > 2 * MAX_STEPS ? 2 * MAX_STEPS : t);
}

// Set the pair's last step to `total` (1..128). Past 64 the B section takes
// over and the pair is linked automatically (spec 5-d / 5-e); at or below 64
// the pattern is a plain A-section pattern again.
static void pair_set_total_length(uint8_t patsel, uint8_t total) {
  Sequence &a = engine.pattern[Engine::section_a_of(patsel)];
  Sequence &b = engine.pattern[Engine::section_b_of(patsel)];
  if (total < 1) total = 1;
  if (total > 2 * MAX_STEPS) total = uint8_t(2 * MAX_STEPS);
  if (total > MAX_STEPS) {
    // Growing past the section links the pair structurally (saved memory) and
    // enters the A/B mode so the B half is heard immediately.
    engine.set_pair_linked(patsel, true);
    s_ab_mode = true;
    engine.ApplyLength(a, uint8_t(MAX_STEPS));
    engine.ApplyLength(b, uint8_t(total - MAX_STEPS));
  } else {
    engine.set_pair_linked(patsel, false);
    engine.ApplyLength(a, total);
  }
}

// Spec 5-b/5-c/5-e: FUNCTION + UP doubles the pair's last step, up to 128. A
// tail that already holds NOTES (retained from an earlier halve) is simply
// revealed; an empty tail is filled with a copy of the first half, time and
// pitch alike, spilling into the B section and linking the pair when it must.
static void pair_double_length(uint8_t patsel) {
  Sequence &a = engine.pattern[Engine::section_a_of(patsel)];
  Sequence &b = engine.pattern[Engine::section_b_of(patsel)];
  const uint8_t sec_cap = a.is_triplet_mode() ? uint8_t(TRIPLET_MAX_STEPS)
                                              : uint8_t(MAX_STEPS);
  const uint8_t len = pair_total_length(patsel);
  uint16_t want = uint16_t(len) * 2;
  const uint16_t cap = uint16_t(sec_cap) * 2;
  if (want > cap) want = cap;
  const uint8_t nlen = uint8_t(want);
  if (nlen <= len) return;

  bool tail_empty = true;
  for (uint8_t i = len; i < nlen; ++i)
    if (pair_time_at(a, b, i) != 0) { tail_empty = false; break; }

  // Snapshot the source half before the sections are resized: the copy reads
  // from the same pair it writes into, and growing B renumbers nothing but
  // does make its own tail readable.
  if (tail_empty) {
    uint8_t src_t[MAX_STEPS];
    uint8_t src_p[MAX_STEPS];
    for (uint8_t j = 0; j < len && j < MAX_STEPS; ++j) {
      src_t[j] = pair_time_at(a, b, j);
      src_p[j] = pair_pitch_at(a, b, j);
    }
    pair_set_total_length(patsel, nlen);
    for (uint8_t i = len; i < nlen; ++i) {
      const uint8_t j = uint8_t(i - len);
      Sequence &d = (i < MAX_STEPS) ? a : b;
      const uint8_t di = (i < MAX_STEPS) ? i : uint8_t(i - MAX_STEPS);
      sequence_set_time_at(d, di, src_t[j]);
      if (src_t[j] == 1 && src_p[j] != PITCH_EMPTY) {
        const uint8_t slot = d.pitch_index_for_note(di);
        if (slot < MAX_STEPS) {
          d.pitch[slot] = src_p[j];
          if (slot >= d.get_pitch_count()) d.set_pitch_count(uint8_t(slot + 1));
        }
      }
    }
    sequence_rebuild_pitch_count(a);
    sequence_ensure_pitch_for_notes(a);
    sequence_rebuild_pitch_count(b);
    sequence_ensure_pitch_for_notes(b);
    engine.stale = true;
  } else {
    pair_set_total_length(patsel, nlen);
  }
}

static void ProcessLengthEditor(bool dial_pattern_write) {
  // Always-solid FUNCTION_MODE_LED so it stays visible when the clock is stopped
  // (clk_count is frozen and may sit at 0, hiding any blink mask).
  Leds::Set(FUNCTION_MODE_LED, true);

  // FN + ACCENT / FN + SLIDE: live force at the CV stage; only active while held.
  // No persistent stamp — handled below in the DAC output block via fn_mod.

  // Length display and edits are Pattern Write only; FN in Pattern Play shows
  // nothing (it is just the keyboard-mode / direction-mode modifier there).

  if (dial_pattern_write) {
    // FN hold + step press: pattern length editor (Pattern Write only)
    // Black keys set 8-step base (C#=8, D#=16, F#=24, G#=32; +32 in extended mode)
    // White keys add fine offset +1 to +8
    // A# toggles extended mode (bases += 32)

    // LED: show current length position
    // White key: remainder within current 8-step block
    // Black keys: cumulative block coverage (solid = covered, blink = extended block covered)
    // Spec 5-d: holding the B (SLIDE) button while FUNCTION moves the whole
    // editor into steps 65-128, i.e. the B section's own 1-64. Picking a last
    // step there links the pair automatically. A pattern that is already on
    // the B section alone cannot exceed step 64, so the modifier is ignored
    // there ("the LAST STEP cannot exceed the 64th step").
    const uint8_t lpat  = engine.get_patsel();
    const bool on_b     = Engine::is_section_b(lpat);
    // A B section selected on its own is an ordinary 1-64 pattern: only a
    // linked pair (or the A section, which can grow into one) edits pair-wide.
    const bool pair_len = (engine.get_edit_var() == 0) &&
                          !(on_b && !s_ab_mode);
    const bool sec_b    = pair_len && !on_b && inputs[SLIDE_KEY].held();
    const uint8_t cur_len = sec_b
        ? uint8_t(pair_total_length(lpat) > MAX_STEPS
                      ? pair_total_length(lpat) - MAX_STEPS
                      : 0)
        : engine.edit_seq_view().length;
    const bool blink_w = bool((millis() >> 8) & 1); // ~2 Hz, clock-independent
    if (cur_len) Leds::Set(OutputIndex((cur_len - 1) & 0x7), true);
    if (s_len_extended || cur_len > 32) {
      Leds::Set(ASHARP_KEY_LED, blink_w);
      Leds::Set(CSHARP_KEY_LED, cur_len > 32 ? blink_w : false);
      Leds::Set(DSHARP_KEY_LED, cur_len > 40 ? blink_w : false);
      Leds::Set(FSHARP_KEY_LED, cur_len > 48 ? blink_w : false);
      Leds::Set(GSHARP_KEY_LED, cur_len > 56 ? blink_w : false);
    } else {
      Leds::Set(ASHARP_KEY_LED, false);
      Leds::Set(CSHARP_KEY_LED, true);           // always: len >= 1
      Leds::Set(DSHARP_KEY_LED, cur_len > 8);
      Leds::Set(FSHARP_KEY_LED, cur_len > 16);
      Leds::Set(GSHARP_KEY_LED, cur_len > 24);
    }

    if (MAX_STEPS > 32 && inputs[ASHARP_KEY].rising()) s_len_extended = !s_len_extended;

    const uint8_t ext_add = s_len_extended ? 32 : 0;
    // Black key → select base range only; white key below applies the length.
    // C# = range 1-8 (base 0), D# = 9-16 (base 8), F# = 17-24 (base 16), G# = 25-32 (base 24).
    // In extended mode each base += 32: C#=33-40, D#=41-48, F#=49-56, G#=57-64.
    if (inputs[CSHARP_KEY].rising()) { s_len_black_base =  0 + ext_add; s_len_black_pressed = true; }
    if (inputs[DSHARP_KEY].rising()) { s_len_black_base =  8 + ext_add; s_len_black_pressed = true; }
    if (inputs[FSHARP_KEY].rising()) { s_len_black_base = 16 + ext_add; s_len_black_pressed = true; }
    if (inputs[GSHARP_KEY].rising()) { s_len_black_base = 24 + ext_add; s_len_black_pressed = true; }
    // White keys → fine offset from base (1–8); if no black pressed yet, set 1–8 directly
    for (uint8_t wi = 0; wi < 8; ++wi) {
      if (inputs[kCfgWhiteKeys[wi]].rising()) {
        const uint8_t base = s_len_black_pressed ? s_len_black_base : 0;
        const uint8_t sel  = uint8_t(base + wi + 1);
        if (sec_b) {
          // Step 64 + sel: crosses into the B section, so the pair links.
          pair_set_total_length(lpat, uint8_t(MAX_STEPS + sel));
          midi_send_pattern_update(lpat);
          midi_send_pattern_update(Engine::section_b_of(lpat));
        } else if (pair_len) {
          pair_set_total_length(lpat, sel);
          midi_send_length_update(lpat, engine.edit_seq_view().length, 0);
        } else {
          engine.SetLength(sel);
          midi_send_length_update(lpat, engine.edit_seq_view().length, engine.get_edit_var());
        }
      }
    }
  }
}

// =============================================================================
// Global CLEAR combos (CLEAR held, no FN; Pattern Write only). All destructive:
// rotate, randomize, mutate, shift, reverse, clear-only, copy/paste.
// =============================================================================
static void ProcessClearCombos(bool clear_mod, bool fn_mod, bool dial_pattern_write,
                               bool pitch_mod, bool time_mod, bool clk_run) {
  if (!clear_mod || fn_mod || !dial_pattern_write) return;
  // Matrix ghost guard (no diodes): in Pattern Write the mode dial closes
  // (PH0,PA0), so a held pattern key (PH0/PH1, PBc) plus CLEAR (PH2,PA0)
  // reads a phantom same-column key on the CLEAR row -- DOWN/UP/ACCENT/SLIDE.
  // The phantom fired the combo on the clear gesture itself (pat key + CLEAR
  // randomized/rotated/mutated the just-cleared pattern) and then masked the
  // real press's rising edge ("randomize does nothing after clear"). No CLEAR
  // combo on these four keys involves a held pattern key (copy/paste uses
  // C#/D#), so drop their edges while any pattern key is down.
  bool pat_key_down = false;
  for (uint8_t i = 0; i < 8; ++i)
    if (inputs[i].held()) { pat_key_down = true; break; }
  bool pat_changed = false;
  // CLEAR + ACCENT rising: randomize the whole pattern.
  if (inputs[ACCENT_KEY].rising() && !pat_key_down) {
    engine.RandomizeFullPattern();
    pat_changed = true;
  }
  // CLEAR + DOWN rising: rotate time data one step LEFT within length.
  // CLEAR + PITCH + DOWN rising: rotate pitch data only one step LEFT.
  if (inputs[DOWN_KEY].rising() && !pat_key_down) {
    if (pitch_mod) engine.RotatePitchLeft();
    else           engine.RotateTimeLeft();
    pat_changed = true;
  }
  // CLEAR + UP rising: rotate time data one step RIGHT within length.
  // CLEAR + PITCH + UP rising: rotate pitch data only one step RIGHT.
  if (inputs[UP_KEY].rising() && !pat_key_down) {
    if (pitch_mod) engine.RotatePitchRight();
    else           engine.RotateTimeRight();
    pat_changed = true;
  }
  // CLEAR + SLIDE rising: Mutate current pattern (small random perturbation).
  if (inputs[SLIDE_KEY].rising() && !pat_key_down) {
    engine.Mutate();
    pat_changed = true;
  }
  // CLEAR + BACK rising: shift whole pattern (pitch+time) one step LEFT.
  if (inputs[BACK_KEY].rising()) {
    engine.ShiftPatternLeft();
    pat_changed = true;
  }
  // CLEAR + TAP_NEXT rising: shift whole pattern (pitch+time) one step RIGHT.
  if (inputs[TAP_NEXT].rising() && !pitch_mod) {
    engine.ShiftPatternRight();
    pat_changed = true;
  }
  // CLEAR + F# rising: reverse entire pattern (pitch+time) within length.
  if (inputs[FSHARP_KEY].rising()) {
    engine.ReversePattern();
    pat_changed = true;
  }
  // CLEAR + G# rising: clear pitches only (keep time data).
  if (inputs[GSHARP_KEY].rising()) {
    engine.ClearPitchesOnly();
    pat_changed = true;
  }
  // CLEAR + A# rising: clear time data only (keep pitches).
  if (inputs[ASHARP_KEY].rising()) {
    engine.ClearTimesOnly();
    pat_changed = true;
  }
  // Individual-attribute randomize (CLEAR + PITCH_KEY held + white key rising):
  //   C = semitones, D = octaves, E = accents, F = slides,
  //   G = full pitch data (sem+oct+acc+slide), A = time data
  if (pitch_mod && !time_mod) {
    if (inputs[C_KEY].rising())     { engine.RandomizeSemitones();   pat_changed = true; }
    if (inputs[D_KEY].rising())     { engine.RandomizeOctaves();     pat_changed = true; }
    if (inputs[E_KEY].rising())     { engine.RandomizeAccentData();  pat_changed = true; }
    if (inputs[F_KEY].rising())     { engine.RandomizeSlideData();   pat_changed = true; }
    if (inputs[G_KEY].rising())     { engine.RandomizePitchData();   pat_changed = true; }
    if (inputs[A_KEY].rising())     { engine.RandomizeTimeData();    pat_changed = true; }
  }
  if (pat_changed) {
    // Incremental sync drains 2 steps per loop iteration so rapid
    // randomize presses can't saturate the MIDI TX ring. The 147-byte
    // 0x11 blob was blocking clock RX and causing timing drift.
    s_pat_sync_pat = engine.get_patsel();
    s_pat_sync_pos = 0;
    s_pat_sync_len = engine.get_length();
  }
  // CLEAR + C# held + pat key rising: copy pattern (current bank) to clipboard.
  // CLEAR + D# held + pat key rising: paste clipboard into that pattern slot.
  static uint8_t s_clip_buf[PATTERN_SIZE];
  static bool    s_clip_valid = false;
  if (inputs[CSHARP_KEY].held()) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (inputs[i].rising()) {
        const uint8_t bank_now = (engine.get_patsel() >> 3) & 1;
        engine.export_pattern_blob(bank_now * 8 + i, s_clip_buf);
        s_clip_valid = true;
        break;
      }
    }
  } else if (inputs[DSHARP_KEY].held()) {
    if (s_clip_valid) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].rising()) {
          const uint8_t bank_now = (engine.get_patsel() >> 3) & 1;
          const uint8_t dst = bank_now * 8 + i;
          engine.import_pattern_blob(dst, s_clip_buf, !clk_run);
          midi_send_pattern_update(dst);
          break;
        }
      }
    }
  }
}

// =============================================================================
// CV output: running = sequenced pitch + engine gate/accent/slide; stopped =
// keys / MIDI live play / audition previews.
// =============================================================================
static void OutputDAC(bool clk_run, bool write_mode, bool track_mode, bool edit_mode,
                      bool pitch_mod, bool fn_mod, bool dial_play_mode) {
  // PITCH_KEY + ACCENT / SLIDE held: live force-accent / force-slide override
  // applied at the CV output stage. Pattern Play / Track Play only -- blocked
  // in Pattern Write and Track Write so it can't interfere with note entry.
  // Releases return to the pattern's stored flags. (FN + ACCENT / FN + SLIDE
  // variant was dropped; PITCH_KEY is the canonical force modifier.)
  const bool force_accent_live =
      pitch_mod && !fn_mod && dial_play_mode && clk_run && inputs[ACCENT_KEY].held();
  const bool force_slide_live  =
      pitch_mod && !fn_mod && dial_play_mode && clk_run && inputs[SLIDE_KEY].held();

  if (clk_run && s_keyboard_mode) {
    // Keyboard play overrides sequenced CV with the audition preview values
    // so the 303 voice tracks the player's keys while the sequencer keeps
    // running its own pattern (sequencer MIDI out is unaffected).
    DAC::SetPitch(s_tap_pitch_preview_cv);
    DAC::SetSlide(s_tap_pitch_preview_slide);
    DAC::SetAccent(s_tap_pitch_preview_accent);
    DAC::SetGate(s_tap_pitch_preview_gate &&
                 (s_tap_pitch_preview_retrig == 0));
    if (s_tap_pitch_preview_retrig) --s_tap_pitch_preview_retrig;
  } else if (clk_run) {
    // Stuck-gate guard only: the click normally ends via the clock-tick
    // counter in the tick loop; this fires if the external clock dies mid-click.
    if (s_metro_gate_pulse && s_metro_gate_timer > 500) {
      s_metro_gate_pulse = false;
      midi_metronome_stop();
    }
    // Force-slide-live: gate stays HIGH across all non-rest steps so the
    // envelope doesn't retrigger between notes. Combined with the slide pin
    // held high, this makes every transition a continuous portamento -- the
    // "infinite slide" the user wants. Rests still drop the gate naturally
    // (engine.resting captures the current step's REST state).
    const bool gate_running = force_slide_live
        ? !engine.resting
        : engine.get_gate();
    // Tap-write voice (one shared voice): the metronome click cuts through
    // for its 2 ticks even over a held note (user request: the beat stays
    // audible while holding; the gate stays high so the note is not
    // retriggered, the pitch just dips to the click), then the monitor
    // (accept tick until release), then the pattern. In the RECORD phase the
    // engine voice is SILENT, exactly the ROM's measure (its notes doubling
    // the monitor was audible glitching); in the OVERDUB phase the recorded
    // pattern plays clean with no clicks. The idle decay tail holds the last
    // event's pitch -- snapping back to the resting pitch mid-decay played
    // the tail low, a thump the d650c doesn't have.
    const bool tap_monitor  = s_metronome_active && s_metro_monitor_gate;
    const bool engine_voice = !s_metronome_active || !s_metro_record_phase;
    uint8_t pitch_cv;
    if (s_metro_gate_pulse)
      pitch_cv = s_metro_pitch_cv;
    else if (tap_monitor)
      pitch_cv = s_metro_tap_monitor_cv;
    else if (engine_voice && gate_running)
      pitch_cv = clamp_cv(int(engine.get_pitch_scaled()) + total_transpose);
    else if (s_metronome_active && s_metro_record_phase)
      // RECORD only: hold the last click/monitor pitch through the decay.
      // In OVERDUB the engine is the voice; its decays must ring at the
      // pattern's own pitch exactly like normal playback, not at a stale
      // click pitch.
      pitch_cv = s_metro_tail_cv;
    else
      pitch_cv = clamp_cv(int(engine.get_pitch_scaled()) + total_transpose);
    DAC::SetPitch(pitch_cv);
    DAC::SetSlide((engine_voice && engine.get_slide_dac()) || force_slide_live);
    DAC::SetAccent((tap_monitor ? s_metro_tap_monitor_accent
                                : (engine_voice && engine.get_accent())) ||
                   force_accent_live);
    DAC::SetGate((engine_voice && gate_running) || s_metro_gate_pulse || tap_monitor);
  } else {
    bool gate = midi_live_gate();
    // Live MIDI play (clock stopped): drive the CV from the received note, not the
    // sequencer's resting pitch -- otherwise every incoming note plays the resting
    // slot (C1). Note 36 maps to linear 0 (matches engine pitch / get_midi_note).
    uint8_t pitch_cv = gate
        ? clamp_cv(int(midi_live_note()) - 36 + total_transpose)
        : clamp_cv(int(engine.get_pitch()) + total_transpose);
    // Auditioning a non-CV variation (var2/var3) is MIDI-only: the 303 analog voice
    // must stay silent. Only variation-1 edits open the audition CV/gate; the MIDI
    // audition (sent on the variation's channel) is unaffected.
    const bool audition_cv = (engine.get_edit_var() == 0);
    if (audition_cv && s_tap_pitch_preview_gate) {
      pitch_cv = s_tap_pitch_preview_cv;
      gate = true;
      if (s_tap_pitch_preview_retrig) { gate = false; --s_tap_pitch_preview_retrig; }
    } else if (audition_cv && s_back_pitch_preview_gate) {
      pitch_cv = s_back_pitch_preview_cv;
      gate = true;
    } else if (audition_cv && write_mode && !track_mode && s_cfg_menu == CfgMenu::Off && !edit_mode &&
               engine.get_mode() == PITCH_MODE && check_pitch_inputs()) {
      gate = true;
    }

    bool slide_cv = inputs[SLIDE_KEY].held() || midi_live_slide();
    // Audition slide is the captured intent, not a live seq.get_slide() read.
    // Live read would pick up the stale slide bit at the post-advance pitch_pos
    // and slide from the overwritten note to the just-written one.
    if (gate && (s_tap_pitch_preview_gate || s_back_pitch_preview_gate))
      slide_cv = slide_cv || s_tap_pitch_preview_slide;

    // Only drive the pitch CV while the gate is open. With the gate closed the
    // note is released; snapping the CV back to the sequencer's resting pitch
    // slot makes an external (still-tracking) VCO jump to a different note on
    // release. Holding the last latched pitch keeps the released note steady.
    if (gate) DAC::SetPitch(pitch_cv);
    DAC::SetSlide(slide_cv);
    // Accent during audition (TAP-held edit, BACK preview, or live key-write)
    // is locked to the preview state captured at audition entry. Letting the
    // raw ACCENT_KEY drive the analog accent pin while a note is sustaining
    // spikes the envelope and retriggers the voice -- user reported this as
    // "ACCENT retriggers the audtioned note." Toggling accent via ACCENT_KEY
    // still flips the stored flag (handled in input_pitch / step-edit); the
    // user must press BACK + TAP to re-audition with the new accent.
    const bool audition_active = s_tap_pitch_preview_gate || s_back_pitch_preview_gate;
    DAC::SetAccent(audition_active
        ? s_tap_pitch_preview_accent
        : (inputs[ACCENT_KEY].held() || midi_live_accent()));
    DAC::SetGate(gate);
  }
}

// =============================================================================
// loop: poll, MIDI/clock, UI dispatch, DAC output every iteration
// =============================================================================
void loop() {
  // Scan the button matrix once per LED-refresh ISR tick (~1 ms at the 977 Hz
  // rate), gated on the refresh's own tick counter -- NOT a free-running
  // microsecond timer. PollInputs blanks the matrix for its whole scan
  // (~150-200us) and Redraw() then holds the last-driven row until the next ISR
  // tick. Gating on EVERY tick means that held row is (isr_tick & 3), which
  // advances 0,1,2,3 on consecutive scans -- every row gets the same brief hold
  // once per 4 scans (~244 Hz, invisible). Gating on 2+ ticks would land the
  // scan on a fixed subset of ticks, so Redraw would favour some rows over
  // others (static dimming, and a beat when loop jitter flips which rows) --
  // that was the residual SuperOS flicker. Between scans, re-push the last raw
  // samples so PinState debounce/edge semantics stay per-iteration.
  static uint8_t s_last_scan_tick = 0;
  const uint8_t now_tick = Leds::isr_tick;
  if ((uint8_t)(now_tick - s_last_scan_tick) >= 1) {
    s_last_scan_tick = now_tick;
    Leds::PauseRefresh();
    PollInputs(inputs);
    // Re-light the matrix immediately with the last frame so the scan leaves no
    // dark tail, without re-phasing Timer3 or advancing the row/PWM counters.
    Leds::Redraw();
    Leds::ResumeRefresh();
  } else {
    for (uint8_t i = 0; i < INPUT_COUNT; ++i) inputs[i].push(inputs[i].read());
  }

#if DEBUG
  if (Serial.available() && Serial.read()) {
    const uint8_t s = engine.abs_slot(engine.get_patsel());
    uint8_t noteSteps = 0, ties = 0, notes = 0;
    for (uint8_t st = 0; st < engine.poly_.length; ++st) {
      const uint8_t t = engine.poly_.time(st);
      if (t == 1) ++noteSteps; else if (t == 2) ++ties;
      notes += engine.poly_.voice_count(st);
    }
    Serial.printf("POLY slot=%u flag=%u active=%u len=%u noteSteps=%u ties=%u notes=%u tpos=%u rest=%u chord=%u evar=%u mode=%u\n",
                  s, GlobalSettings.var3_is_poly(s), engine.poly_active_, engine.poly_.length,
                  noteSteps, ties, notes, engine.poly_time_pos_, engine.poly_resting_,
                  engine.poly_chord_pos_, engine.edit_var_, engine.get_mode());
  }
#endif

  // All modal-modifier reads sampled once per iteration. Legacy locals are
  // initialized from `ins` so existing call sites compile unchanged. Phase 1
  // and later will replace `track_mode`/`write_mode`/etc. with `ins.*` and
  // dispatch on `dial` as gating moves into per-mode handlers.
  const InputState ins = read_input_state(inputs);
  const DialMode dial  = dial_mode_of(ins);

  const bool track_mode = ins.track_sel;
  const bool write_mode = ins.write_mode;
  const bool clear_mod  = ins.clear;
  const bool edit_mode  = ins.edit;

  // Release the "pattern-cleared LEDs lit while held" flag once CLEAR is up.
  if (!clear_mod) s_pat_cleared_hold = false;

  const bool fn_mod    = ins.fn;
  const bool pitch_mod = ins.pitch;
  const bool time_mod  = ins.time;

  // Dial-mode gates: destructive / structural ops belong to PatternWrite only.
  // Force-accent / force-slide live overrides belong to play modes only.
  const bool dial_pattern_write = (dial == DialMode::PatternWrite);
  const bool dial_play_mode     = (dial == DialMode::PatternPlay) ||
                                  (dial == DialMode::TrackPlay);
  const bool dial_track_write   = (dial == DialMode::TrackWrite);
  const bool dial_track_play    = (dial == DialMode::TrackPlay);
  const bool dial_track_mode    = dial_track_write || dial_track_play;

  total_transpose = int16_t(int(transpose) + int(engine.get_pattern_transpose()));
  // In Track mode, also add the per-chain-step transpose for the slot the
  // chain cursor is currently on. Encoding: TRACK_TRANSPOSE_ZERO (=12) means
  // "no transpose", so we subtract that offset before adding.
  if (engine.track_active && engine.track_has_chain()) {
    const int8_t step_off = int8_t(engine.TrackGetTranspose(engine.get_chain_pos()))
                          - int8_t(TRACK_TRANSPOSE_ZERO);
    total_transpose = int16_t(int(total_transpose) + int(step_off));
  }

  // Read track index from the 3-bit dial (TRACK_BIT0..2). Tracks 0..7 select
  // 1-of-8 track slots. In track modes, the track index also forces the active
  // bank (bank = track >> 1, matches OS-303 layout).
  const uint8_t cur_tracknum = uint8_t(inputs[TRACK_BIT0].held()
                                  | (inputs[TRACK_BIT1].held() << 1)
                                  | (inputs[TRACK_BIT2].held() << 2));

  if (s_cfg_menu == CfgMenu::Off && !edit_mode && fn_mod &&
      inputs[CLEAR_KEY].rising()) {
    s_cfg_menu = CfgMenu::Midi;
    s_cfg_suppress_clear_exit = true;
  }

  const bool clk_run =
      inputs[RUN].held() || (midi_clk && GlobalSettings.midi_clock_receive);

  const bool prev_midi_clk = midi_clk;
  uint8_t midi_clock_pulses = 0;
  midi_poll(engine, clk_run, midi_clk, midi_clock_pulses);
  // SysEx 0x22 may have updated led_brightness; mirror into the LED driver.
  Leds::brightness = GlobalSettings.led_brightness;
  // Persist any pending SysEx config change to EEPROM on the very next idle
  // iteration. Waiting for WRITE/RUN falling loses config if the user powers
  // off without ever entering edit/run (common with web-only workflows).
  midi_flush_pending_saves();
  // Persist web-edited patterns in the background: one pattern per idle tick,
  // only after a quiet period so rapid edits coalesce. Ensures patterns pushed
  // from the web editor survive a power cycle without requiring a RUN/WRITE
  // toggle. Safe during playback - EEPROM write is ~3-100ms per pattern.
  midi_flush_pending_pattern_saves(engine);
  // Persist hardware-edited resident patterns in the background too. engine.stale
  // is otherwise only flushed on transport stop / dial-mode change, so a user who
  // writes a pattern and powers off without ever running the transport or moving
  // the dial loses the edits. Save when stopped and quiet (2s) so the flash stall
  // happens between keypresses, not on every one.
  {
    // Covers aux_dirty() too: variation 2/3, poly and probability edits do not
    // set `stale`, so gating on `stale` alone left them in RAM until a slot
    // change or transport stop and a power-off lost them.
    static bool     s_stale_prev = false;
    static uint32_t s_stale_ms   = 0;
    const bool dirty = engine.stale || engine.aux_dirty();
    if (dirty && !s_stale_prev) s_stale_ms = millis();
    s_stale_prev = dirty;
    if (dirty && !clk_run && (millis() - s_stale_ms) >= 2000) engine.Save();
  }
  // Detect MIDI clock Start rising edge (midi_clk just became true this frame).
  const bool midi_clk_rose = (!prev_midi_clk && midi_clk && GlobalSettings.midi_clock_receive);

  // Determine how many clock ticks to process this iteration.
  // MIDI clock: may be >1 if multiple 0xF8 bytes arrived during a single poll
  // (e.g. interleaved with a long SysEx). DIN sync: always 0 or 1.
  uint8_t clock_ticks = 0;
  if (midi_clock_pulses > 0) {
    clock_ticks = midi_clock_pulses;
  } else if (!midi_clk) {
    clock_ticks = inputs[CLOCK].rising() ? 1 : 0;
  }
  const bool clocked = (clock_ticks > 0);

  // Save pattern data on transport stop or dial-mode change.
  // Dial-mode change covers PatternWrite -> any other dial position (replaces
  // the prior WRITE_MODE.falling() trigger and additionally handles the
  // PatternWrite <-> TrackWrite transition where WRITE_MODE stays high).
  bool dial_changed = false;
  {
    static DialMode s_last_dial = dial;
    static bool     s_last_dial_init = false;
    // First iteration also counts: if we boot with the dial in a Track mode,
    // we need to LoadTrack and set track_active just like a real transition.
    if (!s_last_dial_init || dial != s_last_dial) dial_changed = true;
    s_last_dial = dial;
    s_last_dial_init = true;
  }
  // Save patterns when the transport STOPS, from ANY clock source: the internal
  // RUN button, MIDI clock Stop, or DIN sync stop. (Previously gated to
  // `RUN.falling() && !midi_clk`, which never fired under external MIDI/DIN
  // sync, so externally-synced edits were lost on power-off.) midi_clk already
  // reflects this frame because midi_poll() ran above. Saving on dial-mode
  // change is still avoided (it would block the loop ~25-100 ms mid-playback);
  // edits not stopped through stay in RAM and are lost on power-cycle.
  const bool running_now =
      inputs[RUN].held() || (midi_clk && GlobalSettings.midi_clock_receive);
  static bool s_was_running = false;
  const bool transport_stopped = s_was_running && !running_now;
  s_was_running = running_now;
  if (transport_stopped) {
    engine.Save();
    if (engine.track_stale) engine.SaveTrack();
    midi_flush_pending_saves();
    if (dial_track_mode) emit_track_state(dial, /*clk_run=*/false, cur_tracknum & 0x07);
    // On clock stop: if browsing a different group, switch to it. The
    // group-change detector after the clock loop broadcasts it (once).
    if (s_display_group != engine.get_group())
      engine.SetGroup(s_display_group);
  }
  // Reset edit-mode UI overlays whenever the dial leaves Pattern Write
  // (covers WRITE.falling() and PatternWrite -> TrackWrite, etc).
  if (dial_changed) {
    s_len_extended     = false;
    s_len_black_pressed = false;
    s_step_sel_mode = false;
    s_step_sel_edit = false;
    s_step_sel_time = false;
    s_step_sel      = -1;
    s_step_sel_ext  = false;
    s_dir_mode      = false;
    s_scale_mode    = false;
    s_prob_mode     = false;
    s_prob_step     = -1;
    s_prob_ext      = false;
    s_prob_base     = 0;
    if (s_keyboard_mode) {
      s_keyboard_mode = false;
      s_tap_pitch_preview_gate = false;
      midi_kb_all_notes_off();
      kb_stack_clear();
    }
    // Config menu must not survive a dial move; it otherwise keeps consuming
    // keys in the new dial position until CLEAR or FN is pressed.
    s_cfg_menu = CfgMenu::Off;
    s_cfg_suppress_clear_exit = false;
    // Exit any in-progress edit mode so PITCH_MODE / TIME_MODE state cannot
    // bleed across dial positions.
    engine.SetMode(NORMAL_MODE);
    // Persist any in-RAM chain edits before switching away from track mode,
    // otherwise edits made while running are lost when the dial moves to a
    // pattern position (or back later to a different track index).
    if (engine.track_stale) engine.SaveTrack();
    s_track_arm_last = false;
    // Entering Track mode: force bank = track >> 1, load that track. Discard
    // any active Pattern-mode chain so it can't override track playback.
    if (dial_track_mode) {
      const uint8_t track_idx = cur_tracknum & 0x07;
      const uint8_t bank      = uint8_t(track_idx >> 1);
      if (bank != engine.get_group()) engine.SetGroup(bank);
      engine.LoadTrack(track_idx);
      // Pattern-mode chain state must not leak into Track mode; clear it.
      s_chain_active     = false;
      s_chain_len        = 0;
      s_chain_queue_len  = 0;
      s_chain_anchor_key = 0xff;
      s_chain_hold_key   = 0xff;
      s_chain_hold_target_pat = 0xff;
      // TrackPlay: chain advance fires on every wrap. TrackWrite: chain
      // advance stays off so each pat-key just changes the playing pattern.
      // p_select is intentionally NOT reset here -- on entering TrackPlay the
      // user should still see the last pattern that was playing (typically
      // the last one written in TrackWrite) until they press CLEAR.
      engine.track_active = dial_track_play;
    } else {
      engine.track_active = false;
    }
    emit_track_state(dial, clk_run, cur_tracknum & 0x07);
  }

  // Track-index switch while staying in Track mode: persist the outgoing
  // track, then load the new one. Each dial position is its own track slot.
  if (dial_track_mode) {
    static uint8_t s_last_track_idx = 0xFF;
    const uint8_t cur_track_idx = cur_tracknum & 0x07;
    if (s_last_track_idx != 0xFF && cur_track_idx != s_last_track_idx) {
      if (engine.track_stale) engine.SaveTrack();
      s_track_arm_last = false;
      const uint8_t bank = uint8_t(cur_track_idx >> 1);
      if (bank != engine.get_group()) engine.SetGroup(bank);
      engine.LoadTrack(cur_track_idx);
      engine.track_active = dial_track_play;
      emit_track_state(dial, clk_run, cur_track_idx);
    }
    s_last_track_idx = cur_track_idx;
  }

  // Apply chain state received from web editor (SysEx 0x1A), or re-broadcast on config request
  {
    uint8_t rx_al, rx_ap[4], rx_ql, rx_qp[4];
    if (midi_get_received_chain(&rx_al, rx_ap, &rx_ql, rx_qp)) {
      if (rx_al == 0xff) {
        // Config request sentinel: just re-broadcast current chain state, no change
        emit_chain_state();
      } else {
        // Apply new chain state from web
        if (rx_al > 1) {
          // Same active chain re-sent (the web echoes the full state when it
          // only queued something): keep the running position and wrap intent
          // untouched, or the chain would jump to its last entry.
          bool same_active = s_chain_active && (s_chain_len == rx_al);
          for (uint8_t ci = 0; same_active && ci < rx_al; ++ci)
            if (s_chain_pats[ci] != rx_ap[ci]) same_active = false;
          if (!same_active) {
            s_chain_active = true;
            s_chain_len    = rx_al;
            for (uint8_t ci = 0; ci < rx_al; ++ci) s_chain_pats[ci] = rx_ap[ci];
            chain_intent_reset();
            if (!clk_run) {
              s_chain_pos = 0;
              engine.SetPattern(chain_entry_start(rx_ap[0]), true);
            } else {
              s_chain_pos = s_chain_len - 1; // chain advance will queue pats[0] next
            }
          }
        } else if (rx_al == 1) {
          s_chain_active = false;
          s_chain_len    = 1;
          s_chain_pats[0] = rx_ap[0];
          chain_intent_reset();
          engine.SetPattern(chain_entry_start(rx_ap[0]), !clk_run);
        } else {
          s_chain_active = false;
          s_chain_len    = 0;
          chain_intent_reset();
        }
        s_chain_queue_len = rx_ql;
        for (uint8_t ci = 0; ci < rx_ql; ++ci) s_chain_queued[ci] = rx_qp[ci];
        s_chain_anchor_key      = 0xff;
        s_chain_hold_key        = 0xff;
        s_chain_hold_loop       = false;
        s_chain_hold_target_pat = 0xff;
        emit_chain_state();
      }
    }
  }

  // Ignore RUN button while MIDI clock is actively driving the sequencer:
  // pressing RUN mid-playback would call engine.Reset() and stutter.
  const bool run_rising_effective = inputs[RUN].rising() && !midi_clk;
  if (run_rising_effective || midi_clk_rose) {
    // midi_poll already called engine.Reset() on MIDI Start; only reset for hardware button.
    if (!midi_clk_rose) engine.Reset();
    chain_intent_reset();
    // Restart chain from first pattern on every start (hardware or MIDI clock).
    // The Reset() above only reset the sequence selected AT STOP; when either
    // branch moves patsel, the landed sequence still holds its old position
    // and the run would start mid-pattern -- reset it too.
    if (s_chain_active && s_chain_len > 1) {
      s_chain_pos       = 0;
      s_chain_queue_len = 0;
      engine.SetPattern(chain_entry_start(s_chain_pats[0]), true);
      engine.get_sequence().Reset();
    } else if (s_ab_mode && Engine::is_section_b(engine.get_patsel())) {
      // No chain: a run that previously stopped on the B half of a linked
      // pair must restart from A, not resume alternating from B.
      engine.SetPattern(Engine::section_a_of(engine.get_patsel()), true);
      engine.get_sequence().Reset();
    }
    emit_chain_state();
    if (dial_track_mode) emit_track_state(dial, /*clk_run=*/true, cur_tracknum & 0x07);
  }

  // While stopped, keep the shadow voices loaded for the active slot so editing
  // variation 2/3 targets (and displays) the selected pattern's data. While
  // running, the clock-tick loop handles reload (with note-off) at the switch.
  if (!clk_run && engine.ShadowsNeedReload()) {
    engine.persist_shadows();
    engine.ReloadShadows();
    engine.edit_var_ = 0;
  }

  // -=-=- Process inputs and set LEDs -=-=-

  if (s_scale_mode) {
    ProcessScaleMode();
  } else if (s_cfg_menu != CfgMenu::Off) {
    process_config_menu();
  } else if (!clk_run && dial_pattern_write && engine.get_mode() == PITCH_MODE &&
             engine.edit_var_ == 2 && engine.poly_active_) {
    ProcessPolyEdit();
  } else if (edit_mode && !fn_mod && !clk_run && engine.get_mode() != NORMAL_MODE) {
    ProcessEdit(write_mode, clk_run);
  } else if (s_dir_mode) {
    ProcessDirectionMode(dial_pattern_write);
  } else if (dial_track_mode) {
    ProcessTrackUI(dial, dial_track_write, clk_run,
                   uint8_t(cur_tracknum & 0x07), fn_mod, clear_mod, pitch_mod);
  } else {
    // Reset step-select detail-editor sub-state whenever we're not currently in step-select.
    if (!s_step_sel_mode && s_step_sel_edit) { s_step_sel_edit = false; s_step_sel_time = false; }
    // FN + TIME_KEY rising → enter direction mode (allowed in pattern write mode; the
    // TIME_MODE set at line ~1050 is gated by !fn_mod).
    if (fn_mod && inputs[TIME_KEY].rising()) {
      s_dir_mode = true;
      // Overlay modes sit on top of NORMAL_MODE: drop any active PITCH/TIME
      // submode so exiting the overlay always returns to normal.
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (fn_mod && inputs[ACCENT_KEY].rising() && dial_pattern_write) {
      s_scale_mode = true;
      s_scale_fn_entry = true; // FN is down from the entry combo; its release must not exit
      s_scale_cycle_root = 0xFF;
      s_scale_cycle_idx = 0;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (fn_mod && inputs[SLIDE_KEY].rising() && !s_prob_mode && dial_pattern_write) {
      s_prob_mode = true;
      s_prob_base = 0;
      s_prob_ext  = false;
      s_prob_step = -1;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (fn_mod && inputs[PITCH_KEY].rising() && !s_step_sel_mode && dial_pattern_write) {
      s_step_sel_mode = true;
      s_step_sel_chain_view = 0; // always default to first chain slot on entry
      s_step_sel_base = 0;
      s_step_sel_ext  = false;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (s_prob_mode) {
      ProcessProbabilityMode(clk_run, dial_pattern_write);
    } else if (s_step_sel_mode) {
      ProcessStepSelect(clk_run, dial_pattern_write);
    } else if (s_keyboard_mode && (dial == DialMode::PatternPlay)) {
      ProcessKeyboardPlay();
    } else if (pitch_mod && !fn_mod && !clear_mod && !write_mode && !s_keyboard_mode) {
      ProcessPitchMod();
    } else if (time_mod) {
      Leds::Set(FUNCTION_MODE_LED, true);
      // TODO: performance time effects
    } else if (fn_mod && !s_fn_chain_saved) {
      ProcessLengthEditor(dial_pattern_write);
    } else if (edit_mode && dial_pattern_write && !fn_mod && !clear_mod &&
               !s_metronome_active && !s_metro_tap_swallow &&
               engine.get_mode() == NORMAL_MODE) {
      // Hold TAP_NEXT in Pattern Write/normal mode: edit-variation picker.
      // s_metro_tap_swallow keeps a tap-write session's held TAP from landing
      // here when the session auto-exits at the wrap.
      ProcessEditVarPicker(clk_run);
    } else {
      ProcessDefault(write_mode, clear_mod, clk_run, dial_pattern_write);
    }
  }

  // ── Pattern chain advance ──
  // Pattern-mode chains never run in Track mode; engine-side track_advance_chain
  // is the chain driver there.
  if (s_chain_active && s_chain_len > 1 && clk_run && !dial_track_mode) {
    const uint8_t cur = engine.get_patsel();
    bool chain_state_changed = false;

    // Position tracking by INTENT: we remember what we queued for the next
    // wrap (s_chain_expect / s_chain_expect_pos / s_chain_expect_handoff) and
    // act when the wrap lands on it. Deriving position or queue promotion
    // from the playing pattern VALUE alone aliases chains with repeated
    // entries: 2,1,2 would snap back to the first 2, and a queued chain
    // starting with a pattern the current chain also contains would promote
    // mid-pass instead of at the end of the chain. The value search below is
    // only the fallback for external moves (user tap, run start).
    const uint8_t tp      = engine.get_time_pos();
    const bool    wrapped = (tp == 0 && s_chain_prev_tp != 0 && s_chain_prev_tp != 0xFF);
    s_chain_prev_tp = tp;

    // Chain-wide A/B: the first member's link flag makes EVERY member play
    // A then B; a member's own flag still counts (pattern linked before it
    // was chained).
    const bool cur_linked = s_ab_mode;
    if (wrapped) {
      if (s_chain_expect != 0xFF && cur == s_chain_expect) {
        if (s_chain_expect_handoff && s_chain_queue_len > 0) {
          // The end-of-chain handoff we queued has landed: promote the queued
          // chain (or drop to a single pattern) exactly at the chain boundary.
          if (s_chain_queue_len > 1) {
            for (uint8_t ci = 0; ci < s_chain_queue_len; ++ci)
              s_chain_pats[ci] = s_chain_queued[ci];
            s_chain_len = s_chain_queue_len;
            s_chain_pos = 0;
          } else {
            s_chain_active = false;
            s_chain_len    = 0;
          }
          s_chain_queue_len   = 0;
          chain_state_changed = true;
        } else {
          s_chain_pos = s_chain_expect_pos;
        }
      } else {
        // External move: re-derive by value, searching from the cursor. A
        // linked pair's B half matches its entry too.
        for (uint8_t k = 0; k < s_chain_len; ++k) {
          const uint8_t ci = uint8_t((s_chain_pos + k) % s_chain_len);
          const uint8_t e  = s_chain_pats[ci];
          if (e == cur || (cur_linked && (e & 7) == (cur & 7))) { s_chain_pos = ci; break; }
        }
      }
      s_chain_expect_handoff = false;
    }

    if (s_chain_active && s_chain_len > 1) {
      // Queue next: linked-pair B half > hold-loop > queued chain > advance.
      uint8_t next_pat;
      uint8_t next_pos = s_chain_pos;
      bool    handoff  = false;
      if (cur_linked && !Engine::is_section_b(cur)) {
        // A linked pair inside a chain plays its whole A-B before the chain
        // advances: hand the wrap to the B section of the SAME chain entry.
        next_pat = Engine::section_b_of(cur);
      } else if (s_chain_hold_loop && (s_chain_hold_target_pat == 0xff || cur == s_chain_hold_target_pat ||
                 (cur_linked && (s_chain_hold_target_pat & 7) == (cur & 7)))) {
        // Loop: a linked pair loops A->B->A->B, an unlinked pattern loops itself
        next_pat = cur_linked ? Engine::section_a_of(cur) : cur;
      } else if (s_chain_queue_len > 0 && s_chain_pos == s_chain_len - 1) {
        // At the last pattern of the current chain: hand off to the queued
        // chain. Entered at A when the named entry is a linked pair (the tap
        // paths normalize queued[0], the hold+tap build path does not).
        next_pat = chain_entry_start(s_chain_queued[0]);
        handoff  = true;
      } else {
        next_pos = uint8_t((s_chain_pos + 1) % s_chain_len);
        next_pat = s_chain_pats[next_pos];
        // In A/B mode entries are entered at their A section regardless of
        // which section the chain named, so every pass plays A then B.
        if (s_ab_mode)
          next_pat = Engine::section_a_of(next_pat);
      }
      s_chain_expect         = next_pat;
      s_chain_expect_pos     = next_pos;
      s_chain_expect_handoff = handoff;
      engine.SetPattern(next_pat);
    }

    if (chain_state_changed) emit_chain_state();
  } else if (clk_run && !dial_track_mode && s_ab_mode) {
    // A/B mode with no user chain: queue the other section so the
    // engine hands over at the wrap. A plays, then B, then A again -- one
    // pattern of up to 128 steps (spec 1-a / 5-d / 5-e).
    const uint8_t cur  = engine.get_patsel();
    const uint8_t nxt  = engine.get_next();
    const uint8_t want = Engine::is_section_b(cur) ? Engine::section_a_of(cur)
                                                   : Engine::section_b_of(cur);
    // A defer parked on a different pair is stale (the pair was unlinked or
    // the pattern moved externally before it could land): drop it.
    if (s_pair_defer != 0xFF && s_pair_defer_pair != (cur & 7)) {
      s_pair_defer      = 0xFF;
      s_pair_defer_pair = 0xFF;
    }
    if (nxt == cur) {
      // Idle wrap: steer the A/B alternation -- or land a deferred switch now
      // that the B half has finished (the pair is ONE pattern; a switch never
      // cuts it in half).
      if (s_pair_defer != 0xFF && Engine::is_section_b(cur)) {
        engine.SetPattern(s_pair_defer);
        s_pair_defer      = 0xFF;
        s_pair_defer_pair = 0xFF;
      } else {
        engine.SetPattern(want);
      }
    } else if (!Engine::is_section_b(cur) && nxt != Engine::section_b_of(cur) &&
               engine.pending_group_ == 0xff) {
      // A switch queued while the A half plays waits for the pair to finish:
      // park it, hand the wrap to B, and land it at B's wrap (first branch).
      // A later tap overwrites the parked target, so the newest choice wins.
      // Group-switch queues are exempt: the pending group applies at the very
      // next wrap regardless, so deferring the pattern would strand it.
      s_pair_defer      = nxt;
      s_pair_defer_pair = uint8_t(cur & 7);
      engine.SetPattern(Engine::section_b_of(cur));
    }
  }

  // show all pressed buttons. Probability mode renders its own step / picker /
  // level LEDs, and its note keys (percentage entry) and black keys share the
  // step-LED matrix, so the raw echo would spuriously light step LEDs while
  // setting a percentage -- suppress it there (same reason as keyboard mode).
  if (s_cfg_menu == CfgMenu::Off && !s_prob_mode) {
    for (uint8_t i = 0; i < 16; ++i) {
      const InputIndex b = switched_leds[i].button;
      if (!inputs[b].held()) continue;
      if (b == UP_KEY && inputs[C_KEY2].held()) continue;
      // Keyboard mode: raw held reads of note keys include matrix phantoms;
      // ProcessKeyboardPlay lights the real (stack) keys instead.
      if (s_keyboard_mode &&
          (b <= C_KEY2 || b == CSHARP_KEY || b == DSHARP_KEY ||
           b == FSHARP_KEY || b == GSHARP_KEY))
        continue;
      Leds::Set(OutputIndex(i), true);
    }
    // A# is a direct LED (switched_leds[17]) not covered by the 0-15 loop above
    if (inputs[ASHARP_KEY].held() && !s_keyboard_mode)
      Leds::Set(ASHARP_KEY_LED, true);
  }

  // Metronome: auto-exit if transport stopped or write mode released. These
  // are the ONLY session exits -- CLEAR+TIME while active RESTARTS the
  // session (see the gesture below), it never toggles off.
  if (s_metronome_active && (!clk_run || !write_mode)) {
    s_metronome_active = false;
    s_metro_gate_pulse = false;
    midi_metronome_stop();
    midi_audition_note_off();
  }
  // Per-frame TAP tracking for the recorder. A press ARMS the pending flag
  // and starts the monitor voice IMMEDIATELY (ROM-measured: the heard note
  // follows the finger from the press, not from the accept tick). The write
  // itself still lands on the tick grid below. Release ends the monitor and
  // the tie chain immediately.
  if (s_metronome_active) {
    if (inputs[TAP_NEXT].rising()) {
      s_metro_press_pending = true;
      if (s_metro_bar_started) {
        // Predict the step this press is aimed at (same rule the accept tick
        // applies) so the monitor previews the pitch that note will consume.
        // Pre-write, pitch_index_for_note gives the same stream slot the
        // write will map, so the pitch matches the accept-time result.
        const uint8_t t_dec = (engine.step_period() >= 8) ? 3 : 2;
        const uint8_t k     = engine.get_time_pos();
        const uint8_t tgt   = (s_metro_step_tick < t_dec) ? k : uint8_t(k + 1);
        metro_start_monitor(engine.get_sequence(), tgt);
      }
    }
    if (inputs[TAP_NEXT].falling()) {
      s_metro_note_active  = false;
      s_metro_monitor_gate = false;
      midi_audition_note_off();     // monitor follows the finger's release
    }
  }
  // TAP stays owned by tap-write until released, even across the auto-exit.
  if (s_metronome_active && inputs[TAP_NEXT].held()) s_metro_tap_swallow = true;
  else if (!inputs[TAP_NEXT].held())                 s_metro_tap_swallow = false;

  Leds::Swap();

  // Pattern group dial (positions 1-2=group0, 3-4=group1, 5-6=group2, 7=group3):
  // the knob's 7 detents pair into the panel's 4 pattern groups I-IV. Track mode
  // uses the raw 0..7 for its 8 track slots.
  // Debounced: require GROUP_DEBOUNCE_FRAMES consecutive frames before accepting a new group.
  if (!inputs[TRACK_SEL].held()) {
    const uint8_t new_group = uint8_t(cur_tracknum <= 1 ? 0 : cur_tracknum <= 3 ? 1 : cur_tracknum <= 5 ? 2 : 3);
    if (s_prev_tracknum == 0xff) {
      // First frame: initialize without debounce
      s_display_group = new_group;
      s_group_debounce_val = new_group;
      s_group_debounce_count = GROUP_DEBOUNCE_FRAMES;
      if (new_group != engine.get_group())
        engine.SetGroup(new_group);
    } else if (new_group != s_display_group) {
      if (new_group == s_group_debounce_val) {
        s_group_debounce_count++;
        if (s_group_debounce_count >= GROUP_DEBOUNCE_FRAMES) {
          s_display_group = new_group;
          s_group_debounce_val = 0xff;
          s_group_debounce_count = 0;
          if (!clk_run)
            engine.SetGroup(new_group);
        }
      } else {
        s_group_debounce_val = new_group;
        s_group_debounce_count = 1;
      }
    }
    s_prev_tracknum = cur_tracknum;
  }

  if (inputs[FUNCTION_KEY].rising() && !edit_mode) {
    if (s_keyboard_mode) {
      s_keyboard_mode = false;
      kb_stack_clear();
      s_tap_pitch_preview_gate = false;
      midi_kb_all_notes_off();
    } else if (s_step_sel_mode) {
      s_step_sel_mode = false;
      s_step_sel_edit = false;
      s_step_sel_time = false;
      engine.SetMode(NORMAL_MODE, !clk_run);
    } else if (s_prob_mode) {
      s_prob_mode = false;
      s_prob_step = -1;
    } else if (s_dir_mode) {
      s_dir_mode = false;
    } else if (s_scale_mode) {
      // FN is also the scale-preset modifier; the exit fires on FN FALLING
      // (tap with no preset note) inside ProcessScaleMode, not on rising.
    } else if (s_cfg_menu == CfgMenu::Midi) {
      s_cfg_menu = CfgMenu::Off;
    } else if (s_cfg_menu == CfgMenu::Off) {
      engine.SetMode(NORMAL_MODE, !clk_run);
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    // PITCH_MODE / TIME_MODE entry: Pattern Write only, and never while an
    // overlay mode owns the keys (direction / scale / step-select). Without
    // the overlay gate, pressing PITCH or TIME inside an overlay silently
    // queued a submode that popped up the moment the overlay exited (and in
    // step-select made bare step presses open the audition gate).
    const bool overlay_mode = s_dir_mode || s_scale_mode || s_step_sel_mode || s_prob_mode;
    const bool in_poly_edit = (engine.get_mode() == PITCH_MODE && engine.edit_var_ == 2 && engine.poly_active_);
    // !s_metronome_active: a stray TIME/PITCH press during a tap-write
    // session would leave NORMAL_MODE and dead-lock the CLEAR+TIME restart
    // gesture (it requires NORMAL_MODE); the recorder owns the panel.
    if (inputs[TIME_KEY].rising()  && dial_pattern_write && !clear_mod && !fn_mod && !edit_mode && !in_poly_edit && !overlay_mode && !s_metronome_active) { engine.SetMode(TIME_MODE, !clk_run); s_time_edit_steps = 0; }
    if (inputs[PITCH_KEY].rising() && dial_pattern_write && !fn_mod && !edit_mode && !clear_mod && !overlay_mode && !s_metronome_active) engine.SetMode(PITCH_MODE, !clk_run);

    // Keyboard play mode toggle: FN + PITCH_KEY rising while dial is in Pattern Play.
    if (fn_mod && inputs[PITCH_KEY].rising() && (dial == DialMode::PatternPlay) &&
        !edit_mode && !clear_mod) {
      s_keyboard_mode = !s_keyboard_mode;
      kb_stack_clear();
      if (!s_keyboard_mode) {
        s_tap_pitch_preview_gate = false;
        midi_kb_all_notes_off();
      }
    }

    // CLEAR + TIME_KEY: START metronome tap-write (running + Pattern Write +
    // NORMAL_MODE). NOT a toggle: with the session looping endlessly and an
    // all-OVERDUB pass being silent (no clicks, patterns play normally), the
    // user cannot HEAR whether a session is still active -- a blind toggle
    // made every other CLEAR+TIME an invisible no-op exit ("tap-write only
    // works every second try / after a transport restart"). Now the gesture
    // always means "record from the top": inactive = begin, active = restart
    // the session fresh. A session ends on transport stop or leaving
    // Pattern Write (flick the dial to Pattern Play and back to exit without
    // stopping).
    // D# guard: with CLEAR + D# held (paste gesture) the diode-less matrix reads a
    // phantom TIME via the dial contact (PH0,PA0) -- same ghost class as the
    // ProcessClearCombos pattern-key guard -- which would start a tap-write
    // session mid-paste and wipe the pattern's time data.
    // Either press order fires: CLEAR held + TIME pressed, or TIME held +
    // CLEAR pressed. With only the first form, pressing both together could
    // miss (TIME's edge debouncing in a frame before CLEAR reads held), and
    // the press was silently swallowed -- "I had to press it twice".
    const bool metro_gesture =
        (clear_mod && inputs[TIME_KEY].rising()) ||
        (inputs[CLEAR_KEY].rising() && inputs[TIME_KEY].held());
    // TIME_MODE is accepted too: when both keys go down together, TIME's
    // edge can debounce one frame before CLEAR reads held, opening
    // TIME_MODE instead of the gesture -- which then blocked the gesture
    // until TIME_MODE auto-exited ("press twice and wait"). The session
    // forces the mode back to NORMAL in metro_session_begin().
    // !s_step_sel_mode: the step editor runs in NORMAL_MODE and owns
    // TIME + CLEAR as its A/B pin gesture; starting a tap-write session from
    // there would wipe the unit's time data mid-edit.
    if (metro_gesture && !fn_mod && !s_step_sel_mode &&
        !inputs[DSHARP_KEY].held() && !inputs[CSHARP_KEY].held() &&
        clk_run && dial_pattern_write &&
        (engine.get_mode() == NORMAL_MODE || engine.get_mode() == TIME_MODE)) {
      s_metronome_active = true;
      metro_session_begin();
    }

    // CLEAR rising with a pat key held: clear that pattern (only clear path).
    // Pattern Write only -- destructive op. Suppressed during tap-write:
    // pattern keys mean SUSTAIN there, and CLEAR is half of the restart
    // gesture, so CLEAR while sustaining must not wipe a pattern.
    if (inputs[CLEAR_KEY].rising() && !fn_mod && !edit_mode &&
        dial_pattern_write && !s_metronome_active) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (inputs[i].held()) {
          const uint8_t pat = uint8_t((engine.get_patsel() >> 3) * 8 + i);
          // Spec 2: clearing one section leaves the other alone -- unless the
          // A/B mode is on, in which case the whole pattern goes.
          const bool both = s_ab_mode;
          // ClearPattern wipes reserved[], and with it the saved A/B memory
          // on the A section; restore the flag if it was saved.
          const bool saved_ab = engine.pair_linked(pat);
          engine.ClearPattern(pat);
          midi_send_pattern_update(pat);
          if (both) {
            const uint8_t other = Engine::is_section_b(pat) ? Engine::section_a_of(pat)
                                                            : Engine::section_b_of(pat);
            engine.ClearPattern(other);
            midi_send_pattern_update(other);
          }
          if (saved_ab) engine.set_pair_linked(pat, true);
          pattern_cleared_flash_timer = 0;
          s_pat_cleared_hold = true;
          break;
        }
      }
    }

    // Global CLEAR combos: rotate / randomize / mutate / shift / reverse /
    // clear-only / copy-paste. Destructive, Pattern Write only. Suppressed in
    // probability mode (CLEAR runs its randomize gestures) and during
    // tap-write (TAP belongs to the recorder; with CLEAR still held from the
    // entry gesture, every tap would also fire CLEAR+TAP = shift right).
    if (!s_prob_mode && !s_metronome_active)
      ProcessClearCombos(clear_mod, fn_mod, dial_pattern_write, pitch_mod,
                         time_mod, clk_run);

    // Spec 5-a: holding FUNCTION opens on the page that CONTAINS the current
    // last step, so the display starts where the pattern actually ends instead
    // of at page 1. Nothing is written until a white key picks a new last step,
    // so paging past the end to look around is free.
    // Spec 3-a: "Once the desired PATTERN length is reached, press the
    // FUNCTION BUTTON to exit PITCH MODE." The same press must not also open
    // the length editor, so it is consumed here and the editor picks up on the
    // next FUNCTION hold (spec 5, "adjusted after the fact").
    if (inputs[FUNCTION_KEY].rising() && dial_pattern_write && !clk_run &&
        engine.get_mode() == PITCH_MODE) {
      engine.SetMode(NORMAL_MODE, true);
    }
    // Chain-save / memory-clear gesture. While the chain-build keys are held
    // (hold 1, tap 2/3/4...) with a chain of >= 2, pressing FUNCTION stores
    // the chain on its FIRST pattern together with the current A/B mode, so
    // recalling it re-enters the mode. With a SINGLE pattern key held (no
    // chain), FUNCTION clears that pattern's stored chain AND its saved A/B
    // memory. The press is consumed so the same FN hold cannot also drive
    // the length editor.
    if (inputs[FUNCTION_KEY].rising() && dial_pattern_write) {
      int8_t held_key = -1;
      for (uint8_t i = 0; i < 8; ++i)
        if (inputs[i].held()) { held_key = int8_t(i); break; }
      if (held_key >= 0 && s_chain_len >= 2) {
        store_chain_on(s_chain_pats[0], s_chain_pats, s_chain_len);
        engine.set_pair_linked(s_chain_pats[0], s_ab_mode);
        pattern_cleared_flash_timer = 0;  // mode-LED flash = "saved"
        s_fn_chain_saved = true;
      } else if (held_key >= 0) {
        // Single pattern held: store the CURRENT state. A/B mode on = save
        // the A/B memory on this pattern (stored chain untouched); mode off =
        // clear its saved A/B memory AND its stored chain (the unlink
        // gesture).
        const uint8_t pat = uint8_t((engine.get_patsel() >> 3) * 8 + held_key);
        if (s_ab_mode) {
          engine.set_pair_linked(pat, true);
        } else {
          Sequence &cs = engine.pattern[pat & 0x0F];
          cs.reserved[4] = 0; cs.reserved[5] = 0; cs.reserved[6] = 0;
          engine.set_pair_linked(pat, false);
        }
        engine.stale = true;
        midi_send_pattern_update(Engine::section_a_of(pat));
        pattern_cleared_flash_timer = 0;  // mode-LED flash = "saved/cleared"
        s_fn_chain_saved = true;
      }
    }
    if (inputs[FUNCTION_KEY].rising() && dial_pattern_write && !s_fn_chain_saved) {
      const uint8_t cl = engine.edit_seq_view().length;
      s_len_extended      = (MAX_STEPS > 32) && (cl > 32);
      s_len_black_base    = uint8_t(((cl - 1) / 8) * 8);
      s_len_black_pressed = true;
    }
    if (inputs[FUNCTION_KEY].falling()) {
      s_len_black_pressed = false;
      s_len_extended = false;
      s_fn_chain_saved = false;
    }
  }

  midi_leader_transport(clocked, clk_run, midi_clk,
                        inputs[RUN].rising(), inputs[RUN].falling());

  // Process every accumulated clock tick. Normally clock_ticks is 0 or 1; if
  // multiple piled up (e.g. SysEx on the same DIN port as the clock) we must
  // drain them in this iteration so the sequencer plays at the actual clock
  // rate. Earlier prototypes capped at 1/iteration but that fell behind when
  // the main loop was slower than the clock rate (audible tempo drop).
  for (uint8_t ct = 0; ct < clock_ticks; ++ct) {
    ++clk_count %= 24;

    // Metronome click length counts CLOCK TICKS (2 = shorter than the 3-tick
    // step gate) so it scales with tempo like the d650c's.
    if (s_metro_gate_ticks && --s_metro_gate_ticks == 0) {
      s_metro_gate_pulse = false;
      midi_metronome_stop();
    }

    if (clk_run) {
      const bool step_boundary = engine.Clock();
      if (step_boundary) {
        if (engine.ShadowsNeedReload()) { // slot/group changed
          engine.persist_shadows();           // flush any RAM shadow edits first
          midi_shadows_all_notes_off(engine); // close old notes on their old channels
          engine.ReloadShadows();
          engine.edit_var_ = 0;               // new pattern/group -> edit variation 1
        }
        engine.AdvanceShadows();              // advance shadows + compute their gate state
        // Wrap-only anchor: send the playhead position via 0x15 only when
        // time_pos transitions back to 0. The web editor counts MIDI clock
        // bytes to interpolate steps between anchors, so we avoid the
        // per-16th SysEx burst that coupled into the audio rail.
        {
          static uint8_t s_anchor_prev_tp = 0xFF;
          const uint8_t tp = engine.get_time_pos();
          if (tp == 0 && s_anchor_prev_tp != 0) {
            midi_send_step_position(engine.get_patsel(), 0);
            // Track view: cursor / next_p may have advanced on this wrap.
            if (dial_track_mode)
              emit_track_state(dial, /*clk_run=*/true, cur_tracknum & 0x07);
          }
          s_anchor_prev_tp = tp;
        }
        // Metronome tap-write, boundary duties: bar/chain bookkeeping and the
        // metronome click. All WRITING happens on the tick grid below, where
        // the ROM does it (accept/decision ticks), not at boundaries.
        if (s_metronome_active) {
          const uint8_t cur_pat = engine.get_patsel();
          const uint8_t tp      = engine.get_time_pos();
          if (tp == 0) {
            if (!s_metro_bar_started) {
              s_metro_bar_started = true;        // bar reset landed: recording on
            } else {
              if (s_metro_record_phase && !s_metro_pass_accept) {
                // ROM bar validation: a RECORD pass none of whose taps was
                // still held at its accept tick ends as an EMPTY bar -- stale
                // writes are discarded and the metronome keeps looping,
                // exactly like the ROM's endless empty-measure record loop.
                Sequence &pseq = engine.pattern[s_metro_prev_pat];
                if (pseq.note_count()) {
                  for (uint8_t i = 0; i < pseq.length; ++i)
                    sequence_set_time_at(pseq, i, 0);
                  engine.stale = true;
                  midi_send_pattern_update(s_metro_prev_pat);
                }
              }
              // One guided pass through the whole unit, then DONE: at the
              // wrap that completes a full pass (every chain member's bar,
              // linked halves counted), the session AUTO-EXITS if anything
              // was recorded -- the take is finished, the metronome and the
              // TIME LED stop, and the patterns play what was tapped. An
              // all-empty pass keeps looping with the metronome so the
              // clicks guide until the first real take (the ROM's endless
              // empty measure). Runs AFTER the bar validation above so a
              // discarded all-stale take does not count as recorded.
              if (++s_metro_bar_count >= s_metro_pass_bars) {
                s_metro_bar_count = 0;
                bool recorded = false;
                for (uint8_t p = 0; p < NUM_PATTERNS; ++p)
                  if ((s_metro_unit_mask & uint16_t(1u << p)) &&
                      engine.pattern[p].note_count()) { recorded = true; break; }
                if (recorded) {
                  s_metronome_active   = false;
                  s_metro_monitor_gate = false;
                  s_metro_gate_pulse   = false;
                  s_metro_gate_ticks   = 0;
                  midi_metronome_stop();
                  midi_audition_note_off();
                }
              }
            }
            if (s_metronome_active) {
              // Patterns pulled in AFTER entry (new chain builds, queued
              // taps) keep their content and play clean in OVERDUB (the
              // phase computation below sees their notes), taps recording
              // on top. Later passes preserve earlier ones -- see the
              // rest-skip in the tick decision below.
              s_metro_prev_pat    = cur_pat;
              s_metro_pass_accept = false;   // fresh validation window per pass
              // Phase for the pass that starts NOW: a pattern that has
              // content graduates to OVERDUB (clicks off, engine voice on);
              // an empty one stays in RECORD (metronome keeps guiding).
              if (engine.get_sequence().note_count())
                s_metro_recorded_mask |= uint16_t(1u << cur_pat);
              else
                s_metro_recorded_mask &= uint16_t(~(1u << cur_pat));
              s_metro_record_phase =
                  !(s_metro_recorded_mask & uint16_t(1u << cur_pat));
              if (!s_metro_record_phase) {
                // An OVERDUB pass begins: the pattern is the voice. A finger
                // still held from the recording bar must not carry over --
                // the tie chain would write TIEs over this pattern's rests
                // (permanently extending its saved notes), and the monitor
                // voice would override the engine's pitch while both gate.
                // Holds still tie across RECORD wraps (a linked pair being
                // recorded is one 64-step pattern), only content-bearing
                // passes cut them.
                s_metro_note_active = false;
                if (s_metro_monitor_gate) {
                  s_metro_monitor_gate = false;
                  midi_audition_note_off();
                }
              }
            }
          }
          // Metronome click (measured on the d650c): LOW (DAC 51, ~327 Hz on
          // factory trim) at the bar start, HIGH (DAC 63) on the other 8ths,
          // 2 clock ticks long. Unlike the ROM (which mutes the click under a
          // held note), the click keeps ticking THROUGH held notes -- user
          // request: the beat must stay audible while holding. Clicks only
          // run in the RECORD phase; an input pattern plays clean.
          if (s_metronome_active && s_metro_record_phase && (tp % 2) == 0) {
            s_metro_gate_pulse = true;
            s_metro_gate_ticks = 2;
            s_metro_gate_timer = 0;
            const bool bar_start = (tp == 0);
            s_metro_pitch_cv = bar_start ? 51 : 63;
            // A click during a held note must not steal the release tail:
            // the decay after letting go should ring at the note, not 63.
            if (!s_metro_monitor_gate) s_metro_tail_cv = s_metro_pitch_cv;
            midi_metronome_tick(bar_start);
          }
        }
      }
      // Tap-write tick grid (the ROM's actual sampling). Tick 0 = the step
      // boundary; accepts run on ticks t_dec..period-1, the step's own
      // note/tie/rest decision on tick t_dec (1/3 into the step).
      if (s_metronome_active) {
        if (step_boundary) s_metro_step_tick = 0;
        else if (s_metro_step_tick < 250) ++s_metro_step_tick;
      }
      if (s_metronome_active && s_metro_bar_started) {
        const uint8_t period = engine.step_period();
        const uint8_t t_dec  = (period >= 8) ? 3 : 2;
        const uint8_t t      = s_metro_step_tick;
        Sequence &sseq       = engine.get_sequence();
        const uint8_t k      = engine.get_time_pos();
        const uint8_t len    = engine.get_length();
        const bool held      = inputs[TAP_NEXT].held();
        bool wrote_note_now  = false;
        if (t >= t_dec && s_metro_press_pending) {
          // Accept tick with an armed press. ROM-measured law: the note is
          // written whether or not the key is still held (a "stale" press
          // still lands on its target step). Held state only feeds the tie
          // chain and the pass-validation flag; a RECORD pass none of whose
          // taps were held at their accept tick is discarded at the wrap,
          // which is the ROM's endless empty-bar loop.
          s_metro_press_pending = false;
          uint8_t target = k;
          bool ok = true;
          if (t > t_dec) {
            // Late accept: the tap was aimed at the NEXT step's downbeat.
            // Past the bar end it drops (the manual's "you cannot write
            // correctly if tapping between two measures").
            if (uint8_t(k + 1) >= len) ok = false;
            else target = uint8_t(k + 1);
          }
          if (ok) {
            sequence_write_time_with_pitch_sync(sseq, target, 1);
            engine.stale = true;
            s_metro_any_note = true;
            if (held) {
              s_metro_note_active = true;
              s_metro_pass_accept = true;
            }
            if (target != k) s_metro_step_prewritten = true;
            else             wrote_note_now = true;
            uint8_t pb = PITCH_EMPTY;
            const uint8_t slot = sseq.pitch_index_for_note(target);
            if (slot < sseq.get_pitch_count()) pb = sseq.pitch[slot];
            midi_send_step_update(engine.get_patsel(), target, pb, 1);
          }
        }
        if (t == t_dec) {
          // This step's decision instant. RESTs are never WRITTEN: the entry
          // clear made the bar empty, so pass one is identical to the ROM,
          // and later overdub passes preserve what earlier ones recorded. A
          // held note's TIE claims the step outright; SELECTOR sustain only
          // fills steps that are still RESTs.
          if (s_metro_step_prewritten) {
            s_metro_step_prewritten = false;   // a late accept already wrote it
          } else if (!wrote_note_now) {
            // A TIE never overwrites an existing NOTE: later passes (and the
            // saved content of chained / linked patterns playing in OVERDUB)
            // are preserved. A held finger crossing a NOTE step gets cut by
            // that note instead of erasing it.
            bool tie = false;
            if (s_metro_note_active && held && sseq.time(k) != 1) tie = true;
            else if (s_metro_any_note && metro_sustain_held() &&
                     sseq.time(k) == 0)                           tie = true;
            if (tie) {
              sequence_write_time_with_pitch_sync(sseq, k, 2);
              engine.stale = true;
              midi_send_step_update(engine.get_patsel(), k, PITCH_EMPTY, 2);
            }
          }
        }
      }
      // Every clock tick: MIDI note on/off follows the analog gate (all 3 voices),
      // so MIDI sustain matches the 303 hardware gate exactly. The tick's sends are
      // batched and flushed as one burst (USB in a single packet, then DIN) so all
      // variations' notes land together instead of serializing per voice.
      midi_tick_begin();
      midi_seq_gate_tick(engine, total_transpose);
      midi_shadows_gate_tick(engine, total_transpose);
      midi_tick_flush(); // all variations' note-ONs first, then the queued offs
    }
  }

  // Idle tempo blink (matches the d650c's stopped-transport fallback): DIN and
  // MIDI masters gate their clock while stopped, which froze clk_count and the
  // pattern LED. When the transport is stopped and no clock tick has arrived
  // for a while, free-run the blink counter at 120 BPM (24 ppqn = 20.833 ms)
  // so the selected pattern LED keeps flashing like a real 303. Blink only:
  // sequencing still runs exclusively off real ticks.
  {
    static uint32_t s_last_real_tick_ms = 0;
    static uint32_t s_idle_blink_us = 0;
    if (clocked) {
      s_last_real_tick_ms = millis();
    } else if (!clk_run && (uint32_t)(millis() - s_last_real_tick_ms) > 400) {
      const uint32_t now_us = micros();
      if ((int32_t)(now_us - s_idle_blink_us) >= 20833) {
        // resync after a long gap instead of bursting the backlog
        if ((uint32_t)(now_us - s_idle_blink_us) > 41666u) s_idle_blink_us = now_us;
        else s_idle_blink_us += 20833;
        ++clk_count %= 24;
      }
    }
  }

  // Single group-change broadcast point: catches queued switches applied at
  // wrap AND the direct SetGroup calls above (dial, transport stop, MIDI PC).
  {
    static uint8_t s_prev_group = 0;
    if (engine.get_group() != s_prev_group) {
      s_prev_group = engine.get_group();
      s_display_group = engine.get_group();
      // 0x1C only: the web editor reacts by bulk-requesting all 48 variation
      // slots itself (paced 0x10/0x11 round trips). Broadcasting 16 unsolicited
      // pattern dumps here overflowed the 512-byte DIN ring (most were dropped)
      // and mis-counted the editor's bulk-reply tracker.
      midi_send_group_update(engine.get_group());
    }
  }

  if (s_cfg_menu == CfgMenu::Off) {
    // FN + UP / FN + DOWN, Pattern Write. The pair does two different jobs
    // depending on the submode, exactly as the spec splits them:
    //   TIME MODE (spec 7)   -- UP toggles triplet timing for this SECTION.
    //   otherwise (spec 5-b) -- UP doubles the last step, DOWN halves it.
    // Both length moves are non-volatile: halving only pulls the last step in,
    // and doubling either restores the retained tail or copies the first half.
    if (fn_mod && !pitch_mod && dial_pattern_write) {
      Sequence &seq = engine.get_edit_sequence();
      if (engine.get_mode() == TIME_MODE) {
        if (inputs[UP_KEY].rising()) {
          // Triplet pages are 12 steps instead of 16, so the ceiling drops
          // from 64 to 48. The time stream is NOT re-paged: it stays linear,
          // so notes that sat on steps 13-16 land on the next page's 1-4
          // (spec 7-a) and the steps past 48 are retained, not deleted.
          const bool now_triplet = !seq.is_triplet_mode();
          seq.set_triplet_mode(now_triplet);
          if (now_triplet && seq.length > TRIPLET_MAX_STEPS)
            engine.ApplyLength(seq, uint8_t(TRIPLET_MAX_STEPS));
          engine.stale = true;
          midi_send_length_update(engine.get_patsel(), seq.length, engine.get_edit_var());
        }
      } else {
        // Variation 1 doubles/halves across the A/B pair (spec 5-e); the
        // MIDI-only shadows have no sections, so they use the plain form.
        const bool pair_scope = (engine.get_edit_var() == 0);
        const uint8_t pat = engine.get_patsel();
        const uint8_t cap = seq.is_triplet_mode() ? uint8_t(TRIPLET_MAX_STEPS)
                                                  : uint8_t(MAX_STEPS);
        if (inputs[UP_KEY].rising()) {
          if (pair_scope) pair_double_length(pat);
          else            sequence_double_length(seq, cap);
          engine.stale = true;
          midi_send_pattern_update(pat);
          if (pair_scope && s_ab_mode)
            midi_send_pattern_update(Engine::section_b_of(pat));
        }
        if (inputs[DOWN_KEY].rising()) {
          if (pair_scope) {
            const uint8_t t = pair_total_length(pat);
            pair_set_total_length(pat, uint8_t(t > 1 ? t / 2 : 1));
            midi_send_pattern_update(pat);
          } else {
            sequence_halve_length(seq);
            midi_send_length_update(pat, seq.length, engine.get_edit_var());
          }
          engine.stale = true;
        }
      }
    }
    // While FN is held in TIME MODE, UP_KEY_LED shows the section's triplet
    // state (spec 7: "the UP LED being lit while FUNCTION is held in TIME
    // MODE"). Outside TIME MODE the key means double, so no state to show.
    if (fn_mod && dial_pattern_write && engine.get_mode() == TIME_MODE) {
      Leds::Set(UP_KEY_LED, engine.edit_seq_view().is_triplet_mode());
    }

    // FN + BACK_KEY: length -1, FN + TAP_NEXT: length +1. Pattern Write only.
    // Skipped while PITCH_KEY is held -- BACK is consumed by step-select mode.
    if (fn_mod && !pitch_mod && dial_pattern_write && inputs[BACK_KEY].rising()) {
      uint8_t cl = engine.edit_seq_view().length;
      engine.SetLength(cl > 1 ? cl - 1 : 1);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
    }
    if (fn_mod && !pitch_mod && dial_pattern_write && inputs[TAP_NEXT].rising()) {
      uint8_t cl = engine.edit_seq_view().length;
      engine.SetLength(cl < MAX_STEPS ? cl + 1 : MAX_STEPS);
      engine.stale = true;
      midi_send_length_update(engine.get_patsel(), engine.edit_seq_view().length, engine.get_edit_var());
    }

    if (inputs[TAP_NEXT].rising() && !fn_mod && !engine.in_poly_pitch_edit()) {
      if (write_mode && !clk_run) {
        if (engine.get_mode() == PITCH_MODE) {
          engine.get_edit_sequence().ensure_pitch_edit_entry();
          // Audition the current pitch slot. PITCH_MODE walks the pitch stream
          // independently of time data, so don't gate on time(time_pos)==1
          // (that gate broke audition for patterns with no NOTE events yet).
          Sequence &auds = engine.get_edit_sequence();
          const uint8_t pc = auds.get_pitch_count();
          if (pc > 0 && auds.pitch_pos >= 0 && auds.pitch_pos < int(pc) &&
              auds.pitch[auds.pitch_pos] != PITCH_EMPTY) {
            uint16_t mn = uint16_t(36 + auds.get_pitch()) + total_transpose;
            if (mn > 127) mn = 127;
            const bool acc = auds.get_accent();
            const uint8_t vel = acc ? 127 : 80;
            s_tap_pitch_preview_cv = clamp_cv(int(auds.get_pitch()) + total_transpose);
            s_tap_pitch_preview_accent = acc;
            s_tap_pitch_preview_slide = false; // previews always trigger clean, never slide
            s_tap_pitch_preview_gate = true;
            midi_audition_note_on(uint8_t(mn), vel);
          }
        } else if (engine.get_mode() == TIME_MODE) {
          engine.AdvanceEditCursor();
          // Preview the note at the new cursor step (NOTE steps only).
          const Sequence &ts = engine.edit_seq_view();
          const uint8_t tp = uint8_t(ts.time_pos & (MAX_STEPS - 1));
          if (tp < ts.length && ts.time(tp) == 1) {
            const uint8_t slot = ts.pitch_index_for_note(tp);
            if (slot < ts.get_pitch_count() && ts.pitch[slot] != PITCH_EMPTY) {
              const uint8_t pb  = ts.pitch[slot];
              const uint8_t lin = unpack_pitch_linear(pb & 0x3f);
              const bool    acc = (pb & (1 << 6)) != 0;
              uint16_t mn = uint16_t(36 + lin) + total_transpose;
              if (mn > 127) mn = 127;
              if (s_tap_pitch_preview_gate) s_tap_pitch_preview_retrig = 2;
              s_tap_pitch_preview_cv     = clamp_cv(int(lin) + total_transpose);
              s_tap_pitch_preview_accent = acc;
              s_tap_pitch_preview_slide  = false; // previews never slide
              s_tap_pitch_preview_gate   = true;
              midi_audition_note_on(uint8_t(mn), acc ? 127 : 80);
            }
          }
        }
      }
    }
    // BACK in regular write (TAP not held): step cursor back through the
    // pitch stream. Clamp at 0 instead of wrapping -- pressing BACK on the
    // first step should stay there, not jump to the last note (which made
    // the next TAP play the last step and trigger the wrap-back-to-0 auto
    // exit). first_step=true so the auto-exit check `!first_step && pitch_pos==0`
    // doesn't fire while the user is parked on step 0.
    if (inputs[BACK_KEY].rising() && !fn_mod && write_mode && !clk_run && !edit_mode &&
        !s_step_sel_mode && engine.get_mode() == PITCH_MODE) {
      Sequence &s = engine.get_edit_sequence();
      const uint8_t pc = s.get_pitch_count();
      if (pc > 0 && s.pitch_pos > 0) {
        s.pitch_pos = s.pitch_pos - 1;
        s.sync_time_pos_to_pitch_pos();
        s.first_step = true;
      } else if (pc > 0) {
        // Already on step 0: keep cursor pinned, refresh time_pos display,
        // and ensure first_step stays true so we don't auto-exit.
        s.pitch_pos = 0;
        s.sync_time_pos_to_pitch_pos();
        s.first_step = true;
      }
    }
    if (inputs[TAP_NEXT].falling() && !engine.in_poly_pitch_edit()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off(); // close any open audition note
      // PITCH_MODE: on release, step to the next pitch-stream slot and exit
      // after a full pass. Wrap at the stream's end (pitch_count), not at
      // pattern length: REST/TIE steps own no pitch slot, so wrapping at
      // length made every note past the last one a silent blank TAP press.
      if (!clk_run && write_mode && engine.get_mode() == PITCH_MODE) {
        Sequence &es = engine.get_edit_sequence();
        es.first_step = false;
        const uint8_t epc = es.get_pitch_count();
        ++es.pitch_pos;
        if (es.pitch_pos >= int(epc) || es.pitch_pos >= int(es.length))
          es.pitch_pos = 0;
        es.sync_time_pos_to_pitch_pos();
        if (es.pitch_pos == 0 && !es.first_step)
          engine.SetMode(NORMAL_MODE, true);
      }
      if (!clk_run && engine.get_mode() == TIME_MODE &&
          int(engine.edit_seq_view().time_pos) >= int(engine.edit_seq_view().length) - 1)
        engine.SetMode(NORMAL_MODE, true);
    }
  }

  // A/B step-edit target (FN+PITCH step-select and the PITCH/TIME write
  // modes, running, linked pair or chain-wide A/B): PITCH held + CLEAR
  // (either press order) cycles which SECTION the view and step edits land
  // on, instead of following playback's A/B alternation. The pin drops the
  // moment the context does (mode exit, unlink, chain end, transport stop).
  {
    const bool ab_edit_ctx =
        dial_pattern_write && engine.edit_var_ == 0 &&
        (s_step_sel_mode ||
         (clk_run && (engine.get_mode() == PITCH_MODE ||
                      engine.get_mode() == TIME_MODE))) &&
        s_ab_mode;
    if (!ab_edit_ctx) {
      engine.ab_edit_pat_ = 0xFF;
    } else {
      const uint8_t psel = engine.get_patsel();
      // Step editor: PITCH or TIME held + CLEAR (either order) -- TIME+CLEAR
      // is safe there because the tap-write gesture is suppressed during
      // step-select. PITCH/TIME write modes: PITCH-based only (TIME+CLEAR is
      // tap-write in TIME_MODE).
      const bool gesture = s_step_sel_mode
          ? (((pitch_mod || time_mod) && inputs[CLEAR_KEY].rising()) ||
             (clear_mod && (inputs[PITCH_KEY].rising() ||
                            inputs[TIME_KEY].rising())))
          : ((pitch_mod && inputs[CLEAR_KEY].rising()) ||
             (clear_mod && inputs[PITCH_KEY].rising()));
      if (gesture) {
        // Cycle from the section currently viewed/edited: the pin when set,
        // else the step editor's stable default (section A), else the
        // playing section (the write modes follow playback until pinned).
        bool cur_b;
        if (engine.ab_edit_pat_ != 0xFF)
          cur_b = Engine::is_section_b(engine.ab_edit_pat_);
        else if (s_step_sel_mode)
          cur_b = false;
        else
          cur_b = Engine::is_section_b(psel);
        engine.ab_edit_pat_ = cur_b ? Engine::section_a_of(psel)
                                    : Engine::section_b_of(psel);
      } else if (engine.ab_edit_pat_ != 0xFF &&
                 (engine.ab_edit_pat_ & 7) != (psel & 7)) {
        // Chain advanced to another member: keep the pinned side, follow the
        // new pair.
        engine.ab_edit_pat_ = Engine::is_section_b(engine.ab_edit_pat_)
                                  ? Engine::section_b_of(psel)
                                  : Engine::section_a_of(psel);
      }
      // PITCH/TIME write modes, pinned to the non-playing section: its cursor
      // never advances, so mirror the playhead each pass and edits land on
      // the same step of the pinned half. (Step-select edits explicit steps
      // and needs no mirror.)
      if (engine.get_mode() != NORMAL_MODE &&
          engine.ab_edit_pat_ != 0xFF && engine.ab_edit_pat_ != psel) {
        Sequence &es = engine.get_edit_sequence();
        const Sequence &ps = engine.get_sequence();
        uint8_t tp = uint8_t(ps.time_pos);
        if (tp >= es.length) tp = uint8_t(tp % es.length);
        es.time_pos = tp;
        // NOTE steps only, like Sequence::Advance: TIE/REST keeps the held
        // note's slot so pitch edits hit the sounding note, not the next one.
        if (es.time(tp) == 1)
          es.pitch_pos = int(es.pitch_index_for_note(tp));
      }
    }
  }

  // regular pattern write mode (no TAP_NEXT).  Suppressed in direction mode so pitched
  // keys select a direction instead of writing notes into the active pattern.
  // Also suppressed in step-select mode (FN + PITCH held) so its inputs don't leak into writes.
  if (s_cfg_menu == CfgMenu::Off && !edit_mode && write_mode && !track_mode && !s_dir_mode
      && !s_step_sel_mode && !s_scale_mode && !engine.in_poly_pitch_edit()) {

    if (engine.get_mode() == TIME_MODE) {
      // SLIDE_KEY step-lock toggle removed (RAM-only after OS-303 migration).
      if (clk_run) {
        input_time(true, true);
      } else if (!fn_mod && check_time_inputs() &&
                 s_time_edit_steps < engine.edit_seq_view().length) {
        input_time(false, false);
        if (s_time_edit_steps >= engine.edit_seq_view().length)
          engine.SetMode(NORMAL_MODE, true);
      } else if (!clk_run && s_time_edit_steps >= engine.edit_seq_view().length)
        engine.SetMode(NORMAL_MODE, true);
    }

    if (engine.get_mode() == PITCH_MODE && !pitch_mod) {
      const bool check = check_pitch_inputs();
      if (clk_run || check) {
        const uint8_t written_note = input_pitch(clk_run, clk_run);
        // After recording the pitch, open audition on the hardware VCO + MIDI out.
        // Set the preview CV/gate so the DAC plays the written note (not the next step).
        if (!clk_run && check && written_note) {
          uint16_t mn = uint16_t(written_note) + total_transpose;
          if (mn > 127) mn = 127;
          const uint8_t vel = inputs[ACCENT_KEY].held() ? 127 : 80;
          s_tap_pitch_preview_cv  = clamp_cv(int(written_note) - 36 + total_transpose);
          // New-note write: audition reflects the user's modifier state at
          // write time, never stored flags. Without this, the OR with
          // seq.get_slide() below would read the post-advance pitch_pos's
          // stale slide bit and slide from the overwritten note to the new
          // one.
          s_tap_pitch_preview_accent = inputs[ACCENT_KEY].held();
          s_tap_pitch_preview_slide  = inputs[SLIDE_KEY].held();
          s_tap_pitch_preview_gate = true;
          midi_audition_note_on(uint8_t(mn), vel);
        }
      }
      // Close audition when all pitch keys released
      if (!clk_run && !check) {
        s_tap_pitch_preview_gate = false;
        midi_audition_note_off();
      }
      // Exit PITCH_MODE after a full linear loop through all steps (back to step 0).
      // first_step is true until the first write+advance, preventing false exit on entry.
      if (!clk_run
          && !engine.get_sequence().first_step
          && engine.get_sequence().pitch_pos == 0)
        engine.SetMode(NORMAL_MODE, true);
    }
    // If mode exited while a write-preview gate was latched, release it on key-up.
    if (!clk_run && !edit_mode && s_tap_pitch_preview_gate && !check_pitch_inputs()) {
      s_tap_pitch_preview_gate = false;
      midi_audition_note_off();
    }

  }

  // Live-transpose reset on pattern switch in play modes: the performance
  // transpose is not persisted, and resets to the no-transpose default whenever
  // the user switches patterns while in Pattern Play or Track Play.
  {
    static uint8_t s_last_patsel_for_transpose = 0xff;
    const uint8_t cur_patsel = engine.get_patsel();
    if (s_last_patsel_for_transpose != 0xff &&
        cur_patsel != s_last_patsel_for_transpose &&
        dial_play_mode) {
      transpose = 12;
    }
    s_last_patsel_for_transpose = cur_patsel;
  }

  OutputDAC(clk_run, write_mode, track_mode, edit_mode, pitch_mod, fn_mod,
            dial_play_mode);

  if (inputs[RUN].falling() && !midi_clk) {
    DAC::SetGate(false);
    engine.Reset();
    // Reset chain to first pattern on stop so next start begins at chain[0].
    if (s_chain_active && s_chain_len > 1) {
      s_chain_pos       = 0;
      s_chain_queue_len = 0;
      engine.SetPattern(s_chain_pats[0], true);
      emit_chain_state();
    }
  }

  // Incremental pattern sync: send 2 step updates per loop iteration.
  if (s_pat_sync_len > 0) {
    const Sequence &sseq = engine.get_sequence();
    for (uint8_t i = 0; i < 2 && s_pat_sync_pos < s_pat_sync_len; ++i, ++s_pat_sync_pos) {
      const uint8_t ti = s_pat_sync_pos;
      const uint8_t tt = sseq.time(ti);
      uint8_t pb = PITCH_EMPTY;
      if (tt == 1) {
        const uint8_t slot = sseq.pitch_index_for_note(ti);
        if (slot < sseq.get_pitch_count()) pb = sseq.pitch[slot];
      }
      midi_send_step_update(s_pat_sync_pat, ti, pb, tt);
    }
    if (s_pat_sync_pos >= s_pat_sync_len)
      s_pat_sync_len = 0;
  }

  // DAC::Send latches the buffered CV values to the hardware ports and pulses
  // the slide line for ~10us. Calling it every loop iteration (1000+/sec)
  // wastes CPU on hardware I/O and creates a high-frequency pulse train on
  // the slide line. OS-303 throttles to ~555 Hz when running and lets it
  // free-run when stopped (for responsive live audition). Same approach here.
  // On clock-tick iterations force an immediate latch (bypass the throttle):
  // MIDI note on/offs are emitted unthrottled in the clock-tick loop above, so
  // without this the analog gate latches up to ~1.8ms after the MIDI already
  // went out -- audible as MIDI-driven voices triggering before the 303.
  static elapsedMicros dac_timer;
  if (!clk_run || clocked || dac_timer > 1800) {
    DAC::Send();
    dac_timer = 0;
  }
}
