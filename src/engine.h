// Copyright (c) 2026, Nicholas J. Michalek
/*
 * engine.h — TB-303 pattern model + EEPROM; Engine handles patterns, clock, and gate.
 *
 * EEPROM versions:
 *   "PewPewPew!!2" — original: pitch slots packed sequentially by NOTE event
 *   "PewPewPew!!3" — 1:1 pitch/time model, 64-step, 16 patterns
 *   "PewPewPew!!4" — 1:1 pitch/time model, 16-step max, 4 groups x 16 patterns (32 bytes/pattern)
 *   "PewPewPew!!5" — groups 0-2: 16-step max (32 bytes/pattern); group 3: 64-step max (128 bytes/pattern)
 */

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#include "persistent_settings.h"

// =============================================================================
// Engine — patterns + clock + gate
// =============================================================================
struct Engine {
  // pattern storage
  Sequence pattern[NUM_PATTERNS];
  uint8_t p_select = 0;
  uint8_t next_p = 0;           // queued pattern
  uint8_t group_ = 0;           // active pattern group (0-3)
  uint8_t pending_group_ = 0xff; // group to load at next pattern wrap (0xff = none)
                      // TODO: start & end for chains

  SequencerMode      mode_      = NORMAL_MODE;
  SequenceDirection  direction_ = DIR_FORWARD;
  int8_t  pp_dir_        = 1;  ///< Ping-pong direction: +1 or -1
  uint8_t advance_count_ = 0;  ///< Steps since last pattern boundary (non-forward modes)
  /// Queued direction change (applied at next pattern wrap while clock is running).
  SequenceDirection  next_direction_          = DIR_FORWARD;
  bool               direction_change_pending_ = false;
  /// Direction of the last step advance: +1 = moved forward, -1 = moved backward.
  /// Used by slide_from_prev_dir and get_pitch_dir after Clock() to resolve pitch/slide.
  int8_t last_step_dir_ = 1;

  int8_t clk_count = -1;

  bool slide_gate = false; // tie/slide: hold gate across 16ths (firstpr.com 303 slide / gate)
  bool stale = false;
  bool resting = false;

  // Updated when the 16th advances (Clock, Reset, manual advance).
  uint32_t step_start_us_ = 0;

  uint8_t get_group() const { return group_; }
  uint8_t get_pending_group() const { return pending_group_; }

  // Queue a group switch to happen at the next pattern wrap boundary.
  void QueueGroup(uint8_t g) {
    if (g < NUM_GROUPS) pending_group_ = g;
  }

  // Apply the pending group now (called at pattern wrap in Advance, or immediately when stopped).
  void apply_pending_group() {
    if (pending_group_ == 0xff || pending_group_ == group_) { pending_group_ = 0xff; return; }
    stale = true;
    Save();
    group_ = pending_group_;
    pending_group_ = 0xff;
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
      ReadPattern(pattern[i], i, group_);
      if (!pattern[i].length) pattern[i].SetLength(8);
      normalize_pattern_times(pattern[i]);
    }
    stale = false;
  }

  // Switch to a different pattern group: saves current, loads new (immediate, for stopped clock).
  void SetGroup(uint8_t g) {
    if (g >= NUM_GROUPS || g == group_) return;
    pending_group_ = g;
    apply_pending_group();
  }

  void Load() {
    GlobalSettings.Load();
    bool valid = GlobalSettings.Validate();

    if (valid) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        ReadPattern(pattern[i], i, group_);
        if (!pattern[i].length) pattern[i].SetLength(8);
        normalize_pattern_times(pattern[i]);
      }
      GlobalSettings.load_midi_from_storage();
      direction_ = DIR_FORWARD;
    } else {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        pattern[i].Clear();
      GlobalSettings.midi_channel = 1;
      GlobalSettings.midi_clock_receive = true;
      GlobalSettings.midi_thru = false;
      strcpy(GlobalSettings.signature, sig_pew);
      GlobalSettings.Save();
      GlobalSettings.save_midi_to_storage();
      stale = true;
      Save();
    }
  }

  void Save(int pidx = -1) {
    if (!stale) return;
    if (pidx < 0) {
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        WritePattern(pattern[i], i, group_);
    } else
      WritePattern(pattern[pidx], pidx, group_);
    stale = false;
  }

  void Tick() {}

  // After Advance() outside Clock() — re-anchor step time.
  void SyncAfterManualAdvance(bool) { step_start_us_ = micros(); }

  /// Slide CV: only during the *destination* note after a slid step (not during the slid note).
  /// Uses last_step_dir_ so callers (midi_after_clock) get the correct directional result.
  bool get_slide_dac() const {
    return get_sequence().slide_from_prev_dir(uint8_t(direction_), last_step_dir_);
  }

  // returns false for rests
  bool Advance() {
    bool result;
    // Track the direction of movement for this step (used by slide/tie/pitch after the call).
    int8_t step_dir     = 1; // direction we just moved: +1 = forward, -1 = backward
    int8_t next_step_dir = 1; // direction of the *next* step (for is_tied_dir check)

    if (direction_ == DIR_FORWARD) {
      step_dir     = 1;
      next_step_dir = 1;
      result = get_sequence().Advance();
      // Apply queued direction change and/or pattern switch at wrap (time_pos == 0).
      // first_step is still true after a reset-first-step (not a real wrap); guard against it.
      if (0 == get_sequence().time_pos && !get_sequence().first_step) {
        if (direction_change_pending_) {
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
        }
        apply_pending_group();
        if (next_p != p_select) {
          p_select = next_p;
          get_sequence().Reset();
          direction_change_pending_ = false;
          pp_dir_ = 1; advance_count_ = 0;
          result = get_sequence().Advance();
          step_dir = next_step_dir = 1;
        }
      }
    } else {
      // Capture step_dir before AdvanceDirectional may flip pp_dir_ (ping-pong / brownian).
      if (direction_ == DIR_REVERSE)                                 step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) step_dir = pp_dir_;
      else                                                           step_dir = 1; // random: doesn't matter

      result = get_sequence().AdvanceDirectional(uint8_t(direction_), pp_dir_);

      // After advance: pp_dir_ carries the outgoing direction for ping-pong / brownian.
      if (direction_ == DIR_REVERSE)                                 next_step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) next_step_dir = pp_dir_;
      else                                                           next_step_dir = 1; // random: is_tied_dir returns false anyway

      ++advance_count_;
      if (advance_count_ >= get_sequence().length) {
        advance_count_ = 0;
        if (direction_change_pending_) {
          const SequenceDirection old_dir = direction_;
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
          // PINGPONG wraps at two different positions (step 0 on the backward leg,
          // step length-1 on the forward leg), so time_pos is indeterminate when the
          // direction change fires.  Force the pre-start position that the new
          // direction expects so it always lands on the correct first step.
          // REVERSE always wraps at step 0, which already aligns correctly, so
          // no adjustment is needed there.
          if (old_dir == DIR_PINGPONG) {
            if (direction_ == DIR_FORWARD) {
              // The direction change fires on a pingpong endpoint retrigger step.
              // Pre-positioning at len-1 + first_step would add a phantom step before
              // step 0, shifting forward phase by 1.  Instead, play step 0 directly on
              // this tick so timing stays phase-locked with the advance_count_ boundary.
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
              result        = get_sequence().time(0) != 0;
              step_dir      = 1;
              next_step_dir = 1;
            } else if (direction_ == DIR_REVERSE) {
              // AdvanceDirectional(REVERSE): time_pos <= 0 → length-1; force 0.
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
            } else {
              // Switching to another non-forward mode: clean reset to step 0.
              get_sequence().Reset();
            }
          }
        }
        apply_pending_group();
        if (next_p != p_select) {
          p_select = next_p;
          get_sequence().Reset();
          advance_count_ = 0;
          direction_change_pending_ = false;
          pp_dir_ = 1;
        }
      }
    }

    last_step_dir_ = step_dir;

    if (result) {
      // Direction-aware slide_gate: hold gate when current step has slide flag,
      // next step (in play direction) is a TIE, or current TIE was preceded by a slide.
      const bool next_is_tie = get_sequence().is_tied_dir(uint8_t(direction_), next_step_dir);
      const bool tie_slide   = get_sequence().is_tie() &&
                               get_sequence().slide_from_prev_dir(uint8_t(direction_), step_dir);
      // A REST immediately after a slide source cancels the slide — don't hold gate.
      const uint8_t next_pos = uint8_t(
          (unsigned(get_sequence().time_pos) +
           (next_step_dir >= 0 ? 1u : unsigned(get_sequence().length) - 1u)) %
          unsigned(get_sequence().length));
      const bool next_is_rest = (get_sequence().time(next_pos) == 0);
      slide_gate = (!next_is_rest && get_sequence().get_slide()) || next_is_tie || tie_slide;
    }
    resting = !result;
    return result;
  }

  // returns true on step advance (clock divide by 6)
  bool Clock() {
    ++clk_count %= 6;

    if (clk_count == 0) { // sixteenth note advance
      Advance();
      step_start_us_ = micros();
      return true;
    }

    return false;
  }

  void Reset() {
    // Apply any queued direction change immediately on stop/reset so the LED
    // state and the actual direction_ stay in sync for the next run.
    if (direction_change_pending_) {
      direction_ = next_direction_;
      direction_change_pending_ = false;
    }
    get_sequence().Reset();
    clk_count = -1;
    slide_gate = false;
    resting = true;
    step_start_us_ = micros();
    advance_count_ = 0;
    pp_dir_ = 1;
    last_step_dir_ = 1;
  }

  /// Ratchet distribution: ~50% none (1x), ~33% 2x, ~17% 3x.
  static uint8_t random_ratchet_val() {
    const uint8_t r = fast_rand(12);
    return r < 6 ? 0 : r < 10 ? 1 : 2;
  }

  /// Randomize entire pattern: both time data and pitches (NORMAL_MODE).
  void RandomizeFullPattern() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    // First pass: random time data, no ties after rests, no leading tie
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      uint8_t t;
      if (i == 0)       t = fast_rand(2) ? 1 : 0;  // note or rest
      else if (prev==0) t = fast_rand(2) ? 1 : 0;  // after rest: note or rest
      else              t = fast_rand(3);            // note, tie, or rest
      sequence_set_time_at(s, i, t);
      prev = t;
    }
    normalize_pattern_times(s);
    // Second pass: random pitches for NOTE steps. Ratchets are never
    // randomized here — all steps reset to 1x. Use RandomizeRatchetData
    // explicitly if you want random ratchets.
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1) {
        const uint8_t pk  = fast_rand(PITCH_PACK_MAX + 1);
        const uint8_t acc = fast_rand(2) ? 0x40 : 0;
        const uint8_t sl  = fast_rand(2) ? 0x80 : 0;
        s.pitch[i] = pk | acc | sl;
      } else {
        s.pitch[i] = PITCH_DEFAULT;
      }
      s.set_ratchet_val(i, 0);
    }
    stale = true;
  }

  /// Randomize pattern but preserve ratchet bytes (`reserved[1..4]`).
  void RandomizeFullPatternKeepRatchets() {
    Sequence &s = get_sequence();
    uint8_t saved[4];
    for (uint8_t i = 0; i < 4; ++i) saved[i] = s.reserved[1 + i];
    RandomizeFullPattern();
    for (uint8_t i = 0; i < 4; ++i) s.reserved[1 + i] = saved[i];
    stale = true;
  }

  /// Rotate time data one step LEFT within `length` (pitches stay put).
  void RotateTimeLeft() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t first = s.time(0);
    for (uint8_t i = 0; i < len - 1; ++i)
      sequence_set_time_at(s, i, s.time(i + 1));
    sequence_set_time_at(s, len - 1, first);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Rotate time data one step RIGHT within `length` (pitches stay put).
  void RotateTimeRight() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t last = s.time(len - 1);
    for (uint8_t i = len - 1; i > 0; --i)
      sequence_set_time_at(s, i, s.time(i - 1));
    sequence_set_time_at(s, 0, last);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Randomize pitches only — keeps time data intact (PITCH_MODE).
  /// Ratchets are left alone (or cleared when a new slide is rolled, since
  /// slide steps cannot ratchet). Use RandomizeRatchetData for random ratchets.
  void RandomizePitchData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1) {
        const uint8_t pk  = fast_rand(PITCH_PACK_MAX + 1);
        const uint8_t acc = fast_rand(2) ? 0x40 : 0;
        const uint8_t sl  = fast_rand(2) ? 0x80 : 0;
        s.pitch[i] = pk | acc | sl;
        if (sl) s.set_ratchet_val(i, 0);
      }
    }
    stale = true;
  }

  /// Randomize time data only — pushes existing pitches forward into the new
  /// NOTE slots so a pattern's note content survives rhythmic re-roll.
  void RandomizeTimeData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;

    // Capture pre-change pitch stream (NOTE slots first, then stash tail).
    uint8_t stream[MAX_STEPS];
    uint8_t slen = 0;
    for (uint8_t i = 0; i < len; ++i)
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) stream[slen++] = s.pitch[i];
    const uint8_t old_stash = s.get_stash_count();
    for (uint8_t k = 0; k < old_stash && slen < MAX_STEPS; ++k) {
      const uint8_t sb = s.pitch[len + k];
      if (sb != PITCH_EMPTY) stream[slen++] = sb;
    }

    fast_rand_seed();
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      uint8_t t;
      if (i == 0)       t = fast_rand(2) ? 1 : 0;
      else if (prev==0) t = fast_rand(2) ? 1 : 0;
      else              t = fast_rand(3);
      sequence_set_time_at(s, i, t);
      if (t != 1) s.set_ratchet_val(i, 0);
      prev = t;
    }
    normalize_pattern_times(s);

    // Distribute the captured stream into the new NOTE slots; overflow -> stash.
    uint8_t ni = 0, note_count = 0;
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1) {
        s.pitch[i] = (ni < slen) ? stream[ni++] : PITCH_DEFAULT;
        ++note_count;
      } else {
        s.pitch[i] = PITCH_EMPTY;
      }
    }
    uint8_t excess = (slen > note_count) ? uint8_t(slen - note_count) : 0;
    const uint8_t max_stash = uint8_t(MAX_STEPS - len);
    if (excess > max_stash) excess = max_stash;
    for (uint8_t k = 0; k < excess; ++k)
      s.pitch[len + k] = stream[note_count + k];
    s.set_stash_count(excess);

    stale = true;
  }

  /// Reset all ratchet values to 1x (0) — keeps all other data intact.
  void ClearRatchetsOnly() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++)
      s.set_ratchet_val(i, 0);
    stale = true;
  }

  /// Randomize ratchet values only — keeps all other data intact.
  void RandomizeRatchetData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !(s.pitch[i] & 0x80)) {
        // NOTE step without slide: randomize ratchet
        s.set_ratchet_val(i, random_ratchet_val());
      } else {
        s.set_ratchet_val(i, 0);
      }
    }
    stale = true;
  }

  /// Randomize accent flags only (~20% probability) — keeps pitch, slide, and time intact.
  void RandomizeAccentData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (fast_rand(5) == 0)  // ~20%
          s.pitch[i] |=  0x40;
        else
          s.pitch[i] &= ~0x40;
      }
    }
    stale = true;
  }

  /// Mutate: genetic-style nudge. Perturbs a few steps of the current pattern
  /// without full re-randomization (flips a time nibble, re-rolls a pitch,
  /// toggles accent/slide). Keeps ratchets.
  void Mutate() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 1) return;
    fast_rand_seed();
    const uint8_t passes = uint8_t(2 + fast_rand(3)); // 2..4 tweaks
    for (uint8_t n = 0; n < passes; ++n) {
      const uint8_t i = fast_rand(len);
      const uint8_t action = fast_rand(5);
      switch (action) {
        case 0: {
          if (s.time(i) == 1) {
            const uint8_t pk = fast_rand(PITCH_PACK_MAX + 1);
            s.pitch[i] = (s.pitch[i] & 0xC0) | pk;
          }
          break;
        }
        case 1: {
          // Toggle accent on a NOTE step
          if (s.time(i) == 1 && !s.pitch_is_empty(i))
            s.pitch[i] ^= 0x40;
          break;
        }
        case 2: {
          // Toggle slide on a NOTE step (clear ratchet if slide gained)
          if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
            s.pitch[i] ^= 0x80;
            if (s.pitch[i] & 0x80) s.set_ratchet_val(i, 0);
          }
          break;
        }
        case 3: {
          // Nudge time nibble: rest<->note (avoid creating rest->tie issues)
          const uint8_t t = s.time(i);
          const uint8_t nt = (t == 0) ? 1 : (t == 1) ? 0 : 1;
          sequence_set_time_at(s, i, nt);
          break;
        }
        case 4: {
          // Nudge semitone up or down on a NOTE step
          if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
            uint8_t lin = unpack_pitch_linear(s.pitch[i] & 0x3F);
            const int dir = (fast_rand(2) ? 1 : -1);
            int nlin = int(lin) + dir;
            if (nlin < 0) nlin = 0;
            if (nlin > 48) nlin = 48;
            const uint8_t oct = uint8_t(nlin / 12);
            const uint8_t key = uint8_t(nlin - oct * 12);
            const uint8_t pk  = pack_pitch(key, oct);
            s.pitch[i] = (s.pitch[i] & 0xC0) | pk;
          }
          break;
        }
      }
    }
    normalize_pattern_times(s);
    stale = true;
  }

  /// Insert a REST time-step at the current time_pos, shifting subsequent
  /// time nibbles and pitch bytes right by one. Length grows by 1 (capped).
  void InsertTimeStep() {
    Sequence &s = get_sequence();
    const uint8_t gmax = max_steps_for_group(group_);
    if (s.length >= gmax) return;
    const uint8_t at = uint8_t(s.time_pos & (MAX_STEPS - 1));
    // Shift pitch bytes right from `at`
    for (int i = int(s.length); i > int(at); --i) {
      s.pitch[i] = s.pitch[i - 1];
    }
    // Shift time nibbles right from `at`
    for (int i = int(s.length); i > int(at); --i) {
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
    }
    // Shift ratchets right from `at`
    for (int i = int(s.length); i > int(at); --i) {
      s.set_ratchet_val(uint8_t(i), s.get_ratchet_val(uint8_t(i - 1)));
    }
    // Insert rest at `at`
    sequence_set_time_at(s, at, 0);
    s.pitch[at] = PITCH_EMPTY;
    s.set_ratchet_val(at, 0);
    s.length = uint8_t(s.length + 1);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Delete the time-step at the current time_pos, shifting subsequent
  /// time nibbles and pitch bytes left by one. Length shrinks by 1 (min 1).
  void DeleteTimeStep() {
    Sequence &s = get_sequence();
    if (s.length <= 1) return;
    const uint8_t at = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (at >= s.length) return;
    for (uint8_t i = at; i < s.length - 1; ++i) {
      s.pitch[i] = s.pitch[i + 1];
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
      s.set_ratchet_val(i, s.get_ratchet_val(uint8_t(i + 1)));
    }
    const uint8_t last = uint8_t(s.length - 1);
    s.pitch[last] = PITCH_EMPTY;
    sequence_set_time_at(s, last, 0);
    s.set_ratchet_val(last, 0);
    s.length = uint8_t(s.length - 1);
    if (s.time_pos >= s.length) s.time_pos = 0;
    if (s.pitch_pos >= s.length) s.pitch_pos = 0;
    normalize_pattern_times(s);
    stale = true;
  }

  /// Shift entire pattern (pitch + time together) one step LEFT within length.
  void ShiftPatternLeft() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t first_t = s.time(0);
    const uint8_t first_p = s.pitch[0];
    const uint8_t first_r = s.get_ratchet_val(0);
    for (uint8_t i = 0; i < len - 1; ++i) {
      s.pitch[i] = s.pitch[i + 1];
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
      s.set_ratchet_val(i, s.get_ratchet_val(uint8_t(i + 1)));
    }
    s.pitch[len - 1] = first_p;
    sequence_set_time_at(s, uint8_t(len - 1), first_t);
    s.set_ratchet_val(uint8_t(len - 1), first_r);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Shift entire pattern (pitch + time together) one step RIGHT within length.
  void ShiftPatternRight() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t last_t = s.time(uint8_t(len - 1));
    const uint8_t last_p = s.pitch[len - 1];
    const uint8_t last_r = s.get_ratchet_val(uint8_t(len - 1));
    for (int i = int(len - 1); i > 0; --i) {
      s.pitch[i] = s.pitch[i - 1];
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
      s.set_ratchet_val(uint8_t(i), s.get_ratchet_val(uint8_t(i - 1)));
    }
    s.pitch[0] = last_p;
    sequence_set_time_at(s, 0, last_t);
    s.set_ratchet_val(0, last_r);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Rotate pitch bytes only one step LEFT within length (time_data stays put).
  void RotatePitchLeft() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t first = s.pitch[0];
    for (uint8_t i = 0; i < len - 1; ++i) s.pitch[i] = s.pitch[i + 1];
    s.pitch[len - 1] = first;
    stale = true;
  }

  /// Rotate pitch bytes only one step RIGHT within length.
  void RotatePitchRight() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t last = s.pitch[len - 1];
    for (int i = int(len - 1); i > 0; --i) s.pitch[i] = s.pitch[i - 1];
    s.pitch[0] = last;
    stale = true;
  }

  /// Reverse the entire pattern (pitch + time) within length.
  void ReversePattern() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    for (uint8_t i = 0, j = uint8_t(len - 1); i < j; ++i, --j) {
      const uint8_t tp = s.pitch[i]; s.pitch[i] = s.pitch[j]; s.pitch[j] = tp;
      const uint8_t ti = s.time(i);
      const uint8_t tj = s.time(j);
      sequence_set_time_at(s, i, tj);
      sequence_set_time_at(s, j, ti);
      const uint8_t ri = s.get_ratchet_val(i);
      const uint8_t rj = s.get_ratchet_val(j);
      s.set_ratchet_val(i, rj);
      s.set_ratchet_val(j, ri);
    }
    normalize_pattern_times(s);
    stale = true;
  }

  /// Clear pitches only: wipe pitch[0..length-1] to PITCH_EMPTY, keep time data.
  void ClearPitchesOnly() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; ++i) s.pitch[i] = PITCH_EMPTY;
    s.set_stash_count(0);
    stale = true;
  }

  /// Clear time data only: set all steps within length to REST, keep pitches.
  void ClearTimesOnly() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; ++i) sequence_set_time_at(s, i, 0);
    stale = true;
  }

  /// Stamp accent on every NOTE step (or clear all if already fully stamped).
  /// Returns true if any step was changed.
  void StampAllAccent() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    bool all_set = true;
    uint8_t note_count = 0;
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        ++note_count;
        if (!(s.pitch[i] & 0x40)) { all_set = false; }
      }
    }
    if (note_count == 0) return;
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (all_set) s.pitch[i] &= ~0x40;
        else         s.pitch[i] |=  0x40;
      }
    }
    stale = true;
  }

  /// Stamp slide on every NOTE step (or clear all if already fully stamped).
  /// Ratchets are cleared on any step that gains a slide.
  void StampAllSlide() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    bool all_set = true;
    uint8_t note_count = 0;
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        ++note_count;
        if (!(s.pitch[i] & 0x80)) { all_set = false; }
      }
    }
    if (note_count == 0) return;
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (all_set) s.pitch[i] &= ~0x80;
        else       { s.pitch[i] |=  0x80; s.set_ratchet_val(i, 0); }
      }
    }
    stale = true;
  }

  /// Clear every pattern in the active group and reset the engine.
  void ClearAllPatterns() {
    for (uint8_t i = 0; i < 16; ++i) pattern[i].Clear();
    stale = true;
    Reset();
    mode_ = NORMAL_MODE;
  }

  /// Randomize semitone (key index 0..12) only. Preserves octave button,
  /// accent, and slide on every NOTE step. Non-NOTE steps untouched.
  void RandomizeSemitones() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        const uint8_t pk_old = s.pitch[i] & 0x3F;
        const uint8_t oct    = pk_old / 13;
        const uint8_t new_k  = fast_rand(PITCH_KEY_HIGH_C + 1);
        const uint8_t pk_new = pack_pitch(new_k, oct);
        s.pitch[i] = (s.pitch[i] & 0xC0) | pk_new;
      }
    }
    stale = true;
  }

  /// Randomize octave button (0..3) only. Preserves semitone, accent, slide.
  void RandomizeOctaves() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; ++i) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        const uint8_t pk_old = s.pitch[i] & 0x3F;
        const uint8_t key_i  = pk_old % 13;
        const uint8_t new_o  = fast_rand(4);
        const uint8_t pk_new = pack_pitch(key_i, new_o);
        s.pitch[i] = (s.pitch[i] & 0xC0) | pk_new;
      }
    }
    stale = true;
  }

  /// Nudge semitone +/-1 on the current pitch_pos step (PITCH_MODE edit).
  /// Clamps at linear 0 and 48. Preserves accent/slide.
  void NudgeSemitone(int dir) {
    Sequence &s = get_sequence();
    const uint8_t pp = uint8_t(s.pitch_pos & (MAX_STEPS - 1));
    if (s.time(pp) != 1 || s.pitch_is_empty(pp)) return;
    const uint8_t pk  = s.pitch[pp] & 0x3F;
    int lin = int(unpack_pitch_linear(pk)) + dir;
    if (lin < 0)  lin = 0;
    if (lin > 48) lin = 48;
    const uint8_t oct = uint8_t(lin / 12);
    const uint8_t key = uint8_t(lin - oct * 12);
    const uint8_t pk_new = pack_pitch(key, oct);
    s.pitch[pp] = (s.pitch[pp] & 0xC0) | pk_new;
    stale = true;
  }

  /// Nudge ratchet +/-1 on the current time_pos step. Cycles 0..2 (1x..3x).
  /// Slide steps can't ratchet; no-op if current step has slide.
  void NudgeRatchet(int dir) {
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (s.time(tp) != 1) return;
    if (!s.pitch_is_empty(tp) && (s.pitch[tp] & 0x80)) return;
    int r = int(s.get_ratchet_val(tp)) + dir;
    if (r < 0) r = 0;
    if (r > 2) r = 2;
    s.set_ratchet_val(tp, uint8_t(r));
    stale = true;
  }

  /// Set ratchet value (0..2 = 1x/2x/3x) on the current time_pos step directly.
  /// No-op if current step is not a NOTE or has slide set.
  void SetRatchetAtCurrent(uint8_t val) {
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (s.time(tp) != 1) return;
    if (!s.pitch_is_empty(tp) && (s.pitch[tp] & 0x80)) return;
    if (val > 2) val = 2;
    s.set_ratchet_val(tp, val);
    stale = true;
  }

  /// Randomize slide flags only (~15% probability) — keeps pitch, accent, and time intact.
  /// Clears ratchet on any step that gains a slide flag (slide and ratchet are mutually exclusive).
  void RandomizeSlideData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1 && !s.pitch_is_empty(i)) {
        if (fast_rand(20) < 3) { // ~15%
          s.pitch[i] |=  0x80;
          s.set_ratchet_val(i, 0);
        } else {
          s.pitch[i] &= ~0x80;
        }
      }
    }
    stale = true;
  }

  // Direction accessors
  /// Returns the "user-visible" direction: the pending one if a change is queued,
  /// otherwise the active one. Used for LED display so the newly-selected direction
  /// lights up immediately even if it hasn't taken effect yet.
  SequenceDirection get_direction() const {
    return direction_change_pending_ ? next_direction_ : direction_;
  }
  void SetDirection(SequenceDirection d) {
    stale = true;
    if (clk_count == -1) {
      // Clock not running: apply immediately (no wrap boundary to wait for).
      direction_ = d;
      pp_dir_ = 1;
      advance_count_ = 0;
      direction_change_pending_ = false;
    } else {
      // Clock running: queue the change to take effect at the next pattern-cycle
      // boundary, so the "1" (step 0 / downbeat) stays phase-locked to the clock.
      next_direction_ = d;
      direction_change_pending_ = true;
    }
  }

  void ClearPattern(uint8_t idx) {
    pattern[idx].Clear();
    stale = true;
    if (idx == p_select) {
      Reset();
      mode_ = NORMAL_MODE;
    }
  }

  // getters
  SequencerMode get_mode() const { return mode_; }

  Sequence &get_sequence() { return pattern[p_select]; }
  const Sequence &get_sequence() const { return pattern[p_select]; }
  const Sequence &get_pattern(uint8_t idx) const { return pattern[idx & 0xf]; }

  bool get_gate() const {
    if (resting) return false;
    // Slide destination: never ratchet — gate arrives via held CV from source.
    // Check before ratchet logic so a destination with a ratchet value set doesn't fire.
    // Steps that are both a destination AND a source hold the gate all 6 clocks.
    if (get_slide_dac()) return slide_gate ? true : (clk_count < 3);
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    // Slide source with no ratchet: hold gate HIGH all 6 clocks to bridge into destination.
    if (slide_gate && r == 0) return true;
    // Slide source with ratchet: use ratchet pattern but pin clock 5 HIGH so the gate
    // stays up through the step boundary and glides into the destination.
    switch (r) {
      case 1: return (uint8_t(clk_count) % 3u) == 0u || (slide_gate && clk_count == 5); // 2x
      case 2: return (uint8_t(clk_count) % 2u) == 0u || (slide_gate && clk_count == 5); // 3x
      default: break;                                  // r==0 (slide_gate false) falls through
    }
    return clk_count < 3;                              // normal note
  }

  /// Returns true when this MIDI clock tick is a ratchet retrigger within the current step
  /// (i.e., not the step-advance clock 0, but a subsequent sub-note trigger).
  /// Use this in the main loop alongside Clock() to fire midi_ratchet_retrigger().
  bool is_ratchet_retrigger() const {
    if (resting) return false;
    // Slide destination: never ratchet (being slid into, no new envelope trigger expected).
    if (get_slide_dac()) return false;
    // slide_gate true = source note of a slide; ratcheting is allowed on source notes.
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    switch (r) {
      case 1: return clk_count == 3;                    // 2x
      case 2: return clk_count == 2 || clk_count == 4; // 3x
      default: return false;                            // 1x or any stale val >2
    }
  }
  bool get_accent() const {
    // Accent belongs to the source note; suppress on slide destination (no new envelope trigger).
    return !resting && !get_slide_dac() && get_sequence().get_accent();
  }
  uint8_t get_semitone() const {
    return get_sequence().get_semitone();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch_dir(last_step_dir_);
  }
  // MIDI note number: OCTAVE_DOWN C = 36 (C2). Uses get_pitch_dir() so TIE steps in
  // any play direction resolve to the note that started the tie chain.
  uint8_t get_midi_note() const {
    return uint8_t(36 + get_sequence().get_pitch_dir(last_step_dir_));
  }
  bool get_slide() const {
    return get_sequence().get_slide();
  }
  uint8_t get_time_pos() const {
    return get_sequence().time_pos;
  }
  uint8_t get_patsel() const {
    return p_select;
  }
  uint8_t get_next() const {
    return next_p;
  }
  const uint8_t get_time() const {
    return get_sequence().get_time();
  }
  const uint8_t get_length() const {
    return get_sequence().length;
  }

  // setters
  void SetPattern(uint8_t p_, bool override = false) {
    next_p = p_ & 0xf;
    if (override) {
      p_select = next_p;
    }
  }
  void SetLength(uint8_t len) {
    get_sequence().SetLength(len, max_steps_for_group(group_));
    stale = true;
  }
  bool BumpLength() {
    stale = true;
    return get_sequence().BumpLength(max_steps_for_group(group_));
  }
  void SetMode(SequencerMode m, bool reset = false) {
    if (reset && m != mode_) Reset();
    mode_ = m;
  }
  void NudgeOctave(int dir) {
    get_sequence().nudge_octave_buttons(dir);
    stale = true;
  }
  // change pitch, preserving flags
  void SetPitchSemitone(uint8_t p) {
    get_sequence().SetPitchSemitone(p);
    stale = true;
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    get_sequence().SetPitch(p, flags);
    stale = true;
  }
  void SetTime(uint8_t t) {
    get_sequence().SetTime(t);
    stale = true;
  }
  void ToggleSlide() {
    if (mode_ == PITCH_MODE)
      get_sequence().ToggleSlide();
    stale = true;
  }
  void ToggleAccent() {
    if (mode_ == PITCH_MODE)
      get_sequence().ToggleAccent();
    stale = true;
  }

  /// Step back one position in the current edit mode (linear, both TIME and PITCH).
  bool StepBack() {
    bool moved = get_sequence().StepBack();
    if (moved) stale = true;
    return moved;
  }

  /// True when the edited step is note-locked (bit set from TIME_MODE).
  bool is_step_locked() const {
    const Sequence &s = get_sequence();
    if (mode_ == TIME_MODE)
      return s.step_locked(uint8_t(s.time_pos));
    if (mode_ == PITCH_MODE)
      return s.step_locked(uint8_t(s.pitch_pos));
    return false;
  }
  void ToggleStepLockFromTimeMode() {
    if (mode_ != TIME_MODE) return;
    get_sequence().ToggleStepLock(uint8_t(get_sequence().time_pos));
    stale = true;
  }

  /// EEPROM pattern image: 128 bytes from `pitch[0]` through `length` (see WritePattern).
  void export_pattern_blob(uint8_t idx, uint8_t *blob128) const {
    idx &= 0xf;
    memcpy(blob128, pattern[idx].pitch, PATTERN_SIZE);
  }

  /// Replace pattern RAM from host SysEx. If `persist_eeprom`, also write EEPROM.
  bool import_pattern_blob(uint8_t idx, const uint8_t *blob128, bool persist_eeprom = true) {
    idx &= 0xf;
    uint8_t L = blob128[PATTERN_SIZE - 1];
    if (L < 1 || uint8_t(L) > MAX_STEPS)
      return false;
    const uint8_t gmax = max_steps_for_group(group_);
    if (L > gmax) L = gmax;
    memcpy(pattern[idx].pitch, blob128, PATTERN_SIZE);
    pattern[idx].length = L;
    Sequence &s = pattern[idx];
    normalize_pattern_times(s);
    if (s.pitch_pos >= s.length) s.pitch_pos %= s.length;
    if (s.time_pos  >= s.length) s.time_pos  %= s.length;
    if (persist_eeprom)
      WritePattern(pattern[idx], idx, group_);
    return true;
  }

  /// MIDI Note On → current playing pitch step (live). `note` 36..84 → linear 0..48 (matches CV / pack range).
  void midi_apply_note_on(uint8_t note, uint8_t velocity) {
    if (note < 36 || note > 36 + 48)
      return;
    uint8_t lin = uint8_t(note - 36);
    if (lin > 48)
      lin = 48;
    uint8_t oct_btn = lin / 12;
    if (oct_btn > 3)
      oct_btn = 3;
    uint8_t key_idx = uint8_t(lin - oct_btn * 12);
    if (key_idx > PITCH_KEY_HIGH_C)
      key_idx = PITCH_KEY_HIGH_C;
    const uint8_t pk = pack_pitch(key_idx, oct_btn);
    const uint8_t acc = (velocity >= 100) ? uint8_t(1u << 6) : 0;
    Sequence &s = get_sequence();
    const uint8_t st = uint8_t(s.pitch_pos) & (MAX_STEPS - 1);
    const uint8_t slide = s.pitch_is_empty(st) ? 0 : (s.pitch[st] & (1u << 7));
    s.pitch[st] = (pk & 0x3f) | slide | acc;
    stale = true;
  }
};
