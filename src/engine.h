// Copyright (c) 2026, Nicholas J. Michalek
/*
 * engine.h -- TB-303 pattern model + EEPROM; Engine handles patterns, clock, and gate.
 *
 * EEPROM versions:
 *   "PewPewPew!!2" -- original: pitch slots packed sequentially by NOTE event
 *   "PewPewPew!!3" -- 1:1 pitch/time model, 64-step, 16 patterns
 *   "PewPewPew!!4" -- 1:1 pitch/time model, 16-step max, 4 groups x 16 patterns
 *   "PewPewPew!!5" -- groups 0-2: 16-step compact; group 3: 64-step full blob (1:1 model)
 *   "PewPewPew!!6" -- two-stream model: pitch[] in NOTE-event order, pitch_count byte
 *   "superOS-2bit"  -- time_data shrunk from 4-bit nibbles to 2-bit cells
 *                      (8 B/pattern instead of 16 B). PATTERN_SIZE 56->48.
 *                      Wipes EEPROM on first boot since offsets shifted.
 */

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#include "persistent_settings.h"

// =============================================================================
// Engine -- patterns + clock + gate
// =============================================================================
// =============================================================================
// Tracks (OS-303 v0.6 wire format).
// 8 tracks total, mapped to banks via track >> 1 (tracks 0-1 -> bank 0, ..., 6-7 -> bank 3).
// Per-track storage: 32 bytes = p_chain[16] (low 4 bits pattern# | high 4 bits repeats)
// + t_chain[16] (low 7 bits transpose | bit 7 last-step flag).
// EEPROM offset for track t: TRACK_DATA_OFFSET + t*32, with p_chain at +0 and
// t_chain at +16.
// =============================================================================
static constexpr uint8_t MAX_CHAIN  = 16;
static constexpr uint8_t NUM_TRACKS = 8;
static constexpr uint8_t T_CHAIN_LAST_STEP_FLAG = 0x80;
static constexpr uint8_t T_CHAIN_TRANSPOSE_MASK = 0x7F;

struct Engine {
  Sequence pattern[NUM_PATTERNS];
  uint8_t p_select = 0;
  uint8_t next_p = 0;
  uint8_t group_ = 0;
  uint8_t pending_group_ = 0xff;

  // Track-mode state
  uint8_t p_chain[MAX_CHAIN]    = {0};
  uint8_t t_chain[MAX_CHAIN]    = {0};
  uint8_t p_chain_len = 0;        // number of valid chain steps (0 = no track loaded)
  uint8_t p_chain_pos = 0;        // current chain step index
  int8_t  p_repeats = -1;         // -1 = uninitialized; else current repeat count of current step
  uint8_t track_select = 0;       // which track (0..7) is currently loaded
  bool    track_stale = false;    // track-data dirty flag (separate from `stale`)
  bool    track_active = false;   // true while in TrackPlay/TrackWrite playback

  SequencerMode      mode_      = NORMAL_MODE;
  SequenceDirection  direction_ = DIR_FORWARD;
  int8_t  pp_dir_        = 1;
  uint8_t advance_count_ = 0;
  SequenceDirection  next_direction_          = DIR_FORWARD;
  bool               direction_change_pending_ = false;
  int8_t last_step_dir_ = 1;

  int8_t clk_count = -1;

  bool slide_gate = false;
  bool stale = false;
  bool resting = false;

  uint32_t step_start_us_ = 0;

  uint8_t get_group() const { return group_; }
  uint8_t get_pending_group() const { return pending_group_; }

  void QueueGroup(uint8_t g) {
    if (g < NUM_GROUPS) pending_group_ = g;
  }

  void apply_pending_group() {
    if (pending_group_ == 0xff || pending_group_ == group_) { pending_group_ = 0xff; return; }
    // Earlier this forced `stale = true; Save();` to flush edits before the
    // group swap. That's a 16-pattern EEPROM burn (~25-100 ms) which lags the
    // sequencer audibly when group switches happen during playback. Per user
    // spec, saves only happen on clock stop -- if the user switches groups
    // without stopping, edits to the outgoing group are discarded.
    group_ = pending_group_;
    pending_group_ = 0xff;
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
      ReadPattern(pattern[i], i, group_);
      if (!pattern[i].length) pattern[i].SetLength(8);
      sequence_rebuild_pitch_count(pattern[i]);
      normalize_pattern_times(pattern[i]);
    }
    stale = false;
  }

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
        sequence_rebuild_pitch_count(pattern[i]);
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
      memcpy(GlobalSettings.signature, sig_pew, kSigEepromLen);
      GlobalSettings.Save();
      GlobalSettings.save_midi_to_storage();
      // Wipe all 4 banks, not just the active one. Old EEPROM had time_data
      // at different offsets; without this, switching to bank 1/2/3 reads
      // garbage at the new offsets.
      for (uint8_t b = 0; b < NUM_BANKS; ++b)
        for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
          WritePattern(pattern[i], i, b);
      stale = false;
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

  // ---------------------------------------------------------------------------
  // Track storage (OS-303 v0.6 layout)
  // ---------------------------------------------------------------------------
  void LoadTrack(uint8_t track) {
    track &= (NUM_TRACKS - 1);
    track_select = track;
    const int base = TRACK_DATA_OFFSET + (int(track) * 32);
    storage.get(base,                p_chain);
    storage.get(base + int(MAX_CHAIN), t_chain);
    // OS-303's invalid-data sentinel: t_chain[0] == 0xFF -> treat as uninit.
    if (t_chain[0] == 0xFF) {
      memset(p_chain, 0, sizeof(p_chain));
      memset(t_chain, 0, sizeof(t_chain));
      p_chain_len = 0;
    } else {
      // Length: first index whose top bit is set, plus 1.
      p_chain_len = 0;
      for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
        if (t_chain[i] & T_CHAIN_LAST_STEP_FLAG) { p_chain_len = uint8_t(i + 1); break; }
      }
    }
    p_chain_pos = 0;
    p_repeats   = -1;
    track_stale = false;
  }
  void SaveTrack() {
    if (!track_stale) return;
    // Write the last-step marker into t_chain based on p_chain_len.
    for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
      if (p_chain_len > 0 && i == p_chain_len - 1)
        t_chain[i] |= T_CHAIN_LAST_STEP_FLAG;
      else
        t_chain[i] &= T_CHAIN_TRANSPOSE_MASK;
    }
    const int base = TRACK_DATA_OFFSET + (int(track_select) * 32);
    storage.put(base,                p_chain);
    storage.put(base + int(MAX_CHAIN), t_chain);
    track_stale = false;
  }
  uint8_t get_track_select() const { return track_select; }
  uint8_t get_chain_len()    const { return p_chain_len; }
  uint8_t get_chain_pos()    const { return p_chain_pos; }

  // Returns true if track has any chain steps.
  bool track_has_chain() const { return p_chain_len > 0; }

  // Track Write helpers --------------------------------------------------------
  // Write the active pattern + repeats into the current chain step. Repeats
  // are 0..15; 0 = play once, 15 = play 16 times.
  void TrackWriteCurrentStep(uint8_t pattern_in_bank, uint8_t repeats) {
    if (p_chain_pos >= MAX_CHAIN) return;
    p_chain[p_chain_pos] = uint8_t((pattern_in_bank & 0x0F) | ((repeats & 0x0F) << 4));
    if (p_chain_pos + 1 > p_chain_len) p_chain_len = uint8_t(p_chain_pos + 1);
    track_stale = true;
  }
  // Mark current chain step as the last (D.C. equivalent). Sets bit 7 of
  // t_chain[p_chain_pos], clears it elsewhere; truncates p_chain_len.
  void TrackMarkLastStep() {
    if (p_chain_pos >= MAX_CHAIN) return;
    for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
      if (i == p_chain_pos) t_chain[i] |= T_CHAIN_LAST_STEP_FLAG;
      else                  t_chain[i] &= T_CHAIN_TRANSPOSE_MASK;
    }
    p_chain_len = uint8_t(p_chain_pos + 1);
    track_stale = true;
  }
  // Set per-chain-step transpose (low 7 bits, preserves last-step flag).
  void TrackSetTranspose(uint8_t chain_step, uint8_t transpose_semi) {
    if (chain_step >= MAX_CHAIN) return;
    const uint8_t flag = t_chain[chain_step] & T_CHAIN_LAST_STEP_FLAG;
    t_chain[chain_step] = flag | (transpose_semi & T_CHAIN_TRANSPOSE_MASK);
    track_stale = true;
  }
  uint8_t TrackGetRepeats(uint8_t chain_step) const {
    if (chain_step >= MAX_CHAIN) return 0;
    return uint8_t((p_chain[chain_step] >> 4) & 0x0F);
  }
  uint8_t TrackGetPattern(uint8_t chain_step) const {
    if (chain_step >= MAX_CHAIN) return 0;
    return uint8_t(p_chain[chain_step] & 0x0F);
  }
  uint8_t TrackGetTranspose(uint8_t chain_step) const {
    if (chain_step >= MAX_CHAIN) return 0;
    return uint8_t(t_chain[chain_step] & T_CHAIN_TRANSPOSE_MASK);
  }
  void TrackResetCursor() { p_chain_pos = 0; p_repeats = -1; }

  // Called at pattern wrap (time_pos returns to 0). Advances the chain cursor
  // when the current chain step's repeat count is exhausted, and updates
  // next_p so the engine's existing pattern-switch logic picks up the change.
  // Repeat semantics match OS-303: high nibble of p_chain[i] = repeat count
  // (0..15). repeats=0 means "play once before advancing", 1 = "play twice", etc.
  void track_advance_chain() {
    if (!track_active || p_chain_len == 0) return;
    const uint8_t step_repeats = uint8_t((p_chain[p_chain_pos] >> 4) & 0x0F);
    if (p_repeats < 0) {
      // First wrap after starting the track: stay on chain step 0, just init.
      p_repeats = 0;
    } else if (uint8_t(p_repeats) >= step_repeats) {
      // Done with this chain step: advance to next, wrap to 0 at end of chain.
      p_chain_pos = uint8_t((p_chain_pos + 1) % p_chain_len);
      p_repeats = 0;
    } else {
      ++p_repeats;
    }
    next_p = uint8_t(p_chain[p_chain_pos] & 0x0F);
  }
  void TrackAdvanceCursor() {
    if (p_chain_pos + 1 < MAX_CHAIN) ++p_chain_pos;
  }
  void TrackClear() {
    memset(p_chain, 0, sizeof(p_chain));
    memset(t_chain, 0, sizeof(t_chain));
    p_chain_len = 0;
    p_chain_pos = 0;
    p_repeats   = -1;
    track_stale = true;
  }

  void Tick() {}

  void SyncAfterManualAdvance(bool) { step_start_us_ = micros(); }

  bool get_slide_dac() const {
    return get_sequence().slide_from_prev_dir(uint8_t(direction_), last_step_dir_);
  }

  bool Advance() {
    bool result;
    int8_t step_dir     = 1;
    int8_t next_step_dir = 1;

    if (direction_ == DIR_FORWARD) {
      step_dir     = 1;
      next_step_dir = 1;
      result = get_sequence().Advance();
      if (0 == get_sequence().time_pos && !get_sequence().first_step) {
        if (direction_change_pending_) {
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
        }
        apply_pending_group();
        // Track-mode chain advance: updates next_p when current step's repeat
        // count is exhausted. The pattern-switch logic below picks it up.
        track_advance_chain();
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
      if (direction_ == DIR_REVERSE)                                     step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) step_dir = pp_dir_;
      else                                                               step_dir = 1;

      result = get_sequence().AdvanceDirectional(uint8_t(direction_), pp_dir_);

      if (direction_ == DIR_REVERSE)                                     next_step_dir = -1;
      else if (direction_ == DIR_PINGPONG || direction_ == DIR_BROWNIAN) next_step_dir = pp_dir_;
      else                                                               next_step_dir = 1;

      ++advance_count_;
      if (advance_count_ >= get_sequence().length) {
        advance_count_ = 0;
        if (direction_change_pending_) {
          const SequenceDirection old_dir = direction_;
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
          if (old_dir == DIR_PINGPONG) {
            if (direction_ == DIR_FORWARD) {
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
              result        = get_sequence().time(0) != 0;
              step_dir      = 1;
              next_step_dir = 1;
            } else if (direction_ == DIR_REVERSE) {
              get_sequence().time_pos  = 0;
              get_sequence().pitch_pos = 0;
              get_sequence().first_step = true;
            } else {
              get_sequence().Reset();
            }
          }
        }
        apply_pending_group();
        // Track-mode chain advance (matches forward path).
        track_advance_chain();
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
      const bool next_is_tie = get_sequence().is_tied_dir(uint8_t(direction_), next_step_dir);
      const bool tie_slide   = get_sequence().is_tie() &&
                               get_sequence().slide_from_prev_dir(uint8_t(direction_), step_dir);
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

  // Clock period per step:
  //   16ths (default): 24 PPQN / 4 sixteenths-per-beat = 6 ticks per step.
  //   Triplets:        24 PPQN / 3 triplets-per-beat   = 8 ticks per step.
  // Step mode is per-pattern, stored as a flag bit in reserved[0].
  uint8_t step_period() const {
    return get_sequence().is_triplet_mode() ? 8 : 6;
  }
  bool Clock() {
    const uint8_t period = step_period();
    if (++clk_count >= int8_t(period)) clk_count = 0;
    if (clk_count == 0) {
      Advance();
      step_start_us_ = micros();
      return true;
    }
    return false;
  }

  void Reset() {
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

  static uint8_t random_ratchet_val() { return fast_rand_ratchet_weighted(); }

  // ---------------------------------------------------------------------------
  // Bulk ops on the joint (time, pitch) representation.
  // Each captures per-time-step pitch via sequence_pack_per_time, mutates the
  // time stream + the per-time buffer in lockstep, then rebuilds pitch[] +
  // pitch_count via sequence_unpack_per_time.
  // ---------------------------------------------------------------------------

  /// Randomize entire pattern: random time + random pitches in stream order.
  /// Ratchets cleared.
  void RandomizeFullPattern() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      const uint8_t t = fast_rand_time_weighted(prev, i == 0);
      sequence_set_time_at(s, i, t);
      s.set_ratchet_val(i, 0);
      prev = t;
    }
    normalize_pattern_times_only(s);
    const uint8_t nc = s.note_count();
    for (uint8_t k = 0; k < nc; ++k) {
      s.pitch[k] = fast_rand_pitch_byte_weighted()
                   | fast_rand_accent_weighted()
                   | fast_rand_slide_weighted();
    }
    for (uint8_t k = nc; k < MAX_STEPS; ++k) s.pitch[k] = PITCH_EMPTY;
    s.set_pitch_count(nc);
    stale = true;
  }

  void RandomizeFullPatternKeepRatchets() {
    Sequence &s = get_sequence();
    uint8_t saved[4];
    for (uint8_t i = 0; i < 4; ++i) saved[i] = s.reserved[1 + i];
    RandomizeFullPattern();
    for (uint8_t i = 0; i < 4; ++i) s.reserved[1 + i] = saved[i];
    stale = true;
  }

  /// Rotate time data only. Pitch stream stays in place; new NOTE positions
  /// consume pitches in stream order (TB-303 independent-stream semantic).
  void RotateTimeLeft() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t ft = s.time(0);
    for (uint8_t i = 0; i < len - 1; ++i)
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
    sequence_set_time_at(s, len - 1, ft);
    normalize_pattern_times_only(s);
    sequence_ensure_pitch_for_notes(s);
    stale = true;
  }

  void RotateTimeRight() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    const uint8_t lt = s.time(uint8_t(len - 1));
    for (int i = int(len - 1); i > 0; --i)
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
    sequence_set_time_at(s, 0, lt);
    normalize_pattern_times_only(s);
    sequence_ensure_pitch_for_notes(s);
    stale = true;
  }

  /// Randomize pitches only - keeps time data; per-NOTE-slot random pitch.
  void RandomizePitchData() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    for (uint8_t k = 0; k < pc; ++k) {
      s.pitch[k] = fast_rand_pitch_byte_weighted()
                   | fast_rand_accent_weighted()
                   | fast_rand_slide_weighted();
    }
    // Slide steps cannot ratchet - clear ratchets for any NOTE step that
    // gained slide. Ratchet storage is per-time-step.
    uint8_t k = 0;
    for (uint8_t i = 0; i < s.length; ++i) {
      if (s.time(i) == 1) {
        if (k < pc && (s.pitch[k] & 0x80)) s.set_ratchet_val(i, 0);
        ++k;
      }
    }
    stale = true;
  }

  /// Randomize time data only. Existing pitches are preserved in stream order;
  /// pitch_count auto-extends if the new note_count exceeds it.
  void RandomizeTimeData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      const uint8_t t = fast_rand_time_weighted(prev, i == 0);
      sequence_set_time_at(s, i, t);
      if (t != 1) s.set_ratchet_val(i, 0);
      prev = t;
    }
    normalize_pattern_times_only(s);
    sequence_ensure_pitch_for_notes(s);
    stale = true;
  }

  void ClearRatchetsOnly() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; i++) s.set_ratchet_val(i, 0);
    stale = true;
  }

  void RandomizeRatchetData() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    uint8_t k = 0;
    const uint8_t pc = s.get_pitch_count();
    for (uint8_t i = 0; i < len; i++) {
      if (s.time(i) == 1) {
        const uint8_t pb = (k < pc) ? s.pitch[k] : PITCH_EMPTY;
        ++k;
        if (pb != PITCH_EMPTY && !(pb & 0x80))
          s.set_ratchet_val(i, random_ratchet_val());
        else
          s.set_ratchet_val(i, 0);
      } else {
        s.set_ratchet_val(i, 0);
      }
    }
    stale = true;
  }

  void RandomizeAccentData() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      const uint8_t acc = fast_rand_accent_weighted();
      s.pitch[k] = (s.pitch[k] & ~uint8_t(0x40)) | acc;
    }
    stale = true;
  }

  void RandomizeSlideData() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    uint8_t k = 0;
    for (uint8_t i = 0; i < s.length; ++i) {
      if (s.time(i) == 1) {
        if (k < pc && s.pitch[k] != PITCH_EMPTY) {
          const uint8_t sl = fast_rand_slide_weighted();
          s.pitch[k] = (s.pitch[k] & ~uint8_t(0x80)) | sl;
          if (sl) s.set_ratchet_val(i, 0);
        }
        ++k;
      }
    }
    stale = true;
  }

  void Mutate() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 1) return;
    fast_rand_seed();
    const uint8_t passes = uint8_t(2 + fast_rand(3));
    for (uint8_t n = 0; n < passes; ++n) {
      const uint8_t i = fast_rand(len);
      const uint8_t action = fast_rand(5);
      const uint8_t t_i = s.time(i);
      switch (action) {
        case 0: {
          // Reroll pitch on a NOTE step.
          if (t_i == 1) {
            const uint8_t slot = s.pitch_index_for_note(i);
            if (slot < s.get_pitch_count()) {
              const uint8_t pk = fast_rand_pitch_byte_weighted();
              s.pitch[slot] = (s.pitch[slot] & 0xC0) | pk;
            }
          }
          break;
        }
        case 1: {
          if (t_i == 1) {
            const uint8_t slot = s.pitch_index_for_note(i);
            if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY)
              s.pitch[slot] ^= 0x40;
          }
          break;
        }
        case 2: {
          if (t_i == 1) {
            const uint8_t slot = s.pitch_index_for_note(i);
            if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY) {
              s.pitch[slot] ^= 0x80;
              if (s.pitch[slot] & 0x80) s.set_ratchet_val(i, 0);
            }
          }
          break;
        }
        case 3: {
          // Flip rest <-> note (avoid creating rest->tie issues at this step).
          const uint8_t nt = (t_i == 0) ? 1 : (t_i == 1) ? 0 : 1;
          sequence_write_time_with_pitch_sync(s, i, nt);
          if (nt != 1) s.set_ratchet_val(i, 0);
          break;
        }
        case 4: {
          if (t_i == 1) {
            const uint8_t slot = s.pitch_index_for_note(i);
            if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY) {
              uint8_t lin = unpack_pitch_linear(s.pitch[slot] & 0x3F);
              const int dirN = (fast_rand(2) ? 1 : -1);
              int nlin = int(lin) + dirN;
              if (nlin < 0) nlin = 0;
              if (nlin > 48) nlin = 48;
              const uint8_t oct = uint8_t(nlin / 12);
              const uint8_t key = uint8_t(nlin - oct * 12);
              s.pitch[slot] = (s.pitch[slot] & 0xC0) | pack_pitch(key, oct);
            }
          }
          break;
        }
      }
    }
    normalize_pattern_times(s);
    stale = true;
  }

  /// Insert a REST time-step at the current time_pos, shifting later time nibbles
  /// and ratchets right. Pitch stream is left untouched.
  void InsertTimeStep() {
    Sequence &s = get_sequence();
    const uint8_t gmax = MAX_STEPS;
    if (s.length >= gmax) return;
    const uint8_t at = uint8_t(s.time_pos & (MAX_STEPS - 1));
    for (int i = int(s.length); i > int(at); --i)
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
    for (int i = int(s.length); i > int(at); --i)
      s.set_ratchet_val(uint8_t(i), s.get_ratchet_val(uint8_t(i - 1)));
    sequence_set_time_at(s, at, 0);
    s.set_ratchet_val(at, 0);
    s.length = uint8_t(s.length + 1);
    normalize_pattern_times(s);
    stale = true;
  }

  /// Delete the time-step at the current time_pos. Pitch stream is left
  /// untouched; the deleted NOTE's pitch is preserved as a queued slot, so
  /// re-adding a NOTE elsewhere replays the same pitch in stream order.
  void DeleteTimeStep() {
    Sequence &s = get_sequence();
    if (s.length <= 1) return;
    const uint8_t at = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (at >= s.length) return;
    for (uint8_t i = at; i < s.length - 1; ++i) {
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
      s.set_ratchet_val(i, s.get_ratchet_val(uint8_t(i + 1)));
    }
    const uint8_t last = uint8_t(s.length - 1);
    sequence_set_time_at(s, last, 0);
    s.set_ratchet_val(last, 0);
    s.length = uint8_t(s.length - 1);
    if (s.time_pos >= s.length) s.time_pos = 0;
    normalize_pattern_times(s);
    stale = true;
  }

  /// Shift entire pattern (pitch + time + ratchets) one step LEFT within length.
  void ShiftPatternLeft() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    uint8_t per_time[MAX_STEPS];
    sequence_pack_per_time(s, per_time);
    const uint8_t ft = s.time(0);
    const uint8_t fp = per_time[0];
    const uint8_t fr = s.get_ratchet_val(0);
    for (uint8_t i = 0; i < len - 1; ++i) {
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
      per_time[i] = per_time[i + 1];
      s.set_ratchet_val(i, s.get_ratchet_val(uint8_t(i + 1)));
    }
    sequence_set_time_at(s, uint8_t(len - 1), ft);
    per_time[len - 1] = fp;
    s.set_ratchet_val(uint8_t(len - 1), fr);
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  void ShiftPatternRight() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    uint8_t per_time[MAX_STEPS];
    sequence_pack_per_time(s, per_time);
    const uint8_t lt = s.time(uint8_t(len - 1));
    const uint8_t lp = per_time[len - 1];
    const uint8_t lr = s.get_ratchet_val(uint8_t(len - 1));
    for (int i = int(len - 1); i > 0; --i) {
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
      per_time[i] = per_time[i - 1];
      s.set_ratchet_val(uint8_t(i), s.get_ratchet_val(uint8_t(i - 1)));
    }
    sequence_set_time_at(s, 0, lt);
    per_time[0] = lp;
    s.set_ratchet_val(0, lr);
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  /// Rotate pitch stream only (NOTE-event order) one slot left.
  void RotatePitchLeft() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc < 2) return;
    const uint8_t first = s.pitch[0];
    for (uint8_t k = 0; k + 1 < pc; ++k) s.pitch[k] = s.pitch[k + 1];
    s.pitch[pc - 1] = first;
    stale = true;
  }
  void RotatePitchRight() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc < 2) return;
    const uint8_t last = s.pitch[pc - 1];
    for (int k = int(pc - 1); k > 0; --k) s.pitch[k] = s.pitch[k - 1];
    s.pitch[0] = last;
    stale = true;
  }

  /// Reverse the entire pattern (pitch + time + ratchets) within length.
  void ReversePattern() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    uint8_t per_time[MAX_STEPS];
    sequence_pack_per_time(s, per_time);
    for (uint8_t i = 0, j = uint8_t(len - 1); i < j; ++i, --j) {
      const uint8_t ti = s.time(i);
      const uint8_t tj = s.time(j);
      sequence_set_time_at(s, i, tj);
      sequence_set_time_at(s, j, ti);
      const uint8_t pti = per_time[i];
      per_time[i] = per_time[j];
      per_time[j] = pti;
      const uint8_t ri = s.get_ratchet_val(i);
      const uint8_t rj = s.get_ratchet_val(j);
      s.set_ratchet_val(i, rj);
      s.set_ratchet_val(j, ri);
    }
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  void ClearPitchesOnly() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    for (uint8_t k = 0; k < pc; ++k) s.pitch[k] = PITCH_DEFAULT;
    stale = true;
  }

  void ClearTimesOnly() {
    Sequence &s = get_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; ++i) sequence_set_time_at(s, i, 0);
    // Pitch stream preserved (queued for when NOTEs come back).
    stale = true;
  }

  void StampAllAccent() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc == 0) return;
    bool all_set = true;
    uint8_t valid_count = 0;
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      ++valid_count;
      if (!(s.pitch[k] & 0x40)) all_set = false;
    }
    if (valid_count == 0) return;
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      if (all_set) s.pitch[k] &= ~0x40;
      else         s.pitch[k] |=  0x40;
    }
    stale = true;
  }

  void StampAllSlide() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc == 0) return;
    bool all_set = true;
    uint8_t valid_count = 0;
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      ++valid_count;
      if (!(s.pitch[k] & 0x80)) all_set = false;
    }
    if (valid_count == 0) return;
    // Walk pitch slots and clear ratchet on the matching time-step for any NOTE
    // that gains slide. Need a slot->time mapping: walk time_data in order.
    uint8_t k = 0;
    for (uint8_t i = 0; i < s.length && k < pc; ++i) {
      if (s.time(i) == 1) {
        if (s.pitch[k] != PITCH_EMPTY) {
          if (all_set) s.pitch[k] &= ~0x80;
          else       { s.pitch[k] |=  0x80; s.set_ratchet_val(i, 0); }
        }
        ++k;
      }
    }
    stale = true;
  }

  void ClearAllPatterns() {
    for (uint8_t i = 0; i < 16; ++i) pattern[i].Clear();
    stale = true;
    Reset();
    mode_ = NORMAL_MODE;
  }

  void RandomizeSemitones() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      const uint8_t pk_old = s.pitch[k] & 0x3F;
      const uint8_t oct    = pk_old / 13;
      const uint8_t new_k  = fast_rand(PITCH_KEY_HIGH_C + 1);
      const uint8_t pk_new = pack_pitch(new_k, oct);
      s.pitch[k] = (s.pitch[k] & 0xC0) | pk_new;
    }
    stale = true;
  }

  void RandomizeOctaves() {
    Sequence &s = get_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    for (uint8_t k = 0; k < pc; ++k) {
      if (s.pitch[k] == PITCH_EMPTY) continue;
      const uint8_t pk_old = s.pitch[k] & 0x3F;
      const uint8_t key_i  = pk_old % 13;
      const uint8_t new_o  = fast_rand(4);
      s.pitch[k] = (s.pitch[k] & 0xC0) | pack_pitch(key_i, new_o);
    }
    stale = true;
  }

  void NudgeSemitone(int dir) {
    Sequence &s = get_sequence();
    uint8_t slot;
    if (!s.edit_slot_index(slot)) return;
    if (s.pitch[slot] == PITCH_EMPTY) return;
    const uint8_t pk = s.pitch[slot] & 0x3F;
    int lin = int(unpack_pitch_linear(pk)) + dir;
    if (lin < 0)  lin = 0;
    if (lin > 48) lin = 48;
    const uint8_t oct = uint8_t(lin / 12);
    const uint8_t key = uint8_t(lin - oct * 12);
    s.pitch[slot] = (s.pitch[slot] & 0xC0) | pack_pitch(key, oct);
    stale = true;
  }

  void NudgeRatchet(int dir) {
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (s.time(tp) != 1) return;
    const uint8_t slot = s.pitch_index_for_note(tp);
    if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY && (s.pitch[slot] & 0x80))
      return;
    int r = int(s.get_ratchet_val(tp)) + dir;
    if (r < 0) r = 0;
    if (r > 2) r = 2;
    s.set_ratchet_val(tp, uint8_t(r));
    stale = true;
  }

  void SetRatchetAtCurrent(uint8_t val) {
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (s.time(tp) != 1) return;
    const uint8_t slot = s.pitch_index_for_note(tp);
    if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY && (s.pitch[slot] & 0x80))
      return;
    if (val > 2) val = 2;
    s.set_ratchet_val(tp, val);
    stale = true;
  }

  // ---------------------------------------------------------------------------
  // Direction
  // ---------------------------------------------------------------------------
  SequenceDirection get_direction() const {
    return direction_change_pending_ ? next_direction_ : direction_;
  }
  void SetDirection(SequenceDirection d) {
    stale = true;
    if (clk_count == -1) {
      direction_ = d;
      pp_dir_ = 1;
      advance_count_ = 0;
      direction_change_pending_ = false;
    } else {
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

  // ---------------------------------------------------------------------------
  // Getters
  // ---------------------------------------------------------------------------
  SequencerMode get_mode() const { return mode_; }

  Sequence &get_sequence() { return pattern[p_select]; }
  const Sequence &get_sequence() const { return pattern[p_select]; }
  const Sequence &get_pattern(uint8_t idx) const { return pattern[idx & 0xf]; }

  // Gate window: 50% of the step period in 16ths (clk_count < 3 of 6) and in
  // triplets (clk_count < 4 of 8). With 1-bit ratchet (Phase 3): r=0 normal,
  // r=1 = 2x ratchet (two gate pulses, retriggered at the half-step boundary).
  bool get_gate() const {
    if (resting) return false;
    const uint8_t period = step_period();
    const uint8_t half = uint8_t(period >> 1);
    const int8_t  last = int8_t(period - 1);
    if (get_slide_dac()) return slide_gate ? true : (clk_count < int8_t(half));
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    if (slide_gate && r == 0) return true;
    if (r) {
      // 2x ratchet: short gate pulses at clk=0 and clk=half (preserves the
      // legacy hi-hat-style pattern; ratchet_retrigger forces a low edge
      // between the pulses to retrigger the analog envelope).
      return (uint8_t(clk_count) % half) == 0u || (slide_gate && clk_count == last);
    }
    return clk_count < int8_t(half);
  }

  bool is_ratchet_retrigger() const {
    if (resting) return false;
    if (get_slide_dac()) return false;
    const uint8_t r = get_sequence().get_ratchet_val(uint8_t(get_sequence().time_pos));
    if (r) {
      const uint8_t half = uint8_t(step_period() >> 1);
      return uint8_t(clk_count) == half;
    }
    return false;
  }
  bool get_accent() const {
    return !resting && !get_slide_dac() && get_sequence().get_accent();
  }
  uint8_t get_semitone() const {
    return get_sequence().get_semitone();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch_dir(last_step_dir_);
  }
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

  // ---------------------------------------------------------------------------
  // Setters
  // ---------------------------------------------------------------------------
  void SetPattern(uint8_t p_, bool override = false) {
    next_p = p_ & 0xf;
    if (override) p_select = next_p;
  }
  void SetLength(uint8_t len) {
    Sequence &s = get_sequence();
    const uint8_t old_len = s.length;
    // Triplet mode caps at 24 steps (~2 bars of 4/4 in triplet 8ths). 16ths
    // mode caps at MAX_STEPS (32).
    const uint8_t cap = s.is_triplet_mode() ? uint8_t(24) : uint8_t(MAX_STEPS);
    s.SetLength(len, cap);
    if (s.length != old_len) {
      // Length changed: pitch_count may have changed (NOTE events outside new
      // length no longer count). Rebuild from time_data.
      sequence_rebuild_pitch_count(s);
      // Clear pitch[] tail beyond the (possibly shrunk) count.
      for (uint8_t k = s.get_pitch_count(); k < MAX_STEPS; ++k) s.pitch[k] = PITCH_EMPTY;
    }
    stale = true;
  }
  bool BumpLength() {
    stale = true;
    bool ok = get_sequence().BumpLength(MAX_STEPS);
    sequence_rebuild_pitch_count(get_sequence());
    return ok;
  }
  void SetMode(SequencerMode m, bool reset = false) {
    if (reset && m != mode_) Reset();
    mode_ = m;
  }
  void NudgeOctave(int dir) {
    get_sequence().nudge_octave_buttons(dir);
    stale = true;
  }
  void SetPitchSemitone(uint8_t p) {
    get_sequence().SetPitchSemitone(p);
    stale = true;
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    get_sequence().SetPitch(p, flags);
    stale = true;
  }
  /// TIME_MODE write. Surgically maintains pitch_count + pitch[].
  void SetTime(uint8_t t) {
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    sequence_write_time_with_pitch_sync(s, tp, t);
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

  bool StepBack() {
    bool moved = get_sequence().StepBack();
    if (moved) stale = true;
    return moved;
  }

  bool is_step_locked() const {
    const Sequence &s = get_sequence();
    if (mode_ == TIME_MODE)
      return s.step_locked(uint8_t(s.time_pos));
    if (mode_ == PITCH_MODE)
      return s.step_locked(uint8_t(s.time_pos));
    return false;
  }
  void ToggleStepLockFromTimeMode() {
    if (mode_ != TIME_MODE) return;
    get_sequence().ToggleStepLock(uint8_t(get_sequence().time_pos));
    stale = true;
  }

  // ---------------------------------------------------------------------------
  // SysEx blob (PATTERN_SIZE = 48 bytes: pitch[32] + time_data[8] + reserved[5]
  // + transpose + engine_select + length). Layout matches Sequence struct memory.
  // ---------------------------------------------------------------------------
  void export_pattern_blob(uint8_t idx, uint8_t *blob128) const {
    idx &= 0xf;
    memcpy(blob128, pattern[idx].pitch, PATTERN_SIZE);
  }

  bool import_pattern_blob(uint8_t idx, const uint8_t *blob128, bool persist_eeprom = true) {
    idx &= 0xf;
    uint8_t L = blob128[PATTERN_SIZE - 1];
    if (L < 1 || uint8_t(L) > MAX_STEPS) return false;
    const uint8_t gmax = MAX_STEPS;
    if (L > gmax) L = gmax;
    memcpy(pattern[idx].pitch, blob128, PATTERN_SIZE);
    pattern[idx].length = L;
    Sequence &s = pattern[idx];
    sequence_rebuild_pitch_count(s);
    normalize_pattern_times(s);
    if (s.time_pos  >= s.length) s.time_pos  = 0;
    if (s.pitch_pos >= int(s.get_pitch_count()))
      s.pitch_pos = (s.get_pitch_count() > 0) ? int(s.get_pitch_count() - 1) : 0;
    if (persist_eeprom)
      WritePattern(pattern[idx], idx, group_);
    return true;
  }

  /// MIDI Note On -> write the currently-playing pitch slot. No-op on REST.
  /// On TIE step: write the held source NOTE's slot (pitch_pos).
  /// On NOTE step: write the current NOTE's slot (pitch_pos).
  void midi_apply_note_on(uint8_t note, uint8_t velocity) {
    if (note < 36 || note > 36 + 48) return;
    uint8_t lin = uint8_t(note - 36);
    if (lin > 48) lin = 48;
    uint8_t oct_btn = lin / 12;
    if (oct_btn > 3) oct_btn = 3;
    uint8_t key_idx = uint8_t(lin - oct_btn * 12);
    if (key_idx > PITCH_KEY_HIGH_C) key_idx = PITCH_KEY_HIGH_C;
    Sequence &s = get_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    if (s.time(tp) == 0) return; // REST: no slot to target
    const uint8_t pc = s.get_pitch_count();
    if (pc == 0) return;
    int pp = s.pitch_pos;
    if (pp < 0 || pp >= int(pc)) return;
    const uint8_t pk    = pack_pitch(key_idx, oct_btn);
    const uint8_t acc   = (velocity >= 100) ? uint8_t(1u << 6) : 0;
    const uint8_t slide = (s.pitch[pp] == PITCH_EMPTY) ? 0 : (s.pitch[pp] & (1u << 7));
    s.pitch[pp] = (pk & 0x3f) | slide | acc;
    stale = true;
  }
};
