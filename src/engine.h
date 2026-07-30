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
#include "persistent_settings.h"

// =============================================================================
// Engine -- patterns + clock + gate
// =============================================================================
// =============================================================================
// Tracks (chain of patterns + per-step transpose).
// 8 track slots; each can chain up to MAX_CHAIN patterns from its own group
// (group = track >> 1). Per-track storage = 104 bytes:
//   p_chain_packed[32]   : 4 bits/step, low nibble = even step, high = odd step.
//                          Each nibble is pattern index 0..15 in this track's group.
//   t_chain_last[8]      : 64-bit bitmap. Bit i set => chain step i is the last bar.
//   t_chain_transpose[64]: 1 byte/step. Range 0..47, with 12 = "no transpose"
//                          (matches the global performance transpose encoding).
// EEPROM offset for track t: TRACK_DATA_OFFSET + t * TRACK_BYTES.
// =============================================================================
static constexpr uint8_t MAX_CHAIN  = 64;
static constexpr uint8_t NUM_TRACKS = 8;
static constexpr uint8_t P_CHAIN_PACKED_BYTES = (MAX_CHAIN + 1) / 2;     // 32
static constexpr uint8_t T_CHAIN_BITS_BYTES   = (MAX_CHAIN + 7) / 8;     // 8
static constexpr uint8_t TRACK_BYTES = P_CHAIN_PACKED_BYTES
                                     + T_CHAIN_BITS_BYTES
                                     + MAX_CHAIN;                          // 104
static constexpr uint8_t TRACK_TRANSPOSE_ZERO = 12;                        // 0 semitones
static_assert(TRACK_BYTES == FB_TRACK_LEN, "track block size must match flash block length");

struct Engine {
  Sequence pattern[NUM_PATTERNS];
  uint8_t p_select = 0;
  uint8_t next_p = 0;
  uint8_t group_ = 0;
  uint8_t pending_group_ = 0xff;

  // Track-mode state
  uint8_t p_chain_packed[P_CHAIN_PACKED_BYTES] = {0};            // 32 bytes (4 bits/step)
  uint8_t t_chain_last[T_CHAIN_BITS_BYTES]     = {0};            // 8 bytes (1 bit/step)
  uint8_t t_chain_transpose[MAX_CHAIN]         = {0};            // 64 bytes (0..47, 12 = no-op)
  uint8_t p_chain_len = 0;        // number of valid chain steps (0 = no track loaded)
  uint8_t p_chain_pos = 0;        // current chain step index
  uint8_t track_select = 0;       // which track (0..7) is currently loaded
  bool    track_stale = false;    // track-data dirty flag (separate from `stale`)
  bool    track_active = false;   // true while in TrackPlay/TrackWrite playback

  // ---------------------------------------------------------------------------
  // Packed p_chain accessors (4 bits per step, 16 patterns/group fits exactly).
  // ---------------------------------------------------------------------------
  uint8_t p_chain_get(uint8_t step) const {
    if (step >= MAX_CHAIN) return 0;
    const uint8_t b = p_chain_packed[step >> 1];
    return (step & 1) ? uint8_t(b >> 4) : uint8_t(b & 0x0F);
  }
  void p_chain_set(uint8_t step, uint8_t pat) {
    if (step >= MAX_CHAIN) return;
    uint8_t &b = p_chain_packed[step >> 1];
    const uint8_t v = uint8_t(pat & 0x0F);
    b = (step & 1) ? uint8_t((b & 0x0F) | (v << 4))
                   : uint8_t((b & 0xF0) | v);
  }

  // ---------------------------------------------------------------------------
  // Last-step flag bitmap helpers.
  // ---------------------------------------------------------------------------
  bool t_chain_last_get(uint8_t step) const {
    if (step >= MAX_CHAIN) return false;
    return (t_chain_last[step >> 3] >> (step & 7)) & 1;
  }
  void t_chain_last_set(uint8_t step, bool v) {
    if (step >= MAX_CHAIN) return;
    const uint8_t mask = uint8_t(1u << (step & 7));
    if (v) t_chain_last[step >> 3] |= mask;
    else   t_chain_last[step >> 3] &= uint8_t(~mask);
  }
  void t_chain_last_clear_all() {
    for (uint8_t i = 0; i < T_CHAIN_BITS_BYTES; ++i) t_chain_last[i] = 0;
  }

  SequencerMode      mode_      = NORMAL_MODE;
  SequenceDirection  direction_ = DIR_FORWARD;
  int8_t  pp_dir_        = 1;
  uint8_t advance_count_ = 0;
  SequenceDirection  next_direction_          = DIR_FORWARD;
  bool               direction_change_pending_ = false;

  int8_t clk_count = -1;

  bool slide_dac_ = false; // slide CV for the current step (set in Advance)
  bool slide_gate = false;
  bool stale = false;
  uint32_t saved_hash_[NUM_PATTERNS] = {}; // per-pattern content hash at last save/load (dirty detection)
  bool resting = false;

  // Multitimbral shadow voices: variations 2 and 3 of the active slot. The CV/
  // gate always plays variation 1; the shadows are MIDI-only, advanced forward
  // each 16th in lockstep. See ReloadShadows/AdvanceShadows and midi_shadows_gate_tick.
  Sequence shadow_[NUM_VARIATIONS - 1];
  uint8_t  shadow_var_[NUM_VARIATIONS - 1] = {1, 2};
  uint8_t  shadow_notecount_[NUM_VARIATIONS - 1] = {0, 0};
  uint8_t  shadow_last_p_ = 0xff, shadow_last_group_ = 0xff;
  bool     shadow_stale_ = false;          // resident shadow edited; persist on save/reload
  uint32_t shadow_dirty_ms_ = 0;           // millis() of the last shadow edit (idle-flush coalescing)
  uint8_t  edit_var_ = 0;                   // hardware edit target: 0=var1, 1=var2, 2=var3
  // Per-shadow gate state (computed at AdvanceShadows, sampled per tick by the
  // shadow MIDI gate-follow so var2/3 note length tracks the analog gate).
  bool     shadow_resting_[NUM_VARIATIONS - 1]    = {true, true};
  bool     shadow_slide_gate_[NUM_VARIATIONS - 1] = {false, false};
  int8_t   shadow_pp_dir_[NUM_VARIATIONS - 1]     = {1, 1}; // ping-pong/brownian walk state
  int8_t   shadow_step_dir_[NUM_VARIATIONS - 1]   = {1, 1}; // last step direction (slide lookups)

  // Variation 3 polyphonic voice (active slot only, when var3_is_poly). When
  // poly_active_ it plays in place of the mono shadow_[1]; advanced forward each
  // 16th. Step-level accent/slide/time; up to POLY_VOICES notes per step.
  PolyVoice poly_;
  bool     poly_active_     = false;
  bool     poly_stale_      = false; // poly_ edited; persist on save/reload
  bool     poly_reset_      = true;
  uint8_t  poly_time_pos_   = 0;
  uint8_t  poly_chord_pos_  = 0;     // chord-stream index sounding (held across ties)
  bool     poly_resting_    = true;
  bool     poly_slide_gate_ = false;

  // Deferred non-resident poly edits. A per-step web edit (SysEx 0x27) to a poly
  // slot that is NOT the active voice is buffered here and written to flash on the
  // idle save / next reload -- never inline -- so editing a non-playing chord does
  // not stall the playing voice. One slot at a time (the editor shows one pattern);
  // touching a different slot flushes the previous one first.
  PolyVoice poly_edit_;
  int16_t   poly_edit_slot_  = -1;   // absolute slot buffered, or -1 if none
  bool      poly_edit_dirty_ = false;
  uint32_t  poly_edit_ms_    = 0;    // millis() of last buffered edit (idle coalescing)

  // Deferred non-resident variation 2/3 mono blob (web SysEx 0x12). Buffered in
  // RAM and written to flash on the idle save / next persist_shadows -- never
  // inline in the SysEx handler (a page write halts the CPU long enough to drop
  // the next incoming message, and live-edit debounce would wear flash per edit).
  Sequence  shadow_edit_;
  int16_t   shadow_edit_slot_  = -1; // absolute slot buffered, or -1 if none
  uint8_t   shadow_edit_var_   = 1;  // 1 = variation 2, 2 = variation 3 (mono)
  bool      shadow_edit_dirty_ = false;
  uint32_t  shadow_edit_ms_    = 0;  // millis() of last buffered edit (idle coalescing)

  // --- Per-step characteristic probability (variation 1 / CV voice only) ---
  // Three bytes per step (see sequence.h): accent+slide levels in byte0,
  // down+up transpose levels in byte1, up-double flag in byte2. Only the active
  // slot's table is RAM-resident (a full-bank cache won't fit on an 8 KB part);
  // loaded in ReloadShadows, flushed in persist_shadows. Variations 2/3 do not
  // carry probability.
  uint8_t  prob_var1_[3 * MAX_STEPS] = {};
  bool     prob_stale_ = false;       // resident table edited; persist on save/reload
  // Deferred non-resident prob edits (SysEx to a slot != active): the slot's
  // 192-byte table buffered in RAM, flash write deferred (poly 0x27 precedent).
  uint8_t  prob_edit_[3 * MAX_STEPS] = {};
  int16_t  prob_edit_slot_  = -1;
  bool     prob_edit_dirty_ = false;
  uint32_t prob_edit_ms_    = 0;
  // Roll state, latched once per NOTE step in Advance() (output getters are
  // polled every DAC refresh and must never re-roll). TIE holds the source
  // note's latch; REST clears it. Each characteristic rolls independently.
  bool     prob_accent_armed_ = false, prob_accent_pass_ = false;
  int8_t   prob_transpose_ = 0;
  uint8_t  prob_slide_prev_ = 0xFF;   // source-note slide override: 0xFF none, 0 off, 1 on
  uint8_t  prob_slide_pend_ = 0xFF;   // current note's override, consumed next step

  uint8_t get_group() const { return group_; }

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
    load_group_patterns();
    snapshot_pattern_hashes();
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
      load_group_patterns();
      GlobalSettings.load_midi_from_storage();
      direction_ = DIR_FORWARD;
    } else {
      // Signature mismatch (fresh flash or a block-map change): wipe the whole
      // arena so no stale records from an old layout survive at reused block ids.
      g_flash.format();
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i)
        pattern[i].Clear();
      GlobalSettings.midi_channel = 1;
      GlobalSettings.midi_clock_receive = true;
      GlobalSettings.midi_thru = false;
      memcpy(GlobalSettings.signature, sig_pew, kSigEepromLen);
      GlobalSettings.Save();
      // Patterns are written lazily on first edit; unwritten blocks read back as
      // empty (ReadPatternAt clears them), so nothing to pre-write.
      stale = false;
    }
    snapshot_pattern_hashes(); // baseline dirty detection for the active group
    // Track storage format check: bump kTrackFormatVersion when the layout
    // changes. On mismatch, fill the TRACK_DATA region with 0xFF (the AVR
    // "fresh EEPROM" sentinel) so LoadTrack's uninit detection fires and
    // initialises p_chain to 0 + transpose to TRACK_TRANSPOSE_ZERO. Patterns
    // are unaffected.
    if (GlobalSettings.get_track_format() != PersistentSettings::kTrackFormatVersion) {
      uint8_t blank[FB_TRACK_LEN];
      memset(blank, 0xFF, FB_TRACK_LEN);          // LoadTrack treats 0xFF as fresh
      for (uint8_t t = 0; t < NUM_TRACKS; ++t)
        WriteTrackAt(t, blank);
      GlobalSettings.set_track_format(PersistentSettings::kTrackFormatVersion);
    }
    ReloadShadows(); // prime shadow voices for the first running tick
  }

  // 32-bit FNV-1a over a pattern's persisted bytes (the PATTERN_SIZE-byte prefix
  // of Sequence). Used only for change detection; collision ~1/4e9.
  static uint32_t pattern_hash(const Sequence &s) {
    const uint8_t *p = &s.pitch[0];
    uint32_t h = 2166136261UL;
    for (uint8_t i = 0; i < PATTERN_SIZE; ++i) { h ^= p[i]; h *= 16777619UL; }
    return h;
  }
  // Re-baseline the dirty detector for the whole active group (after load/switch).
  void snapshot_pattern_hashes() {
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) saved_hash_[i] = pattern_hash(pattern[i]);
  }
  // Absolute slot index (0..63) for a pattern position in the active group.
  uint8_t abs_slot(uint8_t pat_in_group) const {
    return uint8_t(group_ * NUM_PATTERNS + (pat_in_group & uint8_t(NUM_PATTERNS - 1)));
  }
  // Read the active group's NUM_PATTERNS patterns (variation 1) from flash.
  void load_group_patterns() {
    for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
      ReadPatternAt(pattern[i], abs_slot(i), 0);
      if (!pattern[i].length) pattern[i].SetLength(8);
      sequence_rebuild_pitch_count(pattern[i]);
      normalize_pattern_times(pattern[i]);
    }
  }

  // --- Multitimbral shadow voices (variations 2 and 3 of the active slot) ---
  // Variation 1 (index 0) drives CV/gate + its MIDI; the shadows are MIDI-only.
  void ReloadShadows() {
    const uint8_t s = abs_slot(p_select);
    for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
      const uint8_t var = uint8_t(i + 1); // variation index 1 and 2 (variations 2 and 3)
      ReadPatternAt(shadow_[i], s, var);
      if (!shadow_[i].length) shadow_[i].SetLength(8);
      sequence_rebuild_pitch_count(shadow_[i]);
      normalize_pattern_times(shadow_[i]);
      shadow_[i].Reset();
      shadow_var_[i]       = var;
      shadow_notecount_[i] = shadow_[i].note_count();
      shadow_resting_[i]   = true;
      shadow_slide_gate_[i] = false;
      shadow_pp_dir_[i]    = 1;
      shadow_step_dir_[i]  = 1;
    }
    // Variation 3 (shadow_[1]) may be polyphonic for this slot.
    poly_active_ = GlobalSettings.var3_is_poly(s);
    if (poly_active_) {
      ReadPolyAt(poly_, s);
      if (!poly_.length) poly_.length = 8;
      poly_.ensure_chords_for_notes();   // guarantee a chord for every NOTE event
      poly_reset_      = true;
      poly_time_pos_   = 0;
      poly_chord_pos_  = 0;
      poly_resting_    = true;
      poly_slide_gate_ = false;
      poly_stale_      = false;
      shadow_notecount_[1] = 0; // var3 plays from poly_; silence the mono shadow
    }
    shadow_last_p_     = p_select;
    shadow_last_group_ = group_;
    // Load this slot's variation-1 probability table and re-latch it for the
    // step it is already on (a chain switch advances into the new pattern
    // before the reload, so that roll used the outgoing slot's table).
    ReadProbAt(s, prob_var1_);
    prob_stale_ = false;
    prob_clear_latch();
    if (!resting) prob_roll_var1();
  }
  // True when the active slot or group changed since the last ReloadShadows.
  // Caller must flush open shadow notes before calling ReloadShadows.
  bool ShadowsNeedReload() const {
    return !(p_select == shadow_last_p_ && group_ == shadow_last_group_);
  }
  // Advance each non-empty shadow one 16th (forward only in v1) and compute its
  // gate state (resting + slide_gate) the same way Advance() does for the main
  // voice, so the shadow MIDI can track the analog gate window.
  void AdvanceShadows() {
    for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
      if (!shadow_notecount_[i]) { shadow_resting_[i] = true; shadow_slide_gate_[i] = false; continue; }
      Sequence &s = shadow_[i];
      const uint8_t dir = s.get_direction_stored();   // each variation has its own direction
      bool result;
      int8_t step_dir, next_step_dir;
      if (dir == DIR_FORWARD) {
        result = s.Advance();
        step_dir = next_step_dir = 1;
      } else {
        step_dir = (dir == DIR_REVERSE) ? int8_t(-1)
                 : (dir == DIR_PINGPONG || dir == DIR_BROWNIAN) ? shadow_pp_dir_[i] : int8_t(1);
        result = s.AdvanceDirectional(dir, shadow_pp_dir_[i]);
        next_step_dir = (dir == DIR_REVERSE) ? int8_t(-1)
                      : (dir == DIR_PINGPONG || dir == DIR_BROWNIAN) ? shadow_pp_dir_[i] : int8_t(1);
      }
      shadow_step_dir_[i] = step_dir;
      shadow_resting_[i]  = !result;
      if (result) {
        const bool next_is_tie = s.is_tied_dir(dir, next_step_dir);
        const bool tie_slide   = s.is_tie() && s.slide_from_prev_dir(dir, step_dir);
        const uint8_t len  = s.length ? s.length : 1;
        const uint8_t npos = uint8_t((unsigned(s.time_pos) +
                                      (next_step_dir >= 0 ? 1u : unsigned(len) - 1u)) % unsigned(len));
        const bool next_is_rest = (s.time(npos) == 0);
        shadow_slide_gate_[i] = (!next_is_rest && s.get_slide()) || next_is_tie || tie_slide;
      } else {
        shadow_slide_gate_[i] = false;
      }
    }
    if (poly_active_) AdvancePoly();
  }
  // Advance the poly voice one 16th (forward only in v1). Latches the sounding
  // chord on NOTE steps and holds it across TIE steps; computes the gate window
  // like a shadow so MIDI note length tracks the step.
  void AdvancePoly() {
    PolyVoice &p = poly_;
    const uint8_t len = p.length ? p.length : 1;
    if (poly_reset_) { poly_reset_ = false; poly_time_pos_ = 0; }
    else             poly_time_pos_ = uint8_t((poly_time_pos_ + 1) % len);
    const uint8_t t = p.time(poly_time_pos_);
    poly_resting_ = (t == 0);
    // NOTE pulls the next chord from the stream (K-th NOTE -> chord K); TIE holds it.
    if (t == 1) poly_chord_pos_ = p.chord_index_for_step(poly_time_pos_);
    if (!poly_resting_) {
      const uint8_t np = uint8_t((poly_time_pos_ + 1) % len);
      const uint8_t nt = p.time(np);
      poly_slide_gate_ = (nt == 2) || (p.slide(poly_chord_pos_) && nt != 0);
    } else {
      poly_slide_gate_ = false;
    }
  }
  // Apply a web-editor variation 2/3 blob (raw PATTERN_SIZE bytes, import layout)
  // to the resident shadow voice in RAM -- no flash write, no playback reset, so
  // editing a live variation does not stall the sequencer. Persisted on the next
  // save/reload. Returns false if `pat` is not the active (resident) slot.
  bool apply_shadow_blob(uint8_t pat, uint8_t var, const uint8_t *raw) {
    if (var < 1 || var >= NUM_VARIATIONS) return false;
    if ((pat & uint8_t(NUM_PATTERNS - 1)) != p_select) return false;
    Sequence &sh = shadow_[var - 1];
    memcpy(sh.pitch, raw, PATTERN_SIZE);     // pitch[]+time_data[]+meta, length at end
    uint8_t L = raw[PATTERN_SIZE - 1];
    sh.length = (L >= 1 && L <= MAX_STEPS) ? L : 8;
    sequence_rebuild_pitch_count(sh);
    normalize_pattern_times(sh);
    if (sh.time_pos >= sh.length) sh.time_pos = 0;
    shadow_notecount_[var - 1] = sh.note_count();
    // Variation 3 plays from the poly voice when poly is active: never let an
    // incoming mono var3 blob re-arm the mono shadow (would double the voice).
    if (var == 2 && poly_active_) shadow_notecount_[1] = 0;
    shadow_stale_ = true;
    shadow_dirty_ms_ = millis();
    return true;
  }
  // Apply one buffered poly step edit to a NON-resident slot (absolute). Loads the
  // slot from flash on first touch / slot change (flushing any previous slot first),
  // mutates RAM only. The flash write is deferred to flush_poly_edit().
  void poly_edit_step(uint8_t slot, uint8_t step,
                      uint8_t n0, uint8_t n1, uint8_t n2, uint8_t n3,
                      uint8_t t, bool acc, bool sld) {
    if (poly_edit_slot_ != int16_t(slot)) {
      flush_poly_edit();              // persist the previously buffered slot first
      ReadPolyAt(poly_edit_, slot);   // cheap flash read (no erase)
      poly_edit_.ensure_chords_for_notes();
      poly_edit_slot_ = int16_t(slot);
    }
    poly_edit_.set_step(step, n0, n1, n2, n3, t, acc, sld);
    poly_edit_dirty_ = true;
    poly_edit_ms_    = millis();
  }
  // Write the buffered non-resident poly slot to flash (if dirty) and clear it.
  void flush_poly_edit() {
    if (poly_edit_dirty_ && poly_edit_slot_ >= 0)
      WritePolyAt(poly_edit_, uint8_t(poly_edit_slot_));
    poly_edit_dirty_ = false;
  }
  // Discard buffered edits for `slot` -- a newer full-blob write (0x26) or a poly/
  // mono flag change supersedes them.
  void drop_poly_edit(uint8_t slot) {
    if (poly_edit_slot_ == int16_t(slot)) { poly_edit_slot_ = -1; poly_edit_dirty_ = false; }
  }
  // Buffer a full poly blob (SysEx 0x26) for a NON-resident slot. RAM only; the
  // flash write is deferred to flush_poly_edit() like the per-step 0x27 edits.
  void poly_edit_blob(uint8_t slot, const uint8_t *raw) {
    if (poly_edit_dirty_ && poly_edit_slot_ != int16_t(slot)) flush_poly_edit();
    poly_edit_.deserialize(raw);
    poly_edit_.ensure_chords_for_notes();
    poly_edit_slot_  = int16_t(slot);
    poly_edit_dirty_ = true;
    poly_edit_ms_    = millis();
  }
  // Buffer a full var2/3 mono blob (SysEx 0x12) for a NON-resident slot. RAM
  // only; flushed by flush_shadow_edit(). Caller validates the blob's length.
  void shadow_edit_blob(uint8_t slot, uint8_t var, const uint8_t *raw) {
    if (shadow_edit_dirty_ &&
        (shadow_edit_slot_ != int16_t(slot) || shadow_edit_var_ != var))
      flush_shadow_edit();
    deserialize_pattern(shadow_edit_, raw);
    shadow_edit_.length = raw[PATTERN_SIZE - 1];
    sequence_rebuild_pitch_count(shadow_edit_);
    normalize_pattern_times(shadow_edit_);
    shadow_edit_slot_  = int16_t(slot);
    shadow_edit_var_   = var;
    shadow_edit_dirty_ = true;
    shadow_edit_ms_    = millis();
  }
  void flush_shadow_edit() {
    if (shadow_edit_dirty_ && shadow_edit_slot_ >= 0)
      WritePatternAt(shadow_edit_, uint8_t(shadow_edit_slot_), shadow_edit_var_);
    shadow_edit_dirty_ = false;
  }
  // --- Probability-table edit paths (var1 only; mirror the shadow edit buffer) ---
  // The three prob bytes (b0 accent+slide, b1 down+up, b2 up-double) for a step.
  void prob_set_resident(uint8_t step, uint8_t b0, uint8_t b1, uint8_t b2) {
    const uint8_t k = uint8_t(step & (MAX_STEPS - 1)) * 3;
    prob_var1_[k] = b0; prob_var1_[k + 1] = b1; prob_var1_[k + 2] = b2;
    prob_stale_ = true;
    shadow_dirty_ms_ = millis();
  }
  // Random level for a randomize op: ~1/3 of steps unarmed (0), else 1..13.
  static uint8_t prob_rand_level() {
    return fast_rand(3) ? uint8_t(1 + fast_rand(PROB_LEVEL_MAX)) : 0;
  }
  // Scatter random probabilities across the active pattern's steps (within
  // length) for one characteristic, or all four. kind: 0 accent, 1 slide,
  // 2 down, 3 up, 4 all. Active var1 table only; RAM, persisted lazily.
  void prob_randomize(uint8_t kind) {
    const uint8_t len = get_sequence().length ? get_sequence().length : 1;
    fast_rand_seed();
    for (uint8_t i = 0; i < len && i < MAX_STEPS; ++i) {
      const uint8_t k = i * 3;
      uint8_t acc = prob_accent_level(prob_var1_[k]);
      uint8_t sld = prob_slide_level(prob_var1_[k]);
      uint8_t dn  = prob_down_level(prob_var1_[k + 1]);
      uint8_t up  = prob_up_level(prob_var1_[k + 1]);
      bool    updbl = prob_up_double(prob_var1_[k + 2]);
      if (kind == 0 || kind == 4) acc = prob_rand_level();
      if (kind == 1 || kind == 4) sld = prob_rand_level();
      if (kind == 2 || kind == 4) dn  = prob_rand_level();
      if (kind == 3 || kind == 4) { up = prob_rand_level(); updbl = up && fast_rand(2); }
      prob_var1_[k]     = prob_pack_ac_sl(acc, sld);
      prob_var1_[k + 1] = prob_pack_du(dn, up);
      prob_var1_[k + 2] = updbl ? 1 : 0;
    }
    prob_stale_ = true;
    shadow_dirty_ms_ = millis();
  }
  // Buffer one step edit for a NON-resident slot. RAM only; the flash write (a
  // 192 B read-modify-write of the slot's block) is deferred.
  void prob_edit_set(uint8_t slot, uint8_t step, uint8_t b0, uint8_t b1, uint8_t b2) {
    if (prob_edit_slot_ != int16_t(slot)) {
      flush_prob_edit();
      ReadProbAt(slot, prob_edit_);
      prob_edit_slot_ = int16_t(slot);
    }
    const uint8_t k = uint8_t(step & (MAX_STEPS - 1)) * 3;
    prob_edit_[k] = b0; prob_edit_[k + 1] = b1; prob_edit_[k + 2] = b2;
    prob_edit_dirty_ = true;
    prob_edit_ms_    = millis();
  }
  // Buffer a full 192 B table (null = all zeros) for a NON-resident slot.
  void prob_edit_blob(uint8_t slot, const uint8_t *src) {
    if (prob_edit_dirty_ && prob_edit_slot_ != int16_t(slot)) flush_prob_edit();
    if (src) memcpy(prob_edit_, src, FB_PROB_LEN);
    else     memset(prob_edit_, 0, FB_PROB_LEN);
    prob_edit_slot_  = int16_t(slot);
    prob_edit_dirty_ = true;
    prob_edit_ms_    = millis();
  }
  void flush_prob_edit() {
    if (prob_edit_dirty_ && prob_edit_slot_ >= 0)
      WriteProbAt(uint8_t(prob_edit_slot_), prob_edit_);
    prob_edit_dirty_ = false;
  }
  // Table for the web editor: resident, else the dirty edit buffer, else flash.
  void export_prob_table(uint8_t idx, uint8_t *dst) {
    idx &= uint8_t(NUM_PATTERNS - 1);
    if (idx == p_select) { memcpy(dst, prob_var1_, FB_PROB_LEN); return; }
    const uint8_t s = abs_slot(idx);
    if (prob_edit_dirty_ && prob_edit_slot_ == int16_t(s)) {
      memcpy(dst, prob_edit_, FB_PROB_LEN);
      return;
    }
    ReadProbAt(s, dst);
  }
  // Persist edited shadow voices to flash (called on save and before reload).
  /// True when anything OUTSIDE the variation-1 pattern array is waiting to be
  /// written: resident shadow / poly / probability edits, or a buffered
  /// non-resident one. `stale` does not cover these, so every flush point that
  /// tests `stale` alone would otherwise leave variation 2/3, poly and
  /// probability edits in RAM until a slot change or transport stop.
  bool aux_dirty() const {
    return shadow_stale_ || poly_stale_ || prob_stale_ ||
           poly_edit_dirty_ || shadow_edit_dirty_ || prob_edit_dirty_;
  }

  void persist_shadows() {
    flush_poly_edit();   // commit buffered non-resident poly edits before reload/save
    flush_shadow_edit(); // commit buffered non-resident var2/3 blobs too
    flush_prob_edit();   // commit buffered non-resident probability tables
    // Flush the active slot's probability table (uses the slot it was loaded
    // for, like the shadow write below).
    if (prob_stale_ && shadow_last_p_ != 0xff) {
      const uint8_t ps = uint8_t(shadow_last_group_ * NUM_PATTERNS
                                 + (shadow_last_p_ & uint8_t(NUM_PATTERNS - 1)));
      WriteProbAt(ps, prob_var1_);
      prob_stale_ = false;
    }
    if (!shadow_stale_ && !poly_stale_) return;
    const uint8_t s = uint8_t(shadow_last_group_ * NUM_PATTERNS
                              + (shadow_last_p_ & uint8_t(NUM_PATTERNS - 1)));
    if (shadow_stale_) {
      WritePatternAt(shadow_[0], s, 1);                  // variation 2 (always mono)
      if (!poly_active_) WritePatternAt(shadow_[1], s, 2); // variation 3 mono
    }
    if (poly_active_ && poly_stale_) WritePolyAt(poly_, s); // variation 3 poly
    shadow_stale_ = false;
    poly_stale_   = false;
  }
  // Persist one pattern (variation 1) and re-baseline its hash.
  void persist_pattern(uint8_t idx) {
    idx &= uint8_t(NUM_PATTERNS - 1);
    WritePatternAt(pattern[idx], abs_slot(idx), 0);
    saved_hash_[idx] = pattern_hash(pattern[idx]);
  }

  void Save(int pidx = -1) {
    persist_shadows();
    if (!stale) return;
    if (pidx >= 0) {
      persist_pattern(uint8_t(pidx));
    } else {
      // Write only the patterns whose content changed since the last save/load
      // (one read-modify-write page op per changed pattern, not all 16): less
      // flash wear, shorter LED-refresh stall.
      for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
        const uint32_t h = pattern_hash(pattern[i]);
        if (h != saved_hash_[i]) {
          WritePatternAt(pattern[i], abs_slot(i), 0);
          saved_hash_[i] = h;
        }
      }
    }
    stale = false;
  }

  // ---------------------------------------------------------------------------
  // Track storage (OS-303 v0.6 layout)
  // ---------------------------------------------------------------------------
  void LoadTrack(uint8_t track) {
    track &= (NUM_TRACKS - 1);
    track_select = track;
    uint8_t b[FB_TRACK_LEN];
    const bool have = ReadTrackAt(track, b);
    if (have) {
      memcpy(p_chain_packed, b, P_CHAIN_PACKED_BYTES);
      memcpy(t_chain_last, b + P_CHAIN_PACKED_BYTES, T_CHAIN_BITS_BYTES);
      memcpy(t_chain_transpose, b + P_CHAIN_PACKED_BYTES + T_CHAIN_BITS_BYTES, MAX_CHAIN);
    }
    // Absent block, or 0xFF-filled (track-format wipe): treat as fresh.
    bool uninit = !have || (p_chain_packed[0] == 0xFF);
    if (uninit && have) {
      for (uint8_t i = 0; i < T_CHAIN_BITS_BYTES; ++i)
        if (t_chain_last[i] != 0xFF) { uninit = false; break; }
    }
    if (uninit) {
      memset(p_chain_packed, 0, sizeof(p_chain_packed));
      t_chain_last_clear_all();
      for (uint8_t i = 0; i < MAX_CHAIN; ++i) t_chain_transpose[i] = TRACK_TRANSPOSE_ZERO;
      p_chain_len = 0;
    } else {
      // Length: first chain step whose last-flag bit is set, plus 1.
      p_chain_len = 0;
      for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
        if (t_chain_last_get(i)) { p_chain_len = uint8_t(i + 1); break; }
      }
      // Clamp transpose values in case of stale data.
      for (uint8_t i = 0; i < MAX_CHAIN; ++i)
        if (t_chain_transpose[i] > 47) t_chain_transpose[i] = TRACK_TRANSPOSE_ZERO;
    }
    p_chain_pos = 0;
    track_stale = false;
  }
  void SaveTrack() {
    if (!track_stale) return;
    // Reset the last-step bitmap based on p_chain_len (single bit at len-1).
    t_chain_last_clear_all();
    if (p_chain_len > 0) t_chain_last_set(uint8_t(p_chain_len - 1), true);
    uint8_t b[FB_TRACK_LEN];
    memcpy(b, p_chain_packed, P_CHAIN_PACKED_BYTES);
    memcpy(b + P_CHAIN_PACKED_BYTES, t_chain_last, T_CHAIN_BITS_BYTES);
    memcpy(b + P_CHAIN_PACKED_BYTES + T_CHAIN_BITS_BYTES, t_chain_transpose, MAX_CHAIN);
    WriteTrackAt(track_select, b);
    track_stale = false;
  }
  uint8_t get_chain_len()    const { return p_chain_len; }
  uint8_t get_chain_pos()    const { return p_chain_pos; }

  // Returns true if track has any chain steps.
  bool track_has_chain() const { return p_chain_len > 0; }

  // Track Write helpers --------------------------------------------------------
  // Write the active pattern into the current chain step. `repeats` parameter
  // kept for source compatibility but no longer stored (packed nibble layout).
  void TrackWriteCurrentStep(uint8_t pattern_in_bank, uint8_t /*repeats*/) {
    if (p_chain_pos >= MAX_CHAIN) return;
    p_chain_set(p_chain_pos, pattern_in_bank);
    if (p_chain_pos + 1 > p_chain_len) p_chain_len = uint8_t(p_chain_pos + 1);
    track_stale = true;
  }
  // Mark current chain step as the last bar. Truncates p_chain_len.
  void TrackMarkLastStep() {
    if (p_chain_pos >= MAX_CHAIN) return;
    t_chain_last_clear_all();
    t_chain_last_set(p_chain_pos, true);
    p_chain_len = uint8_t(p_chain_pos + 1);
    track_stale = true;
  }
  uint8_t TrackGetPattern(uint8_t chain_step) const {
    return p_chain_get(chain_step);
  }
  // Per-chain-step transpose. Encoding matches the global performance transpose
  // (0..47, with TRACK_TRANSPOSE_ZERO = 12 meaning "no transpose"). Adding the
  // chain-step transpose to a note linear-semitone gives the effective output.
  uint8_t TrackGetTranspose(uint8_t chain_step) const {
    if (chain_step >= MAX_CHAIN) return TRACK_TRANSPOSE_ZERO;
    return t_chain_transpose[chain_step];
  }
  void TrackSetTranspose(uint8_t chain_step, uint8_t value) {
    if (chain_step >= MAX_CHAIN) return;
    if (value > 47) value = TRACK_TRANSPOSE_ZERO;
    t_chain_transpose[chain_step] = value;
    track_stale = true;
  }
  void TrackResetCursor() { p_chain_pos = 0; }

  // Called at pattern wrap (time_pos returns to 0). Advances the chain cursor
  // and updates next_p so the engine's existing pattern-switch logic picks
  // up the change. Each chain step plays exactly once before advancing.
  // Previously the first wrap re-played chain[cursor] (init branch). With
  // repeats removed, we just advance every wrap; the initial chain[0]
  // playback comes from p_select being set when the track is loaded /
  // CLEAR-reset, not from the chain advance.
  void track_advance_chain() {
    if (!track_active || p_chain_len == 0) return;
    p_chain_pos = uint8_t((p_chain_pos + 1) % p_chain_len);
    next_p = p_chain_get(p_chain_pos);
  }
  void TrackAdvanceCursor() {
    if (p_chain_pos + 1 < MAX_CHAIN) ++p_chain_pos;
  }
  void TrackClear() {
    memset(p_chain_packed, 0, sizeof(p_chain_packed));
    t_chain_last_clear_all();
    for (uint8_t i = 0; i < MAX_CHAIN; ++i) t_chain_transpose[i] = TRACK_TRANSPOSE_ZERO;
    p_chain_len = 0;
    p_chain_pos = 0;
    track_stale = true;
  }

  // Cached once per step in Advance() (slide_from_prev_dir scans time_data,
  // too heavy to recompute on every main-loop DAC refresh).
  bool get_slide_dac() const { return slide_dac_; }

  // Roll variation 1's probability latch for its current step. NOTE = fresh
  // roll of each armed characteristic (accent / slide / transpose independently),
  // REST = clear, TIE = hold the source note's latch (a held note must not
  // change accent/octave mid-tie; matches pitch_pos semantics). Output getters
  // only read the latched result, never re-roll.
  static bool prob_roll(uint8_t level) {
    return level && (fast_rand(PROB_LEVEL_MAX) < level);
  }
  void prob_roll_var1() {
    const Sequence &s = get_sequence();
    const uint8_t t = (s.length && s.time_pos < s.length)
                        ? s.time(uint8_t(s.time_pos)) : 0;
    if (t == 1) {
      const uint8_t k  = uint8_t(s.time_pos) & (MAX_STEPS - 1);
      const uint8_t b0 = prob_var1_[k * 3], b1 = prob_var1_[k * 3 + 1];
      const uint8_t b2 = prob_var1_[k * 3 + 2];
      const uint8_t acc_lvl = prob_accent_level(b0);
      prob_accent_armed_ = (acc_lvl != 0);
      prob_accent_pass_  = prob_roll(acc_lvl);
      const bool slide_armed = prob_slide_level(b0) != 0;
      const bool slide_pass  = prob_roll(prob_slide_level(b0));
      prob_slide_pend_ = slide_armed ? uint8_t(slide_pass ? 1 : 0) : 0xFF;
      // Down (-12) and up (+12/+24) roll independently; if both pass, flip a
      // coin to pick which shift applies this step.
      const bool down_pass = prob_roll(prob_down_level(b1));
      const bool up_pass    = prob_roll(prob_up_level(b1));
      int8_t delta = 0;
      if (down_pass && up_pass) delta = fast_rand(2) ? (prob_up_double(b2) ? 24 : 12) : -12;
      else if (down_pass)       delta = -12;
      else if (up_pass)         delta = prob_up_double(b2) ? 24 : 12;
      prob_transpose_ = delta ? prob_degrade_transpose(
          s.scale_quantize_linear(s.get_pitch()), delta) : 0;
    } else if (t == 0) {
      prob_accent_armed_ = false; prob_accent_pass_ = false;
      prob_transpose_ = 0; prob_slide_pend_ = 0xFF;
    }
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
          direction_ = next_direction_;
          direction_change_pending_ = false;
          pp_dir_ = 1;
          // Re-anchor to step 0 at the bar boundary for every old direction
          // (random/half-rand/brownian leave time_pos at an arbitrary step),
          // so the pattern always hands off in phase with the clock.
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

    // Probability latch: consume the source note's slide override, then roll
    // once for this step (getters below only read the latched result).
    prob_slide_prev_ = prob_slide_pend_;
    prob_roll_var1();
    const bool dir_random = (direction_ == DIR_RANDOM ||
                             direction_ == DIR_HALF_RAND ||
                             direction_ == DIR_BROWNIAN);
    // Effective "the previous note slides into this step": an armed slide char
    // on the source note replaces its stored flag with the roll result. A REST
    // clears the pending override, so rest-cancels-slide is preserved.
    bool src_slide = false;
    if (result && !dir_random) {
      src_slide = (prob_slide_prev_ != 0xFF)
                    ? (!get_sequence().first_step && prob_slide_prev_ == 1)
                    : get_sequence().slide_from_prev_dir(uint8_t(direction_), step_dir);
    }
    slide_dac_ = src_slide;

    if (result) {
      const bool next_is_tie = get_sequence().is_tied_dir(uint8_t(direction_), next_step_dir);
      const bool tie_slide   = get_sequence().is_tie() && src_slide;
      const uint8_t next_pos = uint8_t(
          (unsigned(get_sequence().time_pos) +
           (next_step_dir >= 0 ? 1u : unsigned(get_sequence().length) - 1u)) %
          unsigned(get_sequence().length));
      const bool next_is_rest = (get_sequence().time(next_pos) == 0);
      // This step's own slide: prob override when armed (pend 1/0), else stored.
      const bool cur_slide = (prob_slide_pend_ != 0xFF)
                               ? (prob_slide_pend_ == 1)
                               : get_sequence().get_slide();
      slide_gate = (!next_is_rest && cur_slide) || next_is_tie || tie_slide;
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
    for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
      shadow_[i].Reset();
      shadow_pp_dir_[i] = 1;
      shadow_step_dir_[i] = 1;
    }
    // Restart the poly voice at step 0 too, so it lands on the downbeat with var1/2
    // instead of continuing from a stale position when the transport starts.
    poly_reset_      = true;
    poly_time_pos_   = 0;
    poly_chord_pos_  = 0;
    poly_resting_    = true;
    poly_slide_gate_ = false;
    clk_count = -1;
    slide_dac_ = false;
    slide_gate = false;
    resting = true;
    advance_count_ = 0;
    pp_dir_ = 1;
    prob_clear_latch();
  }

  // Clear all latched probability roll state (transport reset / slot reload).
  void prob_clear_latch() {
    prob_accent_armed_ = false; prob_accent_pass_ = false; prob_transpose_ = 0;
    prob_slide_prev_ = prob_slide_pend_ = 0xFF;
  }

  // ---------------------------------------------------------------------------
  // Bulk ops on the joint (time, pitch) representation.
  // Each captures per-time-step pitch via sequence_pack_per_time, mutates the
  // time stream + the per-time buffer in lockstep, then rebuilds pitch[] +
  // pitch_count via sequence_unpack_per_time.
  // ---------------------------------------------------------------------------

  /// Randomize entire pattern: random time + random pitches in stream order.
  void RandomizeFullPattern() {
    Sequence &s = get_edit_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      const uint8_t t = fast_rand_time_weighted(prev, i == 0);
      sequence_set_time_at(s, i, t);
      prev = t;
    }
    normalize_pattern_times_only(s);
    const uint8_t nc = s.note_count();
    for (uint8_t k = 0; k < nc; ++k) {
      s.pitch[k] = s.scale_apply_packed(fast_rand_pitch_byte_weighted())
                   | fast_rand_accent_weighted()
                   | fast_rand_slide_weighted();
    }
    for (uint8_t k = nc; k < MAX_STEPS; ++k) s.pitch[k] = PITCH_EMPTY;
    s.set_pitch_count(nc);
    stale = true;
  }

  /// Rotate time data only. Pitch stream stays in place; new NOTE positions
  /// consume pitches in stream order (TB-303 independent-stream semantic).
  void RotateTimeLeft() {
    Sequence &s = get_edit_sequence();
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
    Sequence &s = get_edit_sequence();
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
    Sequence &s = get_edit_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    for (uint8_t k = 0; k < pc; ++k) {
      s.pitch[k] = s.scale_apply_packed(fast_rand_pitch_byte_weighted())
                   | fast_rand_accent_weighted()
                   | fast_rand_slide_weighted();
    }
    stale = true;
  }

  /// Randomize time data only. Existing pitches are preserved in stream order;
  /// pitch_count auto-extends if the new note_count exceeds it.
  void RandomizeTimeData() {
    Sequence &s = get_edit_sequence();
    const uint8_t len = s.length;
    fast_rand_seed();
    uint8_t prev = 1;
    for (uint8_t i = 0; i < len; i++) {
      const uint8_t t = fast_rand_time_weighted(prev, i == 0);
      sequence_set_time_at(s, i, t);
      prev = t;
    }
    normalize_pattern_times_only(s);
    sequence_ensure_pitch_for_notes(s);
    stale = true;
  }

  void RandomizeAccentData() {
    Sequence &s = get_edit_sequence();
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
    Sequence &s = get_edit_sequence();
    const uint8_t pc = s.get_pitch_count();
    fast_rand_seed();
    uint8_t k = 0;
    for (uint8_t i = 0; i < s.length; ++i) {
      if (s.time(i) == 1) {
        if (k < pc && s.pitch[k] != PITCH_EMPTY) {
          const uint8_t sl = fast_rand_slide_weighted();
          s.pitch[k] = (s.pitch[k] & ~uint8_t(0x80)) | sl;
        }
        ++k;
      }
    }
    stale = true;
  }

  void Mutate() {
    Sequence &s = get_edit_sequence();
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
              const uint8_t pk = s.scale_apply_packed(fast_rand_pitch_byte_weighted());
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
            if (slot < s.get_pitch_count() && s.pitch[slot] != PITCH_EMPTY)
              s.pitch[slot] ^= 0x80;
          }
          break;
        }
        case 3: {
          // Flip rest <-> note (avoid creating rest->tie issues at this step).
          const uint8_t nt = (t_i == 0) ? 1 : (t_i == 1) ? 0 : 1;
          sequence_write_time_with_pitch_sync(s, i, nt);
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
              nlin = s.scale_quantize_linear(uint8_t(nlin));
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

  /// Shift entire pattern (pitch + time) one step LEFT within length.
  void ShiftPatternLeft() {
    Sequence &s = get_edit_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    uint8_t per_time[MAX_STEPS];
    sequence_pack_per_time(s, per_time);
    const uint8_t ft = s.time(0);
    const uint8_t fp = per_time[0];
    for (uint8_t i = 0; i < len - 1; ++i) {
      sequence_set_time_at(s, i, s.time(uint8_t(i + 1)));
      per_time[i] = per_time[i + 1];
    }
    sequence_set_time_at(s, uint8_t(len - 1), ft);
    per_time[len - 1] = fp;
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  void ShiftPatternRight() {
    Sequence &s = get_edit_sequence();
    const uint8_t len = s.length;
    if (len < 2) return;
    uint8_t per_time[MAX_STEPS];
    sequence_pack_per_time(s, per_time);
    const uint8_t lt = s.time(uint8_t(len - 1));
    const uint8_t lp = per_time[len - 1];
    for (int i = int(len - 1); i > 0; --i) {
      sequence_set_time_at(s, uint8_t(i), s.time(uint8_t(i - 1)));
      per_time[i] = per_time[i - 1];
    }
    sequence_set_time_at(s, 0, lt);
    per_time[0] = lp;
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  /// Rotate pitch stream only (NOTE-event order) one slot left.
  void RotatePitchLeft() {
    Sequence &s = get_edit_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc < 2) return;
    const uint8_t first = s.pitch[0];
    for (uint8_t k = 0; k + 1 < pc; ++k) s.pitch[k] = s.pitch[k + 1];
    s.pitch[pc - 1] = first;
    stale = true;
  }
  void RotatePitchRight() {
    Sequence &s = get_edit_sequence();
    const uint8_t pc = s.get_pitch_count();
    if (pc < 2) return;
    const uint8_t last = s.pitch[pc - 1];
    for (int k = int(pc - 1); k > 0; --k) s.pitch[k] = s.pitch[k - 1];
    s.pitch[0] = last;
    stale = true;
  }

  /// Reverse the entire pattern (pitch + time) within length.
  void ReversePattern() {
    Sequence &s = get_edit_sequence();
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
    }
    normalize_pattern_times_only(s);
    sequence_unpack_per_time(s, per_time);
    stale = true;
  }

  void ClearPitchesOnly() {
    Sequence &s = get_edit_sequence();
    const uint8_t pc = s.get_pitch_count();
    for (uint8_t k = 0; k < pc; ++k) s.pitch[k] = PITCH_DEFAULT;
    stale = true;
  }

  void ClearTimesOnly() {
    Sequence &s = get_edit_sequence();
    const uint8_t len = s.length;
    for (uint8_t i = 0; i < len; ++i) sequence_set_time_at(s, i, 0);
    // Pitch stream preserved (queued for when NOTEs come back).
    stale = true;
  }

  void StampAllAccent() {
    Sequence &s = get_edit_sequence();
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
    Sequence &s = get_edit_sequence();
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
    // Walk pitch slots: set/clear slide on each NOTE's pitch byte.
    uint8_t k = 0;
    for (uint8_t i = 0; i < s.length && k < pc; ++i) {
      if (s.time(i) == 1) {
        if (s.pitch[k] != PITCH_EMPTY) {
          if (all_set) s.pitch[k] &= ~0x80;
          else         s.pitch[k] |=  0x80;
        }
        ++k;
      }
    }
    stale = true;
  }

  void RandomizeSemitones() {
    Sequence &s = get_edit_sequence();
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
    Sequence &s = get_edit_sequence();
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
    // A cleared pattern must not keep ghost step probabilities.
    if (idx == p_select) {
      memset(prob_var1_, 0, sizeof(prob_var1_));
      prob_stale_ = true;
      shadow_dirty_ms_ = millis();
      Reset();
      mode_ = NORMAL_MODE;
    } else {
      prob_edit_blob(abs_slot(idx), nullptr); // stage an all-zero table
    }
  }

  // ---------------------------------------------------------------------------
  // Getters
  // ---------------------------------------------------------------------------
  SequencerMode get_mode() const { return mode_; }
  // True while the panel is editing variation 3's polyphonic chord voice. The mono
  // pitch-write/audition paths defer to ProcessPolyEdit in this state.
  bool in_poly_pitch_edit() const { return mode_ == PITCH_MODE && edit_var_ == 2 && poly_active_; }

  Sequence &get_sequence() { return pattern[p_select]; }
  const Sequence &get_sequence() const { return pattern[p_select]; }
  const Sequence &get_pattern(uint8_t idx) const { return pattern[idx & 0xf]; }

  // Hardware edit target: variation 1 (the playback/CV buffer) or one of the two
  // resident shadow voices (variations 2/3). Playback never uses this; only the
  // hardware pattern-write UI does, so var2/3 edits go to the shadow buffers.
  Sequence &get_edit_sequence() {
    if (edit_var_ == 0) return pattern[p_select];
    shadow_stale_ = true;                 // a shadow is the edit target -> persist on save
    shadow_dirty_ms_ = millis();          // refresh the idle-flush quiet timer
    return shadow_[(edit_var_ - 1) & 0x1];
  }
  // Read-only view of the edit target (no dirty mark) for LED/display code.
  const Sequence &edit_seq_view() const {
    return (edit_var_ == 0) ? pattern[p_select] : shadow_[(edit_var_ - 1) & 0x1];
  }
  uint8_t get_edit_var() const { return edit_var_; }
  // Playhead of the variation being edited (its shadow for var2/3), for the chase
  // LEDs so each variation's chase follows its own length, not variation 1's.
  uint8_t get_edit_time_pos() const {
    // Poly var3 plays from poly_ (its mono shadow is frozen), so its chase must
    // follow poly_time_pos_, not the silenced shadow_[1].
    if (poly_active_ && edit_var_ == 2) return uint8_t(poly_time_pos_ & (MAX_STEPS - 1));
    return uint8_t(edit_seq_view().time_pos & (MAX_STEPS - 1));
  }
  bool SetEditVar(uint8_t v) { if (v >= NUM_VARIATIONS || v == edit_var_) return false; edit_var_ = v; return true; }

  // ---------------------------------------------------------------------------
  // A/B sections (spec 1-a). Slots p and p+8 of the active bank are the A and
  // B sections of one pattern; the link flag is stored on the A section, so
  // both slots answer the same question. A linked pair plays A then B and
  // takes up to 128 steps between them.
  // ---------------------------------------------------------------------------
  static uint8_t section_a_of(uint8_t pat) { return uint8_t(pat & 0x07); }
  static uint8_t section_b_of(uint8_t pat) { return uint8_t((pat & 0x07) + 8); }
  static bool    is_section_b(uint8_t pat) { return (pat & 0x08) != 0; }
  bool pair_linked(uint8_t pat) const { return pattern[section_a_of(pat)].ab_linked(); }
  bool pair_linked() const { return pair_linked(get_patsel()); }
  void set_pair_linked(uint8_t pat, bool on) {
    Sequence &a = pattern[section_a_of(pat)];
    if (a.ab_linked() == on) return;
    a.set_ab_linked(on);
    stale = true;
  }
  /// Total steps across the pair: A alone, or A + B when linked.
  uint8_t pair_length(uint8_t pat) const {
    const uint8_t la = pattern[section_a_of(pat)].length;
    if (!pair_linked(pat)) return la;
    const uint16_t t = uint16_t(la) + uint16_t(pattern[section_b_of(pat)].length);
    return uint8_t(t > 128 ? 128 : t);
  }
  // Advance the edit cursor: variation 1 uses the full engine advance;
  // a shadow just steps its own cursor forward.
  void AdvanceEditCursor() {
    if (edit_var_ != 0) { shadow_[(edit_var_ - 1) & 0x1].Advance(); return; }
    Advance();
  }

  // Gate window: 50% of the step period in 16ths (clk_count < 3 of 6) and in
  // triplets (clk_count < 4 of 8).
  bool get_gate() const {
    if (resting) return false;
    const uint8_t half = uint8_t(step_period() >> 1);
    if (get_slide_dac()) return slide_gate ? true : (clk_count < int8_t(half));
    if (slide_gate) return true;
    return clk_count < int8_t(half);
  }

  bool get_accent() const {
    if (prob_accent_armed_) return !resting && prob_accent_pass_;
    return !resting && get_sequence().get_accent();
  }
  uint8_t get_pitch() const {
    return get_sequence().get_pitch();
  }
  // Playback pitch quantized to the active pattern's scale (identity when the
  // pattern's scale is disabled). Used for CV + MIDI output; non-destructive.
  // The latched probability transpose composes on top of the quantized pitch
  // (octave shifts preserve the pitch class) and is already range-degraded,
  // so the result is -12..52: signed, because shifts may reach the DAC
  // headroom an octave below bottom C / 4 semitones above double-up high C.
  int8_t get_pitch_scaled() const {
    const Sequence &s = get_sequence();
    return int8_t(int(s.scale_quantize_linear(s.get_pitch())) + prob_transpose_);
  }
  uint8_t get_midi_note() const {
    return uint8_t(36 + get_sequence().get_pitch());
  }
  // Signed pattern transpose (-24..+24, 0 = none). Stored as int8 in the byte.
  int8_t get_pattern_transpose() const {
    return int8_t(get_sequence().transpose);
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
  const uint8_t get_length() const {
    return get_sequence().length;
  }

  // ---------------------------------------------------------------------------
  // Setters
  // ---------------------------------------------------------------------------
  void SetPattern(uint8_t p_, bool override = false) {
    next_p = p_ & 0xf;
    if (override) p_select = next_p;
    edit_var_ = 0; // a new pattern always starts on variation 1 for editing
  }
  // Canonical length change: clamps to the triplet cap (48) or MAX_STEPS and
  // rebuilds pitch_count (NOTE events outside the new length no longer count).
  // NON-VOLATILE both ways (spec 5): time_data and pitch[] past the new last
  // step are kept, so shortening then re-lengthening restores the original
  // notes. Used by the hardware editor (via SetLength) and SysEx 0x18.
  void ApplyLength(Sequence &s, uint8_t len) {
    const uint8_t old_len = s.length;
    const uint8_t cap = s.is_triplet_mode() ? uint8_t(TRIPLET_MAX_STEPS)
                                            : uint8_t(MAX_STEPS);
    s.SetLength(len, cap);
    if (s.length != old_len) {
      sequence_rebuild_pitch_count(s);
      sequence_ensure_pitch_for_notes(s);
    }
    stale = true;
  }
  void SetLength(uint8_t len) {
    ApplyLength(get_edit_sequence(), len);
  }
  void SetMode(SequencerMode m, bool reset = false) {
    if (reset && m != mode_) Reset();
    mode_ = m;
  }
  void NudgeOctave(int dir) {
    get_edit_sequence().nudge_octave_buttons(dir);
    stale = true;
  }
  void SetPitchSemitone(uint8_t p) {
    get_edit_sequence().SetPitchSemitone(p);
    stale = true;
  }
  void SetPitch(uint8_t p, uint8_t flags) {
    get_edit_sequence().SetPitch(p, flags);
    stale = true;
  }
  /// TIME_MODE write. Surgically maintains pitch_count + pitch[].
  void SetTime(uint8_t t) {
    Sequence &s = get_edit_sequence();
    const uint8_t tp = uint8_t(s.time_pos & (MAX_STEPS - 1));
    sequence_write_time_with_pitch_sync(s, tp, t);
    stale = true;
  }
  void ToggleSlide() {
    if (mode_ == PITCH_MODE)
      get_edit_sequence().ToggleSlide();
    stale = true;
  }
  void ToggleAccent() {
    if (mode_ == PITCH_MODE)
      get_edit_sequence().ToggleAccent();
    stale = true;
  }

  bool StepBack() {
    bool moved = get_edit_sequence().StepBack();
    if (moved) stale = true;
    return moved;
  }

  bool is_step_locked() const {
    if (mode_ == NORMAL_MODE) return false;
    const Sequence &s = edit_seq_view();
    return s.step_locked(uint8_t(s.time_pos));
  }

  // ---------------------------------------------------------------------------
  // SysEx blob (PATTERN_SIZE = 92 bytes: pitch[64] + time_data[16] + reserved[9]
  // + transpose + engine_select + length). Layout matches Sequence struct memory.
  // ---------------------------------------------------------------------------
  void export_pattern_blob(uint8_t idx, uint8_t *blob) const {
    idx &= 0xf;
    memcpy(blob, pattern[idx].pitch, PATTERN_SIZE);
  }
  // Export one variation's blob: var1 = pattern[idx]; var2/3 = the resident
  // shadow if idx is the active slot, else read from flash. Used to send all 3
  // variations to the web editor on a full request.
  void export_pattern_blob_var(uint8_t idx, uint8_t var, uint8_t *blob) {
    idx &= uint8_t(NUM_PATTERNS - 1);
    if (var == 0 || var >= NUM_VARIATIONS) { memcpy(blob, pattern[idx].pitch, PATTERN_SIZE); return; }
    if (idx == p_select) { memcpy(blob, shadow_[(var - 1) & 0x1].pitch, PATTERN_SIZE); return; }
    // A buffered (not yet flushed) non-resident edit supersedes the flash copy.
    if (shadow_edit_dirty_ && shadow_edit_slot_ == int16_t(abs_slot(idx)) &&
        shadow_edit_var_ == var) {
      memcpy(blob, shadow_edit_.pitch, PATTERN_SIZE);
      return;
    }
    Sequence tmp;
    ReadPatternAt(tmp, abs_slot(idx), var);
    memcpy(blob, tmp.pitch, PATTERN_SIZE);
  }

  bool import_pattern_blob(uint8_t idx, const uint8_t *blob, bool persist_eeprom = true) {
    idx &= 0xf;
    uint8_t L = blob[PATTERN_SIZE - 1];
    if (L < 1 || uint8_t(L) > MAX_STEPS) return false;
    const uint8_t gmax = MAX_STEPS;
    if (L > gmax) L = gmax;
    memcpy(pattern[idx].pitch, blob, PATTERN_SIZE);
    pattern[idx].length = L;
    Sequence &s = pattern[idx];
    sequence_rebuild_pitch_count(s);
    normalize_pattern_times(s);
    if (s.time_pos  >= s.length) s.time_pos  = 0;
    if (s.pitch_pos >= int(s.get_pitch_count()))
      s.pitch_pos = (s.get_pitch_count() > 0) ? int(s.get_pitch_count() - 1) : 0;
    if (persist_eeprom)
      persist_pattern(idx);
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
    Sequence &s = get_edit_sequence();
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
