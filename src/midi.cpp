/*
 * SuperOS-303 pattern SysEx (host ↔ DIN MIDI)
 * API: midi_api.h — implementation in this file.
 *
 * Framing: F0 7D <cmd> ... F7   (manufacturer ID 0x7D non-commercial)
 *
 * Pattern commands:
 *   10h  Host→303  Request one pattern: <pat:0..15>
 *   11h  303→Host  Pattern data: <pat> <xor_lo7> <xor_hi1> <packed 128 bytes>
 *   12h  Host→303  Set pattern: same as 11h body; XOR over raw 128 bytes must match.
 *   13h  Host→303  Request all 16 patterns (303 sends sixteen 11h messages, queued).
 *   14h  303→Host  ACK/NAK: <status> 0=ok 1=bad_checksum 2=bad_pattern 3=blocked_clock
 *   15h  303→Host  Step position: <pat:0..15> <step:0..63>  (sent each 16th while running)
 *   19h  303→Host  Step lock: <pat:0..15> <step:0..63> <locked:0|1>
 *
 * Config commands:
 *   20h  Host→303  Request config
 *   21h  303→Host  Config data: <midi_ch:0..16> <flags: bit0=clock_receive>
 *   22h  Host→303  Set config: same as 21h body
 *
 * Slide fix: note_off_cb tracks the most-recent live note; only clears s_live_gate when
 * the Note Off matches that note. This prevents releasing note 1 from killing note 2
 * during a live slide (two-finger legato play).
 */

#include <Arduino.h>
#include <MIDI.h>
#include <string.h>
#include "engine.h"
#include "pins.h"
#include "midi_api.h"

struct SuperOsMidiSettings {
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const unsigned SysExMaxSize = 256;
  static const bool UseSenderActiveSensing = false;
  static const bool UseReceiverActiveSensing = false;
  static const uint16_t SenderActiveSensingPeriodicity = 0;
};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, SuperOsMidiSettings);

static Engine *g_eng = nullptr;
static bool g_clk_run = false;
static uint8_t s_in_channel = 0; // 0 = omni
static bool s_midi_clock_rx = true;
static bool s_midi_thru = false;
// Deferred settings persistence: set by the 0x22 handler, flushed by
// midi_flush_pending_saves() so EEPROM writes don't stall RX inside SysEx parsing.
static bool s_settings_dirty = false;

// Per-pattern dirty bitmap for deferred pattern-EEPROM persistence. Any
// SysEx that mutates pattern RAM (0x12, 0x16, 0x18, 0x19, 0x1B) sets the
// corresponding bit. midi_flush_pending_pattern_saves() writes one pattern
// per call when idle so the EEPROM stall stays short.
static uint16_t s_pat_dirty_mask = 0;
static uint32_t s_last_web_edit_ms = 0;
static inline void mark_pat_dirty(uint8_t pat) {
  s_pat_dirty_mask |= uint16_t(1u << (pat & 0x0f));
  s_last_web_edit_ms = millis();
}

static uint8_t out_ch() {
  return s_in_channel == 0 ? 1 : s_in_channel;
}

// Per-variation MIDI output channel. Variation 0 (index sentinel 0) routes
// through out_ch() so omni/in-channel behaviour is preserved; variations 1/2
// use the stored var2/var3 channels.
static uint8_t s_var_channel[NUM_VARIATIONS] = {0, 2, 3};
static uint8_t out_ch_for_var(uint8_t var) {
  return (var >= NUM_VARIATIONS || var == 0 || s_var_channel[var] == 0)
         ? out_ch() : s_var_channel[var];
}
void midi_set_var_channels(uint8_t v2, uint8_t v3) {
  s_var_channel[1] = (v2 >= 1 && v2 <= 16) ? v2 : 2;
  s_var_channel[2] = (v3 >= 1 && v3 <= 16) ? v3 : 3;
}

void midi_apply_settings(uint8_t midi_in_channel_0_omni_16, bool midi_clock_receive, bool midi_thru) {
  s_in_channel = midi_in_channel_0_omni_16 <= 16 ? midi_in_channel_0_omni_16 : 1;
  s_midi_clock_rx = midi_clock_receive;
  s_midi_thru = midi_thru;
  if (s_in_channel == 0)
    MIDI.begin(MIDI_CHANNEL_OMNI);
  else
    MIDI.begin(s_in_channel);
  if (s_midi_thru)
    MIDI.turnThruOn();
  else
    MIDI.turnThruOff();
}

uint8_t midi_sequencer_out_channel() { return out_ch(); }

// --- TX queue (non-blocking multi-pattern dump) ---------------------------------
static const uint16_t kTxCap = 512;
static uint8_t s_tx[kTxCap];
static uint16_t s_tx_w, s_tx_r;

static void tx_clear() {
  s_tx_w = s_tx_r = 0;
}

static bool tx_push_byte(uint8_t b) {
  uint16_t n = uint16_t(s_tx_w + 1);
  if (n >= kTxCap) n = 0;
  if (n == s_tx_r) return false;
  s_tx[s_tx_w] = b;
  s_tx_w = n;
  return true;
}

static void midi_tx_drain() {
  while (s_tx_r != s_tx_w && Serial1.availableForWrite() > 0) {
    Serial1.write(s_tx[s_tx_r]);
    s_tx_r++;
    if (s_tx_r >= kTxCap) s_tx_r = 0;
  }
}

static bool tx_push_message(const uint8_t *inner, uint16_t inner_len) {
  // Check space BEFORE writing anything to avoid partial SysEx corruption.
  // A partial F0...no-F7 left in the buffer would break the MIDI output stream.
  uint16_t avail = (s_tx_r + kTxCap - s_tx_w - 1) % kTxCap;
  if (avail < inner_len + 2) return false; // +2 for F0 and F7
  tx_push_byte(0xF0);
  for (uint16_t i = 0; i < inner_len; ++i) tx_push_byte(inner[i]);
  tx_push_byte(0xF7);
  return true;
}

// --- 7-bit pack / unpack (PATTERN_SIZE raw -> kPackedPatternLen packed) -------
static uint16_t pack_7bit(const uint8_t *src, uint16_t len, uint8_t *out) {
  uint16_t o = 0;
  for (uint16_t i = 0; i < len; i += 7) {
    uint8_t msb = 0;
    const uint8_t n = (len - i >= 7) ? 7 : static_cast<uint8_t>(len - i);
    for (uint8_t b = 0; b < n; ++b)
      if (src[i + b] & 0x80) msb |= static_cast<uint8_t>(1u << b);
    out[o++] = msb;
    for (uint8_t b = 0; b < n; ++b)
      out[o++] = src[i + b] & 0x7F;
  }
  return o;
}

// 7-bit packing inflation: every 7 raw bytes -> 1 MSB byte + 7 data bytes.
// 48 raw -> 6 full chunks (48 packed bytes) + 1 partial (1 + 6 = 7) = 55.
static constexpr uint16_t kPackedPatternLen =
    PATTERN_SIZE + ((PATTERN_SIZE + 6) / 7);

static bool unpack_7bit(const uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t out_len) {
  uint16_t oi = 0, ii = 0;
  while (oi < out_len) {
    if (ii >= in_len) return false;
    const uint8_t msb = in[ii++];
    for (uint8_t b = 0; b < 7 && oi < out_len; ++b) {
      if (ii >= in_len) return false;
      out[oi] = static_cast<uint8_t>(in[ii++] | ((msb & (1u << b)) ? 0x80u : 0u));
      ++oi;
    }
  }
  return true;
}

static uint8_t xor_blob_pattern(const uint8_t *p) {
  uint8_t x = 0;
  for (uint16_t i = 0; i < PATTERN_SIZE; ++i)
    x ^= p[i];
  return x;
}

static void enqueue_pattern_reply(uint8_t pat, uint8_t var = 0) {
  if (!g_eng) return;
  uint8_t raw[PATTERN_SIZE];
  g_eng->export_pattern_blob_var(pat, var, raw);
  const uint8_t cx = xor_blob_pattern(raw);
  uint8_t inner[5 + kPackedPatternLen + 1];
  inner[0] = 0x7D;
  inner[1] = 0x11;
  inner[2] = pat & 0x0F;
  inner[3] = static_cast<uint8_t>(cx & 0x7F);
  inner[4] = static_cast<uint8_t>((cx >> 7) & 1);
  const uint16_t pl = pack_7bit(raw, PATTERN_SIZE, inner + 5);
  inner[5 + pl] = static_cast<uint8_t>(var & 0x03); // trailing variation byte
  tx_push_message(inner, static_cast<uint16_t>(5 + pl + 1));
}

static void send_ack(uint8_t status) {
  const uint8_t inner[3] = {0x7D, 0x14, status};
  tx_push_message(inner, 3);
}

// --- Chain state RX (from web via SysEx 0x1A) -----------------------------------
static bool    s_rx_chain_pending     = false;
static uint8_t s_rx_chain_active_len  = 0;
static uint8_t s_rx_chain_active_pats[4] = {};
static uint8_t s_rx_chain_queued_len  = 0;
static uint8_t s_rx_chain_queued_pats[4] = {};

void midi_send_chain_state(uint8_t active_len, const uint8_t *active_pats,
                            uint8_t queued_len, const uint8_t *queued_pats) {
  uint8_t inner[12];
  inner[0] = 0x7D; inner[1] = 0x1A;
  inner[2] = active_len & 0x07;
  for (uint8_t i = 0; i < 4; ++i)
    inner[3 + i] = (active_pats && i < active_len) ? (active_pats[i] & 0x0F) : 0;
  inner[7] = queued_len & 0x07;
  for (uint8_t i = 0; i < 4; ++i)
    inner[8 + i] = (queued_pats && i < queued_len) ? (queued_pats[i] & 0x0F) : 0;
  tx_push_message(inner, 12);
}

bool midi_get_received_chain(uint8_t *out_active_len, uint8_t out_active_pats[4],
                              uint8_t *out_queued_len, uint8_t out_queued_pats[4]) {
  if (!s_rx_chain_pending) return false;
  s_rx_chain_pending = false;
  if (s_rx_chain_active_len == 0xff) {
    // Sentinel: config request asked us to re-broadcast current state.
    // Signal main.cpp with active_len=0xff so it just calls emit_chain_state().
    *out_active_len = 0xff;
    *out_queued_len = 0;
    return true;
  }
  *out_active_len = s_rx_chain_active_len;
  for (uint8_t i = 0; i < 4; ++i) out_active_pats[i] = s_rx_chain_active_pats[i];
  *out_queued_len = s_rx_chain_queued_len;
  for (uint8_t i = 0; i < 4; ++i) out_queued_pats[i] = s_rx_chain_queued_pats[i];
  return true;
}

// --- Sequential dump after 0x13 -------------------------------------------------
static bool s_dump_active = false;
static uint8_t s_dump_next = 0;

static void dump_try_advance() {
  if (!s_dump_active || s_tx_r != s_tx_w) return;
  if (s_dump_next >= 16) { s_dump_active = false; return; }
  enqueue_pattern_reply(s_dump_next++);
}

// --- SysEx parse ----------------------------------------------------------------
static void handle_sysex_body(const uint8_t *p, unsigned n) {
  if (!g_eng || n < 2) return;
  if (p[0] != 0x7D) return;
  const uint8_t cmd = p[1];

  switch (cmd) {
  case 0x10: { // request pattern; optional trailing <var> selects the variation
    if (n < 3) return;
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t var = (n >= 4) ? (p[3] & 0x03) : 0;
    enqueue_pattern_reply(pat, var);
    break;
  }
  case 0x12: { // set pattern. Optional trailing byte = variation (0=var1/default,
               // 1=var2, 2=var3). var1 -> live RAM; var2/3 -> that slot's flash half.
    if (n < 5 + kPackedPatternLen) { send_ack(1); return; }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t cx = static_cast<uint8_t>(p[3] | (p[4] << 7));
    const uint8_t var = (n > 5 + kPackedPatternLen) ? (p[5 + kPackedPatternLen] & 0x03) : 0;
    uint8_t raw[PATTERN_SIZE];
    if (!unpack_7bit(p + 5, kPackedPatternLen, raw, PATTERN_SIZE)) { send_ack(1); return; }
    if (xor_blob_pattern(raw) != cx) { send_ack(1); return; }
    if (var == 0) {
      // Variation 1 (the 303 / CV voice). Never persist inline: a flash write
      // blocks the UART long enough to drop the next SysEx. RAM update only;
      // engine.stale persists at the natural save points (RUN stop, WRITE exit).
      if (!g_eng->import_pattern_blob(pat, raw, /*persist_eeprom=*/false)) { send_ack(2); return; }
      // Apply the blob's per-pattern direction to the live engine if active.
      if (pat == g_eng->get_patsel()) {
        const uint8_t d = g_eng->pattern[pat].get_direction_stored();
        g_eng->SetDirection(static_cast<SequenceDirection>(d));
      }
      g_eng->stale = true;
      mark_pat_dirty(pat);
    } else if (var < NUM_VARIATIONS) {
      // Variations 2/3 are MIDI-only shadow voices. For the active slot, edit the
      // resident shadow in RAM (no flash write -> no sequencer stall); it persists
      // on stop/switch. For a non-active slot (not resident), write flash directly.
      if (!g_eng->apply_shadow_blob(pat, var, raw)) {
        const uint8_t L = raw[PATTERN_SIZE - 1];
        if (L < 1 || L > MAX_STEPS) { send_ack(2); return; }
        Sequence tmp;
        deserialize_pattern(tmp, raw);
        tmp.length = L;
        sequence_rebuild_pitch_count(tmp);
        normalize_pattern_times(tmp);
        WritePatternAt(tmp, g_eng->abs_slot(pat), var);
      }
      s_last_web_edit_ms = millis(); // arm the idle-flush quiet timer for shadows
    }
    send_ack(0);
    break;
  }
  case 0x13: // request all
    s_dump_next = 0;
    s_dump_active = true;
    dump_try_advance();
    break;
  case 0x16: { // host → 303: set single step (pitch + time)
    if (n < 7 || !g_eng) return;
    const uint8_t pat  = p[2] & 0x0F;
    const uint8_t step = p[3] & 0x3F;
    const uint8_t pitchByte = static_cast<uint8_t>((p[4] & 0x7F) | ((p[5] & 0x01) << 7));
    const uint8_t timeNib   = p[6] & 0x0F;
    Sequence &seq = g_eng->pattern[pat];
    if (step >= seq.length) return;
    sequence_write_time_with_pitch_sync(seq, step, timeNib);
    // pitchByte == PITCH_EMPTY (0xFF) is the editor's "time-only edit"
    // sentinel: leave the pitch stream alone (auto-extension already ran).
    if (timeNib == 1 && pitchByte != PITCH_EMPTY) {
      const uint8_t slot = seq.pitch_index_for_note(step);
      if (slot < seq.get_pitch_count())
        seq.pitch[slot] = pitchByte;
    }
    g_eng->stale = true;
    mark_pat_dirty(pat);
    break;
  }
  case 0x18: { // host → 303: set pattern length
    if (n < 4 || !g_eng) return;
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t len = p[3] & 0x7F;
    if (len < 1 || len > MAX_STEPS) return;
    g_eng->pattern[pat].length = len;
    g_eng->stale = true;
    mark_pat_dirty(pat);
    break;
  }
  case 0x19: { // host → 303: set step lock (RAM-only since OS-303 v0.6 layout)
    if (n < 5 || !g_eng) return;
    const uint8_t pat    = p[2] & 0x0F;
    const uint8_t step   = p[3] & 0x3F;
    const bool    locked = (p[4] & 0x01) != 0;
    Sequence &seq = g_eng->pattern[pat];
    if (step >= MAX_STEPS) return;
    const uint8_t mask = uint8_t(1u << (step & 7));
    if (locked)
      seq.step_lock_ram[step >> 3] |= mask;
    else
      seq.step_lock_ram[step >> 3] &= ~mask;
    // step_lock is RAM-only in the OS-303 layout: do not mark stale.
    break;
  }
  case 0x1B: { // host → 303: set step ratchet value
    if (n < 5 || !g_eng) return;
    const uint8_t pat  = p[2] & 0x0F;
    const uint8_t step = p[3] & 0x3F;
    const uint8_t val  = p[4] & 0x03;
    Sequence &seq = g_eng->pattern[pat];
    if (step >= MAX_STEPS) return;
    seq.set_ratchet_val(step, val);
    g_eng->stale = true;
    mark_pat_dirty(pat);
    break;
  }
  case 0x1A: { // set chain state from host
    if (n < 12) return;
    s_rx_chain_active_len = p[2] & 0x07;
    if (s_rx_chain_active_len > 4) s_rx_chain_active_len = 4;
    for (uint8_t i = 0; i < 4; ++i)
      s_rx_chain_active_pats[i] = p[3 + i] & 0x0F;
    s_rx_chain_queued_len = p[7] & 0x07;
    if (s_rx_chain_queued_len > 4) s_rx_chain_queued_len = 4;
    for (uint8_t i = 0; i < 4; ++i)
      s_rx_chain_queued_pats[i] = p[8 + i] & 0x0F;
    s_rx_chain_pending = true;
    break;
  }
  case 0x1D: { // host → 303: queue pattern; immediate when stopped
    if (n < 3 || !g_eng) return;
    g_eng->SetPattern(p[2] & 0x0F, !g_clk_run);
    break;
  }
  case 0x1F: { // host → 303: set the hardware edit-target variation (0..2)
    if (n < 4 || !g_eng) return;
    g_eng->SetEditVar(p[3] & 0x03);
    break;
  }
  case 0x20: { // request config
    const uint8_t fl  = static_cast<uint8_t>((GlobalSettings.midi_clock_receive ? 1 : 0) |
                                              (GlobalSettings.midi_thru          ? 2 : 0));
    const uint8_t dir = g_eng ? static_cast<uint8_t>(g_eng->get_direction()) : 0;
    const uint8_t inner[8] = {0x7D, 0x21, GlobalSettings.midi_channel, fl, dir,
                              GlobalSettings.led_brightness,
                              GlobalSettings.var2_channel, GlobalSettings.var3_channel};
    tx_push_message(inner, 8);
    // Also broadcast current group so web editor syncs on connect
    if (g_eng) {
      const uint8_t grp[3] = {0x7D, 0x1C, g_eng->get_group()};
      tx_push_message(grp, 3);
    }
    s_rx_chain_pending = true; // signal main.cpp to re-broadcast chain state
    s_rx_chain_active_len = 0xff; // sentinel: "just re-emit current state"
    break;
  }
  case 0x22: { // set config
    if (n < 4) return;
    const uint8_t ch = p[2];
    const uint8_t fl = p[3];
    if (ch <= 16) GlobalSettings.midi_channel = ch;
    GlobalSettings.midi_clock_receive = (fl & 1) != 0;
    GlobalSettings.midi_thru          = (fl & 2) != 0;
    if (n >= 5 && g_eng) {
      const SequenceDirection d = static_cast<SequenceDirection>(p[4] & 0x07);
      g_eng->SetDirection(d);
      GlobalSettings.sequence_direction = static_cast<uint8_t>(d);
    }
    if (n >= 6) {
      const uint8_t br = p[5];
      if (br >= 1 && br <= 8) GlobalSettings.led_brightness = br;
      // main.cpp loop syncs Leds::brightness from GlobalSettings each tick.
    }
    if (n >= 8) { // per-variation 2/3 MIDI output channels (1..16)
      const uint8_t v2 = p[6], v3 = p[7];
      if (v2 >= 1 && v2 <= 16) GlobalSettings.var2_channel = v2;
      if (v3 >= 1 && v3 <= 16) GlobalSettings.var3_channel = v3;
      midi_set_var_channels(GlobalSettings.var2_channel, GlobalSettings.var3_channel);
    }
    // Defer EEPROM write to idle; inline save blocks UART RX long enough
    // to drop the next SysEx the web sends.
    s_settings_dirty = true;
    midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
    send_ack(0);
    // Echo back new config
    const uint8_t nfl  = static_cast<uint8_t>((GlobalSettings.midi_clock_receive ? 1 : 0) |
                                               (GlobalSettings.midi_thru          ? 2 : 0));
    const uint8_t ndir = g_eng ? static_cast<uint8_t>(g_eng->get_direction()) : 0;
    const uint8_t reply[8] = {0x7D, 0x21, GlobalSettings.midi_channel, nfl, ndir,
                              GlobalSettings.led_brightness,
                              GlobalSettings.var2_channel, GlobalSettings.var3_channel};
    tx_push_message(reply, 8);
    break;
  }
  default:
    break;
  }
}

static void sysex_cb(byte *data, unsigned sz) {
  if (sz < 4 || data[0] != 0xF0 || data[sz - 1] != 0xF7) return;
  handle_sysex_body(reinterpret_cast<const uint8_t *>(data + 1),
                    static_cast<unsigned>(sz - 2));
}

// --- Bank select state (for Ableton Program Change mapping) ----------------------
// CC 0 = group (0-3), CC 32 = section (0=Bank A patterns 0-7, 1=Bank B patterns 8-15)
// PC value = pattern within section (0-7)
// pattern_index = section * 8 + pc; group selects which group of 16 to load.
static uint8_t s_bank_group   = 0; // last received CC 0 value
static uint8_t s_bank_section = 0; // last received CC 32 value

// --- Live note stack (legato / slide-back) ---------------------------------------
// Tracks all physically-held notes so releasing the top note slides back to
// whatever is still held underneath (e.g. hold C, slide up to B, release B → slide back to C).
static constexpr uint8_t kNoteStackSize = 8;
static uint8_t s_note_stack[kNoteStackSize];
static uint8_t s_note_stack_vel[kNoteStackSize]; // velocity stored per note
static uint8_t s_note_stack_depth = 0;

static void live_stack_clear() {
  s_note_stack_depth = 0;
}

static void live_stack_remove(uint8_t note) {
  for (uint8_t i = 0; i < s_note_stack_depth; ++i) {
    if (s_note_stack[i] == note) {
      for (uint8_t j = i; j < s_note_stack_depth - 1; ++j) {
        s_note_stack[j]     = s_note_stack[j + 1];
        s_note_stack_vel[j] = s_note_stack_vel[j + 1];
      }
      --s_note_stack_depth;
      return;
    }
  }
}

static void live_stack_push(uint8_t note, uint8_t vel) {
  live_stack_remove(note); // remove if already present (avoid duplicates)
  if (s_note_stack_depth < kNoteStackSize) {
    s_note_stack[s_note_stack_depth]     = note;
    s_note_stack_vel[s_note_stack_depth] = vel;
    ++s_note_stack_depth;
  }
}

// --- Live gate / accent / slide -------------------------------------------------
static bool    s_live_accent = false;
static bool    s_live_gate   = false;
static bool    s_live_slide  = false;
static uint8_t s_live_note   = 0;

bool midi_live_accent() { return s_live_accent; }
bool midi_live_gate()   { return s_live_gate; }
bool midi_live_slide()  { return s_live_slide; }

static void note_on_cb(byte ch, byte pitch, byte vel) {
  if (vel == 0) return;
  if (s_in_channel != 0 && ch != s_in_channel) return;

  if (!g_clk_run) {
    const uint8_t prev_note = s_live_note;
    const bool was_playing  = s_live_gate;

    live_stack_push(static_cast<uint8_t>(pitch), static_cast<uint8_t>(vel));

    if (!s_midi_thru) {
      if (was_playing && prev_note != static_cast<uint8_t>(pitch)) {
        // Legato: slide from previous note — Note On new BEFORE Note Off old.
        MIDI.sendNoteOn(pitch, vel, ch);
        MIDI.sendNoteOff(prev_note, 0, ch);
        s_live_slide = true;
      } else {
        MIDI.sendNoteOn(pitch, vel, ch);
        s_live_slide = false;
      }
    } else {
      // Thru: note already forwarded by library; just update slide state.
      s_live_slide = (was_playing && prev_note != static_cast<uint8_t>(pitch));
    }

    s_live_accent = (vel >= 100);
    s_live_gate   = true;
    s_live_note   = static_cast<uint8_t>(pitch);
  } else if (!s_midi_thru) {
    MIDI.sendNoteOn(pitch, vel, ch);
  }

  if (g_eng)
    g_eng->midi_apply_note_on(static_cast<uint8_t>(pitch), static_cast<uint8_t>(vel));
}

static void note_off_cb(byte ch, byte pitch, byte vel) {
  (void)vel;
  if (s_in_channel != 0 && ch != s_in_channel) return;

  if (!g_clk_run) {
    const bool was_top = (static_cast<uint8_t>(pitch) == s_live_note);
    live_stack_remove(static_cast<uint8_t>(pitch));

    if (was_top && s_note_stack_depth > 0) {
      // Slide back: Note On new BEFORE Note Off old for portamento ordering.
      const uint8_t back_note = s_note_stack[s_note_stack_depth - 1];
      const uint8_t back_vel  = s_note_stack_vel[s_note_stack_depth - 1];
      MIDI.sendNoteOn(back_note, back_vel, ch);
      if (!s_midi_thru)
        MIDI.sendNoteOff(static_cast<uint8_t>(pitch), 0, ch);
      s_live_note  = back_note;
      s_live_slide = true;
      s_live_gate  = true;
      if (g_eng)
        g_eng->midi_apply_note_on(back_note, back_vel);
    } else {
      // When thru is off, explicitly send Note Off (covers normal release and
      // duplicate Note Off for already-slid notes, harmless to receiving synth).
      if (!s_midi_thru)
        MIDI.sendNoteOff(static_cast<uint8_t>(pitch), 0, ch);
      if (was_top) {
        s_live_gate  = false;
        s_live_slide = false;
        s_live_note  = 0;
      }
    }
  } else if (!s_midi_thru) {
    MIDI.sendNoteOff(pitch, 0, ch);
  }
}

// --- Audition note (pitch write / edit step preview) ----------------------------
static uint8_t s_audition_note = 0;
static bool    s_audition_on   = false;
static uint8_t s_audition_ch   = 0;

// Audition note (pitch write/edit/step) plays on the channel of the variation
// being edited, so stepping through variation 2/3 sounds the external synth on
// its own MIDI channel (variation 1 = the 303's main channel).
static uint8_t audition_ch() {
  return out_ch_for_var(g_eng ? g_eng->get_edit_var() : 0);
}

void midi_audition_note_on(uint8_t note, uint8_t vel) {
  const byte ch = static_cast<byte>(audition_ch());
  if (s_audition_on && (s_audition_note != note || s_audition_ch != ch)) {
    MIDI.sendNoteOff(s_audition_note, 0, s_audition_ch); // close prev on its channel
    s_audition_on = false;
  }
  if (!s_audition_on) {
    MIDI.sendNoteOn(note, vel, ch);
    s_audition_note = note;
    s_audition_ch   = ch;
    s_audition_on   = true;
  }
}

void midi_audition_note_off() {
  if (s_audition_on) {
    MIDI.sendNoteOff(s_audition_note, 0, s_audition_ch);
    s_audition_on = false;
  }
}

// --- Step position broadcast (SysEx 0x15) ---------------------------------------
// Wrap-only anchor: callers must send this only at pattern wrap (time_pos -> 0)
// or other low-rate events. Per-16th sends produced an audible click because the
// 7-byte burst current-pulse coupled into the analog audio rail. The web editor
// counts incoming 24 PPQN MIDI clock to interpolate steps between anchors.
void midi_send_step_position(uint8_t pat, uint8_t step) {
  const uint8_t grp = g_eng ? g_eng->get_group() : 0;
  const uint8_t inner[5] = {0x7D, 0x15, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F), (uint8_t)(grp & 0x03)};
  tx_push_message(inner, 5);
}

// --- Length update broadcast (SysEx 0x18) ----------------------------------------
void midi_send_length_update(uint8_t pat, uint8_t len, uint8_t var) {
  const uint8_t inner[5] = {0x7D, 0x18, (uint8_t)(pat & 0x0F), (uint8_t)(len & 0x7F),
                            (uint8_t)(var & 0x03)};
  tx_push_message(inner, 5);
}

// --- Direction update broadcast (SysEx 0x17) -------------------------------------
void midi_send_direction_update(uint8_t direction, uint8_t var) {
  const uint8_t inner[4] = {0x7D, 0x17, (uint8_t)(direction & 0x07), (uint8_t)(var & 0x03)};
  tx_push_message(inner, 4);
}

// --- Track state broadcast (SysEx 0x23) -----------------------------------------
// Telemetry for the web editor's Track view. All payload bytes are 7-bit safe.
// Layout: [10 header bytes] [MAX_CHAIN pattern nibbles] [MAX_CHAIN transpose bytes]
//         [MAX_CHAIN last-flag bools]. With MAX_CHAIN=64 inner = 10 + 64*3 = 202 B.
void midi_send_track_state(uint8_t dial_mode, uint8_t track_idx, bool track_active,
                            bool clk_run, Engine &engine) {
  uint8_t inner[10 + MAX_CHAIN * 3];
  inner[0] = 0x7D;
  inner[1] = 0x23;
  inner[2] = dial_mode & 0x03;
  inner[3] = track_idx & 0x07;
  inner[4] = track_active ? 1 : 0;
  inner[5] = clk_run ? 1 : 0;
  inner[6] = engine.get_chain_len() & 0x7F;
  inner[7] = engine.get_chain_pos() & 0x7F;
  inner[8] = engine.get_patsel() & 0x0F;
  inner[9] = engine.get_group() & 0x03;
  for (uint8_t i = 0; i < MAX_CHAIN; ++i) {
    inner[10 + i]                 = engine.p_chain_get(i) & 0x0F;
    inner[10 + MAX_CHAIN + i]     = engine.TrackGetTranspose(i) & 0x7F;
    inner[10 + MAX_CHAIN * 2 + i] = engine.t_chain_last_get(i) ? 1 : 0;
  }
  tx_push_message(inner, sizeof(inner));
}

// --- Group update broadcast (SysEx 0x1C) -----------------------------------------
void midi_send_group_update(uint8_t group) {
  const uint8_t inner[3] = {0x7D, 0x1C, (uint8_t)(group & 0x03)};
  tx_push_message(inner, 3);
}

// Which variation the hardware is currently editing (303<->editor sync, SysEx 0x1F).
void midi_send_edit_variation(uint8_t pat, uint8_t var) {
  const uint8_t inner[4] = {0x7D, 0x1F, (uint8_t)(pat & 0x0F), (uint8_t)(var & 0x03)};
  tx_push_message(inner, 4);
}

// --- Active pattern broadcast (SysEx 0x1E) ---------------------------------------
// Used while stopped so the web editor follows hardware pat-key presses
// without flagging the pill as "playing" (which 0x15 would do). Includes the
// current group so the web can resync even if its hwGroup state is stale.
void midi_send_active_pattern(uint8_t pat) {
  const uint8_t grp = g_eng ? g_eng->get_group() : 0;
  const uint8_t inner[4] = {0x7D, 0x1E, (uint8_t)(pat & 0x0F), (uint8_t)(grp & 0x03)};
  tx_push_message(inner, 4);
}

// --- Metronome MIDI notes --------------------------------------------------------
static uint8_t s_metro_note_on = 0;

void midi_metronome_tick(bool first_beat) {
  // E5 = MIDI 76 (accented downbeat), E6 = MIDI 88 (all other beats)
  const uint8_t note = first_beat ? 76 : 88;
  const uint8_t vel  = first_beat ? 127 : 80;
  if (s_metro_note_on) {
    MIDI.sendNoteOff(s_metro_note_on, 0, static_cast<byte>(out_ch()));
  }
  MIDI.sendNoteOn(note, vel, static_cast<byte>(out_ch()));
  s_metro_note_on = note;
}

void midi_metronome_stop() {
  if (s_metro_note_on) {
    MIDI.sendNoteOff(s_metro_note_on, 0, static_cast<byte>(out_ch()));
    s_metro_note_on = 0;
  }
}

// --- Full pattern broadcast (SysEx 0x11) ----------------------------------------
// Used after hardware edits that change the whole pattern (e.g. Clear).
void midi_send_pattern_update(uint8_t pat) {
  enqueue_pattern_reply(pat & 0x0F);
}

void midi_send_pattern_steps(uint8_t pat, const Sequence &seq, uint8_t len) {
  pat &= 0x0F;
  for (uint8_t k = 0; k < len; ++k) {
    const uint8_t tt = seq.time(k);
    uint8_t pb = PITCH_EMPTY;
    if (tt == 1) {
      const uint8_t slot = seq.pitch_index_for_note(k);
      if (slot < seq.get_pitch_count()) pb = seq.pitch[slot];
    }
    midi_send_step_update(pat, k, pb, tt);
  }
}

// --- Step lock broadcast (SysEx 0x19) -------------------------------------------
void midi_send_step_lock_update(uint8_t pat, uint8_t step, bool locked) {
  const uint8_t inner[5] = {0x7D, 0x19, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F), uint8_t(locked)};
  tx_push_message(inner, 5);
}

// --- Ratchet broadcast (SysEx 0x1B) ---------------------------------------------
// Same format as host-to-device 0x1B. Web editor listens symmetrically.
void midi_send_ratchet_update(uint8_t pat, uint8_t step, uint8_t val, uint8_t var) {
  const uint8_t inner[6] = {
    0x7D, 0x1B,
    static_cast<uint8_t>(pat & 0x0F),
    static_cast<uint8_t>(step & 0x3F),
    static_cast<uint8_t>(val & 0x03),
    static_cast<uint8_t>(var & 0x03)
  };
  tx_push_message(inner, 6);
}

// --- Step edit broadcast (SysEx 0x16) -------------------------------------------
// Trailing <var> tags which variation the edit belongs to (0=var1, 1/2=var2/3).
void midi_send_step_update(uint8_t pat, uint8_t step, uint8_t pitch_byte, uint8_t time_nibble,
                           uint8_t var) {
  const uint8_t inner[8] = {
    0x7D, 0x16,
    static_cast<uint8_t>(pat & 0x0F),
    static_cast<uint8_t>(step & 0x3F),
    static_cast<uint8_t>(pitch_byte & 0x7F),       // low 7 bits
    static_cast<uint8_t>((pitch_byte >> 7) & 0x01), // bit 7 (slide/empty flag)
    static_cast<uint8_t>(time_nibble & 0x0F),
    static_cast<uint8_t>(var & 0x03)
  };
  tx_push_message(inner, 8);
}

// --- midi_init ------------------------------------------------------------------
void midi_init(Engine *engine) {
  g_eng = engine;
  // Enable internal pull-up on the MIDI RX pin so an unplugged 3.5mm jack
  // does not leave PD2 floating and self-clocking the UART from EMI picked
  // up off the LED matrix / DAC port writes.
  pinMode(MIDI_IN_PIN, INPUT_PULLUP);
  Serial1.begin(31250);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleSystemExclusive(sysex_cb);
  MIDI.setHandleNoteOn(note_on_cb);
  MIDI.setHandleNoteOff(note_off_cb);
  tx_clear();
  s_dump_active = false;
}

static uint8_t s_seq_note = 0;
static bool s_seq_note_on = false;
static uint8_t s_seq_note_ch = 0;   // channel the open main note was sent on

void midi_poll(Engine &engine, bool clk_run, bool &midi_clk, uint8_t &midi_clock_pulses) {
  g_eng = &engine;
  if (clk_run && !g_clk_run) {
    // Transitioning to running: discard any held live notes.
    live_stack_clear();
    s_live_gate  = false;
    s_live_slide = false;
    s_live_note  = 0;
  }
  g_clk_run = clk_run;
  midi_clock_pulses = 0;

  if (!s_midi_clock_rx) midi_clk = false;

  if (!clk_run && s_seq_note_on) {
    MIDI.sendNoteOff(s_seq_note, 0, s_seq_note_ch);
    s_seq_note_on = false;
  }
  if (!clk_run) midi_shadows_all_notes_off(engine);

  while (MIDI.read()) {
    const midi::MidiType t = MIDI.getType();
    switch (t) {
    case midi::MidiType::Clock:
      if (s_midi_clock_rx) ++midi_clock_pulses;
      break;
    case midi::MidiType::Start:
      if (s_midi_clock_rx) { midi_clk = true; engine.Reset(); }
      break;
    case midi::MidiType::Stop:
      if (s_midi_clock_rx) { midi_clk = false; engine.Reset(); }
      break;
    case midi::MidiType::ControlChange:
      if (s_in_channel == 0 || MIDI.getChannel() == s_in_channel) {
        const uint8_t cc  = MIDI.getData1();
        const uint8_t val = MIDI.getData2();
        if (cc == 0)  s_bank_group   = val < NUM_GROUPS ? val : NUM_GROUPS - 1;
        if (cc == 32) s_bank_section = val < 2 ? val : 1;
      }
      break;
    case midi::MidiType::ProgramChange:
      if (s_in_channel == 0 || MIDI.getChannel() == s_in_channel) {
        const uint8_t pc  = MIDI.getData1();
        const uint8_t pat = s_bank_section * 8 + (pc < 8 ? pc : 7);
        if (s_bank_group != engine.get_group())
          engine.SetGroup(s_bank_group);
        engine.SetPattern(pat, true);
        engine.get_sequence().Reset();
      }
      break;
    case midi::MidiType::SystemExclusive:
      break;
    default:
      break;
    }
  }

  midi_tx_drain();
  dump_try_advance();
}

void midi_leader_transport(bool clocked, bool clk_run, bool midi_transport_slave,
                           bool run_rising, bool run_falling) {
  if (midi_transport_slave) return;
  if (run_rising)  MIDI.sendStart();
  if (run_falling) MIDI.sendStop();
  if (clocked && clk_run) MIDI.sendClock();
}

static int s_silence_step = -1;
void midi_set_silence_step(int step) { s_silence_step = step; }

// Main voice (variation 1) MIDI, driven from the analog gate every clock tick so
// the MIDI note length tracks the 303 hardware gate exactly: a note sounds only
// while get_gate() is high (half-step for plain notes, extended by ties/slides,
// pulsed by ratchets). Pitch changing while the gate stays high = a slide, so the
// new note is sent before the old note-off (legato overlap).
void midi_seq_gate_tick(Engine &engine, uint8_t transpose) {
  const byte och = static_cast<byte>(out_ch());

  // Output channel moved: close the open note on its old channel first.
  if (s_seq_note_on && och != s_seq_note_ch) {
    MIDI.sendNoteOff(s_seq_note, 0, s_seq_note_ch);
    s_seq_note_on = false;
  }

  bool gate = engine.get_gate();
  // Step-select detail editor mute: don't sound the edited step.
  if (s_silence_step >= 0 && static_cast<int>(engine.get_time_pos()) == s_silence_step)
    gate = false;

  if (!gate) {
    if (s_seq_note_on) { MIDI.sendNoteOff(s_seq_note, 0, och); s_seq_note_on = false; }
    return;
  }

  uint16_t n = static_cast<uint16_t>(engine.get_midi_note()) + transpose;
  if (n > 127) n = 127;
  const uint8_t vel = engine.get_accent() ? 127 : 80;

  if (!s_seq_note_on) {
    MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
    s_seq_note = static_cast<uint8_t>(n);
    s_seq_note_on = true;
    s_seq_note_ch = och;
  } else if (s_seq_note != static_cast<uint8_t>(n)) {
    MIDI.sendNoteOn(static_cast<byte>(n), vel, och);   // slide: new on before old off
    MIDI.sendNoteOff(s_seq_note, 0, och);
    s_seq_note = static_cast<uint8_t>(n);
    s_seq_note_ch = och;
  }
  // else: same note, gate still high -> hold
}

// --- Multitimbral shadow voices (the 2 variations not on CV/gate) -------------
// MIDI-only. Forward-advance each non-empty shadow one 16th, then emit
// notes/ties/rests with per-note slide and accent. No ratchets in v1.
static uint8_t s_shadow_note[NUM_VARIATIONS - 1]    = {0, 0};
static bool    s_shadow_note_on[NUM_VARIATIONS - 1] = {false, false};

// Shadow voices (variations 2/3) MIDI, driven from each shadow's gate state every
// clock tick -- same model as the main voice so var2/3 note length tracks the
// analog gate. Engine::AdvanceShadows() (called at the 16th boundary) computes the
// per-shadow resting/slide_gate; here we sample the gate window with clk_count.
void midi_shadows_gate_tick(Engine &engine, uint8_t transpose) {
  const int8_t half = int8_t(engine.step_period() >> 1);
  const int8_t clk  = engine.clk_count;
  for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
    const byte och = static_cast<byte>(out_ch_for_var(engine.shadow_var_[i]));
    const bool gate = engine.shadow_notecount_[i] && !engine.shadow_resting_[i] &&
                      (engine.shadow_slide_gate_[i] || clk < half);
    if (!gate) {
      if (s_shadow_note_on[i]) {
        MIDI.sendNoteOff(s_shadow_note[i], 0, och);
        s_shadow_note_on[i] = false;
      }
      continue;
    }
    Sequence &sq = engine.shadow_[i];
    uint16_t n = static_cast<uint16_t>(36 + sq.get_pitch()) + transpose;
    if (n > 127) n = 127;
    const uint8_t vel = sq.get_accent() ? 127 : 80;
    if (!s_shadow_note_on[i]) {
      MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
      s_shadow_note[i] = static_cast<uint8_t>(n);
      s_shadow_note_on[i] = true;
    } else if (s_shadow_note[i] != static_cast<uint8_t>(n)) {
      MIDI.sendNoteOn(static_cast<byte>(n), vel, och);   // slide: new on before old off
      MIDI.sendNoteOff(s_shadow_note[i], 0, och);
      s_shadow_note[i] = static_cast<uint8_t>(n);
    }
    // else: same note, gate still high -> hold
  }
}

void midi_shadows_all_notes_off(Engine &engine) {
  for (uint8_t i = 0; i < NUM_VARIATIONS - 1; ++i) {
    if (s_shadow_note_on[i]) {
      const byte och = static_cast<byte>(out_ch_for_var(engine.shadow_var_[i]));
      MIDI.sendNoteOff(s_shadow_note[i], 0, och);
      s_shadow_note_on[i] = false;
    }
  }
}

// Flush pending EEPROM writes accumulated by SysEx handlers.
// Called from the main loop at natural save points (RUN stop, WRITE exit) so
// the 3–100 ms EEPROM stall happens outside the SysEx fast path.
void midi_flush_pending_saves() {
  if (s_settings_dirty) {
    GlobalSettings.save_midi_to_storage();
    s_settings_dirty = false;
  }
}

// Incremental persistence of web-edited patterns. Writes at most ONE pattern
// per call. Only fires when: clock is stopped (EEPROM write halts the CPU and
// would glitch playback audio) AND at least 2s have passed since the last
// SysEx edit (so a burst of edits coalesces into one write per pattern).
void midi_flush_pending_pattern_saves(Engine &engine) {
  if (s_pat_dirty_mask == 0 && !engine.shadow_stale_) return;
  if (g_clk_run) return;
  const uint32_t now = millis();
  // Variation 2/3 (shadow) edits -- hardware or web -- persist after a 2s quiet
  // period (own timer, refreshed on each edit) so bursts coalesce into one flash
  // write instead of stalling the LED ISR on every keypress.
  if (engine.shadow_stale_ && (now - engine.shadow_dirty_ms_) >= 2000) {
    engine.persist_shadows();
    return;
  }
  if (s_pat_dirty_mask == 0) return;
  if (now - s_last_web_edit_ms < 2000) return;
  for (uint8_t i = 0; i < NUM_PATTERNS; ++i) {
    const uint16_t bit = uint16_t(1u << i);
    if (s_pat_dirty_mask & bit) {
      engine.persist_pattern(i);
      s_pat_dirty_mask &= ~bit;
      return; // one per tick
    }
  }
}
