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

static uint8_t out_ch() {
  return s_in_channel == 0 ? 1 : s_in_channel;
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
  if (!tx_push_byte(0xF0)) return false;
  for (uint16_t i = 0; i < inner_len; ++i) {
    if (!tx_push_byte(inner[i])) return false;
  }
  return tx_push_byte(0xF7);
}

// --- 7-bit pack / unpack (128-byte blob) ---------------------------------------
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

static constexpr uint16_t kPacked128Len = 147;

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

static uint8_t xor_blob128(const uint8_t *p) {
  uint8_t x = 0;
  for (uint16_t i = 0; i < PATTERN_SIZE; ++i)
    x ^= p[i];
  return x;
}

static void enqueue_pattern_reply(uint8_t pat) {
  if (!g_eng) return;
  uint8_t raw[PATTERN_SIZE];
  g_eng->export_pattern_blob(pat, raw);
  const uint8_t cx = xor_blob128(raw);
  uint8_t inner[5 + kPacked128Len];
  inner[0] = 0x7D;
  inner[1] = 0x11;
  inner[2] = pat & 0x0F;
  inner[3] = static_cast<uint8_t>(cx & 0x7F);
  inner[4] = static_cast<uint8_t>((cx >> 7) & 1);
  const uint16_t pl = pack_7bit(raw, PATTERN_SIZE, inner + 5);
  tx_push_message(inner, static_cast<uint16_t>(5 + pl));
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
  if (!g_eng || n < 3) return;
  if (p[0] != 0x7D) return;
  const uint8_t cmd = p[1];

  switch (cmd) {
  case 0x10: { // request pattern
    if (n < 3) return;
    const uint8_t pat = p[2] & 0x0F;
    enqueue_pattern_reply(pat);
    break;
  }
  case 0x12: { // set pattern
    if (n < 5 + kPacked128Len) { send_ack(1); return; }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t cx = static_cast<uint8_t>(p[3] | (p[4] << 7));
    uint8_t raw[PATTERN_SIZE];
    if (!unpack_7bit(p + 5, static_cast<uint16_t>(n - 5), raw, PATTERN_SIZE)) { send_ack(1); return; }
    if (xor_blob128(raw) != cx) { send_ack(1); return; }
    const bool persist = !g_clk_run;
    if (!g_eng->import_pattern_blob(pat, raw, persist)) { send_ack(2); return; }
    send_ack(0);
    if (persist) enqueue_pattern_reply(pat);
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
    seq.pitch[step] = pitchByte;
    // Write time nibble at arbitrary index
    uint8_t &td = seq.time_data[step >> 1];
    const uint8_t shift = 4 * (step & 1);
    td = (td & ~(0x0F << shift)) | (timeNib << shift);
    g_eng->stale = true;
    break;
  }
  case 0x18: { // host → 303: set pattern length
    if (n < 4 || !g_eng) return;
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t len = p[3] & 0x7F;
    if (len < 1 || len > MAX_STEPS) return;
    g_eng->pattern[pat].length = len;
    g_eng->stale = true;
    break;
  }
  case 0x19: { // host → 303: set step lock
    if (n < 5 || !g_eng) return;
    const uint8_t pat    = p[2] & 0x0F;
    const uint8_t step   = p[3] & 0x3F;
    const bool    locked = (p[4] & 0x01) != 0;
    Sequence &seq = g_eng->pattern[pat];
    if (step >= MAX_STEPS) return;
    const uint8_t mask = uint8_t(1u << (step & 7));
    if (locked)
      seq.step_lock[step >> 3] |= mask;
    else
      seq.step_lock[step >> 3] &= ~mask;
    g_eng->stale = true;
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
  case 0x20: { // request config
    const uint8_t fl  = static_cast<uint8_t>((GlobalSettings.midi_clock_receive ? 1 : 0) |
                                              (GlobalSettings.midi_thru          ? 2 : 0));
    const uint8_t dir = g_eng ? static_cast<uint8_t>(g_eng->get_direction()) : 0;
    const uint8_t inner[5] = {0x7D, 0x21, GlobalSettings.midi_channel, fl, dir};
    tx_push_message(inner, 5);
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
    GlobalSettings.save_midi_to_storage();
    midi_apply_settings(GlobalSettings.midi_channel, GlobalSettings.midi_clock_receive, GlobalSettings.midi_thru);
    send_ack(0);
    // Echo back new config
    const uint8_t nfl  = static_cast<uint8_t>((GlobalSettings.midi_clock_receive ? 1 : 0) |
                                               (GlobalSettings.midi_thru          ? 2 : 0));
    const uint8_t ndir = g_eng ? static_cast<uint8_t>(g_eng->get_direction()) : 0;
    const uint8_t reply[5] = {0x7D, 0x21, GlobalSettings.midi_channel, nfl, ndir};
    tx_push_message(reply, 5);
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

void midi_audition_note_on(uint8_t note, uint8_t vel) {
  if (s_audition_on && s_audition_note != note) {
    // Pitch changed mid-press — close previous note first
    MIDI.sendNoteOff(s_audition_note, 0, out_ch());
  }
  if (!s_audition_on || s_audition_note != note) {
    MIDI.sendNoteOn(note, vel, out_ch());
    s_audition_note = note;
    s_audition_on   = true;
  }
}

void midi_audition_note_off() {
  if (s_audition_on) {
    MIDI.sendNoteOff(s_audition_note, 0, out_ch());
    s_audition_on = false;
  }
}

// --- Step position broadcast (SysEx 0x15) ---------------------------------------
void midi_send_step_position(uint8_t pat, uint8_t step) {
  const uint8_t inner[4] = {0x7D, 0x15, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F)};
  tx_push_message(inner, 4);
}

// --- Length update broadcast (SysEx 0x18) ----------------------------------------
void midi_send_length_update(uint8_t pat, uint8_t len) {
  const uint8_t inner[4] = {0x7D, 0x18, (uint8_t)(pat & 0x0F), (uint8_t)(len & 0x3F)};
  tx_push_message(inner, 4);
}

// --- Direction update broadcast (SysEx 0x17) -------------------------------------
void midi_send_direction_update(uint8_t direction) {
  const uint8_t inner[3] = {0x7D, 0x17, (uint8_t)(direction & 0x07)};
  tx_push_message(inner, 3);
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

// --- Step lock broadcast (SysEx 0x19) -------------------------------------------
void midi_send_step_lock_update(uint8_t pat, uint8_t step, bool locked) {
  const uint8_t inner[5] = {0x7D, 0x19, (uint8_t)(pat & 0x0F), (uint8_t)(step & 0x3F), locked ? 1u : 0u};
  tx_push_message(inner, 5);
}

// --- Step edit broadcast (SysEx 0x16) -------------------------------------------
void midi_send_step_update(uint8_t pat, uint8_t step, uint8_t pitch_byte, uint8_t time_nibble) {
  const uint8_t inner[7] = {
    0x7D, 0x16,
    static_cast<uint8_t>(pat & 0x0F),
    static_cast<uint8_t>(step & 0x3F),
    static_cast<uint8_t>(pitch_byte & 0x7F),       // low 7 bits
    static_cast<uint8_t>((pitch_byte >> 7) & 0x01), // bit 7 (slide/empty flag)
    static_cast<uint8_t>(time_nibble & 0x0F)
  };
  tx_push_message(inner, 7);
}

// --- midi_init ------------------------------------------------------------------
void midi_init(Engine *engine) {
  g_eng = engine;
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
    MIDI.sendNoteOff(s_seq_note, 0, out_ch());
    s_seq_note_on = false;
  }

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
    case midi::MidiType::ProgramChange:
      if (s_in_channel == 0 || MIDI.getChannel() == s_in_channel)
        engine.SetPattern(MIDI.getData1(), !clk_run);
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

void midi_after_clock(Engine &engine, uint8_t transpose) {
  const byte och = static_cast<byte>(out_ch());

  if (engine.resting) {
    if (s_seq_note_on) {
      MIDI.sendNoteOff(s_seq_note, 0, och);
      s_seq_note_on = false;
    }
    return;
  }

  uint16_t n = static_cast<uint16_t>(engine.get_midi_note()) + transpose;
  if (n > 127) n = 127;
  const uint8_t vel = engine.get_accent() ? 127 : 80;

  if (s_seq_note_on && s_seq_note != static_cast<uint8_t>(n)) {
    const bool slide_midi = engine.get_slide_dac();
    if (slide_midi) {
      MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
      MIDI.sendNoteOff(s_seq_note, 0, och);
    } else {
      MIDI.sendNoteOff(s_seq_note, 0, och);
      MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
    }
    s_seq_note = static_cast<uint8_t>(n);
    return;
  }

  if (!s_seq_note_on) {
    MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
    s_seq_note = static_cast<uint8_t>(n);
    s_seq_note_on = true;
  }
}

void midi_ratchet_retrigger(Engine &engine, uint8_t transpose) {
  if (!s_seq_note_on || engine.resting) return;
  const byte och = static_cast<byte>(out_ch());
  uint16_t n = static_cast<uint16_t>(engine.get_midi_note()) + transpose;
  if (n > 127) n = 127;
  const uint8_t vel = engine.get_accent() ? 127 : 80;
  MIDI.sendNoteOff(s_seq_note, 0, och);
  MIDI.sendNoteOn(static_cast<byte>(n), vel, och);
  s_seq_note = static_cast<uint8_t>(n);
}
