/*
 * SuperOS-303 pattern SysEx (host ↔ DIN MIDI)
 * API: superos_midi.h — implementation in this file (midi.cpp) so #include <MIDI.h>
 * is not shadowed by a project midi.h on case-insensitive filesystems.
 *
 * Framing (between F0 and F7, all data bytes 0..127):
 *   F0 7D <cmd> ... F7
 *
 * Bootloader safety (bootload.c): firmware update uses 7D 01 (flash page) with len>9 and
 * a specific 8-byte header + packed payload, or 7D 02 (jump app). Pattern commands use
 * 10h–14h only; the bootloader ignores them because data[1] must be 01 or 02 for those paths.
 *
 * 7-bit packing (same idea as tools/hex2sysex.py pack_7bit): up to 7 raw bytes → one MSB
 * carry byte + 7 low-7 data bytes. Used for 128-byte pattern blobs.
 *
 * Commands:
 *   10h  Host→303  Request one pattern: <pat:0..15>
 *   11h  303→Host  Pattern data: <pat> <xor_lo7> <xor_hi1> <packed 128 bytes>
 *   12h  Host→303  Set pattern: same as 11h body; XOR over raw 128 bytes must match.
 *   13h  Host→303  Request all 16 patterns (303 sends sixteen 11h messages, queued).
 *   14h  303→Host  ACK/NAK: <status> 0=ok 1=bad_checksum 2=bad_pattern 3=blocked_clock
 *
 * XOR checksum: XOR of 128 raw EEPROM blob bytes; xor_lo7 = c & 0x7F, xor_hi1 = (c>>7) & 1.
 *
 * Note On (any channel, omni): echoed to output; velocity ≥ 100 sets accent on the current
 * pitch step and maps MIDI note 36..72 onto pattern pitch. Note Off is echoed.
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

static uint8_t out_ch() {
  return s_in_channel == 0 ? 1 : s_in_channel;
}

void midi_apply_settings(uint8_t midi_in_channel_0_omni_16, bool midi_clock_receive) {
  s_in_channel = midi_in_channel_0_omni_16 <= 16 ? midi_in_channel_0_omni_16 : 1;
  s_midi_clock_rx = midi_clock_receive;
  if (s_in_channel == 0)
    MIDI.begin(MIDI_CHANNEL_OMNI);
  else
    MIDI.begin(s_in_channel);
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
  if (n >= kTxCap)
    n = 0;
  if (n == s_tx_r)
    return false;
  s_tx[s_tx_w] = b;
  s_tx_w = n;
  return true;
}

static void midi_tx_drain() {
  while (s_tx_r != s_tx_w && Serial1.availableForWrite() > 0) {
    Serial1.write(s_tx[s_tx_r]);
    s_tx_r++;
    if (s_tx_r >= kTxCap)
      s_tx_r = 0;
  }
}

static bool tx_push_message(const uint8_t *inner, uint16_t inner_len) {
  if (!tx_push_byte(0xF0))
    return false;
  for (uint16_t i = 0; i < inner_len; ++i) {
    if (!tx_push_byte(inner[i]))
      return false;
  }
  return tx_push_byte(0xF7);
}

// --- 7-bit pack / unpack (128-byte blob) ---------------------------------------
static uint16_t pack_7bit(const uint8_t *src, uint16_t len, uint8_t *out) {
  uint16_t o = 0;
  for (uint16_t i = 0; i < len; i += 7) {
    uint8_t msb = 0;
    const uint8_t n =
        (len - i >= 7) ? 7 : static_cast<uint8_t>(len - i);
    for (uint8_t b = 0; b < n; ++b)
      if (src[i + b] & 0x80)
        msb |= static_cast<uint8_t>(1u << b);
    out[o++] = msb;
    for (uint8_t b = 0; b < n; ++b)
      out[o++] = src[i + b] & 0x7F;
  }
  return o;
}

static constexpr uint16_t kPacked128Len = 147; // ceil(128/7) groups → 18*8 + 3

static bool unpack_7bit(const uint8_t *in, uint16_t in_len, uint8_t *out,
                          uint16_t out_len) {
  uint16_t oi = 0, ii = 0;
  while (oi < out_len) {
    if (ii >= in_len)
      return false;
    const uint8_t msb = in[ii++];
    for (uint8_t b = 0; b < 7 && oi < out_len; ++b) {
      if (ii >= in_len)
        return false;
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
  if (!g_eng)
    return;
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

// --- Sequential dump after 0x13 -------------------------------------------------
static bool s_dump_active = false;
static uint8_t s_dump_next = 0;

static void dump_try_advance() {
  if (!s_dump_active || s_tx_r != s_tx_w)
    return;
  if (s_dump_next >= 16) {
    s_dump_active = false;
    return;
  }
  enqueue_pattern_reply(s_dump_next++);
}

// --- SysEx parse (callback: array includes F0 ... F7) ----------------------------
static void handle_sysex_body(const uint8_t *p, unsigned n) {
  if (!g_eng || n < 3)
    return;
  if (p[0] != 0x7D)
    return;
  const uint8_t cmd = p[1];

  switch (cmd) {
  case 0x10: { // request pattern
    if (n < 3)
      return;
    const uint8_t pat = p[2] & 0x0F;
    enqueue_pattern_reply(pat);
    break;
  }
  case 0x12: { // set pattern
    if (g_clk_run) {
      send_ack(3);
      return;
    }
    if (n < 5 + kPacked128Len) {
      send_ack(1);
      return;
    }
    const uint8_t pat = p[2] & 0x0F;
    const uint8_t c0 = p[3];
    const uint8_t c1 = p[4];
    const uint8_t cx = static_cast<uint8_t>(c0 | (c1 << 7));
    uint8_t raw[PATTERN_SIZE];
    if (!unpack_7bit(p + 5, static_cast<uint16_t>(n - 5), raw, PATTERN_SIZE)) {
      send_ack(1);
      return;
    }
    if (xor_blob128(raw) != cx) {
      send_ack(1);
      return;
    }
    if (!g_eng->import_pattern_blob(pat, raw)) {
      send_ack(2);
      return;
    }
    send_ack(0);
    enqueue_pattern_reply(pat);
    break;
  }
  case 0x13: // request all (allowed while transport runs; host should avoid floods)
    s_dump_next = 0;
    s_dump_active = true;
    dump_try_advance();
    break;
  default:
    break;
  }
}

static void sysex_cb(byte *data, unsigned sz) {
  // Library passes full buffer including F0 … F7 (see MidiInputCallbacks tests).
  if (sz < 4 || data[0] != 0xF0 || data[sz - 1] != 0xF7)
    return;
  handle_sysex_body(reinterpret_cast<const uint8_t *>(data + 1),
                    static_cast<unsigned>(sz - 2));
}

static void note_on_cb(byte ch, byte pitch, byte vel) {
  if (vel == 0)
    return;
  if (s_in_channel != 0 && ch != s_in_channel)
    return;
  MIDI.sendNoteOn(pitch, vel, ch);
  if (g_eng)
    g_eng->midi_apply_note_on(static_cast<uint8_t>(pitch), static_cast<uint8_t>(vel));
}

static void note_off_cb(byte ch, byte pitch, byte vel) {
  (void)vel;
  if (s_in_channel != 0 && ch != s_in_channel)
    return;
  MIDI.sendNoteOff(pitch, 0, ch);
}

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
  // main calls midi_apply_settings() after engine.Load()
}

static uint8_t s_seq_note = 0;
static bool s_seq_note_on = false;

void midi_poll(Engine &engine, bool clk_run, bool &midi_clk,
               bool &midi_clock_pulse) {
  g_eng = &engine;
  g_clk_run = clk_run;
  midi_clock_pulse = false;

  if (!s_midi_clock_rx)
    midi_clk = false;

  if (!clk_run && s_seq_note_on) {
    MIDI.sendNoteOff(s_seq_note, 0, out_ch());
    s_seq_note_on = false;
  }

  while (MIDI.read()) {
    const midi::MidiType t = MIDI.getType();
    switch (t) {
    case midi::MidiType::Clock:
      if (s_midi_clock_rx)
        midi_clock_pulse = true;
      break;
    case midi::MidiType::Start:
      if (s_midi_clock_rx) {
        midi_clk = true;
        engine.Reset();
      }
      break;
    case midi::MidiType::Stop:
      if (s_midi_clock_rx) {
        midi_clk = false;
        engine.Reset();
      }
      break;
    case midi::MidiType::ProgramChange:
      if (s_in_channel == 0 || MIDI.getChannel() == s_in_channel)
        engine.SetPattern(MIDI.getData1(), !clk_run);
      break;
    case midi::MidiType::SystemExclusive:
      break; // handled in sysex_cb
    default:
      break;
    }
  }

  midi_tx_drain();
  dump_try_advance();
}

void midi_leader_transport(bool clocked, bool clk_run, bool midi_transport_slave,
                           bool run_rising, bool run_falling) {
  if (midi_transport_slave)
    return;
  if (run_rising)
    MIDI.sendStart();
  if (run_falling)
    MIDI.sendStop();
  if (clocked && clk_run)
    MIDI.sendClock();
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
  if (n > 127)
    n = 127;
  const uint8_t vel = engine.get_accent() ? 127 : 80;

  if (s_seq_note_on && s_seq_note != static_cast<uint8_t>(n)) {
    // TB-303 slide: gate stays high while pitch glides. Overlap MIDI notes (new On before
    // old Off) so monophonic / legato voices glide like the hardware slide CV.
    if (engine.get_slide_dac()) {
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
