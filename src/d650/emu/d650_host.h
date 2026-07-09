// d650_host.h — the emulated TB-303 "machine": uPD650 core + external uPD444
// pattern RAM + I/O decode onto SuperOS-303 semantics. Portable C (no AVR/Arduino
// deps) so it runs identically on desktop (validation) and on the AT90USB1286.
//
// The host implements the four ucom4 I/O callbacks:
//   read A/B  -> button/status matrix (from an InputIndex-ordered snapshot)
//   read C    -> uPD444 data bus (only while a chip is selected and WE idle)
//   write C..I-> uPD444 RAM bus + CV/gate/accent/slide + LED matrix
//
// External RAM model (service manual p.4 + MAME roland_tb303 + ROM trace):
//   3x uPD444C, 1024x4 each = 3072 nibbles. PE2/PE3 feed the 4556 (IC-2) which
//   selects one chip (00 = none). PE0/PE1 = A8/A9, PF = A4..A7, PD = A0..A3.
//   PORTI bit 0 is the write strobe: the ROM pulses it high around a write
//   (inverted on the board to the uPD444 _WE); it is parked low for reads.
//   The ROM deselects the chips (CE=00) and re-parks the PORTC latch after
//   every access, so ISR-time PORTC writes never touch the RAM.
//
// CV: pitch (PD + PF0/1) and accent (PE0) are time-shared with RAM addressing,
// so both are sampled only on the PI1 latch rising edge (the 4174 latch clock
// on the voice board). Slide = PI1 level, gate = PI2 level. See PORT_MAP.md.
#pragma once
#include <stdint.h>
#include "ucom4.h"

#ifdef __cplusplus
extern "C" {
#endif

// SuperOS input snapshot: index by InputIndex (pins.h). 1 = key/switch active
// (the 303 matrix is active-high at PA/PB: pressed key + row selected -> 1).
// Layout the host relies on (from pins.h):
//   [row*4 + col]      buttons   (read on PORTB, PH row selected)   idx  0..15
//   [16 + row*4 + col] keys/stat (read on PORTA, PH row selected)   idx 16..31
//   [32 + j]           status    (read on PORTA, all PH high): RUN,TAP,-,CLOCK
#define D650_IN_COUNT 40
#define D650_IN_RUN   32
#define D650_IN_TAP   33
#define D650_IN_CLOCK 35

// External uPD444 store: 3072 nibbles packed 2 per byte (even nibble = low 4
// bits). The packed array is the persistence image — snapshot/restore verbatim.
#define D650_EXT_NIBS  3072
#define D650_EXT_BYTES (D650_EXT_NIBS / 2)

// Driver hooks — set by the platform. Desktop mocks log; AVR wires to DAC::/Leds::.
typedef struct {
  // pitch: 6-bit semitone (0..63); gate/accent/slide: 0/1. Pitch+accent are
  // sampled at the PI1 latch edge; gate/slide follow the PORTI levels.
  void (*set_cv)(void *u, uint8_t pitch, uint8_t gate, uint8_t accent, uint8_t slide);
  // 20 logical LEDs (OutputIndex order): 0..15 matrix, 16..19 direct mode LEDs.
  void (*set_led)(void *u, uint8_t led_index, uint8_t on);
  void *u;
} d650_drivers;

typedef struct {
  ucom4_t  cpu;
  uint8_t  ext[D650_EXT_BYTES]; // uPD444 x3 pattern/track store, packed nibbles
  uint8_t  ext_dirty;    // set when a pattern-RAM nibble changes (flash trigger)
  uint8_t  in[D650_IN_COUNT];
  // Output-latch mirrors (the core updates its port_out AFTER the write
  // callback runs, so the host keeps its own copies).
  uint8_t  lat_c, lat_d, lat_e, lat_f, lat_g, lat_h, lat_i;
  uint8_t  pitch, gate, accent, slide;  // last committed CV state
  d650_drivers drv;
} d650_host;

// Initialize + reset. `rom` = 2 KB TB-303 mask ROM. `drv` may be NULL (no output).
void d650_init(d650_host *h, const uint8_t *rom, const d650_drivers *drv);

// Run one CPU instruction. Returns machine cycles consumed.
uint32_t d650_step(d650_host *h);

// Run at least `cycles` machine cycles (batched; cheaper than stepping).
static inline uint32_t d650_run(d650_host *h, uint32_t cycles) {
  return ucom4_run(&h->cpu, cycles);
}

// Drive the /INT pin (the fixed ~555 Hz sampler line, NOT the tempo clock —
// tempo goes on in[D650_IN_CLOCK]). Rising edge latches the interrupt.
void d650_clock(d650_host *h, int level);

// Snapshot/restore the packed pattern store (persistence layer target).
// Returns nonzero if the store changed since the last clear of ext_dirty.
static inline int d650_dirty(const d650_host *h) { return h->ext_dirty; }
static inline void d650_clear_dirty(d650_host *h) { h->ext_dirty = 0; }

#ifdef __cplusplus
}
#endif
