// ucom4.h — bit-accurate NEC uCOM-43 / uPD650 (D650C) core, standalone C99.
//
// Faithful port of MAME's ucom4 CPU core (src/devices/cpu/ucom4, BSD-3-Clause,
// copyright hap), lifted out of the MAME device framework so it builds for AVR
// and desktop. Semantics, cycle counts, interrupt vector and PC page-wrap all
// match MAME instruction-for-instruction. See emu/PORTING_NOTES.md.
//
// uPD650 specifics baked in: UCOM-43 instruction set, 2 KB ROM (prgmask 0x7FF),
// 96x4 data RAM (datamask 0x7F, DPh 3-bit), 3-level stack, INT vector 0x003C.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ROM fetch — on AVR the 2 KB mask ROM lives in flash (PROGMEM); elsewhere it is
// a plain array. Define UCOM4_ROM_RD before including to override.
// D650_ROM_IN_RAM (combined build): the ROM is a user-uploaded SRAM copy
// (rom_store.h), so the fetch is a plain load even on AVR — one cycle cheaper
// than the LPM it replaces. Local addition, see src/d650/SYNC.md.
#ifndef UCOM4_ROM_RD
#  if defined(D650_ROM_IN_RAM)
#    define UCOM4_ROM_RD(rom, addr) ((rom)[(addr)])
#  elif defined(__AVR__)
#    include <avr/pgmspace.h>
#    define UCOM4_ROM_RD(rom, addr) ((uint8_t)pgm_read_byte(&(rom)[(addr)]))
#  else
#    define UCOM4_ROM_RD(rom, addr) ((rom)[(addr)])
#  endif
#endif

// Port indices — identical to MAME (OP/IP address a port by DPl using these).
enum { UCOM4_PORTA=0, UCOM4_PORTB, UCOM4_PORTC, UCOM4_PORTD,
       UCOM4_PORTE, UCOM4_PORTF, UCOM4_PORTG, UCOM4_PORTH, UCOM4_PORTI };

typedef struct ucom4 ucom4_t;

// Host I/O. read_* return the 4-bit input on ports A/B/C/D (C/D are OR'd with the
// last output latch inside the core, matching MAME open-drain behaviour). Return
// 0 for unused ports. write_port receives port index (C..I) and the 4-bit value.
typedef struct {
  uint8_t (*read_a)(void *user);
  uint8_t (*read_b)(void *user);
  uint8_t (*read_c)(void *user);
  uint8_t (*read_d)(void *user);
  void    (*write_port)(void *user, int port, uint8_t data);
  void   *user;
} ucom4_io;

struct ucom4 {
  const uint8_t *rom;      // 2 KB program
  uint16_t prgmask;        // 0x7FF
  uint8_t  ram[128];       // 96x4 mapped in 0x00-0x4F, 0x70-0x7F; rest reads 0
  uint8_t  datamask;       // 0x7F
  uint8_t  dph_mask;       // 0x07

  uint16_t pc, prev_pc;
  uint16_t stack[3];
  int      stack_levels;   // 3
  uint8_t  acc, dpl, dph;
  uint8_t  carry_f, carry_s_f, timer_f, int_f, inte_f;
  int      int_line;       // last /INT pin level (edge detect)
  uint8_t  op, prev_op, arg, bitmask;
  int      skip;
  uint8_t  port_out[16];

  int16_t  timer_cnt;      // STM countdown in machine cycles (max 4032), <0 = disabled
  uint32_t cyc;            // total machine cycles executed (monotonic)
  uint8_t  step_cyc;       // cycles consumed by the most recent step (1..3)

  ucom4_io io;
};

// Reset to power-on state (PC=0, interrupts disabled, ports cleared).
void ucom4_reset(ucom4_t *c, const uint8_t *rom, const ucom4_io *io);

// Execute exactly one instruction (plus a pending interrupt entry if due).
// Returns the number of machine cycles consumed (also in c->step_cyc).
uint32_t ucom4_step(ucom4_t *c);

// Execute instructions until at least `cycles` machine cycles have elapsed.
// Returns the cycles actually consumed (may overshoot by one instruction).
uint32_t ucom4_run(ucom4_t *c, uint32_t cycles);

// Drive the /INT pin. Rising edge (0->1) latches the interrupt flag.
void ucom4_set_int(ucom4_t *c, int level);

#ifdef __cplusplus
}
#endif
