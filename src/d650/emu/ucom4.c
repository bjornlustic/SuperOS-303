// ucom4.c — bit-accurate NEC uCOM-43 / uPD650 core. Faithful port of MAME's
// ucom4 core (BSD-3-Clause, hap). Cycle counts and semantics match MAME.
//
// Interpreter structure (for AVR real-time: 113.636 k machine cycles/s on a
// 16 MHz AT90USB1286, i.e. a 141-clock budget per machine cycle):
//   * ucom4_run executes batches with the hot CPU state cached in locals;
//   * dispatch is one 256-entry opcode->handler-id table (built at reset)
//     feeding a single dense switch (one jump table);
//   * bit 7 of the table entry marks 2-byte opcodes (arg fetch).
// Architectural state is fully written back after every run and
// ucom4_step == ucom4_run(c, 1), so single-step semantics (tests, MAME
// differential oracle) are unchanged.

// Combined SuperOS build compiles globally at -Os; the interpreter needs -O3
// to hold 113.6 kcyc/s (see PORTING_NOTES.md: -Os and -O2 missed real-time).
#if defined(SUPEROS_COMBINED) && defined(__AVR__)
#pragma GCC optimize("O3")
#endif
#include "ucom4.h"

// ---- memory -----------------------------------------------------------------
// data_96x4 map: 0x00-0x4F and 0x70-0x7F are RAM; 0x50-0x6F unmapped (read 0).
static inline int ram_mapped(uint8_t a) { return a <= 0x4F || a >= 0x70; }

// uCOM-43 extended registers X,Y,R,S,W,Z,F live at datamask-index (0x79-0x7F).
enum { REG_X=0, REG_Y, REG_R, REG_S, REG_W, REG_Z, REG_F };

// ---- ports ------------------------------------------------------------------
static uint8_t input_r(ucom4_t *c, int index) {
  index &= 0xf;
  uint8_t inp = 0;
  switch (index) {
    case UCOM4_PORTA: inp = c->io.read_a ? c->io.read_a(c->io.user) : 0; break;
    case UCOM4_PORTB: inp = c->io.read_b ? c->io.read_b(c->io.user) : 0; break;
    case UCOM4_PORTC: inp = (c->io.read_c ? c->io.read_c(c->io.user) : 0) | c->port_out[UCOM4_PORTC]; break;
    case UCOM4_PORTD: inp = (c->io.read_d ? c->io.read_d(c->io.user) : 0) | c->port_out[UCOM4_PORTD]; break;
    default: break;
  }
  return inp & 0xf;
}
static void output_w(ucom4_t *c, int index, uint8_t data) {
  index &= 0xf; data &= 0xf;
  if (index >= UCOM4_PORTC && index <= UCOM4_PORTI) {
    uint8_t out = (index == UCOM4_PORTI) ? (data & 7) : data;
    if (c->io.write_port) c->io.write_port(c->io.user, index, out);
  }
  c->port_out[index] = data;
}

// ---- stack ------------------------------------------------------------------
static void push_stack_pc(ucom4_t *c, uint16_t pc) {
  for (int i = c->stack_levels - 1; i >= 1; i--) c->stack[i] = c->stack[i-1];
  c->stack[0] = pc;
}
static uint16_t pop_stack(ucom4_t *c) {
  uint16_t pc = c->stack[0] & c->prgmask;
  for (int i = 0; i < c->stack_levels - 1; i++) c->stack[i] = c->stack[i+1];
  return pc;
}

// ---- interrupt --------------------------------------------------------------
void ucom4_set_int(ucom4_t *c, int level) {
  if (c->int_line == 0 && level) c->int_f = 1;  // rising edge
  c->int_line = level ? 1 : 0;
}

// ---- opcode dispatch map ----------------------------------------------------
// opmap[op] = handler id (low 7 bits) | 0x80 if the opcode has an argument
// byte (STM/LDI/CLI/CI = 0x14-0x17, JMP/CAL = 0xA0-0xAF, OCD = 0x1E).
enum {
  H_NOP = 0,
  H_DI, H_S, H_TIT, H_TC, H_TTM, H_DAA, H_TAL, H_AD, H_ADS, H_DAS, H_CLC,
  H_CM, H_INC, H_OP, H_DEC, H_CMA, H_CIA, H_TLA, H_DED, H_STM, H_LDI, H_CLI,
  H_CI, H_EXL, H_ADC, H_XC, H_STC, H_INM, H_OCD, H_DEM,
  H_RAR, H_EI, H_IP, H_IND,
  H_IA, H_JPA, H_TAZ, H_TAW, H_OE, H_TLY, H_THX, H_RT, H_RTS, H_XAZ, H_XAW,
  H_XLS, H_XHR, H_XLY, H_XHX,
  H_FBF, H_TAB, H_XM, H_XMD, H_CMB, H_LM, H_XMI,
  H_TPB, H_TPA, H_TMB, H_FBT, H_RPB, H_REB, H_RMB, H_RFB, H_SPB, H_SEB,
  H_SMB, H_SFB,
  H_LDZ, H_LI, H_JMPCAL, H_CZP, H_JCP
};

static uint8_t opmap[256];
static uint8_t opmap_ready = 0;

static void build_opmap(void) {
  static const uint8_t low20[0x20] = {   // 0x00-0x1F (0x1C illegal -> NOP)
    H_NOP, H_DI, H_S, H_TIT, H_TC, H_TTM, H_DAA, H_TAL,
    H_AD, H_ADS, H_DAS, H_CLC, H_CM, H_INC, H_OP, H_DEC,
    H_CMA, H_CIA, H_TLA, H_DED, H_STM, H_LDI, H_CLI, H_CI,
    H_EXL, H_ADC, H_XC, H_STC, H_NOP, H_INM, H_OCD, H_DEM,
  };
  static const uint8_t grp2x3x[8] = {    // 0x20-0x3F in groups of 4 (except 0x30-0x33)
    H_FBF, H_TAB, H_XM, H_XMD, H_NOP /*0x30 singles*/, H_CMB, H_LM, H_XMI,
  };
  static const uint8_t four4x[16] = {    // 0x40-0x4F (0x45 illegal -> NOP)
    H_IA, H_JPA, H_TAZ, H_TAW, H_OE, H_NOP, H_TLY, H_THX,
    H_RT, H_RTS, H_XAZ, H_XAW, H_XLS, H_XHR, H_XLY, H_XHX,
  };
  static const uint8_t grp5x7x[12] = {   // 0x50-0x7F in groups of 4
    H_TPB, H_TPA, H_TMB, H_FBT, H_RPB, H_REB, H_RMB, H_RFB,
    H_SPB, H_SEB, H_SMB, H_SFB,
  };
  for (int i = 0; i < 256; i++) {
    uint8_t id;
    uint8_t hi = (uint8_t)(i & 0xf0);
    if      (hi == 0x80) id = H_LDZ;
    else if (hi == 0x90) id = H_LI;
    else if (hi == 0xa0) id = H_JMPCAL;
    else if (hi == 0xb0) id = H_CZP;
    else if (hi >= 0xc0) id = H_JCP;
    else if (i <  0x20)  id = low20[i];
    else if (i == 0x30)  id = H_RAR;
    else if (i == 0x31)  id = H_EI;
    else if (i == 0x32)  id = H_IP;
    else if (i == 0x33)  id = H_IND;
    else if (i <  0x40)  id = grp2x3x[(i - 0x20) >> 2];
    else if (i <  0x50)  id = four4x[i - 0x40];
    else                 id = grp5x7x[(i - 0x50) >> 2];
    // 2-byte opcodes
    if ((i & 0xfc) == 0x14 || (i & 0xf0) == 0xa0 || i == 0x1e) id |= 0x80;
    opmap[i] = id;
  }
  opmap_ready = 1;
}

// ---- reset ------------------------------------------------------------------
void ucom4_reset(ucom4_t *c, const uint8_t *rom, const ucom4_io *io) {
  if (!opmap_ready) build_opmap();
  for (unsigned i = 0; i < sizeof *c; i++) ((uint8_t*)c)[i] = 0;
  c->rom = rom;
  c->io = *io;
  c->prgmask = 0x7ff;
  c->datamask = 0x7f;
  c->dph_mask = 0x07;
  c->stack_levels = 3;
  c->timer_cnt = -1;
  c->int_line = 0;
  c->inte_f = 0;            // UCOM-43
  c->pc = 0; c->op = 0; c->skip = 0;
  for (int i = UCOM4_PORTC; i <= UCOM4_PORTI; i++) output_w(c, i, 0);
}

// ---- batch execution --------------------------------------------------------
// Local-state interpreter. All names shadow the struct fields; RAM accesses go
// through the cached `ram` pointer with the same mapping as MAME's data_96x4.
static uint16_t run_chunk(ucom4_t *c, uint16_t cycles) {
  if (!opmap_ready) build_opmap();   // harnesses may bypass ucom4_reset
  const uint8_t *rom = c->rom;
  const uint16_t prgmask = c->prgmask;
  const uint8_t datamask = c->datamask;
  uint8_t *ram = c->ram;

  uint16_t pc   = c->pc;
  uint8_t acc   = c->acc, dpl = c->dpl, dph = c->dph;
  uint8_t op    = c->op, prev_op = c->prev_op, arg = c->arg;
  uint8_t skip  = (uint8_t)(c->skip != 0);
  uint8_t carry = c->carry_f;
  uint8_t inte  = c->inte_f, intf = c->int_f;
  int16_t timer_cnt = c->timer_cnt;
  int16_t rem   = (int16_t)cycles;   // countdown: sign test beats 16-bit compare
  uint8_t step_cyc = 0;

  // RAM/register helpers on the cached state. INC_PC: only the low 8 bits
  // auto-increment (8-bit add, no carry into the page). bm_lut: AVR has no
  // barrel shifter (1 << n is a loop), so look the bitmask up.
  static const uint8_t bm_lut[4] = { 1, 2, 4, 8 };
  #define RAM_A()      ((uint8_t)((((uint16_t)dph << 4) | dpl) & datamask))
  #define RAM_R()      (ram_mapped(RAM_A()) ? (ram[RAM_A()] & 0xf) : 0)
  #define RAM_W(v)     do { uint8_t a_ = RAM_A(); if (ram_mapped(a_)) ram[a_] = (v) & 0xf; } while (0)
  #define REG_R(i)     (ram[(uint8_t)(datamask - (i)) & 0x7F] & 0xf)
  #define REG_W(i, v)  (ram[(uint8_t)(datamask - (i)) & 0x7F] = (v) & 0xf)
  #define INC_PC()     (pc = (uint16_t)((pc & 0xff00u) | (uint8_t)((uint8_t)pc + 1)))

  while (rem > 0) {
    step_cyc = 0;

    // Service interrupt, but not right after LI($9x)/EI($31), nor while skipping.
    if (intf && inte && (op & 0xf0) != 0x90 && op != 0x31 && !skip) {
      step_cyc++;
      push_stack_pc(c, pc);
      pc = 0xf << 2;   // 0x003C
      intf = 0;
      inte = 0;        // UCOM-43: interrupts disabled on entry
    }

    prev_op = op;
#ifndef UCOM4_OMIT_PREV_PC
    // prev_pc is trace/debug state (tests, MAME oracle); nothing functional
    // reads it. On AVR the per-instruction store is worth ~4% of real-time.
    c->prev_pc = pc;
#endif

    step_cyc++;
    op = UCOM4_ROM_RD(rom, pc & prgmask);
    uint8_t bitmask = bm_lut[op & 0x03];
    INC_PC();
    uint8_t h = opmap[op];
    if (h & 0x80) {                  // 2-byte opcode: fetch argument
      step_cyc++;
      arg = UCOM4_ROM_RD(rom, pc & prgmask);
      INC_PC();
      h &= 0x7f;
    }

    if (skip) { skip = 0; op = 0; h = H_NOP; }  // skipped opcode acts as NOP

    switch (h) {
      case H_NOP: break;
      case H_LDZ: /* 8x */ dph = 0; dpl = op & 0x0f; break;
      case H_LI:  /* 9x */ if ((prev_op & 0xf0) != 0x90) acc = op & 0x0f; break;
      case H_JMPCAL: /* Ax */ if (op & 0x08) push_stack_pc(c, pc);
                              pc = (uint16_t)(((op & 0x07) << 8 | arg) & prgmask); break;
      case H_CZP: /* Bx */ push_stack_pc(c, pc); pc = (uint16_t)((op & 0x0f) << 2); break;
      case H_JCP: /* Cx-Fx */ pc = (uint16_t)((pc & ~0x3fu) | (op & 0x3f)); break;

      case H_DI:  inte = 0; break;
      case H_S:   RAM_W(acc); break;
      case H_TIT: skip = (intf != 0); intf = 0; break;
      case H_TC:  skip = (carry != 0); break;
      case H_TTM: skip = (c->timer_f != 0); break;
      case H_DAA: acc = (acc + 6) & 0xf; break;
      case H_TAL: dpl = acc; break;
      case H_AD:  acc += RAM_R(); skip = (acc & 0x10) != 0; acc &= 0xf; break;
      case H_ADS: acc += RAM_R() + carry; carry = (acc >> 4) & 1;
                  acc &= 0xf; skip = (carry != 0); break;
      case H_DAS: acc = (acc + 10) & 0xf; break;
      case H_CLC: carry = 0; break;
      case H_CM:  skip = (acc == RAM_R()); break;
      case H_INC: acc = (acc + 1) & 0xf; skip = (acc == 0); break;
      case H_OP:  output_w(c, dpl, acc); break;
      case H_DEC: acc = (acc - 1) & 0xf; skip = (acc == 0xf); break;
      case H_CMA: acc ^= 0xf; break;
      case H_CIA: acc = ((acc ^ 0xf) + 1) & 0xf; break;
      case H_TLA: acc = dpl; break;
      case H_DED: dpl = (dpl - 1) & 0xf; skip = (dpl == 0xf); break;
      case H_STM: c->timer_f = 0;
                  timer_cnt = (int16_t)(63 * ((arg & 0x3f) + 1)); break;
      case H_LDI: dph = (arg >> 4) & 0xf; dpl = arg & 0x0f; break;
      case H_CLI: skip = (dpl == (arg & 0x0f)); break;
      case H_CI:  skip = (acc == (arg & 0x0f)); break;
      case H_EXL: acc ^= RAM_R(); break;
      case H_ADC: acc += RAM_R() + carry; carry = (acc >> 4) & 1; acc &= 0xf; break;
      case H_XC:  { uint8_t t = carry; carry = c->carry_s_f; c->carry_s_f = t; } break;
      case H_STC: carry = 1; break;
      case H_INM: { uint8_t v = (RAM_R() + 1) & 0xf; RAM_W(v); skip = (v == 0); } break;
      case H_OCD: output_w(c, UCOM4_PORTD, arg >> 4); output_w(c, UCOM4_PORTC, arg & 0xf); break;
      case H_DEM: { uint8_t v = (RAM_R() - 1) & 0xf; RAM_W(v); skip = (v == 0xf); } break;

      case H_RAR: { uint8_t cf = acc & 1; acc = (acc >> 1) | (carry << 3); carry = cf; } break;
      case H_EI:  inte = 1; break;
      case H_IP:  acc = input_r(c, dpl); break;
      case H_IND: dpl = (dpl + 1) & 0xf; skip = (dpl == 0); break;

      case H_IA:  step_cyc++; acc = input_r(c, UCOM4_PORTA); break;
      case H_JPA: step_cyc++; pc = (uint16_t)((pc & ~0x3fu) | (acc << 2)); break;
      case H_TAZ: step_cyc++; REG_W(REG_Z, acc); break;
      case H_TAW: step_cyc++; REG_W(REG_W, acc); break;
      case H_OE:  step_cyc++; output_w(c, UCOM4_PORTE, acc); break;
      case H_TLY: step_cyc++; REG_W(REG_Y, dpl); break;
      case H_THX: step_cyc++; REG_W(REG_X, dph); break;
      case H_RT:  step_cyc++; pc = pop_stack(c); break;
      case H_RTS: step_cyc++; pc = pop_stack(c); skip = 1; break;
      case H_XAZ: step_cyc++; { uint8_t t = acc; acc = REG_R(REG_Z); REG_W(REG_Z, t); } break;
      case H_XAW: step_cyc++; { uint8_t t = acc; acc = REG_R(REG_W); REG_W(REG_W, t); } break;
      case H_XLS: step_cyc++; { uint8_t t = dpl; dpl = REG_R(REG_S); REG_W(REG_S, t); } break;
      case H_XHR: step_cyc++; { uint8_t t = dph; dph = REG_R(REG_R); REG_W(REG_R, t); } break;
      case H_XLY: step_cyc++; { uint8_t t = dpl; dpl = REG_R(REG_Y); REG_W(REG_Y, t); } break;
      case H_XHX: step_cyc++; { uint8_t t = dph; dph = REG_R(REG_X); REG_W(REG_X, t); } break;

      case H_FBF: step_cyc++; skip = ((REG_R(REG_F) & bitmask) == 0); break;
      case H_TAB: skip = ((acc & bitmask) != 0); break;
      case H_XM:  { uint8_t o = acc; acc = RAM_R(); RAM_W(o); dph ^= (op & 3); } break;
      case H_XMD: { uint8_t o = acc; acc = RAM_R(); RAM_W(o); dph ^= (op & 3);
                    dpl = (dpl - 1) & 0xf; skip = (dpl == 0xf); } break;
      case H_CMB: skip = ((acc & bitmask) == (RAM_R() & bitmask)); break;
      case H_LM:  acc = RAM_R(); dph ^= (op & 3); break;
      case H_XMI: { uint8_t o = acc; acc = RAM_R(); RAM_W(o); dph ^= (op & 3);
                    dpl = (dpl + 1) & 0xf; skip = (dpl == 0); } break;

      case H_TPB: skip = ((input_r(c, dpl) & bitmask) != 0); break;
      case H_TPA: skip = ((input_r(c, UCOM4_PORTA) & bitmask) != 0); break;
      case H_TMB: skip = ((RAM_R() & bitmask) != 0); break;
      case H_FBT: step_cyc++; skip = ((REG_R(REG_F) & bitmask) != 0); break;
      case H_RPB: output_w(c, dpl, c->port_out[dpl] & ~bitmask); break;
      case H_REB: step_cyc++; output_w(c, UCOM4_PORTE, c->port_out[UCOM4_PORTE] & ~bitmask); break;
      case H_RMB: RAM_W(RAM_R() & ~bitmask); break;
      case H_RFB: step_cyc++; REG_W(REG_F, REG_R(REG_F) & ~bitmask); break;
      case H_SPB: output_w(c, dpl, c->port_out[dpl] | bitmask); break;
      case H_SEB: step_cyc++; output_w(c, UCOM4_PORTE, c->port_out[UCOM4_PORTE] | bitmask); break;
      case H_SMB: RAM_W(RAM_R() | bitmask); break;
      case H_SFB: step_cyc++; REG_W(REG_F, REG_R(REG_F) | bitmask); break;

      default: break; // illegal: treated as NOP
    }

    // timer countdown (STM/TTM)
    if (timer_cnt >= 0) {
      timer_cnt -= (int16_t)step_cyc;
      if (timer_cnt <= 0) { c->timer_f = 1; timer_cnt = -1; }
    }
    rem -= (int16_t)step_cyc;
  }

  #undef RAM_A
  #undef RAM_R
  #undef RAM_W
  #undef REG_R
  #undef REG_W
  #undef INC_PC

  // write back cached state
  uint16_t spent = (uint16_t)((int16_t)cycles - rem);   // rem <= 0 on exit
  c->pc = pc; c->acc = acc; c->dpl = dpl; c->dph = dph;
  c->op = op; c->prev_op = prev_op; c->arg = arg;
  c->skip = skip; c->carry_f = carry;
  c->inte_f = inte; c->int_f = intf;
  c->timer_cnt = timer_cnt;
  c->step_cyc = step_cyc;          // cycles of the last executed instruction
  c->cyc += spent;
  return spent;
}

uint32_t ucom4_run(ucom4_t *c, uint32_t cycles) {
  uint32_t total = 0;
  while (total < cycles) {
    uint32_t left = cycles - total;
    total += run_chunk(c, left > 0x7000u ? 0x7000u : (uint16_t)left);  // int16 countdown headroom
  }
  return total;
}

uint32_t ucom4_step(ucom4_t *c) { return ucom4_run(c, 1); }
