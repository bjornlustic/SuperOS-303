// d650_host.c — TB-303 machine glue. See d650_host.h and PORT_MAP.md.

// Hot I/O decode path; see the ucom4.c pragma note.
#if defined(SUPEROS_COMBINED) && defined(__AVR__)
#pragma GCC optimize("O3")
#endif

#include "d650_host.h"

// ---- external uPD444 RAM (3x 1024x4 behind the 4556 decoder) -----------------
// Chip select from PE2/PE3 via the 4556: 00 = none, 01 = IC-5, 10 = IC-3,
// 11 = IC-4 (chip index 0..2). Address = chip<<10 | PE1..0<<8 | PF<<4 | PD.
static inline uint8_t ext_sel(const d650_host *h)  { return (h->lat_e >> 2) & 3; }
static inline uint16_t ext_addr(const d650_host *h) {
  return (uint16_t)((ext_sel(h) - 1) << 10)
       | (uint16_t)((h->lat_e & 3) << 8)
       | (uint16_t)(h->lat_f << 4)
       |  h->lat_d;
}
static inline uint8_t ext_get(const d650_host *h, uint16_t a) {
  return (h->ext[a >> 1] >> ((a & 1) << 2)) & 0x0F;
}
static inline void ext_put(d650_host *h, uint16_t a, uint8_t v) {
  uint8_t sh = (uint8_t)((a & 1) << 2);
  uint8_t old = h->ext[a >> 1];
  uint8_t nw  = (uint8_t)((old & (0xF0 >> sh)) | ((v & 0x0F) << sh));
  if (nw != old) { h->ext[a >> 1] = nw; h->ext_dirty = 1; }
}
// Write-transparent while the strobe (PORTI bit 0) is high and a chip is
// selected: any data/address/strobe latch change commits the PORTC latch.
// The ROM only strobes with a stable address+data (SPB 0 / RPB 0 pulse).
static void ext_refresh(d650_host *h) {
  if ((h->lat_i & 1) && ext_sel(h)) ext_put(h, ext_addr(h), h->lat_c);
}

// Selected scan row from PORTH (active-low, one row at a time; the ROM raises
// all four bits then clears one). All-high = the RUN/TAP/CLOCK status group.
// LUT (lowest clear bit, 4 if none): this runs on every PORTG/PORTH write in
// the ROM's scan loop, and the bit-test loop costs a variable shift per pass
// on the shifter-less AVR.
static inline uint8_t row_of(uint8_t ph) {
  static const uint8_t lut[16] =
    { 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4 };
  return lut[ph & 0x0f];
}

// ---- ucom4 I/O callbacks (host is passed as `user`) -------------------------
static uint8_t io_read_a(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) { for (uint8_t j=0;j<4;j++) v |= (h->in[16 + row*4 + j] & 1) << j; }
  else         { for (uint8_t j=0;j<4;j++) v |= (h->in[32 + j] & 1) << j; }  // RUN,TAP,-,CLOCK
  return v & 0x0F;
}
static uint8_t io_read_b(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) for (uint8_t j=0;j<4;j++) v |= (h->in[row*4 + j] & 1) << j;
  return v & 0x0F;
}
static uint8_t io_read_c(void *u) {            // uPD444 data bus read
  d650_host *h = (d650_host *)u;
  if (!ext_sel(h) || (h->lat_i & 1)) return 0; // no chip / write strobe active
  return ext_get(h, ext_addr(h));
}
static uint8_t io_read_d(void *u) { (void)u; return 0; }

// LED index map (pins.h OutputIndex): matrix cell = row*4+col (0..15), direct
// mode LEDs 16..19 (TIME / A# / PITCH / FUNCTION) on the PORTC pins.
static void led_set(d650_host *h, uint8_t idx, uint8_t on) {
  if (h->drv.set_led) h->drv.set_led(h->drv.u, idx, on);
}
// The ROM writes the PORTG column data while all rows are deselected, then
// selects the row — so the matrix image is applied on the PORTH row select
// (and re-applied if PORTG changes while a row is still selected).
static void led_row_apply(d650_host *h, uint8_t row) {
  if (row < 4) for (uint8_t c=0;c<4;c++) led_set(h, (uint8_t)(row*4 + c), (h->lat_g >> c) & 1);
}
static void commit_cv(d650_host *h) {
  if (h->drv.set_cv) h->drv.set_cv(h->drv.u, h->pitch, h->gate, h->accent, h->slide);
}

static void io_write(void *u, int port, uint8_t data) {
  d650_host *h = (d650_host *)u;
  data &= 0x0F;
  switch (port) {
    case UCOM4_PORTC:                           // uPD444 data latch + mode LEDs
      h->lat_c = data;
      ext_refresh(h);
      // On the board the same 4 pins drive the mode LEDs; the ROM parks the
      // LED nibble on PORTC between RAM accesses (X;OP;X in the primitives).
      led_set(h, 16, data & 0x1);               // TIME mode
      led_set(h, 17, (data >> 1) & 0x1);        // A# key
      led_set(h, 18, (data >> 2) & 0x1);        // PITCH mode
      led_set(h, 19, (data >> 3) & 0x1);        // FUNCTION
      break;
    case UCOM4_PORTD: h->lat_d = data; ext_refresh(h); break;  // RAM A0-3 / pitch low
    case UCOM4_PORTE: h->lat_e = data; ext_refresh(h); break;  // RAM A8-9 + CE / accent
    case UCOM4_PORTF: h->lat_f = data; ext_refresh(h); break;  // RAM A4-7 / pitch high
    case UCOM4_PORTG:                           // LED matrix columns
      h->lat_g = data;
      led_row_apply(h, row_of(h->lat_h));
      break;
    case UCOM4_PORTH:                           // matrix row select (active-low)
      h->lat_h = data;
      led_row_apply(h, row_of(data));
      break;
    case UCOM4_PORTI: {                         // WE(bit0), latch+slide(bit1), gate(bit2)
      uint8_t rising1 = (data & 0x2) && !(h->lat_i & 0x2);
      h->gate  = (data >> 2) & 1;
      h->slide = (data >> 1) & 1;
      if (rising1) {                            // PI1 latch: sample pitch + accent
        h->pitch  = (uint8_t)(((h->lat_f & 0x03) << 4) | h->lat_d);
        h->accent = h->lat_e & 1;
      }
      h->lat_i = data;
      ext_refresh(h);                           // SPB 0 write strobe commits here
      commit_cv(h);
      break;
    }
    default:
      break;
  }
}

// ---- public API -------------------------------------------------------------
void d650_init(d650_host *h, const uint8_t *rom, const d650_drivers *drv) {
  for (unsigned i = 0; i < sizeof *h; i++) ((uint8_t*)h)[i] = 0;
  if (drv) h->drv = *drv;
  ucom4_io io = { io_read_a, io_read_b, io_read_c, io_read_d, io_write, h };
  ucom4_reset(&h->cpu, rom, &io);
}
uint32_t d650_step(d650_host *h) { return ucom4_step(&h->cpu); }
void d650_clock(d650_host *h, int level) { ucom4_set_int(&h->cpu, level); }
