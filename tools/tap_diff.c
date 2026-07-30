// tap_diff.c -- DIFFERENTIAL parity test: the SuperOS tap-write recorder
// algorithm (exact port of src/main.cpp's tick logic) vs the LIVE TB-303
// mask ROM running on the d650c emulator core. Every scenario is executed
// on both and the recorded measure is compared step for step.
//
// Build:
//   cc -std=c11 -O2 -I <emu> tap_diff.c <emu>/d650_host.c <emu>/ucom4.c -o tap_diff
//
// Timing note: the ROM's key scan runs ~0.02 step ahead of the nominal tick
// grid, so press/release instants within EPS of a tick boundary (multiples
// of 1/6 step) are below scan resolution and are excluded from the sweep,
// exactly as tools/tap_parity.c documents.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "d650_host.h"
#include "tb303_rom.h"

#define MAXTAPS 8
typedef struct { double press, release; } Tap;

// ---------------------------------------------------------------------------
// SuperOS recorder simulation (lockstep with src/main.cpp:3629-3692).
// t_dec = 2 (16ths). Accept at t>=2 with the key still held; t==2 targets the
// current step, t>2 targets the next (dropped past the bar end). Decision at
// t==2: prewritten skip, else TIE while the tapped finger holds, sustain
// tie-fill on RESTs after the first note, else REST.
// ---------------------------------------------------------------------------
static void superos_sim(const Tap *taps, int ntaps, int sustain, char out[17]) {
  int pending = 0, note_active = 0, any_note = 0, prewritten = 0, had_accept = 0;
  int consumed[MAXTAPS]; memset(consumed, 0, sizeof consumed);
  char steps[17]; memset(steps, '.', 16); steps[16] = 0;
  for (int tick = 0; tick < 16 * 6; tick++) {
    double T = tick / 6.0;
    int k = tick / 6, t = tick % 6;
    for (int i = 0; i < ntaps; i++)
      if (!consumed[i] && taps[i].press <= T && taps[i].press > T - 1.0 / 6.0) {
        pending = 1; consumed[i] = 1;
      }
    int held = 0;
    for (int i = 0; i < ntaps; i++)
      if (taps[i].press <= T && T < taps[i].release) held = 1;
    if (!held) note_active = 0;
    int wrote_now = 0;
    if (t >= 2 && pending) {
      // write regardless of held (stale presses still land); held feeds the
      // tie chain and the bar-validation flag only
      pending = 0;
      int target = (t == 2) ? k : k + 1;
      if (target < 16) {
        steps[target] = 'N';
        any_note = 1;
        if (held) { note_active = 1; had_accept = 1; }
        if (target != k) prewritten = 1; else wrote_now = 1;
      }
    }
    if (t == 2) {
      if (prewritten) prewritten = 0;
      else if (!wrote_now) {
        if (note_active && held)                       steps[k] = 't';
        else if (any_note && sustain && steps[k] == '.') steps[k] = 't';
      }
    }
  }
  // bar validation: no held accept all bar -> the recording is discarded
  if (!had_accept) memset(steps, '.', 16);
  memcpy(out, steps, 17);
}

// ---------------------------------------------------------------------------
// Live-ROM driver (from test/tapmode_sweep.c)
// ---------------------------------------------------------------------------
static d650_host H;
#define INT_PERIOD_CYC  205u
#define CLK_HALF_CYC   1184u
#define TICK_CYC       (2u * CLK_HALF_CYC)
#define STEP_CYC       (6u * TICK_CYC)

typedef struct { uint32_t cyc; uint8_t pitch, on, acc; } Ev;
static Ev ev[8192]; static int nev;
static uint8_t last_gate;

static void mock_cv(void *u, uint8_t p, uint8_t g, uint8_t a, uint8_t s) {
  (void)u; (void)s;
  if (g != last_gate && nev < 8192) {
    ev[nev].cyc = H.cpu.cyc; ev[nev].pitch = p; ev[nev].on = g; ev[nev].acc = a; nev++;
  }
  last_gate = g;
}
static void mock_led(void *u, uint8_t i, uint8_t on) { (void)u;(void)i;(void)on; }

static void run_until(uint32_t target) {
  while (H.cpu.cyc < target) {
    d650_clock(&H, (H.cpu.cyc % INT_PERIOD_CYC) < 8);
    H.in[D650_IN_CLOCK] = (H.cpu.cyc / CLK_HALF_CYC) & 1;
    d650_step(&H);
  }
}
static void run_cycles(uint32_t n) { run_until(H.cpu.cyc + n); }
static void press(int idx) { H.in[idx] = 1; run_cycles(28000); H.in[idx] = 0; run_cycles(17000); }

static void setup_session(void) {
  d650_drivers drv = { mock_cv, mock_led, 0 };
  d650_init(&H, tb303_rom, &drv);
  memset(H.in, 0, sizeof H.in);
  H.in[16] = 1; H.in[17] = 0;
  run_cycles(2000000);
  press(0);
  press(26);
  for (int i = 0; i < 8; i++) press(i);
  press(25);
  press(26);
  nev = 0; last_gate = 0;
  H.in[32] = 1;
  run_cycles(60000);
}

static uint32_t start_session(void) {
  int base = nev;
  H.in[24] = 1;
  uint32_t deadline = H.cpu.cyc + 3 * STEP_CYC;
  uint32_t bar = 0;
  while (H.cpu.cyc < deadline && !bar) {
    run_cycles(500);
    for (int i = base; i < nev; i++)
      if (ev[i].on && ev[i].pitch == 51) { bar = ev[i].cyc; break; }
  }
  run_cycles(8000);
  H.in[24] = 0;
  return bar;
}

static int decode_bar(uint32_t bar_cyc, uint8_t steps[16]) {
  memset(steps, 0, 16);
  int notes = 0;
  for (int i = 0; i < nev; i++) {
    if (!ev[i].on) continue;
    if (ev[i].pitch == 51 || ev[i].pitch == 63) continue;
    double p = (double)(int64_t)(ev[i].cyc - bar_cyc) / STEP_CYC;
    if (p < -0.3 || p >= 15.7) continue;
    int k = (int)(p + 0.5);
    if (k < 0 || k > 15) continue;
    uint32_t offc = ev[i].cyc + STEP_CYC / 2;
    for (int j = i + 1; j < nev; j++)
      if (!ev[j].on) { offc = ev[j].cyc; break; }
    double len = (double)(offc - ev[i].cyc) / STEP_CYC;
    steps[k] = 1; notes++;
    for (int t = 1; t < 16 && (len > t + 0.25) && (k + t) < 16; t++)
      steps[k + t] = 2;
  }
  return notes;
}

// Run one scenario on the live ROM, return the recorded measure as N/t/.
static int rom_run(const Tap *taps, int ntaps, int sustain, char out[17]) {
  setup_session();
  uint32_t bar = start_session();
  if (!bar) return 0;
  if (sustain) { run_cycles(2000); H.in[0] = 1; }
  // schedule taps in press order; overlapping taps share the one TAP line
  for (int i = 0; i < ntaps; i++) {
    int64_t on  = (int64_t)bar + (int64_t)llround(taps[i].press  * STEP_CYC);
    int64_t off = (int64_t)bar + (int64_t)llround(taps[i].release * STEP_CYC);
    if ((uint32_t)on > H.cpu.cyc) run_until((uint32_t)on);
    H.in[33] = 1;
    if ((uint32_t)off > H.cpu.cyc) run_until((uint32_t)off);
    // keep held if the NEXT tap overlaps this release
    if (!(i + 1 < ntaps && taps[i + 1].press <= taps[i].release))
      H.in[33] = 0;
  }
  run_until(bar + 34 * STEP_CYC);
  H.in[32] = 0; H.in[0] = 0;
  uint8_t st[16];
  decode_bar(bar + 16 * STEP_CYC, st);
  for (int k = 0; k < 16; k++)
    out[k] = st[k] == 1 ? 'N' : st[k] == 2 ? 't' : '.';
  out[16] = 0;
  return 1;
}

// ---------------------------------------------------------------------------
// Scenario driver
// ---------------------------------------------------------------------------
static int fails = 0, runs = 0, skipped = 0;

// Exclusion zones (measured on the ROM, tap_probe modes 3/4):
// - The ROM's key scan LEADS the nominal tick grid: the on-time/late
//   boundary sits at k+0.307 (nominal k+1/3) and the late/stale boundary at
//   k+0.782 (nominal k+5/6). Instants inside (tick - 0.06, tick + 0.01) of
//   any nominal tick are inside that scan-phase sliver and are excluded:
//   both implementations apply the same law there, phase-shifted by a few ms.
// - The ROM requires CLEAR held THROUGH the bar reset (hold < press-to-bar
//   kills the session), so all of step 0 is inside the entry gesture's
//   shadow: its on-time window is dead and its late-accept scan instants sit
//   ~0.1 step later than every other step's (measured: press 0.36 rel 0.56
//   drops, press 1.36 rel 1.48 accepts). Presses before 0.65 are excluded;
//   SuperOS's entry (CLEAR+TIME, looping session) diverges there by design.
static int in_scan_sliver(double x) {
  double f = fmod(x * 6.0, 1.0);
  if (f < 0) f += 1.0;
  return f > 1.0 - 0.06 * 6.0 || f < 0.01 * 6.0;
}
static int scenario_ok(const Tap *taps, int ntaps) {
  for (int i = 0; i < ntaps; i++) {
    if (in_scan_sliver(taps[i].press) || in_scan_sliver(taps[i].release)) return 0;
    if (taps[i].press < 0.65 || taps[i].release > 15.6) return 0;
    if (i && taps[i].press < taps[i - 1].release + 0.05) return 0; // one button: no overlap
  }
  return 1;
}

static void diff_one(const char *tag, const Tap *taps, int ntaps, int sustain,
                     int verbose) {
  if (!scenario_ok(taps, ntaps)) { skipped++; return; }
  char rom[17], sos[17];
  if (!rom_run(taps, ntaps, sustain, rom)) {
    printf("%-52s ROM SESSION FAILED\n", tag); fails++; return;
  }
  superos_sim(taps, ntaps, sustain, sos);
  runs++;
  int ok = strcmp(rom, sos) == 0;
  if (!ok || verbose)
    printf("%-52s rom=%s sos=%s %s\n", tag, rom, sos, ok ? "OK" : "MISMATCH");
  if (!ok) {
    fails++;
    for (int i = 0; i < ntaps; i++)
      printf("    tap[%d] press=%.3f release=%.3f\n", i, taps[i].press, taps[i].release);
  }
}

static unsigned rng = 0x303BEEF;
static double frand(void) {              // deterministic xorshift
  rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
  return (double)(rng & 0xFFFFFF) / 0x1000000;
}

int main(int argc, char **argv) {
  int verbose = argc > 1 && !strcmp(argv[1], "-v");
  if (argc > 2) rng = (unsigned)atoi(argv[2]);
  char tag[96];

  // 1. Fine assignment sweep: single tap, three durations, x = 0.36..3.9
  double durs[3] = { 0.20, 0.30, 0.45 };
  for (int d = 0; d < 3; d++)
    for (double x = 0.36; x <= 3.9; x += 0.04) {
      Tap tp = { x, x + durs[d] };
      snprintf(tag, sizeof tag, "assign x=%.2f dur=%.2f", x, durs[d]);
      diff_one(tag, &tp, 1, 0, verbose);
    }

  // 2. Hold sweep: press at k+0.10 / k+0.55, hold 0.1..5.3
  double offs[4] = { 1.10, 4.10, 2.55, 7.55 };
  for (int o = 0; o < 4; o++)
    for (double d = 0.12; d <= 5.3; d += 0.21) {
      Tap tp = { offs[o], offs[o] + d };
      snprintf(tag, sizeof tag, "hold @%.2f d=%.2f", offs[o], d);
      diff_one(tag, &tp, 1, 0, verbose);
    }

  // 3. Sustain: the transcribed trio plus offset/duration variants, both flags
  for (int sus = 0; sus <= 1; sus++) {
    Tap tri[3] = { {1.10, 1.45}, {5.10, 5.45}, {11.10, 11.45} };
    snprintf(tag, sizeof tag, "sustain=%d trio 1.1/5.1/11.1", sus);
    diff_one(tag, tri, 3, sus, verbose);
    Tap tri2[3] = { {0.55, 1.15}, {6.20, 8.45}, {12.10, 12.25} };
    snprintf(tag, sizeof tag, "sustain=%d mixed 0.55/6.2h/12.1", sus);
    diff_one(tag, tri2, 3, sus, verbose);
    Tap tri3[4] = { {2.15, 2.40}, {3.15, 3.40}, {4.15, 4.40}, {9.55, 11.20} };
    snprintf(tag, sizeof tag, "sustain=%d run-of-3 + hold", sus);
    diff_one(tag, tri3, 4, sus, verbose);
  }

  // 4. Dense rhythms: consecutive on-time taps (every step / every 2nd step)
  {
    Tap every[8];
    for (int i = 0; i < 8; i++) { every[i].press = i + 1.10; every[i].release = i + 1.40; }
    diff_one("dense: 8 taps steps 1-8", every, 8, 0, verbose);
    for (int i = 0; i < 8; i++) { every[i].press = 2 * i + 1.10; every[i].release = 2 * i + 1.40; }
    diff_one("dense: 8 taps every 2nd step", every, 8, 0, verbose);
    // late-aimed taps in a row (each targets the NEXT step)
    for (int i = 0; i < 6; i++) { every[i].press = 2 * i + 0.55; every[i].release = 2 * i + 0.95; }
    diff_one("dense: 6 late taps (next-step aims)", every, 6, 0, verbose);
  }

  // 5. Randomized scenarios: 2-5 taps, random safe positions, both sustains
  for (int n = 0; n < 1200; n++) {
    Tap tp[5]; int nt = 2 + (int)(frand() * 4);
    double t0 = 0.4;
    int bad = 0;
    for (int i = 0; i < nt; i++) {
      t0 += 0.15 + frand() * (13.0 / nt - 0.2);
      if (t0 < 0.7) t0 = 0.7;
      while (in_scan_sliver(t0)) t0 += 0.02;      // snap out of the scan sliver
      double dur = 0.15 + frand() * 2.2;
      while (in_scan_sliver(t0 + dur)) dur += 0.02;
      tp[i].press = t0; tp[i].release = t0 + dur;
      t0 += dur + 0.06 + frand() * 0.3;           // one button: strictly sequential
      if (tp[i].release > 15.5) bad = 1;
    }
    if (bad || !scenario_ok(tp, nt)) { skipped++; continue; }
    int sus = n & 1;
    snprintf(tag, sizeof tag, "random #%d ntaps=%d sus=%d", n, nt, sus);
    diff_one(tag, tp, nt, sus, verbose);
  }

  printf("\n%d scenarios compared, %d skipped (grid-adjacent), %d mismatches\n",
         runs, skipped, fails);
  printf(fails ? "DIFFERENTIAL PARITY FAILED\n"
               : "DIFFERENTIAL PARITY: SuperOS == TB-303 ROM on all compared scenarios\n");
  return fails != 0;
}
