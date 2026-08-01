#pragma once
// prob_codec.h -- step-probability table storage codec (variation 1).
//
// Split out of persistent_settings.h so it can be exercised on the host.
// Include AFTER the FB_PROB_* constants, MAX_STEPS
// and the g_flash instance are in scope.

// Probability tables: 3 bytes/step in RAM, sparse on flash. See the region
// layout in flash_persist.h. PROB_SLOTS_PER_BLOCK slots share one record, so an
// armed slot costs a third of a page instead of a whole one.
inline uint16_t prob_group_block(uint8_t abs_slot) {
  return uint16_t(FB_PROB_BASE + abs_slot / PROB_SLOTS_PER_BLOCK);
}
inline uint8_t prob_region_index(uint8_t abs_slot) {
  return uint8_t(abs_slot % PROB_SLOTS_PER_BLOCK);
}

static constexpr uint8_t PR_ARMED_OFF = 1;
static constexpr uint8_t PR_UPDBL_OFF  = uint8_t(1 + MAX_STEPS / 8);
static constexpr uint8_t PR_PAY_OFF    = uint8_t(1 + 2 * (MAX_STEPS / 8));

inline void ReadProbAt(uint8_t abs_slot, uint8_t *dst192) {
  memset(dst192, 0, FB_PROB_LEN);
  uint8_t blk[FB_PROB_BLK_LEN];
  if (g_flash.read(prob_group_block(abs_slot), blk, FB_PROB_BLK_LEN) != FB_PROB_BLK_LEN)
    return;                                     // never written: table is all zeros
  const uint8_t *r = blk + prob_region_index(abs_slot) * PROB_REGION;
  const uint8_t n = r[0];
  if (n == 0xFF) {                              // too dense for the region
    g_flash.read(uint16_t(FB_PROBX_BASE + abs_slot), dst192, FB_PROB_LEN);
    return;
  }
  const uint8_t *pay = r + PR_PAY_OFF;
  uint8_t seen = 0;
  for (uint8_t step = 0; step < MAX_STEPS && seen < n; ++step) {
    if (!((r[PR_ARMED_OFF + (step >> 3)] >> (step & 7)) & 1)) continue;
    const uint8_t k = uint8_t(step * 3);
    dst192[k]     = pay[seen * 2];
    dst192[k + 1] = pay[seen * 2 + 1];
    dst192[k + 2] = (r[PR_UPDBL_OFF + (step >> 3)] >> (step & 7)) & 1;
    ++seen;
  }
}

inline void WriteProbAt(uint8_t abs_slot, const uint8_t *src192) {
  uint8_t armed[MAX_STEPS / 8] = {0};
  uint8_t updbl[MAX_STEPS / 8] = {0};
  uint8_t n = 0;
  for (uint8_t step = 0; step < MAX_STEPS; ++step) {
    const uint8_t k = uint8_t(step * 3);
    if (!src192[k] && !src192[k + 1] && !src192[k + 2]) continue;
    armed[step >> 3] |= uint8_t(1u << (step & 7));
    if (src192[k + 2]) updbl[step >> 3] |= uint8_t(1u << (step & 7));
    ++n;
  }

  const uint16_t gblk = prob_group_block(abs_slot);
  uint8_t blk[FB_PROB_BLK_LEN];
  const bool have = (g_flash.read(gblk, blk, FB_PROB_BLK_LEN) == FB_PROB_BLK_LEN);
  // All-zero table and no group record yet: skip, so unarmed slots never
  // consume a page. If the group exists it must be rewritten to clear this
  // slot's region.
  if (n == 0 && !have) return;
  if (!have) memset(blk, 0, sizeof(blk));

  uint8_t *r = blk + prob_region_index(abs_slot) * PROB_REGION;
  memset(r, 0, PROB_REGION);
  if (n > PROB_SPARSE_MAX) {
    // Dense fallback: the region just points at the per-slot overflow block.
    r[0] = 0xFF;
    g_flash.write(uint16_t(FB_PROBX_BASE + abs_slot), src192, FB_PROB_LEN);
  } else {
    r[0] = n;
    memcpy(r + PR_ARMED_OFF, armed, sizeof(armed));
    memcpy(r + PR_UPDBL_OFF, updbl, sizeof(updbl));
    uint8_t *pay = r + PR_PAY_OFF;
    uint8_t seen = 0;
    for (uint8_t step = 0; step < MAX_STEPS; ++step) {
      if (!((armed[step >> 3] >> (step & 7)) & 1)) continue;
      const uint8_t k = uint8_t(step * 3);
      pay[seen * 2]     = src192[k];
      pay[seen * 2 + 1] = src192[k + 1];
      ++seen;
    }
  }
  g_flash.write(gblk, blk, FB_PROB_BLK_LEN);
}

// Variation 3 in poly form: its own dedicated block (poly slots only).
