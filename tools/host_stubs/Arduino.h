#pragma once
#include <stdint.h>
#include <string.h>
static inline unsigned long micros() { return 123457; }
static inline unsigned long millis() { return 123457; }
template <typename T> static inline T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline long constrain(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
