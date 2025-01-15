#pragma once

#include <cmath>

static inline float minimum_mod_diff(float a, float b) {
    float diff = fmodf(a - b + 180, 360) - 180;
    return diff + (diff < -180) * 360;
}

static inline float sinc(float x) {
    if (fabsf(x) < 1e-3) return 1 - (x*x/6.0f) + (x*x*x*x/120.0f);
    return sinf(x) / x;
}