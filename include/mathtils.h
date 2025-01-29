#pragma once

#include <cmath>
#include <vector>

#define deg(x) (x * 57.2957795131f)
#define rad(x) (x * 0.0174532925199f)

static inline float minimum_mod_diff(float a, float b, float mod) {
    float diff = fmodf(a - b + mod/2, mod) - mod/2;
    return diff + (diff < -mod/2) * mod;
}

static inline float sinc(float x) {
    if (fabsf(x) < 1e-3) return 1 - (x*x/6.0f) + (x*x*x*x/120.0f);
    return sinf(x) / x;
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
static inline float average(const std::vector<T>& vec) {
    if (vec.empty()) return 0;

    // https://en.wikipedia.org/wiki/Kahan_summation_algorithm
    double sum = 0; double z = 0;
    for (auto& val : vec) {
        double y = val - z;
        double t = sum + y;
        z = (t - sum) - y;
        sum = t;
    }
    return static_cast<float>(sum / vec.size());
}

static inline float modfix(float x, float y) {
    float result = fmodf(x, y);
    return result + (result < 0) * y;
}