
#include "waveforms.h"
#include "math_constants.h"
#include <algorithm>
#include <cmath>

float triangle_with_plateau(float x) {
    const float period = Constants::TWO_PI_F;
    const float segment = period * 0.16666666666f;
    float t = fmodf(x, period);  // wrap x to [0, 2π)

    if (t < segment) {
        return t / segment;  // 0 → +1
    } else if (t < 2 * segment) {
        return 1.0f - (t - segment) / segment;  // +1 → 0
    } else if (t < 3 * segment) {
        return 0.0f;  // plateau
    } else if (t < 4 * segment) {
        return -(t - 3 * segment) / segment;  // 0 → -1
    } else if (t < 5 * segment) {
        return -1.0f + (t - 4 * segment) / segment;  // -1 → 0
    } else {
        return 0.0f;  // plateau
    }
}

float triangle_wave(float x) {
    const float period = Constants::TWO_PI_F;
    float t = fmodf(x, period);

    // Normalize t to [0, 1)
    float phase = t / period;

    // Scale to triangle shape in [-1, 1]
    if (phase < 0.25f)
        return 4.0f * phase;             // 0 → +1
    else if (phase < 0.75f)
        return 2.0f - 4.0f * phase;      // +1 → -1
    else
        return -4.0f + 4.0f * phase;     // -1 → 0
}

float trapezoidal_wave(float x, float plateau_fraction) {
    const float two_pi = 2.0f * M_PI;

    // Clamp plateau_fraction to valid range
    plateau_fraction = std::clamp(plateau_fraction, 0.0f, 0.4999f);

    // Calculate segment widths
    float plateau_width = plateau_fraction * two_pi;
    float ramp_width = (two_pi - 2 * plateau_width) / 2.0f;

    // Wrap x into [0, 2π)
    float phase = fmodf(x, two_pi);
    if (phase < 0.0f) phase += two_pi;

    if (phase < plateau_width) {
        // Bottom plateau
        return -1.0f;
    } else if (phase < plateau_width + ramp_width) {
        // Rising edge
        return -1.0f + 2.0f * (phase - plateau_width) / ramp_width;
    } else if (phase < plateau_width + ramp_width + plateau_width) {
        // Top plateau
        return 1.0f;
    } else {
        // Falling edge
        return 1.0f - 2.0f * (phase - (2 * plateau_width + ramp_width)) / ramp_width;
    }
}

