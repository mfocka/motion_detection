#pragma once

#include <stdint.h>

namespace motion_detection {

struct FirstOrderLowPass {
    float a0; float b1; float y1;
    void reset(float cutoff_hz, float fs_hz) {
        float dt = 1.0f / fs_hz;
        float rc = 1.0f / (6.28318530718f * cutoff_hz);
        float alpha = dt / (rc + dt);
        a0 = alpha; b1 = 1.0f - alpha; y1 = 0.0f;
    }
    float step(float x) { y1 = a0 * x + b1 * y1; return y1; }
};

struct FirstOrderHighPass {
    float a0; float a1; float b1; float x1; float y1;
    void reset(float cutoff_hz, float fs_hz) {
        float dt = 1.0f / fs_hz;
        float rc = 1.0f / (6.28318530718f * cutoff_hz);
        float alpha = rc / (rc + dt);
        a0 = alpha; a1 = -alpha; b1 = alpha; x1 = 0.0f; y1 = 0.0f;
    }
    float step(float x) { float y = a0 * x + a1 * x1 + b1 * y1; x1 = x; y1 = y; return y; }
};

struct RmsAccumulator {
    uint32_t window; uint32_t count; float sumsq;
    void reset(uint32_t n) { window = n; count = 0; sumsq = 0.0f; }
    void push(float v) { sumsq += v*v; if (count < window) count++; else sumsq -= sumsq / (float)window; }
    void push3(float x, float y, float z) { float m = x*x + y*y + z*z; push(sqrtf(m)); }
    float rms() const { if (count == 0) return 0.0f; return sqrtf(sumsq / (float)count); }
    uint32_t sampleCount() const { return count; }
};

struct RollingVariance {
    uint32_t window; uint32_t count; float mean; float m2;
    void reset(uint32_t n) { window = n; count = 0; mean = 0.0f; m2 = 0.0f; }
    void push(float x) {
        // Simple Welford without removal (approx for short windows)
        count = (count < window) ? (count + 1) : window;
        float delta = x - mean; mean += delta / (float)count; m2 += delta * (x - mean);
    }
    float variance() const { if (count < 2) return 0.0f; return m2 / (float)(count - 1); }
};

// Physics-aware helper: constrain yaw using gravity projection
void constrainYawWithGravity(float& yaw_rad, float& pitch_rad);

} // namespace motion_detection


