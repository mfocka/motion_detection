#pragma once

#include "quaternion.h"

namespace motionfx_wrapper {

struct Gbias { float gx; float gy; float gz; };
struct Orientation { float qw; float qx; float qy; float qz; };

// Initialize MotionFX with default 6-axis config at ~104 Hz
void init_104hz();

// Advanced configuration helpers (no-ops if not supported by the linked MotionFX variant)
void set_frequency_hz(int hz);
void enable_6x(bool enable);
void set_orientation(const char* orientation_str);

// If available, set internal knobs structure (X-CUBE-MEMS1 exposes MFX_knobs_t)
// We expose a raw pointer to avoid header coupling; pass pointer to the exact struct from motion_fx.h
void set_knobs_raw(const void* knobs_struct);

// Set/get gyroscope bias (deg/s)
void set_gbias(const Gbias& b);
Gbias get_gbias();

// Perform one update step. Inputs must be NED (g, deg/s). Returns quaternion (w, x, y, z)
Orientation update(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps);

} // namespace motionfx_wrapper


