#include "motionfx_wrapper.h"

#include <string.h>

// Prefer the official MotionFX header if present (X-CUBE-MEMS1 or stm32duino)
#ifdef __has_include
#  if __has_include(<motion_fx.h>)
#    include <motion_fx.h>
#    define HAVE_MOTIONFX 1
#  endif
#endif

#ifndef HAVE_MOTIONFX
// Fallback: allow build without the library (no-op integration returning identity)
#define HAVE_MOTIONFX 0
#endif

namespace motionfx_wrapper {

static Gbias s_bias{0.0f, 0.0f, 0.0f};

#if HAVE_MOTIONFX
static MFX_output_t s_mfx_output;
#ifdef MFX_knobs_t
static MFX_knobs_t s_knobs;
#endif

void init_104hz() {
    // Initialize and configure defaults
#ifdef MFX_init
    MFX_init();
#endif
#ifdef MFX_setFrequency
    MFX_setFrequency(104);
#endif
#ifdef MFX_enable6X
    MFX_enable6X(1);
#endif
#ifdef MFX_getKnobs
    MFX_getKnobs(&s_knobs);
#ifdef MFX_setKnobs
    MFX_setKnobs(&s_knobs);
#endif
#endif
}

void set_frequency_hz(int hz) {
#ifdef MFX_setFrequency
    MFX_setFrequency(hz);
#else
    (void)hz;
#endif
}

void enable_6x(bool enable) {
#ifdef MFX_enable6X
    MFX_enable6X(enable ? 1 : 0);
#else
    (void)enable;
#endif
}

void set_orientation(const char* orientation_str) {
#ifdef MFX_setOrientation
    MFX_setOrientation((char*)orientation_str);
#elif defined(MotionFX_setOrientation)
    MotionFX_setOrientation((char*)orientation_str);
#else
    (void)orientation_str;
#endif
}

void set_knobs_raw(const void* knobs_struct) {
#ifdef MFX_knobs_t
    if (knobs_struct) {
        memcpy(&s_knobs, knobs_struct, sizeof(MFX_knobs_t));
#ifdef MFX_setKnobs
        MFX_setKnobs(&s_knobs);
#endif
    }
#else
    (void)knobs_struct;
#endif
}

void set_gbias(const Gbias& b) {
    s_bias = b;
    // If API supports direct bias set, apply here; otherwise keep locally
}

Gbias get_gbias() {
    return s_bias;
}

Orientation update(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps) {
    MFX_input_t in;
    memset(&in, 0, sizeof(in));
    // Inputs in g and dps per MotionFX API
    in.acc[0] = ax_g; in.acc[1] = ay_g; in.acc[2] = az_g;
    in.gyro[0] = gx_dps - s_bias.gx;
    in.gyro[1] = gy_dps - s_bias.gy;
    in.gyro[2] = gz_dps - s_bias.gz;

    MFX_update(&in, &s_mfx_output);
    // Output quaternion order: w, x, y, z
    return Orientation{ s_mfx_output.quaternion[0], s_mfx_output.quaternion[1], s_mfx_output.quaternion[2], s_mfx_output.quaternion[3] };
}

#else

void init_104hz() {}
void set_frequency_hz(int) {}
void enable_6x(bool) {}
void set_orientation(const char*) {}
void set_knobs_raw(const void*) {}
void set_gbias(const Gbias& b) { s_bias = b; }
Gbias get_gbias() { return s_bias; }
Orientation update(float, float, float, float, float, float) { return Orientation{1.0f, 0.0f, 0.0f, 0.0f}; }

#endif

} // namespace motionfx_wrapper


