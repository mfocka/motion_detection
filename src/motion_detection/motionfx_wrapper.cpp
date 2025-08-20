#include "motionfx_wrapper.h"

#include <string.h>

// Prefer the official MotionFX header if present (X-CUBE-MEMS1 or stm32duino)
#ifdef __has_include
#  if __has_include(<motion_fx.h>)
#    include <motion_fx.h>
#    define HAS_MFX_HEADER 1
#  endif
#endif

#ifndef HAS_MFX_HEADER
#define HAS_MFX_HEADER 0
#endif

namespace motionfx_wrapper {

static Gbias s_bias{0.0f, 0.0f, 0.0f};
static float s_dt = 1.0f / 104.0f;

static uint8_t s_state_buffer[8192];
static MFX_output_t s_output;
static MFX_knobs_t s_knobs;

void init_104hz() {
    // Initialize state
    MotionFX_initialize((MFXState_t)s_state_buffer);
    // Knobs
    MotionFX_getKnobs((MFXState_t)s_state_buffer, &s_knobs);
    // Default: 6X mode enabled by caller or here
    MotionFX_enable_6X((MFXState_t)s_state_buffer, MFX_ENGINE_ENABLE);
}

void set_frequency_hz(int hz) {
    if (hz > 0) s_dt = 1.0f / (float)hz;
}

void enable_6x(bool enable) {
    MotionFX_enable_6X((MFXState_t)s_state_buffer, enable ? MFX_ENGINE_ENABLE : MFX_ENGINE_DISABLE);
}

void set_orientation(const char* orientation_str) {
    if (!orientation_str) return;
    // Expect 3 chars (e/n/u or E/N/U). Apply to acc and gyro orientations.
    s_knobs.acc_orientation[0] = orientation_str[0];
    s_knobs.acc_orientation[1] = orientation_str[1];
    s_knobs.acc_orientation[2] = orientation_str[2];
    s_knobs.acc_orientation[3] = '\0';
    s_knobs.gyro_orientation[0] = orientation_str[0];
    s_knobs.gyro_orientation[1] = orientation_str[1];
    s_knobs.gyro_orientation[2] = orientation_str[2];
    s_knobs.gyro_orientation[3] = '\0';
    s_knobs.output_type = MFX_ENGINE_OUTPUT_ENU;
    MotionFX_setKnobs((MFXState_t)s_state_buffer, &s_knobs);
}

void set_knobs_raw(const void* knobs_struct) {
    if (!knobs_struct) return;
    memcpy(&s_knobs, knobs_struct, sizeof(MFX_knobs_t));
    MotionFX_setKnobs((MFXState_t)s_state_buffer, &s_knobs);
}

void set_gbias(const Gbias& b) {
    s_bias = b;
    float gbias[3] = { b.gx, b.gy, b.gz };
    MotionFX_setGbias((MFXState_t)s_state_buffer, gbias);
}

Gbias get_gbias() {
    float gbias[3] = {0,0,0};
    MotionFX_getGbias((MFXState_t)s_state_buffer, gbias);
    return Gbias{ gbias[0], gbias[1], gbias[2] };
}

Orientation update(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps) {
    MFX_input_t in{};
    in.acc[0] = ax_g; in.acc[1] = ay_g; in.acc[2] = az_g;
    in.gyro[0] = gx_dps - s_bias.gx;
    in.gyro[1] = gy_dps - s_bias.gy;
    in.gyro[2] = gz_dps - s_bias.gz;
    // No mag in 6X mode
    float dt = s_dt;
    MotionFX_propagate((MFXState_t)s_state_buffer, &in, &dt);
    MotionFX_update((MFXState_t)s_state_buffer, &s_output, &in, &dt, nullptr);
    return Orientation{ s_output.quaternion[0], s_output.quaternion[1], s_output.quaternion[2], s_output.quaternion[3] };
}


} // namespace motionfx_wrapper

