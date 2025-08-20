#include <cmath>

// Provide the weak/stubbed MotionFX wrapper referenced in motion_detection.cpp
namespace motionfx_wrapper {

struct Gbias { float gx; float gy; float gz; };
struct Orientation { float qw; float qx; float qy; float qz; };

static Gbias g_bias{0.0f, 0.0f, 0.0f};
static Orientation g_last{1.0f, 0.0f, 0.0f, 0.0f};

void init_104hz() {
    // no-op for stub
}

void set_gbias(const Gbias& b) { g_bias = b; (void)g_bias; }
Gbias get_gbias() { return g_bias; }

Orientation update(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps) {
    // Extremely naive integrator for stub: integrate gyro (deg/s) into quaternion.
    // This is ONLY for offline tests without real MotionFX.
    const float sample_rate_hz = 104.0f;
    const float dt = 1.0f / sample_rate_hz;

    float gx = (gx_dps) * (3.1415926535f/180.0f);
    float gy = (gy_dps) * (3.1415926535f/180.0f);
    float gz = (gz_dps) * (3.1415926535f/180.0f);

    float wx = gx * dt * 0.5f;
    float wy = gy * dt * 0.5f;
    float wz = gz * dt * 0.5f;

    // Quaternion derivative qdot = 0.5 * q ⊗ omega
    float qw = g_last.qw; float qx = g_last.qx; float qy = g_last.qy; float qz = g_last.qz;
    float dq_w = - (qx*wx + qy*wy + qz*wz);
    float dq_x =   (qw*wx + qy*wz - qz*wy);
    float dq_y =   (qw*wy - qx*wz + qz*wx);
    float dq_z =   (qw*wz + qx*wy - qy*wx);

    qw += dq_w; qx += dq_x; qy += dq_y; qz += dq_z;
    float n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n == 0.0f) { qw = 1.0f; qx = qy = qz = 0.0f; }
    else { float inv = 1.0f/n; qw*=inv; qx*=inv; qy*=inv; qz*=inv; }

    g_last = Orientation{qw, qx, qy, qz};
    (void)ax_g; (void)ay_g; (void)az_g; // unused in stub
    return g_last;
}

}

