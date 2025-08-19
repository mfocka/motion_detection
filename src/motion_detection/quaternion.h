#pragma once

#include <math.h>

namespace quat_math {

struct Quaternion { float w; float x; float y; float z; };
struct Vector3 { float x; float y; float z; };

inline Quaternion normalize(const Quaternion& q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n == 0.0f) return Quaternion{1,0,0,0};
    float inv = 1.0f / n;
    return Quaternion{q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

inline Quaternion conjugate(const Quaternion& q) { return Quaternion{q.w, -q.x, -q.y, -q.z}; }

inline Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return Quaternion{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

inline Vector3 rotateVector(const Quaternion& q, const Vector3& v) {
    // v' = q * (0, v) * q^{-1}
    Quaternion vq{0.0f, v.x, v.y, v.z};
    Quaternion qi = conjugate(normalize(q));
    Quaternion t = multiply(q, multiply(vq, qi));
    return Vector3{t.x, t.y, t.z};
}

inline Quaternion quaternionFromRotationMatrix(float r00, float r01, float r02,
                                               float r10, float r11, float r12,
                                               float r20, float r21, float r22) {
    float trace = r00 + r11 + r22;
    float w, x, y, z;
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (r21 - r12) / s;
        y = (r02 - r20) / s;
        z = (r10 - r01) / s;
    } else if ((r00 > r11) && (r00 > r22)) {
        float s = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
        w = (r21 - r12) / s;
        x = 0.25f * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        float s = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25f * s;
        z = (r12 + r21) / s;
    } else {
        float s = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25f * s;
    }
    return normalize(Quaternion{w, x, y, z});
}

inline void eulerFromQuaternionYawPitchRoll(const Quaternion& q, float& yaw, float& pitch, float& roll) {
    // Yaw-Pitch-Roll intrinsic ZYX
    float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    yaw = atan2f(siny_cosp, cosy_cosp);

    float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    if (fabsf(sinp) >= 1.0f) pitch = copysignf(3.1415926535f/2.0f, sinp); else pitch = asinf(sinp);

    float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    roll = atan2f(sinr_cosp, cosr_cosp);
}

} // namespace quat_math


