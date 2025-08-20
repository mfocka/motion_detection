#include "filters.h"

#include <math.h>

namespace motion_detection {

void constrainYawWithGravity(float& yaw_rad, float& pitch_rad) {
    // Relaxed heuristic: modestly reduce yaw magnitude when pitch is near zero to limit drift,
    // but keep enough sensitivity for real yaw changes even when level.
    float pitch_abs = fabsf(pitch_rad);
    float scale = 0.8f + 0.2f * fminf(1.0f, pitch_abs / (15.0f * 3.1415926535f / 180.0f));
    yaw_rad *= scale;
}

} // namespace motion_detection


