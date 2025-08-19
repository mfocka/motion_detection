#include "filters.h"

#include <math.h>

namespace motion_detection {

void constrainYawWithGravity(float& yaw_rad, float& pitch_rad) {
    // Very light heuristic: reduce yaw magnitude when pitch is near zero (gravity alignment weak for yaw)
    // and allow more yaw when pitch is larger (non-zero Ax, Ay give better heading constraint).
    float pitch_abs = fabsf(pitch_rad);
    float scale = 0.3f + 0.7f * fminf(1.0f, pitch_abs / (15.0f * 3.1415926535f / 180.0f));
    yaw_rad *= scale;
}

} // namespace motion_detection


