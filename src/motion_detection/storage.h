#pragma once

#include <stdint.h>
#include "motion_detection.h"
#include "quaternion.h"

namespace motion_detection { namespace storage {

static const uint16_t kStorageVersion = 1;

struct StorageBlob {
    uint16_t version;
    ThresholdsDeg thresholds;
    WindowsSec windows;
    GyroBias gbias;
    quat_math::Quaternion q_reference;
};

bool load(StorageBlob& out);
bool save(const StorageBlob& in);

}} // namespace motion_detection::storage


