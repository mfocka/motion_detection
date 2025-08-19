#include "storage.h"

#include <string.h>

namespace motion_detection { namespace storage {

bool load(StorageBlob& out) {
    StorageBlob temp;
    bool ok = loadFromEEPROM(&temp, sizeof(StorageBlob));
    if (!ok) return false;
    if (temp.version != kStorageVersion) return false;
    memcpy(&out, &temp, sizeof(StorageBlob));
    return true;
}

bool save(const StorageBlob& in) {
    StorageBlob temp;
    memcpy(&temp, &in, sizeof(StorageBlob));
    temp.version = kStorageVersion;
    return saveToEEPROM(&temp, sizeof(StorageBlob));
}

}} // namespace motion_detection::storage


