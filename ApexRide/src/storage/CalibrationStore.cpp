#include "CalibrationStore.h"

#if defined(ARDUINO)

#include <Preferences.h>

#include "../core/Log.h"

namespace apex {
namespace {

constexpr const char* kKeyGyroBias   = "gyroBias";
constexpr const char* kKeyMounting   = "mountQuat";
constexpr const char* kKeyCalVersion = "calVer";

struct StoredVec3 {
    float x;
    float y;
    float z;
};

struct StoredQuaternion {
    float w;
    float x;
    float y;
    float z;
};

}  // namespace

CalibrationStore::CalibrationStore(const char* nvsNamespace) : namespace_(nvsNamespace) {}

bool CalibrationStore::begin() {
    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/false)) {
        APEX_LOGE("NVS namespace '%s' unavailable", namespace_);
        return false;
    }
    preferences.end();

    ready_ = true;
    return true;
}

bool CalibrationStore::loadImuCalibration(ImuCalibration& out) const {
    if (!ready_) return false;

    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/true)) {
        return false;
    }

    StoredVec3 stored{};
    if (!preferences.isKey(kKeyGyroBias)) {
        preferences.end();
        return false;
    }
    const size_t got = preferences.getBytes(kKeyGyroBias, &stored, sizeof(stored));
    const uint16_t version = preferences.getUShort(kKeyCalVersion, 0);
    preferences.end();

    if (got != sizeof(stored)) {
        return false;
    }

    out.gyroBias      = Vec3(stored.x, stored.y, stored.z);
    out.gyroBiasValid = true;
    out.version       = version;

    APEX_LOGI("Loaded gyro bias %.3f %.3f %.3f dps (cal v%u)",
            static_cast<double>(out.gyroBias.x * kRadToDeg),
            static_cast<double>(out.gyroBias.y * kRadToDeg),
            static_cast<double>(out.gyroBias.z * kRadToDeg),
            static_cast<unsigned>(version));
    return true;
}

bool CalibrationStore::saveImuCalibration(const ImuCalibration& calibration) {
    if (!ready_ || !calibration.gyroBiasValid) return false;

    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/false)) {
        return false;
    }

    const StoredVec3 stored{calibration.gyroBias.x, calibration.gyroBias.y, calibration.gyroBias.z};
    const bool written = preferences.putBytes(kKeyGyroBias, &stored, sizeof(stored)) ==
                         sizeof(stored);
    preferences.putUShort(kKeyCalVersion, calibration.version);
    preferences.end();

    return written;
}

bool CalibrationStore::loadMountingOffset(Quaternion& out) const {
    if (!ready_) return false;

    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/true)) {
        return false;
    }

    StoredQuaternion stored{};
    if (!preferences.isKey(kKeyMounting)) {
        preferences.end();
        return false;
    }
    const size_t got = preferences.getBytes(kKeyMounting, &stored, sizeof(stored));
    preferences.end();

    if (got != sizeof(stored)) {
        return false;
    }

    out = Quaternion(stored.w, stored.x, stored.y, stored.z);
    out.normalize();

    APEX_LOGI("Loaded mounting offset: roll %.2f deg, pitch %.2f deg",
            static_cast<double>(out.eulerX() * kRadToDeg),
            static_cast<double>(-out.eulerY() * kRadToDeg));
    return true;
}

bool CalibrationStore::saveMountingOffset(const Quaternion& offset) {
    if (!ready_) return false;

    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/false)) {
        return false;
    }

    const StoredQuaternion stored{offset.w, offset.x, offset.y, offset.z};
    const bool written = preferences.putBytes(kKeyMounting, &stored, sizeof(stored)) ==
                         sizeof(stored);
    preferences.end();

    return written;
}

bool CalibrationStore::clear() {
    if (!ready_) return false;

    Preferences preferences;
    if (!preferences.begin(namespace_, /*readOnly=*/false)) {
        return false;
    }
    const bool cleared = preferences.clear();
    preferences.end();

    APEX_LOGW("Calibration cleared");
    return cleared;
}

}  // namespace apex

#endif  // ARDUINO
