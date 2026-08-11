#pragma once
//
// Persists calibration in NVS so it survives a reboot.
//
// Two independent things are stored:
//   - gyro zero-rate bias (a property of the sensor)
//   - the mounting offset quaternion (a property of how the box sits on the bike)
//
// NVS is used because these are small and written rarely. High-rate telemetry
// goes to LittleFS instead; writing samples into NVS would destroy it.
//

#if defined(ARDUINO)

#include "../core/MathUtils.h"
#include "../sensors/ImuManager.h"

namespace moto {

class CalibrationStore {
public:
    explicit CalibrationStore(const char* nvsNamespace);

    bool begin();

    bool loadImuCalibration(ImuCalibration& out) const;
    bool saveImuCalibration(const ImuCalibration& calibration);

    bool loadMountingOffset(Quaternion& out) const;
    bool saveMountingOffset(const Quaternion& offset);

    /// Forgets everything, so the next boot starts from an uncalibrated state.
    bool clear();

private:
    const char* namespace_;
    bool        ready_ = false;
};

}  // namespace moto

#endif  // ARDUINO
