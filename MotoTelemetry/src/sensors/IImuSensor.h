#pragma once
//
// Hardware abstraction for the inertial sensor.
//
// V1 ships MockImuSensor. When the ICM-20948 arrives, add Icm20948Sensor
// implementing this interface and change one line in the composition root —
// nothing above this layer needs to know.
//
// Implementations report RAW sensor-frame data. Axis remapping, bias removal
// and unit conversion are ImuManager's job.
//

#include "../core/Types.h"

namespace moto {

/// Raw sample straight off the sensor, in the sensor's own axes.
struct RawImuSample {
    uint64_t timestampUs = 0;

    Vec3 accel;  ///< m/s^2
    Vec3 gyro;   ///< rad/s
    Vec3 mag;    ///< microtesla

    bool magValid = false;
};

class IImuSensor {
public:
    virtual ~IImuSensor() = default;

    virtual bool begin() = 0;

    /// Non-blocking. Returns true and fills `out` when a new sample is ready.
    virtual bool read(RawImuSample& out) = 0;

    /// Nominal output data rate, used to size buffers and derive dt bounds.
    virtual float sampleRateHz() const = 0;

    virtual const char* name() const = 0;
};

}  // namespace moto
