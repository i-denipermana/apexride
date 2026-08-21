#pragma once
//
// Owns the IMU device and turns raw samples into calibrated body-frame data.
//
// Responsibilities, in order of application:
//   1. axis remapping    — breakout orientation -> body frame
//   2. bias removal      — measured gyro zero-rate offset
//   3. stationary detect — used to trigger bias capture and by RideManager
//
// This is the "sensor calibration" layer from the handoff. The separate
// "mounting calibration" layer lives in Orientation, because it is a property
// of the attitude estimate rather than of the sensor.
//

#include "../core/Types.h"
#include "IImuSensor.h"

namespace apex {

/// Maps sensor axes onto body axes. Each entry names the source axis index
/// (0=X, 1=Y, 2=Z) in the sensor frame and its sign.
struct AxisMap {
    uint8_t sourceIndex[3] = {0, 1, 2};
    int8_t  sign[3]        = {1, 1, 1};

    Vec3 apply(const Vec3& sensor) const {
        const float components[3] = {sensor.x, sensor.y, sensor.z};
        return Vec3(components[sourceIndex[0]] * sign[0], components[sourceIndex[1]] * sign[1],
                    components[sourceIndex[2]] * sign[2]);
    }
};

/// Persisted in NVS so calibration survives a reboot.
struct ImuCalibration {
    Vec3     gyroBias;              ///< rad/s, subtracted from every sample
    uint16_t version    = 0;        ///< bumped on each successful calibration
    bool     gyroBiasValid = false;
};

class ImuManager {
public:
    enum class CalibrationState : uint8_t {
        Idle,
        WaitingForStill,
        Capturing,
    };

    struct Config {
        AxisMap axisMap;

        /// Number of samples averaged when capturing gyro bias.
        uint16_t biasSampleCount = 400;

        /// Stationary thresholds. The bike is considered still when the gyro
        /// magnitude and the deviation of |accel| from 1 g are both below these.
        float stationaryGyroDps       = 3.0f;
        float stationaryAccelMps2     = 0.6f;
        uint32_t stationaryHoldMs     = 750;

        /// Capture is stricter than the general stationary detector. Any
        /// violating sample aborts the batch and returns to WaitingForStill.
        float calibrationMaxGyroDps       = 5.0f;
        float calibrationMaxAccelMps2     = 0.45f;
        float calibrationMaxGyroStdDps    = 0.30f;
        float calibrationMaxAccelStdMps2  = 0.18f;
        float calibrationMaxLevelAngleDeg = 15.0f;
    };

    /// Sample timestamps come from the sensor itself, so no clock is needed here.
    explicit ImuManager(IImuSensor& sensor);

    bool begin(const Config& config);

    /// Reads at most one sample. Returns true when `out` holds a fresh,
    /// calibrated body-frame reading.
    bool poll(ImuReading& out);

    /// True when the last `stationaryHoldMs` of samples all looked still.
    bool isStationary() const { return stationary_; }

    /// Requests a robust gyro + level-reference capture. It waits for the
    /// stationary hold, rejects an entire batch if motion is seen, and only
    /// publishes a result after its variance checks pass.
    void startGyroBiasCapture();
    bool calibratingGyroBias() const { return calibrationState_ != CalibrationState::Idle; }
    CalibrationState calibrationState() const { return calibrationState_; }
    const char* calibrationStateName() const;
    uint32_t calibrationRejectionCount() const { return calibrationRejections_; }

    /// Returns the averaged level sample once for the fusion/mounting layer.
    bool consumeLevelReference(ImuReading& out);

    const ImuCalibration& calibration() const { return calibration_; }
    void                  setCalibration(const ImuCalibration& calibration);

    /// Most recent calibrated reading, whether or not it was consumed.
    const ImuReading& lastReading() const { return lastReading_; }
    const Vec3& lastRawGyro() const { return lastRawGyro_; }

    uint32_t sampleCount() const { return sampleCount_; }
    uint32_t readErrorCount() const { return sensor_.readErrorCount(); }
    uint32_t droppedSampleCount() const { return droppedSamples_; }

private:
    void updateStationary(const ImuReading& reading);
    void beginCalibrationBatch();
    void rejectCalibration(const char* reason);
    void updateCalibration(const ImuReading& mappedRaw);

    IImuSensor& sensor_;
    Config      config_{};

    ImuCalibration calibration_{};
    ImuReading     lastReading_{};
    Vec3           lastRawGyro_{};

    uint32_t sampleCount_    = 0;
    uint32_t droppedSamples_ = 0;
    uint64_t previousSampleUs_ = 0;

    CalibrationState calibrationState_ = CalibrationState::Idle;
    uint16_t calibrationSamples_ = 0;
    Vec3     gyroAccumulator_;
    Vec3     gyroSquares_;
    Vec3     accelAccumulator_;
    Vec3     accelSquares_;
    ImuReading levelReference_{};
    bool       levelReferencePending_ = false;
    uint32_t   calibrationRejections_ = 0;

    bool     stationary_          = false;
    bool     movingSinceValid_    = false;
    uint32_t stationarySinceMs_   = 0;
};

}  // namespace apex
