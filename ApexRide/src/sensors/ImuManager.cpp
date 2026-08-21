#include "ImuManager.h"

#include <math.h>

#include "../core/Log.h"

namespace apex {

ImuManager::ImuManager(IImuSensor& sensor) : sensor_(sensor) {}

bool ImuManager::begin(const Config& config) {
    config_ = config;

    if (!sensor_.begin()) {
        APEX_LOGE("IMU %s failed to start", sensor_.name());
        return false;
    }

    APEX_LOGI("IMU %s ready at %.0f Hz", sensor_.name(), static_cast<double>(sensor_.sampleRateHz()));
    return true;
}

void ImuManager::setCalibration(const ImuCalibration& calibration) {
    calibration_ = calibration;
}

void ImuManager::startGyroBiasCapture() {
    calibrationState_ = CalibrationState::WaitingForStill;
    calibrationSamples_ = 0;
    levelReferencePending_ = false;
    APEX_LOGI("IMU calibration waiting for %u ms of stillness",
            static_cast<unsigned>(config_.stationaryHoldMs));
}

const char* ImuManager::calibrationStateName() const {
    switch (calibrationState_) {
        case CalibrationState::WaitingForStill: return "WAIT";
        case CalibrationState::Capturing: return "CAPTURE";
        case CalibrationState::Idle: return calibration_.gyroBiasValid ? "VALID" : "IDLE";
    }
    return "?";
}

bool ImuManager::consumeLevelReference(ImuReading& out) {
    if (!levelReferencePending_) return false;
    out = levelReference_;
    levelReferencePending_ = false;
    return true;
}

void ImuManager::beginCalibrationBatch() {
    calibrationState_ = CalibrationState::Capturing;
    calibrationSamples_ = 0;
    gyroAccumulator_ = Vec3();
    gyroSquares_ = Vec3();
    accelAccumulator_ = Vec3();
    accelSquares_ = Vec3();
    APEX_LOGI("IMU calibration capture started (%u samples)",
            static_cast<unsigned>(config_.biasSampleCount));
}

void ImuManager::rejectCalibration(const char* reason) {
    ++calibrationRejections_;
    calibrationState_ = CalibrationState::WaitingForStill;
    calibrationSamples_ = 0;
    stationary_ = false;
    movingSinceValid_ = false;
    APEX_LOGW("IMU calibration rejected (%s, attempt %u); waiting for stillness",
            reason, static_cast<unsigned>(calibrationRejections_));
}

void ImuManager::updateCalibration(const ImuReading& mappedRaw) {
    if (calibrationState_ == CalibrationState::WaitingForStill) {
        if (stationary_) beginCalibrationBatch();
        return;
    }
    if (calibrationState_ != CalibrationState::Capturing) return;

    const float gyroDps = mappedRaw.gyro.norm() * kRadToDeg;
    const float accelDeviation = fabsf(mappedRaw.accel.norm() - kGravityMps2);
    if (gyroDps > config_.calibrationMaxGyroDps ||
        accelDeviation > config_.calibrationMaxAccelMps2) {
        rejectCalibration("movement");
        return;
    }

    gyroAccumulator_ = gyroAccumulator_ + mappedRaw.gyro;
    gyroSquares_ = gyroSquares_ + Vec3(mappedRaw.gyro.x * mappedRaw.gyro.x,
                                        mappedRaw.gyro.y * mappedRaw.gyro.y,
                                        mappedRaw.gyro.z * mappedRaw.gyro.z);
    accelAccumulator_ = accelAccumulator_ + mappedRaw.accel;
    accelSquares_ = accelSquares_ + Vec3(mappedRaw.accel.x * mappedRaw.accel.x,
                                          mappedRaw.accel.y * mappedRaw.accel.y,
                                          mappedRaw.accel.z * mappedRaw.accel.z);
    ++calibrationSamples_;
    if (calibrationSamples_ < config_.biasSampleCount) return;

    const float inv = 1.0f / static_cast<float>(calibrationSamples_);
    const Vec3 meanGyro = gyroAccumulator_ * inv;
    const Vec3 meanAccel = accelAccumulator_ * inv;
    const Vec3 gyroVariance = gyroSquares_ * inv -
        Vec3(meanGyro.x * meanGyro.x, meanGyro.y * meanGyro.y, meanGyro.z * meanGyro.z);
    const Vec3 accelVariance = accelSquares_ * inv -
        Vec3(meanAccel.x * meanAccel.x, meanAccel.y * meanAccel.y, meanAccel.z * meanAccel.z);
    const float gyroStdDps = sqrtf(fmaxf(0.0f, gyroVariance.x) +
                                   fmaxf(0.0f, gyroVariance.y) +
                                   fmaxf(0.0f, gyroVariance.z)) * kRadToDeg;
    const float accelStd = sqrtf(fmaxf(0.0f, accelVariance.x) +
                                 fmaxf(0.0f, accelVariance.y) +
                                 fmaxf(0.0f, accelVariance.z));
    const float leanDeg = atan2f(meanAccel.y, meanAccel.z) * kRadToDeg;
    const float pitchDeg = -atan2f(-meanAccel.x,
        sqrtf(meanAccel.y * meanAccel.y + meanAccel.z * meanAccel.z)) * kRadToDeg;

    if (gyroStdDps > config_.calibrationMaxGyroStdDps ||
        accelStd > config_.calibrationMaxAccelStdMps2) {
        rejectCalibration("unstable variance");
        return;
    }
    if (fabsf(leanDeg) > config_.calibrationMaxLevelAngleDeg ||
        fabsf(pitchDeg) > config_.calibrationMaxLevelAngleDeg) {
        rejectCalibration("surface not level");
        return;
    }

    calibration_.gyroBias = meanGyro;
    calibration_.gyroBiasValid = true;
    ++calibration_.version;
    calibrationState_ = CalibrationState::Idle;

    levelReference_ = mappedRaw;
    levelReference_.accel = meanAccel;
    levelReference_.gyro = Vec3();
    levelReferencePending_ = true;

    APEX_LOGI("IMU calibration accepted: bias %.3f %.3f %.3f dps, std %.3f dps / %.3f m/s2",
            static_cast<double>(meanGyro.x * kRadToDeg),
            static_cast<double>(meanGyro.y * kRadToDeg),
            static_cast<double>(meanGyro.z * kRadToDeg),
            static_cast<double>(gyroStdDps), static_cast<double>(accelStd));
    APEX_LOGI("IMU level reference: lean %.2f deg, pitch %.2f deg (cal v%u)",
            static_cast<double>(leanDeg), static_cast<double>(pitchDeg),
            static_cast<unsigned>(calibration_.version));
}

bool ImuManager::poll(ImuReading& out) {
    RawImuSample raw;
    if (!sensor_.read(raw)) {
        return false;
    }

    ImuReading reading;
    reading.timestampUs = raw.timestampUs;
    reading.accel       = config_.axisMap.apply(raw.accel);
    reading.gyro        = config_.axisMap.apply(raw.gyro);
    reading.mag         = config_.axisMap.apply(raw.mag);
    reading.magValid    = raw.magValid;
    lastRawGyro_ = reading.gyro;

    if (calibration_.gyroBiasValid) {
        reading.gyro = reading.gyro - calibration_.gyroBias;
    }

    updateStationary(reading);
    ImuReading mappedRaw = reading;
    mappedRaw.gyro = lastRawGyro_;
    updateCalibration(mappedRaw);

    lastReading_ = reading;
    sampleCount_++;
    if (previousSampleUs_ != 0 && reading.timestampUs > previousSampleUs_ &&
        sensor_.sampleRateHz() > 0.0f) {
        const uint64_t nominalUs = static_cast<uint64_t>(1e6f / sensor_.sampleRateHz());
        const uint64_t gapUs = reading.timestampUs - previousSampleUs_;
        if (gapUs > nominalUs + nominalUs / 2u) {
            const uint32_t intervals = static_cast<uint32_t>((gapUs + nominalUs / 2u) / nominalUs);
            if (intervals > 1) droppedSamples_ += intervals - 1;
        }
    }
    previousSampleUs_ = reading.timestampUs;
    out = reading;
    return true;
}

void ImuManager::updateStationary(const ImuReading& reading) {
    const float gyroDps      = reading.gyro.norm() * kRadToDeg;
    const float accelDeviate = fabsf(reading.accel.norm() - kGravityMps2);

    const bool stillNow =
        gyroDps < config_.stationaryGyroDps && accelDeviate < config_.stationaryAccelMps2;

    const uint32_t nowMs = static_cast<uint32_t>(reading.timestampUs / 1000u);

    if (!stillNow) {
        stationary_       = false;
        movingSinceValid_ = false;
        return;
    }

    if (!movingSinceValid_) {
        stationarySinceMs_ = nowMs;
        movingSinceValid_  = true;
    }

    if (nowMs - stationarySinceMs_ >= config_.stationaryHoldMs) {
        stationary_ = true;
    }
}

}  // namespace apex
