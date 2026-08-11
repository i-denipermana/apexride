#include "ImuManager.h"

#include <math.h>

#include "../core/Log.h"

namespace moto {

ImuManager::ImuManager(IImuSensor& sensor) : sensor_(sensor) {}

bool ImuManager::begin(const Config& config) {
    config_ = config;

    if (!sensor_.begin()) {
        MT_LOGE("IMU %s failed to start", sensor_.name());
        return false;
    }

    MT_LOGI("IMU %s ready at %.0f Hz", sensor_.name(), static_cast<double>(sensor_.sampleRateHz()));
    return true;
}

void ImuManager::setCalibration(const ImuCalibration& calibration) {
    calibration_ = calibration;
}

void ImuManager::startGyroBiasCapture() {
    biasCaptureRemaining_ = config_.biasSampleCount;
    biasAccumulator_      = Vec3();
    MT_LOGI("IMU gyro bias capture started (%u samples) — keep the bike still",
            static_cast<unsigned>(biasCaptureRemaining_));
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

    if (biasCaptureRemaining_ > 0) {
        // Bias is measured on axis-mapped but otherwise uncorrected data.
        biasAccumulator_ = biasAccumulator_ + reading.gyro;
        if (--biasCaptureRemaining_ == 0) {
            const float inv = 1.0f / static_cast<float>(config_.biasSampleCount);
            calibration_.gyroBias      = biasAccumulator_ * inv;
            calibration_.gyroBiasValid = true;
            calibration_.version++;
            MT_LOGI("IMU gyro bias: %.3f %.3f %.3f dps (cal v%u)",
                    static_cast<double>(calibration_.gyroBias.x * kRadToDeg),
                    static_cast<double>(calibration_.gyroBias.y * kRadToDeg),
                    static_cast<double>(calibration_.gyroBias.z * kRadToDeg),
                    static_cast<unsigned>(calibration_.version));
        }
    }

    if (calibration_.gyroBiasValid) {
        reading.gyro = reading.gyro - calibration_.gyroBias;
    }

    updateStationary(reading);

    lastReading_ = reading;
    sampleCount_++;
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

}  // namespace moto
