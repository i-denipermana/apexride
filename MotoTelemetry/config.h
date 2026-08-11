#pragma once
//
// Build-time configuration for MotoTelemetry V1.
//
// Only the sketch reads this file. Every module takes its settings through a
// Config struct instead, so tests can run the same code with different values.
//
// Target hardware:
//   ESP32-S3 N16R8 DevKitC-1   (16 MB flash, 8 MB PSRAM)
//   ICM-20948 9DOF breakout    (I2C)
//   ATGM336H GNSS + antenna    (UART)
//   3.7 V 1500 mAh LiPo -> TP4056 -> SPDT switch -> MT3608 (~5.0 V) -> 5V pin
//

// ---------------------------------------------------------------------------
// Sensor selection
// ---------------------------------------------------------------------------
//
// V1 runs on simulated sensors so the pipeline can be developed and tested
// before the hardware arrives. Set these to 0 as each real driver lands.

#define MT_USE_MOCK_IMU  1
#define MT_USE_MOCK_GNSS 1

// ---------------------------------------------------------------------------
// Pin assignment
// ---------------------------------------------------------------------------
//
// Not yet verified against the board — check against the DevKitC-1 pinout
// before wiring anything. GPIO 19/20 are USB D-/D+ and 26-32 are the internal
// SPI flash and PSRAM; none of those are usable.

#define MT_I2C_SDA_PIN 8
#define MT_I2C_SCL_PIN 9
#define MT_I2C_FREQUENCY 400000

#define MT_IMU_INT_PIN 7  ///< ICM-20948 data-ready interrupt (not yet used)

#define MT_GNSS_UART_NUM 1
#define MT_GNSS_RX_PIN   17  ///< ESP32 RX  <- GNSS TX
#define MT_GNSS_TX_PIN   18  ///< ESP32 TX  -> GNSS RX
#define MT_GNSS_PPS_PIN  16  ///< pulse-per-second (reserved for timing work)
#define MT_GNSS_BAUD     9600

#define MT_STATUS_LED_PIN 48  ///< onboard addressable LED on the DevKitC-1

// ---------------------------------------------------------------------------
// Sampling rates
// ---------------------------------------------------------------------------

#define MT_IMU_SAMPLE_RATE_HZ 200.0f  ///< raw IMU reads, all integrated by the filter
#define MT_FUSION_RATE_HZ     100.0f  ///< attitude estimates published
#define MT_IMU_LOG_RATE_HZ    50.0f   ///< samples actually written to flash
#define MT_GNSS_RATE_HZ       5.0f    ///< ATGM336H needs a command to exceed 1 Hz

// ---------------------------------------------------------------------------
// Fusion tuning
// ---------------------------------------------------------------------------

#define MT_MAHONY_KP 0.6f
#define MT_MAHONY_KI 0.02f

/// Subtract omega x v using GNSS speed before using the accelerometer as a
/// gravity reference. Without this, lean angle sags toward upright in a
/// sustained corner. See Orientation.h.
#define MT_USE_KINEMATIC_CORRECTION 1
#define MT_MIN_SPEED_FOR_CORRECTION_MPS 3.0f

// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------

/// Match the LittleFS block size so a flush is one erase-write cycle.
#define MT_RECORD_BLOCK_BYTES 4096

/// Total staging buffer. Two blocks, so encoding continues during a flush.
#define MT_RECORD_BUFFER_BYTES 8192

#define MT_MAX_FLUSH_INTERVAL_MS 4000
#define MT_SUMMARY_INTERVAL_MS   10000

// ---------------------------------------------------------------------------
// Ride detection
// ---------------------------------------------------------------------------

#define MT_RIDE_START_SPEED_MPS 2.5f
#define MT_RIDE_STOP_SPEED_MPS  1.0f
#define MT_RIDE_START_HOLD_MS   1500
#define MT_RIDE_WAITING_ENTER_MS 8000
#define MT_RIDE_WAITING_ENTER_NO_GNSS_MS 45000
#define MT_RIDE_WAITING_TIMEOUT_MS 120000
#define MT_RIDE_SLEEP_TIMEOUT_MS   300000

// ---------------------------------------------------------------------------
// Storage
// ---------------------------------------------------------------------------

#define MT_RIDES_DIRECTORY "/rides"

/// Recording will not start below this much free space.
#define MT_MIN_FREE_BYTES_TO_START (512u * 1024u)

/// An in-progress ride is closed below this, leaving room to finish cleanly.
#define MT_MIN_FREE_BYTES_TO_CONTINUE (96u * 1024u)

/// NVS namespace holding gyro bias and the mounting offset.
#define MT_NVS_NAMESPACE "moto"

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

#define MT_SERIAL_BAUD 115200
#define MT_STATUS_INTERVAL_MS 1000

/// Wait for the USB serial monitor before running setup(). Handy on the bench,
/// but it must be 0 for a battery-powered ride or the device will hang.
#define MT_WAIT_FOR_SERIAL 0
