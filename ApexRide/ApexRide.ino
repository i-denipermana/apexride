//
// ApexRide V1 — offline-first motorcycle telemetry logger.
//
// Milestone 1: the full recording pipeline running on simulated sensors.
// Everything except the two sensor drivers is production code; flipping
// APEX_USE_MOCK_IMU / APEX_USE_MOCK_GNSS in config.h swaps in the real hardware.
//
// Serial commands (115200 baud, one letter then Enter):
//   s  start a ride manually        c  capture the mounting offset
//   x  stop the ride                g  re-measure gyro bias
//   l  list stored rides            k  clear stored calibration
//   i  storage and device info      y  mark every ride synced (sync stand-in)
//   d  dump the newest ride as hex  f  FORMAT the filesystem (destroys rides)
//   p  print the sync API responses  h  this help
//

#include "config.h"
#include "src/core/Clock.h"
#include "src/core/Log.h"
#include "src/ride/TelemetrySystem.h"
#include "src/storage/CalibrationStore.h"
#include "src/storage/LittleFsRideStore.h"
#include "src/sync/SyncProtocol.h"
#include "src/sync/SyncService.h"
#include "src/sync/TelemetryStatusSource.h"

#if APEX_USE_MOCK_IMU || APEX_USE_MOCK_GNSS
#include "src/sim/RideSimulator.h"
#endif

#if APEX_USE_MOCK_IMU
#include "src/sensors/MockImuSensor.h"
#else
#error "Real ICM-20948 driver not implemented yet — keep APEX_USE_MOCK_IMU set to 1"
#endif

#if APEX_USE_MOCK_GNSS
#include "src/sensors/MockGnssSensor.h"
#else
#error "Real ATGM336H driver not implemented yet — keep APEX_USE_MOCK_GNSS set to 1"
#endif

using namespace apex;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

namespace {

SystemClock       g_clock;
LittleFsRideStore g_store;
CalibrationStore  g_calibration(APEX_NVS_NAMESPACE);

RideSimulator  g_simulator;
MockImuSensor  g_imuSensor(g_clock, g_simulator);
MockGnssSensor g_gnssSensor(g_clock, g_simulator);

TelemetrySystem g_system(g_imuSensor, g_gnssSensor, g_store, g_clock);

TelemetryStatusSource g_statusSource(g_system);
SyncService           g_sync(g_system.storage(), g_clock);
SyncProtocol          g_syncProtocol(g_sync);

/// Buffer for sync JSON responses. Large enough for a full ride manifest, so
/// it comes from PSRAM alongside the telemetry buffer.
char*  g_syncBuffer     = nullptr;
size_t g_syncBufferSize = 0;

/// Telemetry staging buffer. Placed in PSRAM at boot; the static array is the
/// fallback for a board without working PSRAM.
uint8_t  g_fallbackBuffer[APEX_RECORD_BUFFER_BYTES];
uint8_t* g_recordBuffer = g_fallbackBuffer;

uint32_t g_lastStatusMs   = 0;
uint16_t g_savedCalVersion = 0;

// ---------------------------------------------------------------------------
// Logging bridge
// ---------------------------------------------------------------------------

void serialLogSink(LogLevel level, const char* line) {
    static const char* const kPrefix[] = {"ERR ", "WARN", "INFO", "DBG "};
    Serial.print('[');
    Serial.print(kPrefix[static_cast<int>(level)]);
    Serial.print("] ");
    Serial.println(line);
}

// ---------------------------------------------------------------------------
// Configuration assembly
// ---------------------------------------------------------------------------

TelemetrySystem::Config buildConfig() {
    TelemetrySystem::Config config;

#if APEX_USE_MOCK_IMU
    // The mock emulates a breakout rotated 90 degrees about the vertical axis,
    // so the demo exercises the axis map rather than bypassing it:
    //   body X = -sensor Y,  body Y = +sensor X,  body Z = +sensor Z
    config.imu.axisMap.sourceIndex[0] = 1;
    config.imu.axisMap.sign[0]        = -1;
    config.imu.axisMap.sourceIndex[1] = 0;
    config.imu.axisMap.sign[1]        = 1;
    config.imu.axisMap.sourceIndex[2] = 2;
    config.imu.axisMap.sign[2]        = 1;
#else
    // TODO(hardware): set this from how the ICM-20948 actually sits on the bike.
    // Check it by resting the device level: accel must read (0, 0, +9.81).
    config.imu.axisMap = AxisMap();
#endif

    config.orientation.kp                       = APEX_MAHONY_KP;
    config.orientation.ki                       = APEX_MAHONY_KI;
    config.orientation.useKinematicCorrection   = APEX_USE_KINEMATIC_CORRECTION;
    config.orientation.minSpeedForCorrectionMps = APEX_MIN_SPEED_FOR_CORRECTION_MPS;

    config.ride.startSpeedMps         = APEX_RIDE_START_SPEED_MPS;
    config.ride.stopSpeedMps          = APEX_RIDE_STOP_SPEED_MPS;
    config.ride.startHoldMs           = APEX_RIDE_START_HOLD_MS;
    config.ride.waitingEnterMs        = APEX_RIDE_WAITING_ENTER_MS;
    config.ride.waitingEnterNoGnssMs  = APEX_RIDE_WAITING_ENTER_NO_GNSS_MS;
    config.ride.waitingTimeoutMs      = APEX_RIDE_WAITING_TIMEOUT_MS;
    config.ride.sleepTimeoutMs        = APEX_RIDE_SLEEP_TIMEOUT_MS;

    config.recorder.imuLogRateHz       = APEX_IMU_LOG_RATE_HZ;
    config.recorder.blockSizeBytes     = APEX_RECORD_BLOCK_BYTES;
    config.recorder.maxFlushIntervalMs = APEX_MAX_FLUSH_INTERVAL_MS;
    config.recorder.summaryIntervalMs  = APEX_SUMMARY_INTERVAL_MS;

    config.storage.basePath               = APEX_RIDES_DIRECTORY;
    config.storage.minFreeBytesToStart    = APEX_MIN_FREE_BYTES_TO_START;
    config.storage.minFreeBytesToContinue = APEX_MIN_FREE_BYTES_TO_CONTINUE;

    config.fusionRateHz = APEX_FUSION_RATE_HZ;

    return config;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void printStorageBudget() {
    const uint64_t total = g_system.storage().totalBytes();
    const float    bytesPerSecond =
        APEX_IMU_LOG_RATE_HZ * (sizeof(RecordHeader) + sizeof(ImuRecord)) +
        APEX_GNSS_RATE_HZ * (sizeof(RecordHeader) + sizeof(GnssRecord));

    Serial.printf("Storage budget: %.0f B/s -> %.1f MB/hour; %llu KB holds ~%.1f hours\n",
                  bytesPerSecond, bytesPerSecond * 3600.0f / (1024.0f * 1024.0f),
                  static_cast<unsigned long long>(total / 1024),
                  static_cast<float>(total) / bytesPerSecond / 3600.0f);
}

void printStatus() {
    const TelemetrySystem::Status status = g_system.status();

    Serial.printf(
        "%-9s ride %-4u | lean %+6.1f pitch %+6.1f | %5.1f km/h | GNSS %s %2u sat | "
        "%6u smp %5u KB | free %5llu KB | unsynced %u\n",
        toString(status.state), static_cast<unsigned>(status.activeRideId),
        static_cast<double>(status.fused.rollDeg()),
        static_cast<double>(status.fused.pitchDeg()),
        static_cast<double>(status.speedMps * 3.6f), status.gnssFix ? "fix" : "---",
        static_cast<unsigned>(status.satellites), static_cast<unsigned>(status.imuSamples),
        static_cast<unsigned>(status.rideBytes / 1024),
        static_cast<unsigned long long>(status.freeBytes / 1024),
        static_cast<unsigned>(status.unsyncedRides));
}

void listRides() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    Serial.printf("\n%u ride(s), %llu KB free of %llu KB\n",
                  static_cast<unsigned>(storage.rideCount()),
                  static_cast<unsigned long long>(storage.freeBytes() / 1024),
                  static_cast<unsigned long long>(storage.totalBytes() / 1024));
    Serial.println("  id      dur    dist     max     lean L/R      size  state");

    for (size_t i = 0; i < storage.rideCount(); ++i) {
        const RideStorage::RideEntry& entry = storage.rideAt(i);

        if (!entry.summaryValid) {
            Serial.printf("  R%06u  (summary missing or corrupt, %u KB of data)\n",
                          static_cast<unsigned>(entry.rideId), entry.dataBytes / 1024);
            continue;
        }

        const RideSummary& s = entry.summary;
        Serial.printf("  R%06u  %4us  %6.2fkm  %3ukm/h  %5.1f/%5.1f  %5uKB  %s%s%s\n",
                      static_cast<unsigned>(s.rideId), static_cast<unsigned>(s.durationMs / 1000),
                      s.distanceCm / 100000.0, static_cast<unsigned>(s.maxSpeed * 36 / 1000),
                      s.maxLeanLeft / 100.0, s.maxLeanRight / 100.0, s.fileSizeBytes / 1024,
                      s.isSynced() ? "synced" : "UNSYNCED",
                      (s.flags & kRideFlagRecovered) ? " recovered" : "",
                      (s.flags & kRideFlagTruncated) ? " truncated" : "");
    }
    Serial.println();
}

void dumpNewestRide() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    if (storage.rideCount() == 0) {
        Serial.println("No rides stored.");
        return;
    }

    const uint32_t rideId = storage.rideAt(storage.rideCount() - 1).rideId;

    IRideFile* file = storage.openRideFile(rideId, FileMode::Read);
    if (file == nullptr) {
        Serial.println("Could not open the ride file.");
        return;
    }

    Serial.printf("\nR%06u first 256 bytes:\n", static_cast<unsigned>(rideId));

    uint8_t      chunk[16];
    size_t       offset = 0;
    while (offset < 256) {
        const size_t got = file->read(chunk, sizeof(chunk));
        if (got == 0) break;

        Serial.printf("%04x  ", static_cast<unsigned>(offset));
        for (size_t i = 0; i < got; ++i) {
            Serial.printf("%02x ", chunk[i]);
        }
        Serial.println();
        offset += got;
    }

    file->close();
    delete file;
    Serial.println();
}

/// Stand-in for the BLE/Wi-Fi sync flow that arrives in a later milestone.
/// Marking rides synced from the console is the only way to exercise the
/// retention policy until then.
void markAllSynced() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    uint32_t marked = 0;
    while (true) {
        uint32_t target = 0;
        for (size_t i = 0; i < storage.rideCount(); ++i) {
            const RideStorage::RideEntry& entry = storage.rideAt(i);
            if (entry.summaryValid && !entry.summary.isSynced()) {
                target = entry.rideId;
                break;
            }
        }
        if (target == 0) break;
        if (!storage.markSynced(target)) break;
        ++marked;
    }

    Serial.printf("Marked %u ride(s) synced.\n", static_cast<unsigned>(marked));
}

void printHelp() {
    Serial.println(
        "\nCommands: s=start x=stop c=calibrate-mounting g=gyro-bias l=list i=info\n"
        "          d=dump-newest y=mark-all-synced p=sync-api k=clear-calibration\n"
        "          f=format h=help\n");
}

/// Runs the sync API locally and prints what it returns.
///
/// There is no radio yet, so this is how the protocol gets exercised on real
/// hardware during bring-up: the same route() the Wi-Fi handler will call, with
/// the serial monitor standing in for the phone.
void printSyncApi() {
    if (g_syncBuffer == nullptr) {
        Serial.println("Sync buffer unavailable.");
        return;
    }

    struct Probe {
        const char* method;
        const char* path;
    };
    const Probe probes[] = {
        {"GET", "/status"},
        {"POST", "/sync/begin"},
        {"GET", "/sync/pending"},
        {"GET", "/rides"},
        {"POST", "/sync/end"},
    };

    Serial.println();
    for (const Probe& probe : probes) {
        const SyncResponse response =
            g_syncProtocol.route(probe.method, probe.path, nullptr, g_syncBuffer,
                                 g_syncBufferSize);

        Serial.printf("%-4s %-14s -> %u  ", probe.method, probe.path, response.status);
        if (response.isRideData) {
            Serial.printf("<%u bytes of ride data>\n", response.dataLength);
        } else {
            Serial.println(response.body);
        }
    }
    Serial.println();
}

void printInfo() {
    Serial.printf("\nApexRide firmware 0x%04x, format v%u\n", kFirmwareVersion,
                  kFormatVersion);
    Serial.printf("Sensors: %s + %s (mock)\n", "IMU", "GNSS");
    Serial.printf("Heap: %u free, PSRAM: %u free\n", static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getFreePsram()));
    Serial.printf("Record sizes: IMU %u B, GNSS %u B, summary %u B\n",
                  static_cast<unsigned>(sizeof(RecordHeader) + sizeof(ImuRecord)),
                  static_cast<unsigned>(sizeof(RecordHeader) + sizeof(GnssRecord)),
                  static_cast<unsigned>(sizeof(RideSummary)));
    printStorageBudget();
    Serial.println();
}

/// Persists whatever calibration has been captured, if it changed.
void saveCalibrationIfChanged() {
    const ImuCalibration& calibration = g_system.imu().calibration();
    if (calibration.gyroBiasValid && calibration.version != g_savedCalVersion) {
        g_calibration.saveImuCalibration(calibration);
        g_savedCalVersion = calibration.version;
    }
}

void handleSerialCommand(char command) {
    switch (command) {
        case 's':
            Serial.println(g_system.startRideManually() ? "Ride started." : "Could not start.");
            break;
        case 'x':
            Serial.println(g_system.stopRideManually() ? "Ride stopped." : "No ride was running.");
            break;
        case 'c':
            if (g_system.calibrateMounting()) {
                g_calibration.saveMountingOffset(g_system.orientation().mountingOffset());
                Serial.println("Mounting offset captured and saved.");
            } else {
                Serial.println("Hold the bike upright and still, then try again.");
            }
            break;
        case 'g':
            g_system.calibrateGyroBias();
            Serial.println("Measuring gyro bias — keep the bike still.");
            break;
        case 'l': listRides(); break;
        case 'i': printInfo(); break;
        case 'd': dumpNewestRide(); break;
        case 'y': markAllSynced(); break;
        case 'p': printSyncApi(); break;
        case 'k':
            g_calibration.clear();
            g_system.orientation().clearMountingOffset();
            Serial.println("Calibration cleared. Reboot to start fresh.");
            break;
        case 'f':
            Serial.println("Formatting — every stored ride will be lost.");
            g_system.stopRideManually();
            g_store.format();
            Serial.println("Done. Reboot the device.");
            break;
        case 'h':
        case '?': printHelp(); break;
        default: break;
    }
}

void pollSerialCommands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command != '\n' && command != '\r' && command != ' ') {
            handleSerialCommand(command);
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(APEX_SERIAL_BAUD);
#if APEX_WAIT_FOR_SERIAL
    while (!Serial) {
        delay(10);
    }
#else
    delay(500);  // give the USB CDC port a moment to enumerate
#endif

    setLogSink(serialLogSink);
    setLogLevel(LogLevel::Info);

    Serial.println("\n\n=== ApexRide V1 ===");

    // Prefer PSRAM for the telemetry buffer, keeping internal SRAM for stacks.
    if (psramFound()) {
        uint8_t* psramBuffer = static_cast<uint8_t*>(ps_malloc(APEX_RECORD_BUFFER_BYTES));
        if (psramBuffer != nullptr) {
            g_recordBuffer = psramBuffer;
            APEX_LOGI("Telemetry buffer allocated in PSRAM (%u bytes)", APEX_RECORD_BUFFER_BYTES);
        }
    } else {
        APEX_LOGW("No PSRAM detected — using internal SRAM for the telemetry buffer");
    }

    // First boot has no filesystem yet; allow exactly one automatic format.
    if (!g_store.begin()) {
        APEX_LOGW("Mount failed — formatting for first use");
        if (!g_store.format() || !g_store.begin()) {
            APEX_LOGE("Filesystem unavailable; recording is disabled");
        }
    }

    g_calibration.begin();

#if APEX_USE_MOCK_IMU || APEX_USE_MOCK_GNSS
    size_t             segmentCount = 0;
    const RideSegment* script       = defaultRideScript(segmentCount);
    g_simulator.begin(RideSimulator::Config(), script, segmentCount);
    APEX_LOGW("Running on SIMULATED sensors — a scripted %u second ride starts at boot",
            static_cast<unsigned>(g_simulator.totalDurationMs() / 1000));
#endif

    MockImuSensor::Config imuConfig;
    imuConfig.sampleRateHz = APEX_IMU_SAMPLE_RATE_HZ;
    g_imuSensor.setConfig(imuConfig);

    MockGnssSensor::Config gnssConfig;
    gnssConfig.updateRateHz = APEX_GNSS_RATE_HZ;
    g_gnssSensor.setConfig(gnssConfig);

    if (!g_system.begin(buildConfig(), g_recordBuffer, APEX_RECORD_BUFFER_BYTES)) {
        APEX_LOGE("Telemetry system failed to start");
        return;
    }

    // Restore calibration so the device is usable immediately after a reboot.
    ImuCalibration storedCalibration;
    if (g_calibration.loadImuCalibration(storedCalibration)) {
        g_system.imu().setCalibration(storedCalibration);
        g_savedCalVersion = storedCalibration.version;
    }

    Quaternion storedMounting;
    if (g_calibration.loadMountingOffset(storedMounting)) {
        g_system.orientation().setMountingOffset(storedMounting);
    } else {
        APEX_LOGW("No mounting calibration stored — park upright and level, then press 'c'");
    }

    // Sync layer. It has no transport yet — BLE and the Wi-Fi AP are the next
    // milestone — but wiring it now means the handler is a single call away,
    // and 'p' exercises the whole protocol over serial in the meantime.
    g_syncBufferSize = SyncProtocol::manifestBufferSize(RideStorage::kMaxRides);
    g_syncBuffer     = static_cast<char*>(psramFound() ? ps_malloc(g_syncBufferSize)
                                                       : malloc(g_syncBufferSize));
    if (g_syncBuffer == nullptr) {
        APEX_LOGE("Could not allocate the %u byte sync buffer",
                  static_cast<unsigned>(g_syncBufferSize));
        g_syncBufferSize = 0;
    }

    SyncService::Config syncConfig;
    syncConfig.deviceName = APEX_DEVICE_NAME;
    g_sync.begin(syncConfig);
    g_sync.setStatusSource(&g_statusSource);

    printInfo();
    printHelp();
    listRides();
}

void loop() {
    g_system.update();
    g_sync.update();
    pollSerialCommands();
    saveCalibrationIfChanged();

    const uint32_t nowMs = millis();
    if (nowMs - g_lastStatusMs >= APEX_STATUS_INTERVAL_MS) {
        g_lastStatusMs = nowMs;
        printStatus();
    }
}
