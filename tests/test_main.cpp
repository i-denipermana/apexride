//
// Host-side verification of the ApexRide recording pipeline.
//
// Runs the full firmware stack — mock sensors, fusion, ride detection,
// encoding, buffering, flash persistence, summaries, recovery and retention —
// against a simulated motorcycle ride, with no hardware involved.
//
// Build and run:  make -C tests
//

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "../ApexRide/src/core/Clock.h"
#include "../ApexRide/src/core/Crc32.h"
#include "../ApexRide/src/core/Log.h"
#include "../ApexRide/src/format/TelemetryFormat.h"
#include "../ApexRide/src/ride/TelemetrySystem.h"
#include "../ApexRide/src/sensors/MockGnssSensor.h"
#include "../ApexRide/src/sensors/MockImuSensor.h"
#include "../ApexRide/src/sim/RideSimulator.h"
#include "HostRideStore.h"

using namespace apex;

// ---------------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------------

namespace {

int g_checks   = 0;
int g_failures = 0;

void check(bool condition, const char* what) {
    ++g_checks;
    if (condition) {
        printf("  \033[32mPASS\033[0m  %s\n", what);
    } else {
        ++g_failures;
        printf("  \033[31mFAIL\033[0m  %s\n", what);
    }
}

void checkNear(double actual, double expected, double tolerance, const char* what) {
    ++g_checks;
    const double delta = fabs(actual - expected);
    if (delta <= tolerance) {
        printf("  \033[32mPASS\033[0m  %s (%.3f, expected %.3f +/- %.3f)\n", what, actual, expected,
               tolerance);
    } else {
        ++g_failures;
        printf("  \033[31mFAIL\033[0m  %s (%.3f, expected %.3f +/- %.3f)\n", what, actual, expected,
               tolerance);
    }
}

void section(const char* title) {
    printf("\n\033[1m== %s\033[0m\n", title);
}

void logToStdout(LogLevel level, const char* line) {
    static const char* kPrefix[] = {"E", "W", "I", "D"};
    printf("    [%s] %s\n", kPrefix[static_cast<int>(level)], line);
}

// ---------------------------------------------------------------------------
// Ride file reader, standing in for what the phone app will do
// ---------------------------------------------------------------------------

struct ParsedRide {
    bool        headerValid = false;
    FileHeader  header{};
    uint32_t    imuCount   = 0;
    uint32_t    gnssCount  = 0;
    uint32_t    eventCount = 0;
    uint32_t    unknownCount = 0;
    bool        truncated  = false;
    uint32_t    fileCrc    = 0;
    size_t      fileSize   = 0;

    std::vector<ImuRecord>   imu;
    std::vector<GnssRecord>  gnss;
    std::vector<EventRecord> events;
};

bool readWholeFile(const std::string& path, std::vector<uint8_t>& out) {
    FILE* handle = fopen(path.c_str(), "rb");
    if (handle == nullptr) return false;

    fseek(handle, 0, SEEK_END);
    const long size = ftell(handle);
    fseek(handle, 0, SEEK_SET);

    out.resize(static_cast<size_t>(size < 0 ? 0 : size));
    const size_t got = out.empty() ? 0 : fread(out.data(), 1, out.size(), handle);
    fclose(handle);

    return got == out.size();
}

ParsedRide parseRideFile(const std::string& path) {
    ParsedRide parsed;

    std::vector<uint8_t> bytes;
    if (!readWholeFile(path, bytes) || bytes.size() < sizeof(FileHeader)) {
        return parsed;
    }

    parsed.fileSize = bytes.size();
    parsed.fileCrc  = crc32(bytes.data(), bytes.size());

    memcpy(&parsed.header, bytes.data(), sizeof(FileHeader));

    const uint32_t expectedCrc = crc32(bytes.data(), sizeof(FileHeader) - sizeof(uint32_t));
    parsed.headerValid = parsed.header.magic == kFileMagic &&
                         parsed.header.formatVersion == kFormatVersion &&
                         parsed.header.headerCrc == expectedCrc;

    size_t offset = sizeof(FileHeader);
    while (offset + sizeof(RecordHeader) <= bytes.size()) {
        RecordHeader recordHeader;
        memcpy(&recordHeader, bytes.data() + offset, sizeof(recordHeader));
        offset += sizeof(recordHeader);

        if (offset + recordHeader.length > bytes.size()) {
            parsed.truncated = true;
            break;
        }

        const uint8_t* payload = bytes.data() + offset;

        switch (static_cast<RecordType>(recordHeader.type)) {
            case RecordType::Imu: {
                ImuRecord record;
                memcpy(&record, payload, sizeof(record));
                parsed.imu.push_back(record);
                ++parsed.imuCount;
                break;
            }
            case RecordType::Gnss: {
                GnssRecord record;
                memcpy(&record, payload, sizeof(record));
                parsed.gnss.push_back(record);
                ++parsed.gnssCount;
                break;
            }
            case RecordType::Event: {
                EventRecord record;
                memcpy(&record, payload, sizeof(record));
                parsed.events.push_back(record);
                ++parsed.eventCount;
                break;
            }
            default:
                ++parsed.unknownCount;
                break;
        }

        offset += recordHeader.length;
    }

    if (offset != bytes.size()) {
        parsed.truncated = true;
    }

    return parsed;
}

// ---------------------------------------------------------------------------
// Simulated ride run
// ---------------------------------------------------------------------------

struct RideRunResult {
    bool     started            = false;
    uint32_t rideId             = 0;
    float    maxTrueLeanDeg     = 0.0f;
    float    maxFusedErrorDeg   = 0.0f;
    float    rmsFusedErrorDeg   = 0.0f;
    float    maxCorneringErrorDeg = 0.0f;
    float    maxNaiveErrorDeg   = 0.0f;
    float    naiveErrorAtMaxLeanDeg = 0.0f;
    float    trueDistanceM      = 0.0f;
    uint32_t rideEndedAtMs      = 0;
};

TelemetrySystem::Config makeTestConfig(bool useKinematicCorrection) {
    TelemetrySystem::Config config;

    // The mock breakout is rotated 90 degrees about the vertical axis:
    //   body X = -sensor Y,  body Y = +sensor X,  body Z = +sensor Z
    config.imu.axisMap.sourceIndex[0] = 1;
    config.imu.axisMap.sign[0]        = -1;
    config.imu.axisMap.sourceIndex[1] = 0;
    config.imu.axisMap.sign[1]        = 1;
    config.imu.axisMap.sourceIndex[2] = 2;
    config.imu.axisMap.sign[2]        = 1;

    config.imu.biasSampleCount = 300;

    config.orientation.useKinematicCorrection = useKinematicCorrection;

    // Shortened so the whole ride lifecycle fits inside one simulated script.
    config.ride.startHoldMs      = 1000;
    config.ride.waitingEnterMs   = 4000;
    config.ride.waitingTimeoutMs = 5000;

    config.recorder.blockSizeBytes     = 4096;
    config.recorder.maxFlushIntervalMs = 4000;
    config.recorder.summaryIntervalMs  = 10000;

    config.storage.minFreeBytesToStart    = 64 * 1024;
    config.storage.minFreeBytesToContinue = 16 * 1024;

    return config;
}

RideRunResult runSimulatedRide(HostRideStore& store, bool useKinematicCorrection, bool verbose) {
    RideRunResult result;

    VirtualClock clock;

    RideSimulator          simulator;
    RideSimulator::Config  simConfig;
    size_t                 segmentCount = 0;
    const RideSegment*     script       = defaultRideScript(segmentCount);
    simulator.begin(simConfig, script, segmentCount);

    MockImuSensor  imuSensor(clock, simulator);
    MockGnssSensor gnssSensor(clock, simulator);

    MockGnssSensor::Config gnssConfig;
    // Drop the fix during straight-line acceleration: enough to exercise fix
    // loss, the distance-chain break and the IMU motion fallback, without
    // making the cornering accuracy measurement a test of dead reckoning.
    gnssConfig.dropoutStartMs = 55000;
    gnssConfig.dropoutEndMs   = 61000;
    gnssSensor.setConfig(gnssConfig);

    TelemetrySystem system(imuSensor, gnssSensor, store, clock);

    static uint8_t recorderBuffer[8192];
    if (!system.begin(makeTestConfig(useKinematicCorrection), recorderBuffer,
                      sizeof(recorderBuffer))) {
        return result;
    }

    const uint32_t totalMs = simulator.totalDurationMs() + 15000;

    bool  calibrated       = false;
    float naiveZeroRad     = 0.0f;
    double squaredErrorSum = 0.0;
    uint32_t errorSamples  = 0;
    uint32_t lastRideId    = 0;

    for (uint32_t nowMs = 0; nowMs <= totalMs; ++nowMs) {
        clock.advanceMillis(1);
        system.update();

        // Mounting calibration: bike upright and stationary, before it moves.
        if (!calibrated && nowMs == 5000) {
            calibrated = system.calibrateMounting();
            const ImuReading& reading = system.imu().lastReading();
            naiveZeroRad = atan2f(reading.accel.y, reading.accel.z);
            if (verbose) {
                printf("    calibrated at t=5s, naive zero offset %.2f deg\n",
                       static_cast<double>(naiveZeroRad * kRadToDeg));
            }
        }

        if (!calibrated) {
            continue;
        }

        if (system.recorder().isRecording()) {
            lastRideId = system.recorder().currentRideId();
        } else if (lastRideId != 0 && result.rideEndedAtMs == 0) {
            result.rideEndedAtMs = nowMs;
        }

        // Compare the estimate against simulator ground truth.
        const float trueLeanDeg = simulator.trueLeanRad() * kRadToDeg;
        const float fusedDeg    = system.status().fused.rollDeg();

        const ImuReading& reading = system.imu().lastReading();
        const float naiveDeg =
            (atan2f(reading.accel.y, reading.accel.z) - naiveZeroRad) * kRadToDeg;

        const float fusedError = fabsf(fusedDeg - trueLeanDeg);
        const float naiveError = fabsf(naiveDeg - trueLeanDeg);

        if (fabsf(trueLeanDeg) > result.maxTrueLeanDeg) {
            result.maxTrueLeanDeg          = fabsf(trueLeanDeg);
            result.naiveErrorAtMaxLeanDeg  = naiveError;
        }

        if (fusedError > result.maxFusedErrorDeg) result.maxFusedErrorDeg = fusedError;
        if (naiveError > result.maxNaiveErrorDeg) result.maxNaiveErrorDeg = naiveError;

        // Steady cornering is where accelerometer-only estimation fails worst,
        // so it gets its own metric.
        if (fabsf(trueLeanDeg) > 10.0f && fusedError > result.maxCorneringErrorDeg) {
            result.maxCorneringErrorDeg = fusedError;
        }

        squaredErrorSum += static_cast<double>(fusedError) * fusedError;
        ++errorSamples;
    }

    if (errorSamples > 0) {
        result.rmsFusedErrorDeg = static_cast<float>(sqrt(squaredErrorSum / errorSamples));
    }

    // Close anything still open, as a clean shutdown would.
    system.stopRideManually();

    result.started       = lastRideId != 0;
    result.rideId        = lastRideId;
    result.trueDistanceM = simulator.trueDistanceM();

    return result;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

std::string g_scratchRoot = "./.testdata";

void testRecordingPipeline() {
    section("End-to-end simulated ride");

    HostRideStore store(g_scratchRoot + "/ride", 12u * 1024u * 1024u);
    check(store.reset(), "scratch filesystem prepared");

    const RideRunResult run = runSimulatedRide(store, /*useKinematicCorrection=*/true,
                                               /*verbose=*/true);

    check(run.started, "a ride was detected and recorded automatically");
    check(run.rideId == 1, "first ride was allocated id 1");
    check(run.rideEndedAtMs > 0, "ride was closed automatically after the bike stopped");

    // --- Ride file structure ------------------------------------------------
    section("Ride file structure");

    char path[64];
    snprintf(path, sizeof(path), "%s/rides/R%06u.bin", store.rootPath().c_str(),
             static_cast<unsigned>(run.rideId));

    const ParsedRide parsed = parseRideFile(path);

    check(parsed.headerValid, "file header magic, version and CRC are valid");
    check(parsed.header.rideId == run.rideId, "header carries the correct ride id");
    check(!parsed.truncated, "record stream ends exactly on a record boundary");
    check(parsed.unknownCount == 0, "no unknown record types");
    check(parsed.imuCount > 0, "IMU records were written");
    check(parsed.gnssCount > 0, "GNSS records were written");
    check(parsed.eventCount >= 2, "ride start and end events were written");

    // GNSS must be its own stream, not duplicated into every IMU record.
    check(parsed.gnssCount < parsed.imuCount / 5,
          "GNSS is a separate low-rate stream, not duplicated per IMU sample");

    // --- Summary ------------------------------------------------------------
    section("Ride summary");

    RideStorage  storage;
    RideStorage::Config storageConfig;
    check(storage.begin(store, storageConfig), "storage index reopened");

    RideSummary summary{};
    check(storage.readSummary(run.rideId, summary), "summary file exists and passes CRC");
    check(summary.isClosed(), "summary is marked closed");
    check(!summary.isSynced(), "a freshly recorded ride is not synced");

    checkNear(summary.imuSampleCount, parsed.imuCount, 0,
              "summary IMU count matches the records on disk");
    checkNear(summary.gnssSampleCount, parsed.gnssCount, 0,
              "summary GNSS count matches the records on disk");
    checkNear(summary.eventCount, parsed.eventCount, 0,
              "summary event count matches the records on disk");
    check(summary.dataCrc == parsed.fileCrc, "summary data CRC matches the file contents");
    check(summary.fileSizeBytes == parsed.fileSize, "summary file size matches the file on disk");

    printf("    ride: %u s, %.2f km, max %u km/h, lean %.1f L / %.1f R deg\n",
           static_cast<unsigned>(summary.durationMs / 1000),
           summary.distanceCm / 100000.0, static_cast<unsigned>(summary.maxSpeed * 36 / 1000),
           summary.maxLeanLeft / 100.0, summary.maxLeanRight / 100.0);
    printf("    peak accel %.2f g, peak braking %.2f g\n", summary.maxAcceleration / 1000.0,
           summary.maxBraking / 1000.0);

    // The script corners at 30 and 40 degrees right, 35 left.
    checkNear(summary.maxLeanLeft / 100.0, 35.0, 3.0, "max left lean recovered from the ride");
    checkNear(summary.maxLeanRight / 100.0, 40.0, 3.0, "max right lean recovered from the ride");
    checkNear(summary.distanceCm / 100.0, run.trueDistanceM, run.trueDistanceM * 0.06,
              "trip distance within 6 percent of ground truth");
    checkNear(summary.maxSpeed / 100.0, 90.0 / 3.6, 1.5, "max speed matches the ride script");

    // --- Fusion accuracy ----------------------------------------------------
    section("Lean angle accuracy");

    printf("    fusion  : max error %.2f deg, RMS %.2f deg, cornering max %.2f deg\n",
           static_cast<double>(run.maxFusedErrorDeg), static_cast<double>(run.rmsFusedErrorDeg),
           static_cast<double>(run.maxCorneringErrorDeg));
    printf("    naive   : max error %.2f deg (error at peak lean of %.1f deg: %.2f deg)\n",
           static_cast<double>(run.maxNaiveErrorDeg), static_cast<double>(run.maxTrueLeanDeg),
           static_cast<double>(run.naiveErrorAtMaxLeanDeg));

    check(run.maxCorneringErrorDeg < 2.0f, "fused lean stays within 2 deg through corners");
    check(run.rmsFusedErrorDeg < 1.0f, "fused lean RMS error under 1 deg");

    // The whole reason this project does sensor fusion at all.
    check(run.naiveErrorAtMaxLeanDeg > 20.0f,
          "accelerometer-only lean is wrong by more than 20 deg at peak lean");
    check(run.maxNaiveErrorDeg > run.maxFusedErrorDeg * 3.0f,
          "fusion is dramatically better than accelerometer-only");
}

void testKinematicCorrectionMatters() {
    section("Kinematic correction ablation");

    HostRideStore withStore(g_scratchRoot + "/with", 12u * 1024u * 1024u);
    HostRideStore withoutStore(g_scratchRoot + "/without", 12u * 1024u * 1024u);
    withStore.reset();
    withoutStore.reset();

    setLogLevel(LogLevel::Error);
    const RideRunResult with    = runSimulatedRide(withStore, true, false);
    const RideRunResult without = runSimulatedRide(withoutStore, false, false);
    setLogLevel(LogLevel::Info);

    printf("    with GNSS speed correction   : cornering max error %.2f deg\n",
           static_cast<double>(with.maxCorneringErrorDeg));
    printf("    plain Mahony (no correction) : cornering max error %.2f deg\n",
           static_cast<double>(without.maxCorneringErrorDeg));

    check(with.maxCorneringErrorDeg < without.maxCorneringErrorDeg,
          "GNSS-aided correction improves sustained cornering accuracy");

    // Plain Mahony is pulled back toward upright by an accelerometer that reads
    // level mid-corner. This is the physics, not a tuning problem — it is why
    // the correction exists.
    check(without.maxCorneringErrorDeg > 10.0f,
          "plain Mahony sags badly through a sustained corner");
    check(with.maxCorneringErrorDeg < without.maxCorneringErrorDeg / 5.0f,
          "the correction removes most of that error");
}

void testRecoveryAfterUncleanShutdown() {
    section("Recovery after an unclean shutdown");

    HostRideStore store(g_scratchRoot + "/recovery", 12u * 1024u * 1024u);
    store.reset();

    setLogLevel(LogLevel::Error);
    const RideRunResult run = runSimulatedRide(store, true, false);
    setLogLevel(LogLevel::Info);
    check(run.started, "reference ride recorded");

    RideStorage         storage;
    RideStorage::Config storageConfig;
    storage.begin(store, storageConfig);

    RideSummary original{};
    check(storage.readSummary(run.rideId, original), "original summary readable");

    // Simulate a battery dying: the summary file never got its final write.
    char summaryPath[64];
    snprintf(summaryPath, sizeof(summaryPath), "/rides/R%06u.met",
             static_cast<unsigned>(run.rideId));
    check(store.remove(summaryPath), "summary file deleted to simulate power loss");

    RideStorage reopened;
    check(reopened.begin(store, storageConfig), "storage reopened with a missing summary");
    check(reopened.unsyncedCount() == 1, "ride with no summary counts as unsynced, not lost");

    const uint32_t repaired = reopened.recoverIncompleteRides();
    check(repaired == 1, "exactly one ride was recovered");

    RideSummary rebuilt{};
    check(reopened.readSummary(run.rideId, rebuilt), "rebuilt summary is readable and valid");

    check((rebuilt.flags & kRideFlagRecovered) != 0, "rebuilt summary is flagged as recovered");
    check((rebuilt.flags & kRideFlagTruncated) == 0, "clean file is not flagged truncated");

    checkNear(rebuilt.imuSampleCount, original.imuSampleCount, 0,
              "recovered IMU count matches the original");
    checkNear(rebuilt.gnssSampleCount, original.gnssSampleCount, 0,
              "recovered GNSS count matches the original");
    checkNear(rebuilt.distanceCm, original.distanceCm, 0,
              "recovered distance matches the original");
    checkNear(rebuilt.maxLeanLeft, original.maxLeanLeft, 0,
              "recovered max left lean matches the original");
    checkNear(rebuilt.maxLeanRight, original.maxLeanRight, 0,
              "recovered max right lean matches the original");
    check(rebuilt.dataCrc == original.dataCrc, "recovered data CRC matches the original");

    // Now cut the tail off the data file, as a power cut mid-block would.
    char dataPath[64];
    snprintf(dataPath, sizeof(dataPath), "/rides/R%06u.bin", static_cast<unsigned>(run.rideId));
    check(store.truncate(dataPath, original.fileSizeBytes - 7),
          "data file truncated mid-record");
    check(store.remove(summaryPath), "summary removed again");

    RideStorage afterTruncation;
    afterTruncation.begin(store, storageConfig);
    check(afterTruncation.recoverIncompleteRides() == 1, "truncated ride still recovers");

    RideSummary truncatedSummary{};
    check(afterTruncation.readSummary(run.rideId, truncatedSummary),
          "summary rebuilt from a truncated file");
    check((truncatedSummary.flags & kRideFlagTruncated) != 0,
          "partial trailing record is flagged as truncated");
    check(truncatedSummary.imuSampleCount > 0, "usable samples survived the truncation");
    check(truncatedSummary.imuSampleCount <= original.imuSampleCount,
          "truncated ride has no more samples than the original");
}

void testStorageExhaustion() {
    section("Running out of storage mid-ride");

    // Far too small to hold the whole ride, so recording must stop partway.
    HostRideStore store(g_scratchRoot + "/full", 100u * 1024u);
    store.reset();

    setLogLevel(LogLevel::Error);
    const RideRunResult run = runSimulatedRide(store, true, false);
    setLogLevel(LogLevel::Info);

    check(run.started, "a ride still started on a nearly full filesystem");

    RideStorage         storage;
    RideStorage::Config storageConfig;
    storage.begin(store, storageConfig);

    RideSummary summary{};
    check(storage.readSummary(run.rideId, summary),
          "the truncated ride still has a valid summary");
    check(summary.isClosed(), "the ride was closed cleanly rather than abandoned");
    check(summary.imuSampleCount > 0, "usable telemetry was captured before space ran out");

    // The critical property: whatever survived must still verify, or the phone
    // would reject the ride and it could never be synced or reclaimed.
    char path[64];
    snprintf(path, sizeof(path), "%s/rides/R%06u.bin", store.rootPath().c_str(),
             static_cast<unsigned>(run.rideId));
    const ParsedRide parsed = parseRideFile(path);

    check(parsed.headerValid, "header of the truncated ride is intact");
    check(summary.dataCrc == parsed.fileCrc,
          "summary CRC matches what actually reached the disk");
    check(summary.fileSizeBytes == parsed.fileSize, "summary size matches the file on disk");
    check(store.usedBytes() <= store.totalBytes(), "the emulated volume was never overfilled");

    printf("    captured %u IMU samples in %u KB before the volume filled\n",
           static_cast<unsigned>(summary.imuSampleCount),
           static_cast<unsigned>(summary.fileSizeBytes / 1024));
}

/// Writes a placeholder ride straight to the store, so retention can be tested
/// without recording several full rides.
void createStubRide(HostRideStore& store, RideStorage& storage, uint32_t rideId,
                    uint32_t dataBytes, bool synced) {
    char path[64];
    snprintf(path, sizeof(path), "/rides/R%06u.bin", static_cast<unsigned>(rideId));

    IRideFile* file = store.open(path, FileMode::Write);
    std::vector<uint8_t> filler(dataBytes, 0xA5);
    file->write(filler.data(), filler.size());
    file->flush();
    file->close();
    delete file;

    RideSummary summary{};
    summary.magic          = kSummaryMagic;
    summary.version        = kFormatVersion;
    summary.rideId         = rideId;
    summary.flags          = kRideFlagClosed | (synced ? kRideFlagSynced : 0);
    summary.fileSizeBytes  = dataBytes;
    summary.imuSampleCount = 100;

    storage.refresh();
    storage.writeSummary(summary);
    storage.refresh();
}

void testRetentionPolicy() {
    section("Storage retention policy");

    HostRideStore store(g_scratchRoot + "/retention", 1024u * 1024u);
    store.reset();

    RideStorage         storage;
    RideStorage::Config storageConfig;
    storageConfig.minFreeBytesToStart = 64 * 1024;
    check(storage.begin(store, storageConfig), "empty storage initialised");

    createStubRide(store, storage, 1, 200 * 1024, /*synced=*/true);
    createStubRide(store, storage, 2, 200 * 1024, /*synced=*/true);
    createStubRide(store, storage, 3, 200 * 1024, /*synced=*/false);
    createStubRide(store, storage, 4, 200 * 1024, /*synced=*/false);

    check(storage.rideCount() == 4, "four rides indexed");
    check(storage.unsyncedCount() == 2, "two rides are unsynced");
    check(storage.nextRideId() == 5, "next ride id follows the highest on disk");

    check(!storage.deleteRide(3), "deleting an unsynced ride is refused");
    check(storage.findRide(3) != nullptr, "the unsynced ride is still there");

    // Ask for more room than is free; only synced rides may be sacrificed.
    check(storage.reclaimSpace(400 * 1024), "space reclaimed from synced rides");
    check(storage.findRide(1) == nullptr, "oldest synced ride was deleted first");
    check(storage.findRide(3) != nullptr, "unsynced rides survived reclamation");
    check(storage.findRide(4) != nullptr, "second unsynced ride survived reclamation");

    // Now demand more than can possibly be freed.
    check(!storage.reclaimSpace(900 * 1024),
          "reclamation fails rather than deleting unsynced rides");
    check(storage.unsyncedCount() == 2, "both unsynced rides are still present");

    // Marking synced makes a ride eligible.
    check(storage.markSynced(3), "ride marked synced after a verified transfer");
    const RideStorage::RideEntry* entry = storage.findRide(3);
    check(entry != nullptr && entry->summary.isSynced(), "synced flag persisted to disk");
    check(storage.deleteRide(3), "a synced ride can now be deleted");
}

void testFormatInvariants() {
    section("Binary format invariants");

    check(sizeof(FileHeader) == 32, "FileHeader is 32 bytes");
    check(sizeof(ImuRecord) == 22, "ImuRecord is 22 bytes");
    check(sizeof(GnssRecord) == 26, "GnssRecord is 26 bytes");
    check(sizeof(RideSummary) == 72, "RideSummary is 72 bytes");

    // Known-answer test, so a phone app can be checked against the same value.
    const char* reference = "123456789";
    checkNear(crc32(reference, 9), 0xCBF43926u, 0, "CRC-32 matches the standard check value");

    checkNear(toCentiDegrees(-38.42f), -3842, 0, "-38.42 deg encodes as -3842");
    checkNear(toDegreesE7(-6.2088123), -62088123, 0, "-6.2088123 deg encodes as -62088123");

    // Saturation rather than wraparound on out-of-range input.
    check(toCentiDegrees(1.0e9f) == 32767, "over-range angle saturates positive");
    check(toCentiDegrees(-1.0e9f) == -32768, "over-range angle saturates negative");
    check(toMilliG(0.0f / 0.0f) == 0, "NaN encodes as zero rather than garbage");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1) {
        g_scratchRoot = argv[1];
    }

    setLogSink(logToStdout);
    setLogLevel(LogLevel::Info);

    printf("\033[1mApexRide host tests\033[0m\n");
    printf("scratch: %s\n", g_scratchRoot.c_str());

    const std::string mk = "mkdir -p '" + g_scratchRoot + "'";
    if (system(mk.c_str()) != 0) {
        printf("could not create scratch directory\n");
        return 2;
    }

    testFormatInvariants();
    testRecordingPipeline();
    testKinematicCorrectionMatters();
    testRecoveryAfterUncleanShutdown();
    testStorageExhaustion();
    testRetentionPolicy();

    printf("\n\033[1m%d checks, %d failures\033[0m\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
