//
// ridedump — read an ApexRide ride file and show what is in it.
//
//   ridedump R000001.bin              report, validation and terminal charts
//   ridedump R000001.bin --csv-imu    IMU records as CSV
//   ridedump R000001.bin --csv-gnss   GNSS records as CSV
//   ridedump R000001.bin --gpx        GPX track for any mapping tool
//   ridedump R000001.bin --events     event log only
//
// This is also the reference decoder for the format: the phone app has to do
// exactly what parseRide() below does. Keeping it as a working program means
// the format documentation cannot quietly drift away from the truth.
//

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <string>
#include <vector>

#include "../ApexRide/src/core/Crc32.h"
#include "../ApexRide/src/core/MathUtils.h"
#include "../ApexRide/src/format/SummaryBuilder.h"
#include "../ApexRide/src/format/TelemetryFormat.h"

using namespace apex;

namespace {

struct Ride {
    FileHeader               header{};
    std::vector<ImuRecord>   imu;
    std::vector<GnssRecord>  gnss;
    std::vector<EventRecord> events;

    uint32_t unknownRecords = 0;
    bool     headerValid    = false;
    bool     truncated      = false;
    uint32_t crc            = 0;
    size_t   fileSize       = 0;
};

const char* eventName(uint8_t code) {
    switch (static_cast<EventCode>(code)) {
        case EventCode::RideStart:          return "RideStart";
        case EventCode::RideEnd:            return "RideEnd";
        case EventCode::GnssFixAcquired:    return "GnssFixAcquired";
        case EventCode::GnssFixLost:        return "GnssFixLost";
        case EventCode::CalibrationApplied: return "CalibrationApplied";
        case EventCode::StorageLow:         return "StorageLow";
        case EventCode::RecordingPaused:    return "RecordingPaused";
        case EventCode::RecordingResumed:   return "RecordingResumed";
        case EventCode::BufferOverrun:      return "BufferOverrun";
        case EventCode::StorageFull:        return "StorageFull";
    }
    return "Unknown";
}

/// Seconds of a record relative to the ride start.
///
/// Signed on purpose: events are written before the first IMU sample, so an
/// unsigned subtraction underflows into 4294967 seconds.
double relativeSeconds(uint32_t timestampMs, uint32_t originMs) {
    return static_cast<int32_t>(timestampMs - originMs) / 1000.0;
}

std::string formatUnixTime(uint32_t unixTime) {
    if (unixTime == 0) return "(no GNSS time)";

    const time_t seconds = static_cast<time_t>(unixTime);
    struct tm    parts {};
    gmtime_r(&seconds, &parts);

    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S UTC", &parts);
    return buffer;
}

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

bool parseRide(const std::string& path, Ride& ride) {
    std::vector<uint8_t> bytes;
    if (!readWholeFile(path, bytes)) {
        fprintf(stderr, "ridedump: cannot read %s\n", path.c_str());
        return false;
    }
    if (bytes.size() < sizeof(FileHeader)) {
        fprintf(stderr, "ridedump: %s is too short to be a ride file\n", path.c_str());
        return false;
    }

    ride.fileSize = bytes.size();
    ride.crc      = crc32(bytes.data(), bytes.size());
    memcpy(&ride.header, bytes.data(), sizeof(FileHeader));

    const uint32_t expected = crc32(bytes.data(), sizeof(FileHeader) - sizeof(uint32_t));
    ride.headerValid = ride.header.magic == kFileMagic &&
                       ride.header.formatVersion == kFormatVersion &&
                       ride.header.headerCrc == expected;

    // Length-prefixed records: an unknown type is skipped, not fatal.
    size_t offset = ride.header.headerSize != 0 ? ride.header.headerSize : sizeof(FileHeader);
    while (offset + sizeof(RecordHeader) <= bytes.size()) {
        RecordHeader recordHeader;
        memcpy(&recordHeader, bytes.data() + offset, sizeof(recordHeader));
        offset += sizeof(recordHeader);

        if (offset + recordHeader.length > bytes.size()) {
            ride.truncated = true;
            break;
        }

        const uint8_t* payload = bytes.data() + offset;

        if (static_cast<RecordType>(recordHeader.type) == RecordType::Imu &&
            recordHeader.length == sizeof(ImuRecord)) {
            ImuRecord record;
            memcpy(&record, payload, sizeof(record));
            ride.imu.push_back(record);
        } else if (static_cast<RecordType>(recordHeader.type) == RecordType::Gnss &&
                   recordHeader.length == sizeof(GnssRecord)) {
            GnssRecord record;
            memcpy(&record, payload, sizeof(record));
            ride.gnss.push_back(record);
        } else if (static_cast<RecordType>(recordHeader.type) == RecordType::Event &&
                   recordHeader.length == sizeof(EventRecord)) {
            EventRecord record;
            memcpy(&record, payload, sizeof(record));
            ride.events.push_back(record);
        } else {
            ++ride.unknownRecords;
        }

        offset += recordHeader.length;
    }

    if (offset != bytes.size()) {
        ride.truncated = true;
    }

    return true;
}

bool loadSummary(const std::string& binPath, RideSummary& out) {
    std::string metPath = binPath;
    const size_t dot = metPath.find_last_of('.');
    if (dot == std::string::npos) return false;
    metPath.replace(dot, std::string::npos, ".met");

    std::vector<uint8_t> bytes;
    if (!readWholeFile(metPath, bytes) || bytes.size() != sizeof(RideSummary)) {
        return false;
    }

    memcpy(&out, bytes.data(), sizeof(RideSummary));
    return true;
}

// ---------------------------------------------------------------------------
// Terminal charts
// ---------------------------------------------------------------------------

/// Draws the min/max envelope of a series, so a brief spike is visible even
/// when hundreds of samples collapse into one column.
void plotEnvelope(const std::vector<float>& xs, const std::vector<float>& ys, int width,
                  int height, float lo, float hi, const char* unit, bool markZero) {
    if (xs.empty() || hi <= lo) return;

    std::vector<float> columnMin(width, 0.0f);
    std::vector<float> columnMax(width, 0.0f);
    std::vector<bool>  columnUsed(width, false);

    const float xMin = xs.front();
    const float xMax = xs.back();
    const float span = xMax > xMin ? xMax - xMin : 1.0f;

    for (size_t i = 0; i < xs.size(); ++i) {
        int column = static_cast<int>((xs[i] - xMin) / span * (width - 1));
        if (column < 0) column = 0;
        if (column >= width) column = width - 1;

        if (!columnUsed[column]) {
            columnMin[column] = columnMax[column] = ys[i];
            columnUsed[column] = true;
        } else {
            if (ys[i] < columnMin[column]) columnMin[column] = ys[i];
            if (ys[i] > columnMax[column]) columnMax[column] = ys[i];
        }
    }

    // Carry the previous column across gaps so the trace stays continuous.
    for (int c = 1; c < width; ++c) {
        if (!columnUsed[c]) {
            columnMin[c] = columnMin[c - 1];
            columnMax[c] = columnMax[c - 1];
            columnUsed[c] = columnUsed[c - 1];
        }
    }

    const float step = (hi - lo) / height;

    for (int row = 0; row < height; ++row) {
        const float rowTop    = hi - row * step;
        const float rowBottom = rowTop - step;

        printf("  %7.1f %s|", static_cast<double>(rowTop), unit);
        for (int c = 0; c < width; ++c) {
            if (!columnUsed[c]) {
                putchar(' ');
                continue;
            }
            const bool intersects = columnMax[c] >= rowBottom && columnMin[c] <= rowTop;
            if (intersects) {
                putchar('#');
            } else if (markZero && rowBottom <= 0.0f && rowTop > 0.0f) {
                putchar('-');
            } else {
                putchar(' ');
            }
        }
        putchar('\n');
    }

    printf("  %*s+", 9, "");
    for (int c = 0; c < width; ++c) putchar('-');
    printf("\n  %*s0 s%*.0f s\n", 9, "", width - 4, static_cast<double>(xMax));
}

// ---------------------------------------------------------------------------
// Output modes
// ---------------------------------------------------------------------------

void printReport(const std::string& path, const Ride& ride) {
    printf("\nFile      %s (%zu bytes)\n", path.c_str(), ride.fileSize);

    char magic[5] = {0};
    memcpy(magic, &ride.header.magic, 4);
    printf("Magic     %s   format v%u   firmware 0x%04x   %s\n", magic,
           ride.header.formatVersion, ride.header.firmwareVersion,
           ride.headerValid ? "header CRC OK" : "*** HEADER INVALID ***");

    printf("Ride      R%06u\n", ride.header.rideId);
    printf("Started   %s   (device clock %u ms)\n",
           formatUnixTime(ride.header.startUnixTime).c_str(), ride.header.startMillis);
    printf("Log rate  %u Hz IMU   calibration v%u\n", ride.header.imuLogRateHz,
           ride.header.calibrationVersion);

    printf("\nRecords   %zu IMU · %zu GNSS · %zu events", ride.imu.size(), ride.gnss.size(),
           ride.events.size());
    if (ride.unknownRecords > 0) printf(" · %u unknown (skipped)", ride.unknownRecords);
    printf("\n          stream %s\n", ride.truncated ? "TRUNCATED — ends mid-record"
                                                     : "ends cleanly on a record boundary");
    printf("CRC-32    0x%08x", ride.crc);

    RideSummary summary{};
    const bool  haveSummary = loadSummary(path, summary);

    if (haveSummary) {
        const bool crcMatch     = summary.dataCrc == ride.crc;
        const bool summaryValid = validateSummary(summary);
        printf("   %s\n", crcMatch ? "matches the summary" : "*** DOES NOT MATCH SUMMARY ***");

        printf("\nSummary   %s\n", summaryValid ? "(.met file, CRC OK)"
                                                : "(.met file, *** CRC BAD ***)");
        printf("  duration    %u s\n", summary.durationMs / 1000);
        printf("  distance    %.2f km\n", summary.distanceCm / 100000.0);
        printf("  max speed   %.1f km/h\n", summary.maxSpeed * 0.036);
        printf("  lean        %.1f deg left / %.1f deg right\n", summary.maxLeanLeft / 100.0,
               summary.maxLeanRight / 100.0);
        printf("  pitch       %.1f deg up / %.1f deg down\n", summary.maxPitchUp / 100.0,
               summary.maxPitchDown / 100.0);
        printf("  peak accel  %.2f g      peak braking %.2f g\n",
               summary.maxAcceleration / 1000.0, summary.maxBraking / 1000.0);
        printf("  samples     %u IMU / %u GNSS / %u events\n", summary.imuSampleCount,
               summary.gnssSampleCount, summary.eventCount);
        printf("  flags       %s%s%s%s\n", summary.isClosed() ? "closed " : "OPEN ",
               summary.isSynced() ? "synced" : "UNSYNCED",
               (summary.flags & kRideFlagRecovered) ? " recovered" : "",
               (summary.flags & kRideFlagTruncated) ? " truncated" : "");
    } else {
        printf("\n\nSummary   no readable .met file alongside this ride\n");
    }

    if (ride.imu.empty()) {
        return;
    }

    // --- Charts -------------------------------------------------------------
    // The header's start time is the origin, not the first IMU sample: events
    // are written before any sample arrives.
    const uint32_t t0 = ride.header.startMillis;

    std::vector<float> imuTime, lean, longitudinal;
    imuTime.reserve(ride.imu.size());
    for (const ImuRecord& record : ride.imu) {
        imuTime.push_back(static_cast<float>(relativeSeconds(record.timestampMs, t0)));
        lean.push_back(fromCentiDegrees(record.roll));

        const float pitchRad = fromCentiDegrees(record.pitch) * kDegToRad;
        const float cosPitch = cosf(pitchRad);
        float       g        = fromMilliG(record.accelX);
        if (fabsf(cosPitch) > 0.1f) g = (g - sinf(pitchRad)) / cosPitch;
        longitudinal.push_back(g);
    }

    printf("\nLean angle over the ride  (negative = left, positive = right)\n");
    plotEnvelope(imuTime, lean, 76, 11, -50.0f, 50.0f, "d ", true);

    if (!ride.gnss.empty()) {
        std::vector<float> gnssTime, speed;
        for (const GnssRecord& record : ride.gnss) {
            if (record.fixType == 0) continue;  // no fix: nothing meaningful to plot
            gnssTime.push_back(static_cast<float>(relativeSeconds(record.timestampMs, t0)));
            speed.push_back(record.speed * 0.036f);
        }
        if (!speed.empty()) {
            printf("\nGNSS speed (km/h)\n");
            plotEnvelope(gnssTime, speed, 76, 8, 0.0f, 100.0f, "  ", false);
        }
    }

    printf("\nLongitudinal acceleration (g, gravity removed)\n");
    plotEnvelope(imuTime, longitudinal, 76, 7, -1.0f, 1.0f, "g ", true);

    if (!ride.events.empty()) {
        printf("\nEvents\n");
        for (const EventRecord& record : ride.events) {
            printf("  %7.1f s  %-20s value %d\n",
                   relativeSeconds(record.timestampMs, t0), eventName(record.code), record.value);
        }
    }

    printf("\n");
}

void printImuCsv(const Ride& ride) {
    printf("timestamp_ms,roll_deg,pitch_deg,yaw_deg,accel_x_g,accel_y_g,accel_z_g,"
           "gyro_x_dps,gyro_y_dps,gyro_z_dps\n");
    for (const ImuRecord& r : ride.imu) {
        printf("%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n", r.timestampMs,
               fromCentiDegrees(r.roll), fromCentiDegrees(r.pitch), fromCentiDegrees(r.yaw),
               fromMilliG(r.accelX), fromMilliG(r.accelY), fromMilliG(r.accelZ),
               fromDeciDps(r.gyroX), fromDeciDps(r.gyroY), fromDeciDps(r.gyroZ));
    }
}

void printGnssCsv(const Ride& ride) {
    printf("timestamp_ms,unix_time,latitude,longitude,speed_kmh,heading_deg,altitude_m,"
           "hdop,satellites,fix\n");
    for (const GnssRecord& r : ride.gnss) {
        printf("%u,%u,%.7f,%.7f,%.2f,%.2f,%.1f,%.1f,%u,%u\n", r.timestampMs, r.unixTime,
               fromDegreesE7(r.latitude), fromDegreesE7(r.longitude), r.speed * 0.036,
               r.heading / 100.0, r.altitude / 10.0, r.hdop / 10.0, r.satellites, r.fixType);
    }
}

void printGpx(const Ride& ride) {
    printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    printf("<gpx version=\"1.1\" creator=\"ApexRide ridedump\" "
           "xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
    printf("  <trk><name>R%06u</name><trkseg>\n", ride.header.rideId);

    for (const GnssRecord& r : ride.gnss) {
        if (r.fixType == 0) continue;

        printf("    <trkpt lat=\"%.7f\" lon=\"%.7f\">", fromDegreesE7(r.latitude),
               fromDegreesE7(r.longitude));
        printf("<ele>%.1f</ele>", r.altitude / 10.0);
        if (r.unixTime != 0) {
            const time_t seconds = static_cast<time_t>(r.unixTime);
            struct tm    parts {};
            gmtime_r(&seconds, &parts);
            char stamp[32];
            strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", &parts);
            printf("<time>%s</time>", stamp);
        }
        printf("</trkpt>\n");
    }

    printf("  </trkseg></trk>\n</gpx>\n");
}

void printEvents(const Ride& ride) {
    for (const EventRecord& r : ride.events) {
        printf("%u\t%s\t%d\n", r.timestampMs, eventName(r.code), r.value);
    }
}

void printUsage() {
    fprintf(stderr,
            "usage: ridedump <ride.bin> [--csv-imu | --csv-gnss | --gpx | --events]\n"
            "\n"
            "  (no flag)     report, integrity check and terminal charts\n"
            "  --csv-imu     IMU records as CSV on stdout\n"
            "  --csv-gnss    GNSS records as CSV on stdout\n"
            "  --gpx         GPX track, for any mapping tool\n"
            "  --events      event log, tab separated\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 2;
    }

    const std::string path = argv[1];
    const std::string mode = argc > 2 ? argv[2] : "";

    Ride ride;
    if (!parseRide(path, ride)) {
        return 1;
    }

    if (mode.empty()) {
        printReport(path, ride);
    } else if (mode == "--csv-imu") {
        printImuCsv(ride);
    } else if (mode == "--csv-gnss") {
        printGnssCsv(ride);
    } else if (mode == "--gpx") {
        printGpx(ride);
    } else if (mode == "--events") {
        printEvents(ride);
    } else {
        printUsage();
        return 2;
    }

    // A ride whose CRC does not match, or which ends mid-record, is still worth
    // reading — but the exit code has to say so for scripting.
    return ride.headerValid && !ride.truncated ? 0 : 1;
}
