#include "AutoSyncClient.h"

#include <stdio.h>
#include <string.h>

#include <set>
#include <string>

namespace apex {
namespace {

/// Locates `"key":` and returns the first character of its value.
const char* findKey(const char* json, const char* key) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);

    const char* found = strstr(json, pattern);
    return found != nullptr ? found + strlen(pattern) : nullptr;
}

}  // namespace

bool jsonUInt(const char* json, const char* key, uint32_t& valueOut, const char** afterOut) {
    if (json == nullptr) return false;

    const char* cursor = findKey(json, key);
    if (cursor == nullptr || *cursor < '0' || *cursor > '9') return false;

    uint32_t value = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10u + static_cast<uint32_t>(*cursor - '0');
        ++cursor;
    }

    valueOut = value;
    if (afterOut != nullptr) *afterOut = cursor;
    return true;
}

bool jsonHexString(const char* json, const char* key, uint32_t& valueOut, const char** afterOut) {
    if (json == nullptr) return false;

    const char* cursor = findKey(json, key);
    if (cursor == nullptr || *cursor != '"') return false;
    ++cursor;

    uint32_t value  = 0;
    int      digits = 0;
    while (*cursor != '"' && *cursor != '\0') {
        uint32_t nibble;
        if (*cursor >= '0' && *cursor <= '9') {
            nibble = static_cast<uint32_t>(*cursor - '0');
        } else if (*cursor >= 'a' && *cursor <= 'f') {
            nibble = static_cast<uint32_t>(*cursor - 'a' + 10);
        } else if (*cursor >= 'A' && *cursor <= 'F') {
            nibble = static_cast<uint32_t>(*cursor - 'A' + 10);
        } else {
            return false;
        }
        value = (value << 4) | nibble;
        ++cursor;
        if (++digits > 8) return false;
    }

    if (digits == 0) return false;

    valueOut = value;
    if (afterOut != nullptr) *afterOut = cursor;
    return true;
}

bool AutoSyncClient::fetchPending(std::vector<PendingRide>& out) {
    out.clear();

    std::vector<uint8_t> body;
    if (transport_.request("GET", "/sync/pending", nullptr, body) != 200) {
        return false;
    }

    body.push_back('\0');
    const char* json = reinterpret_cast<const char*>(body.data());

    uint32_t count = 0;
    if (!jsonUInt(json, "count", count) || count == 0) {
        return true;  // nothing outstanding is a success, not a failure
    }

    // Each entry emits id, bytes then crc32 in that order, so walking forward
    // from each "id" is enough without a full parser.
    const char* cursor = strstr(json, "\"rides\":[");
    if (cursor == nullptr) return true;

    for (uint32_t i = 0; i < count; ++i) {
        PendingRide ride;
        const char* afterId = nullptr;
        if (!jsonUInt(cursor, "id", ride.rideId, &afterId)) break;
        if (!jsonUInt(afterId, "bytes", ride.bytes)) break;
        if (!jsonHexString(afterId, "crc32", ride.crc32)) break;

        out.push_back(ride);
        cursor = afterId;
    }

    return true;
}

bool AutoSyncClient::syncRide(const PendingRide& ride, const Config& config, Report& report) {
    char path[48];
    char ackPath[56];
    snprintf(path, sizeof(path), "/rides/R%06u/data", ride.rideId);
    snprintf(ackPath, sizeof(ackPath), "/rides/R%06u/ack", ride.rideId);

    for (int attempt = 0; attempt < config.attemptsPerRide; ++attempt) {
        if (attempt > 0) ++report.retries;

        uint32_t offset = sink_.bytesHeld(ride.rideId);

        // More bytes held than the device says exist means the local copy is
        // from an older, different file. Start again.
        if (offset > ride.bytes) {
            sink_.discard(ride.rideId);
            offset = 0;
        }

        bool transferOk = true;
        while (offset < ride.bytes) {
            char query[64];
            snprintf(query, sizeof(query), "offset=%u&length=%u", offset, config.chunkBytes);

            std::vector<uint8_t> chunk;
            const int status = transport_.request("GET", path, query, chunk);

            if (status != 200 || chunk.empty()) {
                // A dropped link leaves what we have intact; the next attempt
                // resumes from here rather than starting over.
                transferOk = false;
                break;
            }

            if (!sink_.append(ride.rideId, chunk.data(), chunk.size())) {
                transferOk = false;
                break;
            }

            offset += static_cast<uint32_t>(chunk.size());
            report.bytesTransferred += chunk.size();
        }

        if (!transferOk) {
            continue;
        }

        // The checksum is computed over the phone's own copy — using the
        // device's figure here would make the whole verification circular.
        char query[32];
        snprintf(query, sizeof(query), "crc=%08x", sink_.checksum(ride.rideId));

        std::vector<uint8_t> ackBody;
        const int status = transport_.request("POST", ackPath, query, ackBody);

        if (status == 200) {
            return true;
        }

        if (status == 409) {
            // The device rejected our checksum: what we hold is corrupt, so
            // resuming would preserve the corruption. Throw it away.
            sink_.discard(ride.rideId);
            continue;
        }

        // 404 or 500: nothing to be gained by retrying this ride now.
        break;
    }

    return false;
}

AutoSyncClient::Report AutoSyncClient::run(const Config& config) {
    Report report;

    std::vector<uint8_t> body;

    const int beginStatus = transport_.request("POST", "/sync/begin", nullptr, body);
    if (beginStatus == 409) {
        // A ride is being recorded. Serving bulk data would compete with the
        // recorder; try again when the rider stops.
        report.sessionRefused = true;
        return report;
    }
    if (beginStatus != 200) {
        report.transportFailed = true;
        return report;
    }

    std::set<uint32_t> giveUpOn;

    while (report.ridesSynced + report.ridesFailed < config.maxRidesPerSession) {
        std::vector<PendingRide> pending;
        if (!fetchPending(pending)) {
            report.transportFailed = true;
            break;
        }

        // Skip anything already written off this session, or pending would keep
        // handing back the same ride and the loop would never terminate.
        const PendingRide* next = nullptr;
        for (const PendingRide& ride : pending) {
            if (giveUpOn.find(ride.rideId) == giveUpOn.end()) {
                next = &ride;
                break;
            }
        }

        if (next == nullptr) {
            break;  // everything outstanding is either done or written off
        }

        if (syncRide(*next, config, report)) {
            ++report.ridesSynced;
        } else {
            ++report.ridesFailed;
            giveUpOn.insert(next->rideId);
        }
    }

    transport_.request("POST", "/sync/end", nullptr, body);
    return report;
}

}  // namespace apex
