#include "SyncProtocol.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace apex {
namespace {

/// Bounded append-only JSON writer.
///
/// A dedicated writer rather than a JSON library: the output shapes are fixed
/// and small, and overflow has to be detectable so a full buffer produces a
/// 500 rather than a truncated body the phone would try to parse.
class JsonWriter {
public:
    JsonWriter(char* buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {
        if (capacity_ > 0) buffer_[0] = '\0';
    }

    void append(const char* text) {
        const size_t length = strlen(text);
        if (overflowed_ || length_ + length + 1 > capacity_) {
            overflowed_ = true;
            return;
        }
        memcpy(buffer_ + length_, text, length + 1);
        length_ += length;
    }

    void appendFormat(const char* format, ...)
#if defined(__GNUC__)
        __attribute__((format(printf, 2, 3)))
#endif
    {
        if (overflowed_) return;

        va_list args;
        va_start(args, format);
        const int written =
            vsnprintf(buffer_ + length_, capacity_ - length_, format, args);
        va_end(args);

        if (written < 0 || static_cast<size_t>(written) >= capacity_ - length_) {
            overflowed_ = true;
            return;
        }
        length_ += static_cast<size_t>(written);
    }

    bool   overflowed() const { return overflowed_; }
    size_t length() const { return length_; }

private:
    char*  buffer_;
    size_t capacity_;
    size_t length_     = 0;
    bool   overflowed_ = false;
};

SyncResponse errorResponse(uint16_t status, const char* message, char* buffer,
                           size_t bufferSize) {
    JsonWriter json(buffer, bufferSize);
    json.appendFormat("{\"error\":\"%s\"}", message);

    SyncResponse response;
    response.status     = status;
    response.body       = buffer;
    response.bodyLength = json.length();
    return response;
}

uint16_t statusFor(SyncService::Result result) {
    switch (result) {
        case SyncService::Result::Ok:         return 200;
        case SyncService::Result::NotFound:   return 404;
        case SyncService::Result::Conflict:   return 409;
        case SyncService::Result::BadRequest: return 400;
        case SyncService::Result::IoError:    return 500;
    }
    return 500;
}

const char* messageFor(SyncService::Result result) {
    switch (result) {
        case SyncService::Result::Ok:         return "ok";
        case SyncService::Result::NotFound:   return "no such ride";
        case SyncService::Result::Conflict:   return "ride is still recording, or the checksum did not match";
        case SyncService::Result::BadRequest: return "bad request";
        case SyncService::Result::IoError:    return "filesystem error";
    }
    return "error";
}

void writeSummaryFields(JsonWriter& json, const RideSummary& summary) {
    json.appendFormat(
        "\"id\":%u,\"start\":%u,\"end\":%u,\"durationMs\":%u,\"distanceCm\":%u,"
        "\"maxSpeedCmS\":%u,\"maxLeanLeftCdeg\":%u,\"maxLeanRightCdeg\":%u,"
        "\"maxAccelMilliG\":%u,\"maxBrakeMilliG\":%u,"
        "\"imuSamples\":%u,\"gnssSamples\":%u,\"events\":%u,"
        "\"bytes\":%u,\"crc32\":\"%08x\",\"synced\":%s,\"closed\":%s,"
        "\"recovered\":%s,\"truncated\":%s",
        summary.rideId, summary.startUnixTime, summary.endUnixTime, summary.durationMs,
        summary.distanceCm, summary.maxSpeed, summary.maxLeanLeft, summary.maxLeanRight,
        summary.maxAcceleration, summary.maxBraking, summary.imuSampleCount,
        summary.gnssSampleCount, summary.eventCount, summary.fileSizeBytes, summary.dataCrc,
        summary.isSynced() ? "true" : "false", summary.isClosed() ? "true" : "false",
        (summary.flags & kRideFlagRecovered) ? "true" : "false",
        (summary.flags & kRideFlagTruncated) ? "true" : "false");
}

/// Matches `path` against `prefix` and returns what follows, or null.
const char* afterPrefix(const char* path, const char* prefix) {
    const size_t length = strlen(prefix);
    if (strncmp(path, prefix, length) != 0) {
        return nullptr;
    }
    return path + length;
}

}  // namespace

bool parseRideId(const char* text, uint32_t& rideIdOut) {
    if (text == nullptr || text[0] != 'R') {
        return false;
    }

    uint32_t value  = 0;
    int      digits = 0;
    for (const char* c = text + 1; *c != '\0'; ++c) {
        if (*c < '0' || *c > '9') return false;
        value = value * 10u + static_cast<uint32_t>(*c - '0');
        if (++digits > 9) return false;
    }

    if (digits == 0) return false;

    rideIdOut = value;
    return true;
}

namespace {

/// Finds `key=` in a query string, honouring parameter boundaries so that
/// looking for "crc" does not match "srccrc".
const char* findQueryValue(const char* query, const char* key) {
    if (query == nullptr || key == nullptr) return nullptr;

    const size_t keyLength = strlen(key);
    const char*  cursor    = query;

    while (*cursor != '\0') {
        const bool atBoundary = cursor == query || cursor[-1] == '&' || cursor[-1] == '?';
        if (atBoundary && strncmp(cursor, key, keyLength) == 0 && cursor[keyLength] == '=') {
            return cursor + keyLength + 1;
        }
        ++cursor;
    }
    return nullptr;
}

}  // namespace

bool queryUInt(const char* query, const char* key, uint32_t& valueOut) {
    const char* text = findQueryValue(query, key);
    if (text == nullptr || *text < '0' || *text > '9') {
        return false;
    }

    uint32_t value = 0;
    while (*text >= '0' && *text <= '9') {
        value = value * 10u + static_cast<uint32_t>(*text - '0');
        ++text;
    }

    valueOut = value;
    return true;
}

bool queryHex(const char* query, const char* key, uint32_t& valueOut) {
    const char* text = findQueryValue(query, key);
    if (text == nullptr) return false;

    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        text += 2;
    }

    uint32_t value  = 0;
    int      digits = 0;
    while (*text != '\0' && *text != '&') {
        uint32_t nibble;
        if (*text >= '0' && *text <= '9') {
            nibble = static_cast<uint32_t>(*text - '0');
        } else if (*text >= 'a' && *text <= 'f') {
            nibble = static_cast<uint32_t>(*text - 'a' + 10);
        } else if (*text >= 'A' && *text <= 'F') {
            nibble = static_cast<uint32_t>(*text - 'A' + 10);
        } else {
            return false;
        }
        value = (value << 4) | nibble;
        ++text;
        if (++digits > 8) return false;
    }

    if (digits == 0) return false;

    valueOut = value;
    return true;
}

size_t SyncProtocol::manifestBufferSize(size_t rideCount) {
    // ~260 bytes per entry with every field present, plus the wrapper.
    return 256 + rideCount * 288;
}

SyncResponse SyncProtocol::writeStatus(char* buffer, size_t bufferSize) {
    const DeviceStatus status  = service_.deviceStatus();
    RideStorage&       storage = service_.storage();

    JsonWriter json(buffer, bufferSize);
    json.appendFormat(
        "{\"device\":\"%s\",\"firmware\":\"0x%04x\",\"format\":%u,"
        "\"rides\":%u,\"unsynced\":%u,"
        "\"storage\":{\"totalBytes\":%llu,\"freeBytes\":%llu},"
        "\"recording\":%s,\"activeRide\":%u,"
        "\"gnss\":{\"fix\":%s,\"satellites\":%u},",
        service_.config().deviceName, kFirmwareVersion, kFormatVersion,
        static_cast<unsigned>(storage.rideCount()), static_cast<unsigned>(storage.unsyncedCount()),
        static_cast<unsigned long long>(storage.totalBytes()),
        static_cast<unsigned long long>(storage.freeBytes()),
        status.recording ? "true" : "false", status.activeRideId,
        status.gnssFix ? "true" : "false", status.satellites);

    // Reported honestly: V1 hardware cannot measure the cell.
    if (status.batteryAvailable) {
        json.appendFormat("\"battery\":{\"available\":true,\"percent\":%u}}",
                          status.batteryPercent);
    } else {
        json.append("\"battery\":{\"available\":false}}");
    }

    if (json.overflowed()) {
        return errorResponse(500, "status buffer too small", buffer, bufferSize);
    }

    SyncResponse response;
    response.body       = buffer;
    response.bodyLength = json.length();
    return response;
}

SyncResponse SyncProtocol::writeRideList(char* buffer, size_t bufferSize) {
    RideStorage& storage = service_.storage();

    JsonWriter json(buffer, bufferSize);
    json.appendFormat("{\"count\":%u,\"rides\":[", static_cast<unsigned>(storage.rideCount()));

    for (size_t i = 0; i < storage.rideCount(); ++i) {
        const RideStorage::RideEntry& entry = storage.rideAt(i);

        if (i > 0) json.append(",");
        json.append("{");

        if (entry.summaryValid) {
            writeSummaryFields(json, entry.summary);
        } else {
            // A ride whose summary could not be read is still listed: hiding it
            // would make unsynced data invisible to the phone.
            json.appendFormat("\"id\":%u,\"bytes\":%u,\"synced\":false,\"summaryValid\":false",
                              entry.rideId, entry.dataBytes);
        }

        json.append("}");
    }

    json.append("]}");

    if (json.overflowed()) {
        return errorResponse(500, "ride list buffer too small", buffer, bufferSize);
    }

    SyncResponse response;
    response.body       = buffer;
    response.bodyLength = json.length();
    return response;
}

SyncResponse SyncProtocol::writeRideSummary(uint32_t rideId, char* buffer, size_t bufferSize) {
    const RideStorage::RideEntry* entry = service_.storage().findRide(rideId);
    if (entry == nullptr) {
        return errorResponse(404, "no such ride", buffer, bufferSize);
    }
    if (!entry->summaryValid) {
        return errorResponse(409, "ride has no valid summary", buffer, bufferSize);
    }

    JsonWriter json(buffer, bufferSize);
    json.append("{");
    writeSummaryFields(json, entry->summary);
    json.append("}");

    if (json.overflowed()) {
        return errorResponse(500, "summary buffer too small", buffer, bufferSize);
    }

    SyncResponse response;
    response.body       = buffer;
    response.bodyLength = json.length();
    return response;
}

SyncResponse SyncProtocol::route(const char* method, const char* path, const char* query,
                                 char* buffer, size_t bufferSize) {
    if (method == nullptr || path == nullptr || buffer == nullptr || bufferSize < 64) {
        return errorResponse(400, "malformed request", buffer, bufferSize);
    }

    const bool isGet  = strcmp(method, "GET") == 0;
    const bool isPost = strcmp(method, "POST") == 0;

    if (isGet && strcmp(path, "/status") == 0) {
        return writeStatus(buffer, bufferSize);
    }

    if (isGet && strcmp(path, "/rides") == 0) {
        return writeRideList(buffer, bufferSize);
    }

    const char* tail = afterPrefix(path, "/rides/");
    if (tail == nullptr || *tail == '\0') {
        return errorResponse(404, "unknown endpoint", buffer, bufferSize);
    }

    // Split "R000001/data" into the id and the action after it.
    char        rideText[16];
    const char* slash  = strchr(tail, '/');
    const size_t idLen = slash != nullptr ? static_cast<size_t>(slash - tail) : strlen(tail);
    if (idLen == 0 || idLen >= sizeof(rideText)) {
        return errorResponse(400, "malformed ride id", buffer, bufferSize);
    }
    memcpy(rideText, tail, idLen);
    rideText[idLen] = '\0';

    uint32_t rideId = 0;
    if (!parseRideId(rideText, rideId)) {
        return errorResponse(400, "malformed ride id", buffer, bufferSize);
    }

    const char* action = slash != nullptr ? slash + 1 : "";

    if (isGet && action[0] == '\0') {
        return writeRideSummary(rideId, buffer, bufferSize);
    }

    if (isGet && strcmp(action, "data") == 0) {
        const RideStorage::RideEntry* entry = service_.storage().findRide(rideId);
        if (entry == nullptr) {
            return errorResponse(404, "no such ride", buffer, bufferSize);
        }
        if (entry->summaryValid && !entry->summary.isClosed()) {
            return errorResponse(409, "ride is still being recorded", buffer, bufferSize);
        }

        uint32_t offset = 0;
        queryUInt(query, "offset", offset);

        uint32_t length = static_cast<uint32_t>(service_.config().maxChunkBytes);
        queryUInt(query, "length", length);
        if (length > service_.config().maxChunkBytes) {
            length = static_cast<uint32_t>(service_.config().maxChunkBytes);
        }

        const uint32_t total = entry->summaryValid ? entry->summary.fileSizeBytes
                                                   : entry->dataBytes;
        if (offset > total) {
            return errorResponse(400, "offset past end of ride", buffer, bufferSize);
        }
        if (offset + length > total) {
            length = total - offset;
        }

        SyncResponse response;
        response.contentType = "application/octet-stream";
        response.isRideData  = true;
        response.rideId      = rideId;
        response.dataOffset  = offset;
        response.dataLength  = length;
        return response;
    }

    if (isPost && strcmp(action, "ack") == 0) {
        uint32_t checksum = 0;
        if (!queryHex(query, "crc", checksum)) {
            // Refusing an ack with no checksum is the whole point: without one
            // there is nothing to verify, and the ride must stay unsynced.
            return errorResponse(400, "ack requires a crc parameter", buffer, bufferSize);
        }

        const SyncService::Result result = service_.acknowledge(rideId, checksum);
        if (result != SyncService::Result::Ok) {
            return errorResponse(statusFor(result), messageFor(result), buffer, bufferSize);
        }

        JsonWriter json(buffer, bufferSize);
        json.appendFormat("{\"id\":%u,\"synced\":true}", rideId);

        SyncResponse response;
        response.body       = buffer;
        response.bodyLength = json.length();
        return response;
    }

    if (isPost && strcmp(action, "delete") == 0) {
        const SyncService::Result result = service_.deleteRide(rideId);
        if (result != SyncService::Result::Ok) {
            return errorResponse(statusFor(result),
                                 result == SyncService::Result::Conflict
                                     ? "refusing to delete an unsynced ride"
                                     : messageFor(result),
                                 buffer, bufferSize);
        }

        JsonWriter json(buffer, bufferSize);
        json.appendFormat("{\"id\":%u,\"deleted\":true}", rideId);

        SyncResponse response;
        response.body       = buffer;
        response.bodyLength = json.length();
        return response;
    }

    return errorResponse(404, "unknown endpoint", buffer, bufferSize);
}

}  // namespace apex
