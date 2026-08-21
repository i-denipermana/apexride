#include "LittleFsRideStore.h"

#if defined(ARDUINO)

#include <LittleFS.h>
#include <string.h>

#include "../core/Log.h"

namespace apex {
namespace {

class LittleFsRideFile : public IRideFile {
public:
    explicit LittleFsRideFile(File file) : file_(file) {}

    ~LittleFsRideFile() override { close(); }

    size_t write(const void* data, size_t length) override {
        if (!file_) return 0;
        return file_.write(static_cast<const uint8_t*>(data), length);
    }

    size_t read(void* buffer, size_t length) override {
        if (!file_) return 0;
        const int got = file_.read(static_cast<uint8_t*>(buffer), length);
        return got < 0 ? 0 : static_cast<size_t>(got);
    }

    bool flush() override {
        if (!file_) return false;
        file_.flush();
        return true;
    }

    bool seek(size_t offset) override {
        return file_ && file_.seek(static_cast<uint32_t>(offset));
    }

    size_t size() const override {
        return file_ ? static_cast<size_t>(const_cast<File&>(file_).size()) : 0;
    }

    void close() override {
        if (file_) {
            file_.close();
        }
    }

private:
    File file_;
};

const char* modeString(FileMode mode) {
    switch (mode) {
        case FileMode::Read:   return FILE_READ;
        case FileMode::Write:  return FILE_WRITE;
        case FileMode::Append: return FILE_APPEND;
    }
    return FILE_READ;
}

}  // namespace

LittleFsRideStore::LittleFsRideStore() : config_(Config()) {}

LittleFsRideStore::LittleFsRideStore(const Config& config) : config_(config) {}

bool LittleFsRideStore::begin() {
    if (mounted_) {
        return true;
    }

    mounted_ = LittleFS.begin(config_.formatOnFailure, "/littlefs", 10, config_.partitionLabel);
    if (!mounted_) {
        APEX_LOGE("LittleFS mount failed on partition '%s'", config_.partitionLabel);
        return false;
    }

    APEX_LOGI("LittleFS mounted: %llu KB total, %llu KB used",
            static_cast<unsigned long long>(totalBytes() / 1024),
            static_cast<unsigned long long>(usedBytes() / 1024));
    return true;
}

bool LittleFsRideStore::format() {
    APEX_LOGW("Formatting the LittleFS partition — all ride data will be lost");
    mounted_ = false;
    return LittleFS.format();
}

bool LittleFsRideStore::ensureDirectory(const char* path) {
    if (!mounted_) return false;
    if (LittleFS.exists(path)) return true;
    return LittleFS.mkdir(path);
}

IRideFile* LittleFsRideStore::open(const char* path, FileMode mode) {
    if (!mounted_) return nullptr;

    File file = LittleFS.open(path, modeString(mode), mode != FileMode::Read);
    if (!file || file.isDirectory()) {
        return nullptr;
    }

    return new LittleFsRideFile(file);
}

bool LittleFsRideStore::exists(const char* path) {
    return mounted_ && LittleFS.exists(path);
}

bool LittleFsRideStore::remove(const char* path) {
    return mounted_ && LittleFS.remove(path);
}

bool LittleFsRideStore::list(const char* directory, IDirectoryVisitor& visitor) {
    if (!mounted_) return false;

    File dir = LittleFS.open(directory);
    if (!dir || !dir.isDirectory()) {
        return false;
    }

    for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
        if (!entry.isDirectory()) {
            // name() returns the full path on ESP32; the visitor wants the leaf.
            const char* full = entry.name();
            const char* leaf = strrchr(full, '/');
            visitor.onEntry(leaf != nullptr ? leaf + 1 : full, static_cast<size_t>(entry.size()));
        }
        entry.close();
    }

    dir.close();
    return true;
}

uint64_t LittleFsRideStore::totalBytes() const {
    return mounted_ ? LittleFS.totalBytes() : 0;
}

uint64_t LittleFsRideStore::usedBytes() const {
    return mounted_ ? LittleFS.usedBytes() : 0;
}

}  // namespace apex

#endif  // ARDUINO
