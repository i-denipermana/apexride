#include "HostRideStore.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>

namespace apex {
namespace {

class HostRideFile : public IRideFile {
public:
    HostRideFile(FILE* handle, uint64_t remainingCapacity, HostRideStore& owner)
        : handle_(handle), remainingCapacity_(remainingCapacity), owner_(owner) {}

    ~HostRideFile() override { close(); }

    size_t write(const void* data, size_t length) override {
        if (handle_ == nullptr) return 0;

        // Emulate a full filesystem: a short write is exactly what LittleFS
        // does when the volume runs out, and the recorder must handle it.
        const size_t allowed = length <= remainingCapacity_
                                   ? length
                                   : static_cast<size_t>(remainingCapacity_);
        if (allowed == 0) return 0;

        const size_t written = fwrite(data, 1, allowed, handle_);
        remainingCapacity_ -= written;
        owner_.invalidateUsage();
        return written;
    }

    size_t read(void* buffer, size_t length) override {
        if (handle_ == nullptr) return 0;
        return fread(buffer, 1, length, handle_);
    }

    bool flush() override { return handle_ != nullptr && fflush(handle_) == 0; }

    bool seek(size_t offset) override {
        return handle_ != nullptr && fseek(handle_, static_cast<long>(offset), SEEK_SET) == 0;
    }

    size_t size() const override {
        if (handle_ == nullptr) return 0;
        const long current = ftell(handle_);
        fseek(handle_, 0, SEEK_END);
        const long end = ftell(handle_);
        fseek(handle_, current, SEEK_SET);
        return static_cast<size_t>(end < 0 ? 0 : end);
    }

    void close() override {
        if (handle_ != nullptr) {
            fclose(handle_);
            handle_ = nullptr;
        }
    }

private:
    FILE*          handle_;
    uint64_t       remainingCapacity_;
    HostRideStore& owner_;
};

uint64_t directorySize(const std::string& path) {
    DIR* dir = opendir(path.c_str());
    if (dir == nullptr) return 0;

    uint64_t total = 0;
    while (const dirent* entry = readdir(dir)) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        const std::string child = path + "/" + entry->d_name;
        struct stat       info {};
        if (stat(child.c_str(), &info) != 0) continue;

        if (S_ISDIR(info.st_mode)) {
            total += directorySize(child);
        } else {
            total += static_cast<uint64_t>(info.st_size);
        }
    }

    closedir(dir);
    return total;
}

}  // namespace

HostRideStore::HostRideStore(std::string rootPath, uint64_t capacityBytes)
    : rootPath_(std::move(rootPath)), capacityBytes_(capacityBytes) {}

std::string HostRideStore::resolve(const char* path) const {
    if (path == nullptr) return rootPath_;
    if (path[0] == '/') return rootPath_ + path;
    return rootPath_ + "/" + path;
}

bool HostRideStore::reset() {
    const std::string command = "rm -rf '" + rootPath_ + "'";
    if (system(command.c_str()) != 0) {
        return false;
    }
    usageDirty_ = true;
    return mkdir(rootPath_.c_str(), 0755) == 0;
}

bool HostRideStore::begin() {
    struct stat info {};
    if (stat(rootPath_.c_str(), &info) == 0) {
        return S_ISDIR(info.st_mode);
    }
    return mkdir(rootPath_.c_str(), 0755) == 0;
}

bool HostRideStore::ensureDirectory(const char* path) {
    const std::string full = resolve(path);
    struct stat       info {};
    if (stat(full.c_str(), &info) == 0) {
        return S_ISDIR(info.st_mode);
    }
    return mkdir(full.c_str(), 0755) == 0;
}

IRideFile* HostRideStore::open(const char* path, FileMode mode) {
    const char* flags = nullptr;
    switch (mode) {
        case FileMode::Read:   flags = "rb"; break;
        case FileMode::Write:  flags = "wb"; break;
        case FileMode::Append: flags = "ab"; break;
    }

    FILE* handle = fopen(resolve(path).c_str(), flags);
    if (handle == nullptr) {
        return nullptr;
    }

    const uint64_t used      = usedBytes();
    const uint64_t remaining = used >= capacityBytes_ ? 0 : capacityBytes_ - used;

    if (mode != FileMode::Read) {
        usageDirty_ = true;  // truncation or creation changes the total
    }

    return new HostRideFile(handle, mode == FileMode::Read ? UINT64_MAX : remaining, *this);
}

bool HostRideStore::exists(const char* path) {
    struct stat info {};
    return stat(resolve(path).c_str(), &info) == 0;
}

bool HostRideStore::remove(const char* path) {
    const bool removed = ::remove(resolve(path).c_str()) == 0;
    usageDirty_ = true;
    return removed;
}

bool HostRideStore::list(const char* directory, IDirectoryVisitor& visitor) {
    const std::string full = resolve(directory);

    DIR* dir = opendir(full.c_str());
    if (dir == nullptr) {
        return false;
    }

    while (const dirent* entry = readdir(dir)) {
        if (entry->d_name[0] == '.') continue;

        const std::string child = full + "/" + entry->d_name;
        struct stat       info {};
        if (stat(child.c_str(), &info) != 0 || S_ISDIR(info.st_mode)) continue;

        visitor.onEntry(entry->d_name, static_cast<size_t>(info.st_size));
    }

    closedir(dir);
    return true;
}

uint64_t HostRideStore::usedBytes() const {
    if (usageDirty_) {
        cachedUsage_ = directorySize(rootPath_);
        usageDirty_  = false;
    }
    return cachedUsage_;
}

bool HostRideStore::truncate(const char* path, uint64_t newSize) {
    const bool ok = ::truncate(resolve(path).c_str(), static_cast<off_t>(newSize)) == 0;
    usageDirty_ = true;
    return ok;
}

}  // namespace apex
