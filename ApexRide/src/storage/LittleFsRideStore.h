#pragma once
//
// IRideStore backed by LittleFS on the ESP32-S3's internal flash.
//
// LittleFS rather than SPIFFS: it is wear-levelled, has real directories, and
// survives power loss mid-write, which matters for a device that is switched
// off by pulling a slide switch mid-ride.
//
// This is the only file in the storage layer that touches Arduino APIs, which
// is what keeps the rest of the pipeline testable on a host.
//

#if defined(ARDUINO)

#include "IRideStore.h"

namespace apex {

class LittleFsRideStore : public IRideStore {
public:
    struct Config {
        /// Format the partition if it cannot be mounted. Safe on first boot;
        /// after that a mount failure means something is wrong and formatting
        /// would silently destroy unsynced rides — so it defaults to false and
        /// the first-boot format is done explicitly in setup().
        bool formatOnFailure = false;

        /// Partition label from the partition table.
        const char* partitionLabel = "littlefs";
    };

    explicit LittleFsRideStore(const Config& config = Config());

    bool begin() override;
    bool ensureDirectory(const char* path) override;

    IRideFile* open(const char* path, FileMode mode) override;
    bool       exists(const char* path) override;
    bool       remove(const char* path) override;
    bool       list(const char* directory, IDirectoryVisitor& visitor) override;

    uint64_t totalBytes() const override;
    uint64_t usedBytes() const override;

    /// Erases the whole partition. Destroys every ride; call only deliberately.
    bool format();

private:
    Config config_;
    bool   mounted_ = false;
};

}  // namespace apex

#endif  // ARDUINO
