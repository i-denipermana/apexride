#pragma once
//
// Minimal logging facade.
//
// Core code must stay free of Arduino headers so it can be compiled and tested
// on a host machine. Logging is therefore routed through a sink function that
// the platform layer installs (Serial on the ESP32, stdout on the host).
//

#include <stdint.h>

namespace moto {

enum class LogLevel : uint8_t {
    Error = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
};

using LogSink = void (*)(LogLevel level, const char* line);

void     setLogSink(LogSink sink);
void     setLogLevel(LogLevel level);
LogLevel logLevel();

void logPrintf(LogLevel level, const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace moto

#define MT_LOGE(...) ::moto::logPrintf(::moto::LogLevel::Error, __VA_ARGS__)
#define MT_LOGW(...) ::moto::logPrintf(::moto::LogLevel::Warn, __VA_ARGS__)
#define MT_LOGI(...) ::moto::logPrintf(::moto::LogLevel::Info, __VA_ARGS__)
#define MT_LOGD(...) ::moto::logPrintf(::moto::LogLevel::Debug, __VA_ARGS__)
