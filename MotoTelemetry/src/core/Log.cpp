#include "Log.h"

#include <stdarg.h>
#include <stdio.h>

namespace moto {
namespace {

LogSink  g_sink  = nullptr;
LogLevel g_level = LogLevel::Info;

}  // namespace

void setLogSink(LogSink sink) {
    g_sink = sink;
}

void setLogLevel(LogLevel level) {
    g_level = level;
}

LogLevel logLevel() {
    return g_level;
}

void logPrintf(LogLevel level, const char* fmt, ...) {
    if (level > g_level || g_sink == nullptr) {
        return;
    }

    char    line[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    g_sink(level, line);
}

}  // namespace moto
