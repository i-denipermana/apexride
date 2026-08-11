#include "TelemetryFormat.h"

#include <math.h>

#include "../core/MathUtils.h"

namespace moto {
namespace {

int16_t saturateI16(float v) {
    if (!(v == v)) return 0;  // NaN
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return static_cast<int16_t>(lrintf(v));
}

uint16_t saturateU16(float v) {
    if (!(v == v)) return 0;
    if (v > 65535.0f) return 65535;
    if (v < 0.0f) return 0;
    return static_cast<uint16_t>(lrintf(v));
}

}  // namespace

int16_t toCentiDegrees(float degrees) {
    return saturateI16(degrees * 100.0f);
}

int16_t toMilliG(float mps2) {
    return saturateI16(mps2 / kGravityMps2 * 1000.0f);
}

int16_t toDeciDps(float radPerSec) {
    return saturateI16(radPerSec * kRadToDeg * 10.0f);
}

int32_t toDegreesE7(double degrees) {
    if (!(degrees == degrees)) return 0;
    if (degrees > 214.0) return 2140000000;
    if (degrees < -214.0) return -2140000000;
    return static_cast<int32_t>(llround(degrees * 1e7));
}

uint16_t toCmPerSec(float mps) {
    return saturateU16(mps * 100.0f);
}

uint16_t toCentiDegreesUnsigned(float degrees) {
    return saturateU16(degrees * 100.0f);
}

int16_t toDecimetres(float metres) {
    return saturateI16(metres * 10.0f);
}

uint8_t toHdopByte(float hdop) {
    const uint16_t scaled = saturateU16(hdop * 10.0f);
    return scaled > 255 ? 255 : static_cast<uint8_t>(scaled);
}

}  // namespace moto
