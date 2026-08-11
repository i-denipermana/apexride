#pragma once
//
// Small vector/quaternion helpers used by the fusion filter and the ride
// simulator. Header-only and float-based: the ESP32-S3 has a single-precision
// FPU, so float operations are cheap while double operations are emulated.
//

#include <math.h>
#include <stdint.h>

namespace apex {

constexpr float kGravityMps2 = 9.80665f;
constexpr float kPi          = 3.14159265358979f;
constexpr float kDegToRad    = kPi / 180.0f;
constexpr float kRadToDeg    = 180.0f / kPi;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Wrap an angle in radians to (-pi, pi].
inline float wrapPi(float rad) {
    while (rad > kPi) rad -= 2.0f * kPi;
    while (rad < -kPi) rad += 2.0f * kPi;
    return rad;
}

/// Move `current` toward `target` by at most `maxDelta`.
inline float moveToward(float current, float target, float maxDelta) {
    const float diff = target - current;
    if (diff > maxDelta) return current + maxDelta;
    if (diff < -maxDelta) return current - maxDelta;
    return target;
}

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    float norm() const { return sqrtf(x * x + y * y + z * z); }

    /// Returns false (and leaves the vector untouched) for a degenerate vector.
    bool normalize() {
        const float n = norm();
        if (n < 1e-6f) return false;
        const float inv = 1.0f / n;
        x *= inv;
        y *= inv;
        z *= inv;
        return true;
    }
};

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Rotate a vector by -roll about the body X axis (world -> body for pure roll).
inline Vec3 rotateXNeg(const Vec3& v, float roll) {
    const float c = cosf(roll);
    const float s = sinf(roll);
    return {v.x, c * v.y + s * v.z, -s * v.y + c * v.z};
}

/// Rotate a vector by -pitch about the body Y axis (world -> body for pure pitch).
inline Vec3 rotateYNeg(const Vec3& v, float pitch) {
    const float c = cosf(pitch);
    const float s = sinf(pitch);
    return {c * v.x - s * v.z, v.y, s * v.x + c * v.z};
}

/// Unit quaternion, Hamilton convention, rotating body -> world.
struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion identity() { return {}; }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    void normalize() {
        const float n = sqrtf(w * w + x * x + y * y + z * z);
        if (n < 1e-9f) {
            *this = identity();
            return;
        }
        const float inv = 1.0f / n;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
    }

    /// Hamilton product: this ⊗ o.
    Quaternion operator*(const Quaternion& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    // Intrinsic Z-Y-X (yaw, pitch, roll) Euler decomposition.
    //
    // These are the plain mathematical angles of the rotation — they carry no
    // vehicle semantics. Orientation applies the motorcycle sign conventions
    // (see Orientation.h) on top of them.
    float eulerX() const {
        return atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    }

    float eulerY() const {
        return asinf(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
    }

    float eulerZ() const {
        return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    }
};

}  // namespace apex
