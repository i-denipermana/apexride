#pragma once
//
// Hardware abstraction for the GNSS receiver.
//
// V1 ships MockGnssSensor. The ATGM336H driver will parse NMEA off a UART and
// implement this same interface.
//

#include "../core/Types.h"

namespace moto {

class IGnssSensor {
public:
    virtual ~IGnssSensor() = default;

    virtual bool begin() = 0;

    /// Non-blocking. Returns true and fills `out` when a new solution is ready.
    /// Must also be called to service the UART, so call it every loop.
    virtual bool read(GnssReading& out) = 0;

    virtual float updateRateHz() const = 0;

    virtual const char* name() const = 0;
};

}  // namespace moto
