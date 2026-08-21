#pragma once
//
// Hardware abstraction for the GNSS receiver.
//
// Both the ATGM336H UART/NMEA driver and the simulator implement this same
// interface.
//

#include "../core/Types.h"

namespace apex {

class IGnssSensor {
public:
    virtual ~IGnssSensor() = default;

    virtual bool begin() = 0;

    /// Non-blocking. Returns true and fills `out` when a new solution is ready.
    /// Must also be called to service the UART, so call it every loop.
    virtual bool read(GnssReading& out) = 0;

    virtual float updateRateHz() const = 0;

    /// Parser/transport health counters. Hardware drivers override these;
    /// mocks may leave the zero defaults.
    virtual uint32_t packetCount() const { return 0; }
    virtual uint32_t parseErrorCount() const { return 0; }

    virtual const char* name() const = 0;
};

}  // namespace apex
