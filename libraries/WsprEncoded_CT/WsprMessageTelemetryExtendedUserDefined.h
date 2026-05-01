#ifndef WSPR_MESSAGE_TELEMETRY_EXTENDED_USER_DEFINED_H
#define WSPR_MESSAGE_TELEMETRY_EXTENDED_USER_DEFINED_H

#include <cstdint>
#include "WsprMessageTelemetryExtendedCommon.h"

// Extended Telemetry, User-Defined variant.
// This is the only Extended Telemetry subclass; 
// it exists as a thin naming convenience over WsprMessageTelemetryExtendedCommon.
class WsprMessageTelemetryExtendedUserDefined
        : public WsprMessageTelemetryExtendedCommon {
public:
    explicit WsprMessageTelemetryExtendedUserDefined(
            uint8_t field_count = kDefaultFieldCount)
            : WsprMessageTelemetryExtendedCommon(field_count) {}

    // Returns true on successful decode.
    // Returns false on error.
    //
    // An error will occur when:
    // - The HdrTelemetryType is not ExtendedTelemetry
    bool Decode() {
        return WsprMessageTelemetryExtendedCommon::Decode();
    }
};

#endif  // WSPR_MESSAGE_TELEMETRY_EXTENDED_USER_DEFINED_H
