#ifndef WSPR_MESSAGE_TELEMETRY_COMMON_H
#define WSPR_MESSAGE_TELEMETRY_COMMON_H

// WsprMessageTelemetryCommon.h — Shared base for U4B telemetry messages.
//
// Adds the two-character ID13 field (id1 + id3) that identifies the
// sending station, and provides base-36 encode/decode utilities used by
// all telemetry subclasses.

#include <cstdint>
#include <cstring>
#include "WsprMessageRegularType1.h"

class WsprMessageTelemetryCommon : public WsprMessageRegularType1 {
public:
    WsprMessageTelemetryCommon() {
        Reset();
    }

    void Reset() override {
        WsprMessageRegularType1::Reset();
        id13_[0] = '0';
        id13_[1] = '0';
        id13_[2] = '\0';
    }

    // 00 through 09, 10 through 19, Q0 through Q9.
    bool SetId13(const char* id13) {
        bool ret_val = false;
        if (id13 && strlen(id13) == 2) {
            char id1 = id13[0];
            char id3 = id13[1];
            if ((id1 == '0' || id1 == '1' || id1 == 'Q') &&
                ('0' <= id3 && id3 <= '9')) {
                id13_[0] = id1;
                id13_[1] = id3;
                ret_val = true;
            }
        }
        return ret_val;
    }

    // 00 through 09, 10 through 19, Q0 through Q9.
    const char* GetId13() const {
        return id13_;
    }

protected:
    /////////////////////////////////////////////////////////////////
    // Encode/Decode utilities
    /////////////////////////////////////////////////////////////////
    static char EncodeBase36(uint8_t val) {
        // Defensive: a base-36 digit must be in [0, 35]. Clamp out-of-range
        // input rather than producing a non-base36 character. Existing
        // callers should never trigger this, but a future regression in
        // upstream big-number arithmetic would be obvious instead of silent.
        if (val > 35) {
            val = 35;
        }
        char ret_val;
        if (val < 10) {
            ret_val = '0' + val;
        } else {
            ret_val = 'A' + (val - 10);
        }
        return ret_val;
    }

    // Decode a single base-36 character ('0'..'9', 'A'..'Z') back to its
    // 0..35 numeric value. Returns 0 for any out-of-range input rather
    // than producing a silent overflow value (e.g. a space character would
    // otherwise underflow to 240). This is the inverse of EncodeBase36
    // and shares its clamp-on-bad-input policy.
    //
    // The cast through `unsigned char` avoids the sign-extension that
    // would happen on platforms where plain `char` is signed: a high-byte
    // input would otherwise become a large negative int and then narrow
    // unpredictably.
    static uint8_t DecodeBase36(char c) {
        uint8_t ret_val = 0;
        uint8_t c_val = static_cast<unsigned char>(c);
        const uint8_t a_val    = 'A';
        const uint8_t z_val    = 'Z';
        const uint8_t zero_val = '0';
        const uint8_t nine_val = '9';
        if (a_val <= c_val && c_val <= z_val) {
            ret_val = 10 + (c_val - a_val);
        } else if (zero_val <= c_val && c_val <= nine_val) {
            ret_val = c_val - zero_val;
        }
        // else: out of range, leave ret_val == 0
        return ret_val;
    }

    // Map a WSPR power-in-dBm value to its 0-based index in the
    // canonical power list. Returns 0 if `power_dbm` is not in the list
    // (callers that need to detect this should validate via
    // Wspr::PowerDbmInSet first; surfacing failure through a sentinel
    // would change the public ABI).
    static uint8_t DecodePowerDbmToNum(uint8_t power_dbm) {
        uint8_t ret_val = 0;
        const uint8_t* power_dbm_list = Wspr::GetPowerDbmList();
        uint8_t power_dbm_count = Wspr::GetPowerDbmCount();
        for (uint8_t i = 0; i < power_dbm_count; ++i) {
            if (power_dbm == power_dbm_list[i]) {
                ret_val = i;
                break;
            }
        }
        return ret_val;
    }

private:
    char id13_[3];
};

#endif  // WSPR_MESSAGE_TELEMETRY_COMMON_H
