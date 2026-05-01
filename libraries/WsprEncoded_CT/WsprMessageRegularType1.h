#ifndef WSPR_MESSAGE_REGULAR_TYPE_1_H_
#define WSPR_MESSAGE_REGULAR_TYPE_1_H_

#include <cstdint>
#include <cstring>

#include "Wspr.h"
#include "WsprUtl.h"

class WsprMessageRegularType1 {
public:
    WsprMessageRegularType1() {
        Reset();
    }

    // Reset the object back to its default state.
    void Reset() {
        callsign_.Target(callsign_buf_, kCallsignLenMax + 1);
        callsign_.Set("0A0AAA");
        grid4_.Target(grid4_buf_, kGrid4Len + 1);
        grid4_.Set("AA00");
        power_dbm_ = 0;
    }

    // Set the callsign.
    //
    // Returns true on success.
    // Returns false on error.
    //
    // An error occurs when the format of the callsign is invalid.
    bool SetCallsign(const char* callsign) {
        bool ret_val = false;
        if (CallsignIsValid(callsign)) {
            callsign_.Set(callsign);
            ret_val = true;
        }
        return ret_val;
    }

    // Get the callsign.
    const char* GetCallsign() const {
        return callsign_.Get();
    }

    // 'A' through 'X' for chars 1 and 2.
    // '0' through '9' for chars 3 and 4.
    bool SetGrid4(const char* grid4) {
        bool ret_val = false;
        if (Grid4IsValid(grid4)) {
            grid4_.Set(grid4);
            ret_val = true;
        }
        return ret_val;
    }

    // 'A' through 'X' for chars 1 and 2.
    // '0' through '9' for chars 3 and 4.
    const char* GetGrid4() const {
        return grid4_.Get();
    }

    // 0,  3,  7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60
    bool SetPowerDbm(uint8_t power_dbm) {
        bool ret_val = false;
        if (Wspr::PowerDbmInSet(power_dbm)) {
            power_dbm_ = power_dbm;
            ret_val = true;
        }
        return ret_val;
    }

    // 0,  3,  7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60
    uint8_t GetPowerDbm() const {
        return power_dbm_;
    }

    static bool CallsignIsValid(const char* callsign) {
        bool ret_val = false;
        // pointer not null
        if (callsign) {
            // temporary copy of string
            size_t len = strlen(callsign);
            char buf[kCallsignLenMax + 1] = { 0 };
            WsprUtl::CString callsign_check(static_cast<char*>(buf),
                                            kCallsignLenMax + 1);
            callsign_check.Set(callsign);
            bool is_padded_left  = callsign_check.IsPaddedLeft();
            bool is_padded_right = callsign_check.IsPaddedRight();
            bool is_uppercase    = callsign_check.IsUppercase();
            // check criteria
            if (is_padded_left == false &&
                is_padded_right == false &&
                is_uppercase == true &&
                len >= kCallsignLenMin &&
                len <= kCallsignLenMax) {
                ret_val = true;
            }
        }
        return ret_val;
    }

    static bool Grid4IsValid(const char* grid4) {
        bool ret_val = false;
        if (grid4) {
            size_t len = strlen(grid4);
            if (len == kGrid4Len) {
                char g1 = grid4[0];
                char g2 = grid4[1];
                char g3 = grid4[2];
                char g4 = grid4[3];
                if ('A' <= g1 && g1 <= 'R' &&
                    'A' <= g2 && g2 <= 'R' &&
                    '0' <= g3 && g3 <= '9' &&
                    '0' <= g4 && g4 <= '9') {
                    ret_val = true;
                }
            }
        }
        return ret_val;
    }

private:
    static const uint8_t kCallsignLenMax = 6;
    static const uint8_t kCallsignLenMin = 4;
    char callsign_buf_[kCallsignLenMax + 1];
    WsprUtl::CString callsign_;

    static const uint8_t kGrid4Len = 4;
    char grid4_buf_[kGrid4Len + 1];
    WsprUtl::CString grid4_;

    uint8_t power_dbm_;
};

#endif  // WSPR_MESSAGE_REGULAR_TYPE_1_H_
