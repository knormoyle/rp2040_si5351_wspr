#pragma once

#include <cstdint>
#include <cstring>

#include "Wspr.h"
#include "WsprUtl.h"


class WsprMessageRegularType1
{
public:

    WsprMessageRegularType1()
    {
        Reset();
    }

    // Reset the object back to its default state
    void Reset()
    {
        callsign_.Target(callsignBuf_, CALLSIGN_LEN_MAX + 1);
        callsign_.Set("0A0AAA");

        grid4_.Target(grid4Buf_, GRID4_LEN + 1);
        grid4_.Set("AA00");

        powerDbm_ = 0;
    }

    // Set the callsign
    //
    // Returns true on success.
    // Returns false on error.
    //
    // An error occurs when
    // - The format of the callsign is invalid
    bool SetCallsign(const char *callsign)
    {
        bool retVal = false;

        if (CallsignIsValid(callsign))
        {
            callsign_.Set(callsign);

            retVal = true;
        }

        return retVal;
    }

    // Get the callsign
    const char *GetCallsign() const
    {
        return callsign_.Get();
    }
    
    // 'A' through 'X' for chars 1 and 2
    // '0' through '9' for chars 3 and 4
    bool SetGrid4(const char *grid4)
    {
        bool retVal = false;

        if (Grid4IsValid(grid4))
        {
            grid4_.Set(grid4);

            retVal = true;
        }

        return retVal;
    }

    // 'A' through 'X' for chars 1 and 2
    // '0' through '9' for chars 3 and 4
    const char *GetGrid4() const
    {
        return grid4_.Get();
    }

    // 0,  3,  7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60
    bool SetPowerDbm(uint8_t powerDbm)
    {
        bool retVal = false;

        if (Wspr::PowerDbmInSet(powerDbm))
        {
            powerDbm_ = powerDbm;

            retVal = true;
        }

        return retVal;
    }

    // 0,  3,  7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60
    uint8_t GetPowerDbm() const
    {
        return powerDbm_;
    }
    

    static bool CallsignIsValid(const char *callsign)
    {
        bool retVal = false;

        // pointer not null
        if (callsign)
        {
            // temporary copy of string
            size_t len = strlen(callsign);
            char buf[CALLSIGN_LEN_MAX + 1] { 0 };
            WsprUtl::CString callsignCheck((char *)buf, CALLSIGN_LEN_MAX + 1);
            callsignCheck.Set(callsign);

            bool isPaddedLeft  = callsignCheck.IsPaddedLeft();
            bool isPaddedRight = callsignCheck.IsPaddedRight();
            bool isUppercase   = callsignCheck.IsUppercase();

            // check criteria
            if (isPaddedLeft == false &&
                isPaddedRight == false &&
                isUppercase == true &&
                len >= CALLSIGN_LEN_MIN &&
                len <= CALLSIGN_LEN_MAX)
            {
                retVal = true;
            }
        }

        return retVal;
    }

    static bool Grid4IsValid(const char *grid4)
    {
        bool retVal = false;

        if (grid4)
        {
            size_t len = strlen(grid4);

            if (len == GRID4_LEN)
            {
                char g1 = grid4[0];
                char g2 = grid4[1];
                char g3 = grid4[2];
                char g4 = grid4[3];

                if ('A' <= g1 && g1 <= 'R' &&
                    'A' <= g2 && g2 <= 'R' &&
                    '0' <= g3 && g3 <= '9' &&
                    '0' <= g4 && g4 <= '9')
                {
                    retVal = true;
                }
            }
        }

        return retVal;
    }



private:

    static const uint8_t CALLSIGN_LEN_MAX = 6;
    static const uint8_t CALLSIGN_LEN_MIN = 4;
    char callsignBuf_[CALLSIGN_LEN_MAX + 1];
    WsprUtl::CString callsign_;

    static const uint8_t GRID4_LEN = 4;
    char grid4Buf_[GRID4_LEN + 1];
    WsprUtl::CString grid4_;

    uint8_t powerDbm_;
};

