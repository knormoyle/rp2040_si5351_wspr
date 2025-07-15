#pragma once

#include <array>
#include <cstring>

#include "WsprMessageRegularType1.h"


class WsprMessageTelemetryCommon
: public WsprMessageRegularType1
{
public:

    WsprMessageTelemetryCommon()
    {
        Reset();
    }

    void Reset()
    {
        WsprMessageRegularType1::Reset();

        id13_ = std::array<char, 3>{ "00" };
    }

    // 00 through 09, 10 through 19, Q0 through Q9
    bool SetId13(const char *id13)
    {
        bool retVal = false;

        if (id13 && strlen(id13) == 2)
        {
            char id1 = id13[0];
            char id3 = id13[1];

            if ((id1 == '0' || id1 == '1' || id1 == 'Q') &&
               ('0' <= id3 && id3 <= '9'))
            {
                id13_[0] = id1;
                id13_[1] = id3;

                retVal = true;
            }
        }

        return retVal;
    }

    // 00 through 09, 10 through 19, Q0 through Q9
    const char *GetId13() const
    {
        return (const char *)id13_.data();
    }


protected:

    /////////////////////////////////////////////////////////////////
    // Encoded/Decode Utilities
    /////////////////////////////////////////////////////////////////

    static char EncodeBase36(uint8_t val)
    {
        char retVal;

        if (val < 10)
        {
            retVal = '0' + val;
        }
        else
        {
            retVal = 'A' + (val - 10);
        }

        return retVal;
    }

    static uint8_t DecodeBase36(char c)
    {
        uint8_t retVal = 0;

        uint8_t cVal = c;

        uint8_t aVal    = 'A';
        uint8_t zVal    = 'Z';
        uint8_t zeroVal = '0';

        if (aVal <= cVal && cVal <= zVal)
        {
            retVal = 10 + (cVal - aVal);
        }
        else
        {
            retVal = cVal - zeroVal;
        }

        return retVal;
    }

    static uint8_t DecodePowerDbmToNum(uint8_t powerDbm)
    {
        uint8_t retVal = 0;

        const auto &powerDbmList = Wspr::GetPowerDbmList();

        for (uint8_t i = 0; i < powerDbmList.size(); ++i)
        {
            if (powerDbm == powerDbmList[i])
            {
                retVal = i;
            }
        }

        return retVal;
    }


private:

    std::array<char, 3> id13_;
};