#include <cstdint>
#include <iostream>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "Wspr.h"


bool TestGetBandDataList()
{
    bool retVal = Wspr::GetBandDataList().size() != 0;

    cout << "TestGetBandDataList test: " << retVal << endl;

    return retVal;
}

bool TestGetDialFreqFromBandStr()
{
    static const uint32_t DIAL_FREQ_20M = 14095600;
    static const uint32_t DIAL_FREQ_30M = 10138700;

    vector<InputTest<const char *, uint32_t>> testList = {
        { nullptr,  DIAL_FREQ_20M,  "nullptr",           },
        { "",       DIAL_FREQ_20M,  "empty string",      },
        { " ",      DIAL_FREQ_20M,  "space",             },
        { "20m",    DIAL_FREQ_20M,  "valid 20m",         },
        { "30m",    DIAL_FREQ_30M,  "valid 30m",         },
        { "777m",   DIAL_FREQ_20M,  "non-existent 777m", },
    };

    bool retVal = true;
    for (const auto &test : testList)
    {
        retVal &= CheckErr(test, Wspr::GetDialFreqFromBandStr(test.input));
    }

    cout << "TestGetDialFreqFromBandStr test: " << retVal << endl;

    return retVal;
}

bool TestGetPowerDbmList()
{
    bool retVal = Wspr::GetPowerDbmList().size() != 0;

    cout << "TestGetPowerDbmList test: " << retVal << endl;

    return retVal;
}

bool TestPowerDbmInSet()
{
    vector<InputTest<uint8_t, bool>> testList = {
        { 0,  true,  "valid",   },
        { 1,  false, "invalid", },
        { 60, true,  "valid",   },
        { 61, false, "invalid", },
    };

    bool retVal = true;
    for (const auto &test : testList)
    {
        retVal &= CheckErr(test, Wspr::PowerDbmInSet(test.input));
    }

    cout << "TestPowerDbmInSet test: " << retVal << endl;

    return retVal;
}

int main()
{
    bool retVal = true;

    retVal &= TestGetBandDataList();
    retVal &= TestGetDialFreqFromBandStr();
    retVal &= TestGetPowerDbmList();
    retVal &= TestPowerDbmInSet();

    return !retVal;
}