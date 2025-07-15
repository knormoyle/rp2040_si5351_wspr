#include <iostream>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "WsprChannelMap.h"


bool TestGetChannelDetails()
{
    struct Arg
    {
        const char *band;
        uint16_t channel;
    };

    // subset of fields I care to check
    struct Result
    {
        char     id1;
        char     id3;
        uint8_t  min;
        uint32_t freq;

        // make compare work
        auto operator<=>(const Result &) const = default;
    };

    vector<InputTest<Arg, Result>> testList = {
        { { "20m",   0 }, { '0', '0', 8, 14097020, }, "20m,   0", },
        { { "20m",   1 }, { '0', '0', 0, 14097020, }, "20m,   1", },
        { { "20m", 226 }, { '1', '1', 0, 14097060, }, "20m, 226", },
        { { "20m", 452 }, { 'Q', '2', 2, 14097140, }, "20m, 252", },
        { { "20m", 452 }, { 'Q', '2', 2, 14097140, }, "20m, 252", },
        { { "21m",   1 }, { '0', '0', 0, 14097020, }, "21m,   1", },
        { { "20m", 600 }, { '0', '0', 8, 14097020, }, "20m, 600", },
    };

    bool retVal = true;
    for (const auto &test : testList)
    {
        WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(test.input.band, test.input.channel);

        Result r = { cd.id1, cd.id3, cd.min, cd.freq, };

        if (test.expectedRetVal != r)
        {
            retVal = false;

            cout << "ERR: Got diff val than expected (" << test.comment << ")" << endl;
        }
    }

    cout << "TestGetChannelDetails: " << retVal << endl;

    return retVal;
}

bool TestGetDefaultChannelIfNotValid()
{
    vector<InputTest<uint16_t, uint16_t>> testList = {
        {   0,   0, "valid",   },
        { 187, 187, "valid",   },
        { 599, 599, "valid",   },
        { 600,   0, "invalid", },
    };

    bool retVal = true;
    for (const auto &test : testList)
    {
        retVal &= CheckErr(test, WsprChannelMap::GetDefaultChannelIfNotValid(test.input));
    }

    cout << "TestGetDefaultChannelIfNotValid: " << retVal << endl;

    return retVal;
}

int main()
{
    bool retVal = true;

    retVal &= TestGetDefaultChannelIfNotValid();
    retVal &= TestGetChannelDetails();

    return !retVal;
}
