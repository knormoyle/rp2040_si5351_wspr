#include <iostream>
#include <string>
#include <vector>
using namespace std;


#include "TestUtl.h"
using namespace TestUtl;

#include "WsprMessageRegularType1.h"


bool TestCall()
{
    vector<InputTest<const char *, bool>> testList = {
        { nullptr,      false,  "nullptr",                                              },
        { "",           false,  "empty string",                                         },
        { "K1J",        false,  "too short",                                            },
        { "K1JT",       true,   "valid",                                                },
        { "K1JT2",      true,   "invalid format but right len (we don't check format)", },
        { "K1JT22",     true,   "invalid format but right len (we don't check format)", },
        { "K1JT2 ",     false,  "invalid format -- padded",                             },
        { " K1JT2",     false,  "invalid format -- padded",                             },
        { " K1JT ",     false,  "invalid format -- padded",                             },
        { "K1JKT222",   false,  "too long",                                             },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageRegularType1 msg;
        retVal &= CheckErr(test, msg.SetCallsign(test.input));
    }

    cout << "Call test: " << retVal << endl;

    return retVal;
}

bool TestGrid()
{
    vector<InputTest<const char *, bool>> testList = {
        { nullptr,     false, "nullptr",        },
        { "",          false, "empty string",   },
        { "F",         false, "too short",      },
        { " F",        false, "padded",         },
        { "F ",        false, "padded",         },
        { " F ",       false, "padded",         },
        { "FN",        false, "too short",      },
        { " FN",       false, "padded",         },
        { "FN ",       false, "padded",         },
        { " FN ",      false, "padded",         },
        { "FNA",       false, "invalid format", },
        { "FN2",       false, "too short",      },
        { "FN20",      true,  "valid",          },
        { "FN2A",      false, "invalid format", },
        { "FNA0",      false, "invalid format", },
        { "FS20",      false, "invalid format", },
        { "SN20",      false, "invalid format", },
        { "FN201",     false, "too long",       },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageRegularType1 msg;
        retVal &= CheckErr(test, msg.SetGrid4(test.input));
    }

    cout << "Grid test: " << retVal << endl;

    return retVal;
}

bool TestPower()
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
        WsprMessageRegularType1 msg;
        retVal &= CheckErr(test, msg.SetPowerDbm(test.input));
    }

    cout << "PowerDbm test: " << retVal << endl;

    return retVal;
}

int main()
{
    bool retVal = true;

    retVal &= TestCall();
    retVal &= TestGrid();
    retVal &= TestPower();

    return !retVal;
}