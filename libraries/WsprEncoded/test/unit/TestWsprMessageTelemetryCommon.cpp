#include <iostream>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "WsprMessageTelemetryCommon.h"


bool TestSetId13()
{
    vector<InputTest<const char *, bool>> testList = {
        { nullptr,  false, "nullptr",        },
        { "",       false, "empty string",   },
        { ".",      false, "punctuation .",  },
        { "0",      false, "too short",      },
        { "00",     true,  "valid",          },
        { "15",     true,  "valid",          },
        { "Q9",     true,  "valid",          },
        { ".9",     false, "invalid 1",      },
        { "1Z",     false, "invalid 3",      },
        { "ZZ",     false, "invalid 13",     },
        { "..",     false, "punctuation ..", },
        { " 8",     false, "space 1",        },
        { "1 ",     false, "space 3",        },
        { "  ",     false, "space 13",       },
        { "   ",    false, "space x 3",      },
        { "AAA",    false, "AAA",            },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryCommon msg;
        retVal &= CheckErr(test, msg.SetId13(test.input));
    }

    cout << "TestSetId13: " << retVal << endl;

    return retVal;
}

bool TestSetters()
{
    bool retVal = true;

    retVal &= TestSetId13();

    return retVal;
}

int main()
{
    bool retVal = true;

    retVal &= TestSetters();

    return !retVal;
}