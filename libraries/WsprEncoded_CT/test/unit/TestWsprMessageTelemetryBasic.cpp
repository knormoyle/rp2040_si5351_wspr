#include <iostream>
#include <string>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "WsprMessageTelemetryBasic.h"


struct Telemetry
{
    string   grid56;
    int32_t  altitudeMeters;
    int32_t  temperatureCelsius;
    double   voltageVolts;
    int32_t  speedKnots;
    bool     gpsIsValid;
};

struct WsprData
{
    string   callsign;
    string   grid4;
    uint8_t  powerDbm;
};


struct TestDecodeData
{
    WsprData   wspr;
    Telemetry  telemetry;
    string     comment;
};

bool TestDecode()
{
    vector<TestDecodeData> testList = {
        {  { "1Y4PAS", "HK08", 10, },  { "XR", 1000, -12, 4.95,  0, true,  },  "", },
        {  { "QX7DGS", "JQ97", 33, },  { "WR", 3000,   0, 3.20, 10, false, },  "", },
        {  { "0X2FDM", "MI65", 27, },  { "WS", 7000,  13, 3.00, 60, true,  },  "", },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;

        msg.SetCallsign(test.wspr.callsign.c_str());
        msg.SetGrid4(test.wspr.grid4.c_str());
        msg.SetPowerDbm(test.wspr.powerDbm);

        msg.Decode();

        string  grid56             = msg.GetGrid56();
        int32_t altitudeMeters     = msg.GetAltitudeMeters();
        int32_t temperatureCelsius = msg.GetTemperatureCelsius();
        double  voltageVolts       = msg.GetVoltageVolts();
        int32_t speedKnots         = msg.GetSpeedKnots();
        bool    gpsIsValid         = msg.GetGpsIsValid();

        bool ok = true;

        if (test.telemetry.grid56 != grid56)
        {
            ok = false;
            cout << "ERR: Decode grid56 expected \"" << test.telemetry.grid56 << "\" but got \"" << grid56 << "\" (" << test.comment << ")" << endl;
        }

        if (test.telemetry.altitudeMeters != altitudeMeters)
        {
            ok = false;
            cout << "ERR: Decode altitudeMeters expected \"" << test.telemetry.altitudeMeters << "\" but got \"" << altitudeMeters << "\" (" << test.comment << ")" << endl;
        }

        if (test.telemetry.temperatureCelsius != temperatureCelsius)
        {
            ok = false;
            cout << "ERR: Decode temperatureCelsius expected \"" << test.telemetry.temperatureCelsius << "\" but got \"" << temperatureCelsius << "\" (" << test.comment << ")" << endl;
        }

        if (test.telemetry.voltageVolts != voltageVolts)
        {
            ok = false;
            cout << "ERR: Decode voltageVolts expected \"" << test.telemetry.voltageVolts << "\" but got \"" << voltageVolts << "\" (" << test.comment << ")" << endl;
        }

        if (test.telemetry.speedKnots != speedKnots)
        {
            ok = false;
            cout << "ERR: Decode speedKnots expected \"" << test.telemetry.speedKnots << "\" but got \"" << speedKnots << "\" (" << test.comment << ")" << endl;
        }

        if (test.telemetry.gpsIsValid != gpsIsValid)
        {
            ok = false;
            cout << "ERR: Decode gpsIsValid expected \"" << test.telemetry.gpsIsValid << "\" but got \"" << gpsIsValid << "\" (" << test.comment << ")" << endl;
        }

        if (ok == false)
        {
            cout
                << "Input: "
                << " " << test.wspr.callsign
                << " " << test.wspr.grid4
                << " " << test.wspr.powerDbm
                << endl;

            cout << endl;
        }

        retVal &= ok;
    }

    return retVal;
}


struct TestEncodeData
{
    string     id13;
    Telemetry  telemetry;
    WsprData   wspr;
    string     comment;
};

bool TestEncode()
{
    vector<TestEncodeData> testList = {
        {  "14", { "XR", 1000, -12, 4.95,  0, true,  },  { "1Y4PAS", "HK08", 10, },  "", },
        {  "Q7", { "WR", 3000,   0, 3.18, 10, false, },  { "QX7DGS", "JQ97", 33, },  "", },
        {  "02", { "WS", 7000,  13, 3.00, 60, true,  },  { "0X2FDM", "MI65", 27, },  "", },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;

        msg.SetGrid56(test.telemetry.grid56.c_str());
        msg.SetAltitudeMeters(test.telemetry.altitudeMeters);
        msg.SetTemperatureCelsius(test.telemetry.temperatureCelsius);
        msg.SetVoltageVolts(test.telemetry.voltageVolts);
        msg.SetSpeedKnots(test.telemetry.speedKnots);
        msg.SetGpsIsValid(test.telemetry.gpsIsValid);

        msg.SetId13(test.id13.c_str());
        msg.Encode();

        string  callsign = msg.GetCallsign();
        string  grid4    = msg.GetGrid4();
        uint8_t powerDbm = msg.GetPowerDbm();

        bool ok = true;
        if (callsign != test.wspr.callsign)
        {
            ok = false;
            cout << "ERR: Encode callsign expected \"" << test.wspr.callsign << "\" but got \"" << callsign << "\" (" << test.comment << ")" << endl;
        }

        if (grid4 != test.wspr.grid4)
        {
            ok = false;
            cout << "ERR: Encode grid4 expected \"" << test.wspr.grid4 << "\" but got \"" << grid4 << "\" (" << test.comment << ")" << endl;
        }

        if (powerDbm != test.wspr.powerDbm)
        {
            ok = false;
            cout << "ERR: Encode powerDbm expected \"" << (int)test.wspr.powerDbm << "\" but got \"" << (int)powerDbm << "\" (" << test.comment << ")" << endl;
        }

        if (ok == false)
        {
            cout
                << "Input: "
                << " " << test.id13
                << " " << test.telemetry.grid56
                << " " << test.telemetry.altitudeMeters
                << " " << test.telemetry.temperatureCelsius
                << " " << test.telemetry.voltageVolts
                << " " << test.telemetry.speedKnots
                << " " << test.telemetry.gpsIsValid
                << endl;

            cout << endl;
        }

        retVal &= ok;
    }

    cout << "TestEncode: " << retVal << endl;

    return retVal;
}

bool TestSetGrid56()
{
    vector<InputTest<const char *, bool>> testList = {
        { nullptr,  false, "nullptr",        },
        { "",       false, "empty string",   },
        { ".",      false, "punctuation .",  },
        { "F",      false, "too short",      },
        { "AA",     true,  "valid",          },
        { "FF",     true,  "valid",          },
        { "XX",     true,  "valid",          },
        { "ZF",     false, "invalid 5",      },
        { "FZ",     false, "invalid 6",      },
        { "ZZ",     false, "invalid 56",     },
        { "..",     false, "punctuation ..", },
        { " F",     false, "space 5",        },
        { "F ",     false, "space 6",        },
        { "  ",     false, "space 56",       },
        { "   ",    false, "space x 3",      },
        { "AAA",    false, "AAA",            },
        { "AAZ",    false, "AAZ",            },
        { "ZAZ",    false, "ZAZ",            },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;
        retVal &= CheckErr(test, msg.SetGrid56(test.input));
    }

    cout << "SetGrid56: " << retVal << endl;

    return retVal;
}

bool TestSetAltitudeMeters()
{
    vector<InputTest<int32_t, bool>> testList = {
        {    -1,  false, "-1",    },
        {     0,  true,  "0",     },
        { 15000,  true,  "15000", },
        { 21340,  true,  "21340", },
        { 21341,  false, "21341", },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;
        retVal &= CheckErr(test, msg.SetAltitudeMeters(test.input));
    }

    cout << "TestSetAltitudeMeters: " << retVal << endl;

    return retVal;
}

bool TestSetTemperatureCelsius()
{
    vector<InputTest<int32_t, bool>> testList = {
        {    -51,  false, "-51", },
        {    -50,  true,  "-50", },
        {      0,  true,  "0",   },
        {      1,  true,  "1",   },
        {     39,  true,  "39",  },
        {     40,  false, "40",  },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;
        retVal &= CheckErr(test, msg.SetTemperatureCelsius(test.input));
    }

    cout << "TestSetTemperatureCelsius: " << retVal << endl;

    return retVal;
}

bool TestSetVoltageVolts()
{
    vector<InputTest<double, bool>> testList = {
        {  2.95,  false, "2.95", },
        {  3.00,  true,  "3.00", },
        {  3.01,  true,  "3.01", },
        {  4.95,  true,  "4.95", },
        {  4.96,  false, "4.96", },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;
        retVal &= CheckErr(test, msg.SetVoltageVolts(test.input));
    }

    cout << "TestSetVoltageVolts: " << retVal << endl;

    return retVal;
}

bool TestSetSetSpeedKnots()
{
    vector<InputTest<int32_t, bool>> testList = {
        {  -1,  false, "-1", },
        {   0,  true,  "0",  },
        {   1,  true,  "1",  },
        {  82,  true,  "82", },
        {  83,  false, "83", },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryBasic msg;
        retVal &= CheckErr(test, msg.SetSpeedKnots(test.input));
    }

    cout << "TestSetSetSpeedKnots: " << retVal << endl;

    return retVal;
}

bool TestSetters()
{
    bool retVal = true;

    retVal &= TestSetGrid56();
    retVal &= TestSetAltitudeMeters();
    retVal &= TestSetTemperatureCelsius();
    retVal &= TestSetVoltageVolts();
    retVal &= TestSetSetSpeedKnots();

    return retVal;
}

int main()
{
    bool retVal = true;

    retVal &= TestSetters();
    retVal &= TestEncode();
    retVal &= TestDecode();

    return !retVal;
}