#include <iostream>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "WsprMessageTelemetryExtendedCommon.h"


bool TestSetClamp()
{
    struct FieldDataSetValue
    {
        const char *name = "";

        double lowValue = 0;
        double highValue = 0;
        double stepSize = 0;

        double setValue = 0;
    };

    struct TestResult
    {
        bool   setOk    = true;
        double getValue = 0;
    };

    // particular test cases
    vector<InputTest<FieldDataSetValue, TestResult>> testList = {
        {
            { "F1", 0, 10, 1, 0 },
            { true, 0 },
            "just a normal set at low end"
        },
        {
            { "F1", 0, 10, 1, 10 },
            { true, 10 },
            "just a normal set at high end"
        },
        {
            { "F1", 0, 10, 1, 5 },
            { true, 5 },
            "just a normal set at mid range"
        },
        {
            { "F1", 0, 10, 1, -1 },
            { false, 0 },
            "set below range"
        },
        {
            { "F1", 0, 10, 1, 11 },
            { false, 10 },
            "set above range"
        },
        {
            { "F1", 0, 10, 1, NAN },
            { false, 0 },
            "set NAN"
        },
        {
            { "F1", 0, 10, 1, (1.0 / 0.0) },
            { false, 0 },
            "set +inf"
        },
        {
            { "F1", 0, 10, 1, (-1.0 / 0.0) },
            { false, 0 },
            "set -inf"
        },
        {
            { "F1", 0, 10, 1, -0.0 },
            { false, 0 },
            "set -0.0"
        },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryExtendedCommon msg;

        msg.DefineField(test.input.name, test.input.lowValue, test.input.highValue, test.input.stepSize);

        bool setOk = msg.Set(test.input.name, test.input.setValue);
        double getValue = msg.Get(test.input.name);

        if (setOk != test.expectedRetVal.setOk)
        {
            retVal = false;

            cout << "ERR: Got setOk " << setOk << ", but expected " << test.expectedRetVal.setOk << " (" << test.comment << ")" << endl;
        }
        if (getValue != test.expectedRetVal.getValue)
        {
            retVal = false;

            cout << "ERR: Got getValue " << getValue << ", but expected " << test.expectedRetVal.getValue << " (" << test.comment << ")" << endl;
        }
    }

    cout << "TestSetClamp: " << retVal << endl;

    return retVal;
}

template <typename T>
void Print(const string &header, T &msg, const vector<const char *> &fieldNameList)
{
    cout << header << endl;
    cout << "-------------------" << endl;
    for (const char *fieldName : fieldNameList)
    {
        double val = NAN;
        msg.Get(fieldName, val);

        cout << fieldName << ": " << val << endl;
    }

    cout << "id13    : " << msg.GetId13()          << endl;
    cout << "callsign: " << msg.GetCallsign()       << endl;
    cout << "grid4   : " << msg.GetGrid4()          << endl;
    cout << "powerDbm: " << (int) msg.GetPowerDbm() << endl;

    cout << endl;
}

struct FieldData
{
    const char *name = "";

    double lowValue = 0;
    double highValue = 0;
    double stepSize = 0;

    double value = 0;
};

struct Type1Data
{
    string  callsign;
    string  grid4;
    uint8_t powerDbm = 0;
};

struct EncodeDecodeTest
{
    const char *id13    = "00";
    uint8_t     hdrSlot = 0;

    vector<FieldData> fieldDataList;

    Type1Data wspr;
};

bool DoEncodeDecodeTest(const EncodeDecodeTest &test)
{
    bool retVal = true;

    // capture field names, including some headers
    vector<const char *> fieldNameList;
    for (const auto &fd : test.fieldDataList)
    {
        fieldNameList.push_back(fd.name);
    }
    fieldNameList.push_back("HdrSlot");
    fieldNameList.push_back("HdrType");
    fieldNameList.push_back("HdrRESERVED");
    fieldNameList.push_back("HdrTelemetryType");

    // create message holding max field count possible
    WsprMessageTelemetryExtendedCommon msg;

    // define fields
    for (const auto &fd : test.fieldDataList)
    {
        msg.DefineField(fd.name, fd.lowValue, fd.highValue, fd.stepSize);
    }

    // Print("After definition", msg, fieldNameList);

    // set field values
    for (const auto &fd : test.fieldDataList)
    {
        msg.Set(fd.name, fd.value);
    }

    // encode
    msg.SetId13(test.id13);
    msg.SetHdrSlot(test.hdrSlot);
    // Print("After Setting", msg, fieldNameList);
    msg.Encode();

    // cout << endl;
    // Print("After Encode", msg, fieldNameList);

    // copy encoded values
    string  callsign = msg.GetCallsign();
    string  grid4    = msg.GetGrid4();
    uint8_t powerDbm = msg.GetPowerDbm();

    // cout << callsign << " " << grid4 << " " << (int)powerDbm << endl;
    // cout << endl;

    // check encode worked as expected
    if (callsign != test.wspr.callsign)
    {
        cout << "callsign mismatch, expected " << test.wspr.callsign << ", but got " << callsign << endl;
        retVal = false;
    }
    if (grid4 != test.wspr.grid4)
    {
        cout << "grid4 mismatch, expected " << test.wspr.grid4 << ", but got " << grid4 << endl;
        retVal = false;
    }
    if (powerDbm != test.wspr.powerDbm)
    {
        cout << "powerDbm mismatch, expected " << (int)test.wspr.powerDbm << ", but got " << (int)powerDbm << endl;
        retVal = false;
    }

    // take a copy before reset for field values
    WsprMessageTelemetryExtendedCommon msgOld = msg;

    // reset
    msg.Reset();

    // Print("After Reset", msg, fieldNameList);

    msg.SetCallsign(callsign.c_str());
    msg.SetGrid4(grid4.c_str());
    msg.SetPowerDbm(powerDbm);

    // Print("After Type1", msg, fieldNameList);

    // decode
    msg.Decode();

    // cout << endl;
    // Print("After Decode", msg, fieldNameList);

    // compare decoded values to the values as they were going into Encode
    // this includes application fields and headers
    for (const auto &fieldName : fieldNameList)
    {
        double valOld = msgOld.Get(fieldName);

        double valNew = msg.Get(fieldName);

        if (valNew != valOld || valOld == NAN || valNew == NAN)
        {
            retVal = false;

            // cout << "Field " << fieldName << " decoded value " << valNew << " isn't the same as what went into encoding (" << valOld << ")" << endl;
        }
    }

    // cout << "DoTestEncodeDecode: " << retVal << endl;
    // cout << endl;

    return retVal;
}

bool TestEncodeDecode()
{
    bool retVal = true;

    vector<EncodeDecodeTest> testList = {
        {   "00", 0, {
            { "Altitude",      0, 21340,    20,     5000 },
            { "Temperature", -50,    39,     1,      -55 },
            { "Voltage",       3,     4.95,  0.05,  2.95 },
            { "Speed",         0,    82,     2,       -5 },
            { "GpsIsValid",    0,     1,     1,       -1 },
        },  { "000AAA", "EM21", 3 }},

        {   "11", 1, {
            { "Altitude",      0, 21340,    20,    15000 },
            { "Temperature", -50,    39,     1,       22 },
            { "Voltage",       3,     4.95,  0.05,  3.15 },
            { "Speed",         0,    82,     2,       40 },
            { "GpsIsValid",    0,     1,     1,        1 },
        },  { "1E1DEK", "IK93", 3 }},

        {   "Q2", 2, {
            { "Altitude",      0, 21340,    20,    22000 },
            { "Temperature", -50,    39,     1,       45 },
            { "Voltage",       3,     4.95,  0.05,  5.15 },
            { "Speed",         0,    82,     2,       95 },
            { "GpsIsValid",    0,     1,     1,        2 },
        },  { "QJ2CSA", "DO00", 53 }},
    };

    for (const auto &test : testList)
    {
        retVal &= DoEncodeDecodeTest(test);
    }

    cout << "TestEncodeDecode: " << retVal << endl;

    return retVal;
}

bool TestDecodeFailure()
{
    bool retVal = true;

    // known Basic Telemetry
    vector<Type1Data> wsprList = {
        { "1Y4PAS", "HK08", 10 },
        { "QX7DGS", "JQ97", 33 },
        { "0X2FDM", "MI65", 27 },
    };

    for (const auto &wspr : wsprList)
    {
        WsprMessageTelemetryExtendedCommon msg;

        msg.SetCallsign(wspr.callsign.c_str());
        msg.SetGrid4(wspr.grid4.c_str());
        msg.SetPowerDbm(wspr.powerDbm);

        retVal &= (msg.Decode() == false);
    }

    cout << "TestDecodeFailure: " << retVal << endl;

    return retVal;
}

bool TestBits()
{
    vector<InputTest<vector<FieldData>, bool>> testList = {
        {{
            }, true, "no fields",
        },
        {
            {
                { "F1", 0, 1, 1 },
            },
            true, "1 field, 1 bit",
        },
        {
            {
                { "F1", 0, 608212403, 1 },
            },
            true, "1 field, 29.179xxx bits (max val)",
        },
        {
            {
                { "F1", 0, 608212404, 1 },
            },
            false, "1 field, 29.180xxx bits (max val + 1)",
        },
        {
            {
                { "Altitude",      0, 21340,    20,    },
                { "Temperature", -50,    39,     1,    },
                { "Voltage",       3,     4.95,  0.05, },
                { "Speed",         0,    82,     2,    },
                { "GpsIsValid",    0,     1,     1,    },
            },
            true, "multiple fields that should fit",
        },
        {
            {
                { "F1", 0, 1, 1 },
                { "F2", 0, 1, 1 },
                { "F3", 0, 1, 1 },
                { "F4", 0, 1, 1 },
                { "F5", 0, 1, 1 },
                { "F6", 0, 1, 1 },
                { "F7", 0, 1, 1 },
                { "F8", 0, 1, 1 },
                { "F9", 0, 1, 1 },
                { "F10", 0, 1, 1 },
                { "F11", 0, 1, 1 },
                { "F12", 0, 1, 1 },
                { "F13", 0, 1, 1 },
                { "F14", 0, 1, 1 },
                { "F15", 0, 1, 1 },
                { "F16", 0, 1, 1 },
                { "F17", 0, 1, 1 },
                { "F18", 0, 1, 1 },
                { "F19", 0, 1, 1 },
                { "F20", 0, 1, 1 },
                { "F21", 0, 1, 1 },
                { "F22", 0, 1, 1 },
                { "F23", 0, 1, 1 },
                { "F24", 0, 1, 1 },
                { "F25", 0, 1, 1 },
                { "F26", 0, 1, 1 },
                { "F27", 0, 1, 1 },
                { "F28", 0, 1, 1 },
                { "F29", 0, 1, 1 },
            },
            true, "29 1-bit fields that should fit",
        },
        {
            {
                { "F1", 0, 1, 1 },
                { "F2", 0, 1, 1 },
                { "F3", 0, 1, 1 },
                { "F4", 0, 1, 1 },
                { "F5", 0, 1, 1 },
                { "F6", 0, 1, 1 },
                { "F7", 0, 1, 1 },
                { "F8", 0, 1, 1 },
                { "F9", 0, 1, 1 },
                { "F10", 0, 1, 1 },
                { "F11", 0, 1, 1 },
                { "F12", 0, 1, 1 },
                { "F13", 0, 1, 1 },
                { "F14", 0, 1, 1 },
                { "F15", 0, 1, 1 },
                { "F16", 0, 1, 1 },
                { "F17", 0, 1, 1 },
                { "F18", 0, 1, 1 },
                { "F19", 0, 1, 1 },
                { "F20", 0, 1, 1 },
                { "F21", 0, 1, 1 },
                { "F22", 0, 1, 1 },
                { "F23", 0, 1, 1 },
                { "F24", 0, 1, 1 },
                { "F25", 0, 1, 1 },
                { "F26", 0, 1, 1 },
                { "F27", 0, 1, 1 },
                { "F28", 0, 1, 1 },
                { "F29", 0, 1, 1 },
                { "F30", 0, 1, 1 },
            },
            false, "30 1-bit fields should not fit",
        },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryExtendedCommon msg;

        bool ok = true;

        for (const auto &fd : test.input)
        {
            ok &= msg.DefineField(fd.name, fd.lowValue, fd.highValue, fd.stepSize);
        }

        retVal &= CheckErr(test, ok);
    }

    cout << "TestBits: " << retVal << endl;

    return retVal;
}


bool TestReset()
{
    bool retVal = true;

    WsprMessageTelemetryExtendedCommon msg;

    msg.DefineField("ABC", 0, 1, 1);

    double valIn = 1;
    msg.Set("ABC", valIn);

    double valOut = msg.Get("ABC");

    msg.Reset();

    double valOutAfter = msg.Get("ABC");

    retVal = valIn  == valOut &&
             valOut != valOutAfter;

    cout << "TestReset: " << retVal << endl;

    return retVal;
}

bool TestResetEverything()
{
    bool retVal = true;

    WsprMessageTelemetryExtendedCommon msg;

    // define field, should be ok to set/get
    bool def1 = msg.DefineField("ABC", 0, 2, 1);

    // set should work
    double valIn = 1;
    bool set1 = msg.Set("ABC", valIn);

    // get should work
    double valOut = msg.Get("ABC");

    // should clobber all field defs and ability to set/get field values
    msg.ResetEverything();

    // try to get a field, should be NAN
    double valOutAfter = msg.Get("ABC");

    // try to set a field, should be false return value
    double valIn2Ok = msg.Set("ABC", 2);

    // 2nd def of ABC should work
    bool def2 = msg.DefineField("ABC", 2, 4, 2);

    // should be default value of lowValue = 2
    double get2 = msg.Get("ABC");

    // should work
    bool set2 = msg.Set("ABC", 4);

    // check assumptions
    retVal = 
        def1 == true &&
        set1 == true &&
        valOut == 1 &&
        std::isnan(valOutAfter) &&
        valIn2Ok == false &&
        def2 == true &&
        get2 == 2 &&
        set2 == true;

    cout << "TestResetEverything: " << retVal << endl;

    return retVal;
}

bool TestDefinePrecision()
{
    // particular test cases
    vector<InputTest<FieldData, bool>> testList = {
        { { "F1", 2.4, 4.35, 0.05 }, true, "" },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryExtendedCommon msg;

        bool ok = msg.DefineField(test.input.name, test.input.lowValue, test.input.highValue, test.input.stepSize);

        if (ok != test.expectedRetVal)
        {
            retVal = false;

            cout << "ERR: Got " << ok << ", but expected " << test.expectedRetVal << " (" << test.comment << ")" << endl;
            cout << "    Reason: " << msg.GetDefineFieldErr() << endl;
        }
    }

    // extreme
    double stepSize = 0.001;
    for (double lowValue = -5; lowValue <= 5; lowValue += stepSize)
    {
        for (double highValue = lowValue + 0.001; highValue <= 5; highValue += stepSize)
        {
            WsprMessageTelemetryExtendedCommon msg;

            bool ok = msg.DefineField("test", lowValue, highValue, stepSize);

            if (ok == false)
            {
                retVal = false;

                cout << "ERR: Got " << ok << ", but expected " << true << endl;
                cout << "    Reason: " << msg.GetDefineFieldErr() << endl;
                cout << "    " << lowValue << ", " << highValue << ", " << stepSize << endl;
                cout << endl;
            }
        }
    }

    cout << "TestDefinePrecision: " << retVal << endl;

    return retVal;
}

bool TestDefineSetGetField()
{
    vector<InputTest<vector<FieldData>, bool>> testList = {
        {   {
            },
            true, "no fields",
        },
        {
            {
                { "F1", 0, 1, 1 },
            },
            true, "1 field",
        },
        {
            {
                { "F1", 0, 1, 1 },
                { "F2", 0, 1, 1 },
            },
            true, "2 diff field",
        },
        {
            {
                { "F1", 0, 1, 1 },
                { "F2", 0, 1, 1 },
                { "F1", 0, 1, 1 },
            },
            false, "2 same name field with middle unique field",
        },
    };

    bool retVal = true;

    for (const auto &test : testList)
    {
        WsprMessageTelemetryExtendedCommon msg;

        bool ok = true;

        for (const auto &fd : test.input)
        {
            ok &= msg.DefineField(fd.name, fd.lowValue, fd.highValue, fd.stepSize);
        }

        retVal &= CheckErr(test, ok);
    }

    cout << "TestDefineSetGetField: " << retVal << endl;

    return retVal;
}

bool TestRawHeaderFields(WsprMessageTelemetryExtendedCommon<> &msg,
                         const vector<const char *>           &fieldNameCanSetList,
                         const vector<const char *>           &fieldNameCanNotSetList,
                         const vector<const char *>           &fieldNameCanGetList)
{
    // check behavior
    bool retVal = true;

    // settable fields are gettable
    for (const char *fieldName : fieldNameCanSetList)
    {
        // setting to 1 is within range for all fields, and isn't their default value
        double setVal = 1;
        bool setOk = msg.Set(fieldName, setVal);

        // should get 1 back
        double getVal = msg.Get(fieldName);

        bool ok = setVal == getVal && setOk == true;

        retVal &= ok;
    }

    // non-settable fields return errors
    for (const char *fieldName : fieldNameCanNotSetList)
    {
        // setting to 1 is within range for all fields, and isn't their default value
        double setVal = 1;

        // set should fail
        bool setOk = msg.Set(fieldName, setVal);

        bool ok = setOk == false;

        retVal &= ok;
    }

    // non-settable fields return errors
    for (const char *fieldName : fieldNameCanGetList)
    {
        // should be able to get a non-NAN value
        double val = msg.Get(fieldName);

        retVal &= (std::isnan(val) == false);
    }

    return retVal;
}

bool TestRawHeaderFieldsHdrTypeSettable()
{
    vector<const char *> fieldNameCanSetList = {
        "HdrType",
        "HdrSlot",
    };

    vector<const char *> fieldNameCanNotSetList = {
        "HdrTelemetryType",
        "HdrRESERVED",
    };

    vector<const char *> fieldNameCanGetList = {
        "HdrTelemetryType",
        "HdrRESERVED",
        "HdrType",
        "HdrSlot",
    };

    WsprMessageTelemetryExtendedCommon msg;

    bool retVal = true;

    retVal &= TestRawHeaderFields(msg,
                                  fieldNameCanSetList,
                                  fieldNameCanNotSetList,
                                  fieldNameCanGetList);

    cout << "TestRawHeaderFieldsHdrTypeSettable: " << retVal << endl;

    return retVal;
}

bool TestRawHeaderFieldsHdrTypeNotSettable()
{
    vector<const char *> fieldNameCanSetList = {
        "HdrSlot",
    };

    vector<const char *> fieldNameCanNotSetList = {
        "HdrTelemetryType",
        "HdrRESERVED",
        "HdrType",
    };

    vector<const char *> fieldNameCanGetList = {
        "HdrTelemetryType",
        "HdrRESERVED",
        "HdrType",
        "HdrSlot",
    };

    WsprMessageTelemetryExtendedCommon msg;

    msg.SetCanSetHdrType(false);

    bool retVal = true;

    retVal &= TestRawHeaderFields(msg,
                                  fieldNameCanSetList,
                                  fieldNameCanNotSetList,
                                  fieldNameCanGetList);

    cout << "TestRawHeaderFieldsHdrTypeNotSettable: " << retVal << endl;

    return retVal;
}

bool TestNamedHeaderFields()
{
    bool retVal = true;

    WsprMessageTelemetryExtendedCommon msg;

    uint8_t getHdrTelemetryType = msg.GetHdrTelemetryType();
    uint8_t getHdrRESERVED = msg.GetHdrRESERVED();

    msg.SetHdrSlot(1);
    uint8_t getHdrSlot = msg.GetHdrSlot();

    WsprMessageTelemetryExtendedCommon<>::HdrType getHdrType = msg.GetHdrType();

    // check assumptions
    retVal =
        getHdrTelemetryType == 0 &&
        getHdrRESERVED == 0 &&
        getHdrSlot == 1 &&
        getHdrType == WsprMessageTelemetryExtendedCommon<>::HdrType::USER_DEFINED;

    cout << "TestNamedHeaderFields: " << retVal << endl;

    return retVal;
}

bool TestGetFieldDefList()
{
    bool retVal = true;

    {
        WsprMessageTelemetryExtendedCommon<> msg;

        retVal &= msg.GetFieldDefListLen() == 0;
    }

    {
        WsprMessageTelemetryExtendedCommon<> msg;

        msg.DefineField("ABC", 0, 1, 1);

        retVal &= msg.GetFieldDefListLen() == 1;
    }

    {
        vector<string> fieldList = {
            "ABC",
            "DEF",
            "GHI",
        };

        WsprMessageTelemetryExtendedCommon<> msg;

        for (const auto & field : fieldList)
        {
            msg.DefineField(field.c_str(), 0, 1, 1);
        }

        retVal &= msg.GetFieldDefListLen() == fieldList.size();

        auto fieldDefList = msg.GetFieldDefList();

        for (int i = 0; i < msg.GetFieldDefListLen(); ++i)
        {
            retVal &= fieldList[i] == fieldDefList[i].name;
        }
    }

    cout << "TestGetFieldDefList: " << retVal << endl;

    return retVal;
}


int main()
{
    bool retVal = true;

    retVal &= TestSetClamp();
    retVal &= TestEncodeDecode();
    retVal &= TestDecodeFailure();
    retVal &= TestBits();
    retVal &= TestReset();
    retVal &= TestResetEverything();
    retVal &= TestDefinePrecision();
    retVal &= TestDefineSetGetField();
    retVal &= TestRawHeaderFieldsHdrTypeSettable();
    retVal &= TestRawHeaderFieldsHdrTypeNotSettable();
    retVal &= TestNamedHeaderFields();
    retVal &= TestGetFieldDefList();

    return !retVal;
}