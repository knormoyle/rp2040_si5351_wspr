#include <iostream>
using namespace std;

#include "WsprMessageTelemetryExtendedVendorDefined.h"


bool TestHdrTypeIsRestricted()
{
    bool retVal = true;

    WsprMessageTelemetryExtendedVendorDefined msg;

    bool ok = msg.Set("HdrType", 1);

    retVal &= (ok == false);

    cout << "TestHdrTypeIsRestricted: " << retVal << endl;

    return retVal;
}

bool TestBadDecode()
{
    // 000AAA AA06 20   // good
    // 000AAA AA06 27   // same input data, but HdrType changed type to 1

    WsprMessageTelemetryExtendedVendorDefined msg1;
    msg1.SetCallsign("000AAA");
    msg1.SetGrid4("AA06");
    msg1.SetPowerDbm(20);
    bool msg1Decode = msg1.Decode();
    
    WsprMessageTelemetryExtendedVendorDefined msg2;
    msg2.SetCallsign("000AAA");
    msg2.SetGrid4("AA00");
    msg2.SetPowerDbm(27);
    bool msg2Decode = msg2.Decode();

    bool retVal =
        msg1Decode == true &&
        msg2Decode == false;

    cout << "TestBadDecode: " << retVal << endl;


    return retVal;
}

bool TestResetEverything()
{
    WsprMessageTelemetryExtendedVendorDefined msg;

    msg.Encode();

    string  callsign = msg.GetCallsign();
    string  grid4    = msg.GetGrid4();
    uint8_t powerDbm = msg.GetPowerDbm();


    msg.ResetEverything();
    msg.Encode();
    

    bool retVal = callsign == msg.GetCallsign() &&
                  grid4    == msg.GetGrid4()    &&
                  powerDbm == msg.GetPowerDbm();

    cout << "TestResetEverything: " << retVal;

    return retVal;
}


int main()
{
    bool retVal = true;

    retVal &= TestHdrTypeIsRestricted();
    retVal &= TestBadDecode();
    retVal &= TestResetEverything();

    return !retVal;
}