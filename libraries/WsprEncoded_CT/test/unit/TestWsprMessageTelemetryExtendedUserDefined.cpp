#include <iostream>
using namespace std;

#include "WsprMessageTelemetryExtendedUserDefined.h"


bool TestHdrTypeIsRestricted()
{
    bool retVal = true;

    WsprMessageTelemetryExtendedUserDefined msg;

    bool ok = msg.Set("HdrType", 1);

    retVal &= (ok == false);

    cout << "TestHdrTypeIsRestricted: " << retVal << endl;

    return retVal;
}

bool TestBadDecode()
{
    // 000AAA EM21 3    // good
    // 000AAA EM21 30   // same input data, but type changed type to 1

    WsprMessageTelemetryExtendedUserDefined msg1;
    msg1.SetCallsign("000AAA");
    msg1.SetGrid4("EM21");
    msg1.SetPowerDbm(3);
    bool msg1Decode = msg1.Decode();
    
    WsprMessageTelemetryExtendedUserDefined msg2;
    msg2.SetCallsign("000AAA");
    msg2.SetGrid4("EM21");
    msg2.SetPowerDbm(30);
    bool msg2Decode = msg2.Decode();

    bool retVal =
        msg1Decode == true &&
        msg2Decode == false;

    cout << "TestBadDecode: " << retVal << endl;


    return retVal;
}

bool TestResetEverything()
{
    WsprMessageTelemetryExtendedUserDefined msg;

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