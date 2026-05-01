#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.
    WsprMessageTelemetryExtendedUserDefined<5> codecGpsMsg;

    /////////////////////////////////////////////////////////////////
    // Define telemetry fields
    /////////////////////////////////////////////////////////////////

    // Define counts of GPS satellites for each constellation type.
    // Values will be clamped between 0 - 128 inclusive.
    // Resolution will be in increments of 4.
    codecGpsMsg.DefineField("SatCountUSA",    0, 128, 4);
    codecGpsMsg.DefineField("SatCountChina",  0, 128, 4);
    codecGpsMsg.DefineField("SatCountRussia", 0, 128, 4);

    // Define a metric for GPS lock times, in seconds.
    // Values will be clamped between 0 - 30 inclusive.
    // Resolution will be in increments of 0.5.
    codecGpsMsg.DefineField("LockTimeSecs",    0, 30, 0.5);
    codecGpsMsg.DefineField("LockTimeSecsAvg", 0, 30, 0.5);


    /////////////////////////////////////////////////////////////////
    // Get encoded WSPR message fields (sourced elsewhere)
    /////////////////////////////////////////////////////////////////

    codecGpsMsg.SetCallsign("036KVF");
    codecGpsMsg.SetGrid4("PP73");
    codecGpsMsg.SetPowerDbm(30);


    /////////////////////////////////////////////////////////////////
    // Decode
    /////////////////////////////////////////////////////////////////

    codecGpsMsg.Decode();


    /////////////////////////////////////////////////////////////////
    // Extract the now-decoded WSPR message fields
    /////////////////////////////////////////////////////////////////

    int satCountUsa    = codecGpsMsg.Get("SatCountUSA");
    int satCountChina  = codecGpsMsg.Get("SatCountChina");
    int satCountRussia = codecGpsMsg.Get("SatCountRussia");

    double lockTimeSecs    = codecGpsMsg.Get("LockTimeSecs");
    double lockTimeSecsAvg = codecGpsMsg.Get("LockTimeSecsAvg");

    cout << "Encoded data"                         << endl;
    cout << "------------"                         << endl;
    cout << "satCountUsa    : " << satCountUsa     << endl; // 12
    cout << "satCountChina  : " << satCountChina   << endl; // 12
    cout << "satCountRussia : " << satCountRussia  << endl; // 0
    cout << "lockTimeSecs   : " << lockTimeSecs    << endl; // 10.5
    cout << "lockTimeSecsAvg: " << lockTimeSecsAvg << endl; // 13

    return 0;
}