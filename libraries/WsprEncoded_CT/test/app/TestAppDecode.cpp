#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main(int argc, char *argv[])
{
    int retVal = 0;

    if (argc != 4)
    {
        cout << "Usage: " << argv[0] << " <callsign> <grid4> <powerDbm>" << endl;

        retVal = 1;
    }
    else
    {
        // Create Basic Telemetry object
        WsprMessageTelemetryBasic tb;

        // Set encoded fields
        tb.SetCallsign(argv[1]);
        tb.SetGrid4(argv[2]);
        tb.SetPowerDbm(atoi(argv[3]));

        // Decode the data
        tb.Decode();

        // Extract the decoded telemetry
        cout << "Decoded data" << endl;
        cout << "------------" << endl;
        cout << "Grid56            : "<< tb.GetGrid56()                  << endl;
        cout << "AltitudeMeters    : "<< tb.GetAltitudeMeters()          << endl;
        cout << "TemperatureCelsius: "<< (int)tb.GetTemperatureCelsius() << endl;
        cout << "VoltageVolts      : "<< tb.GetVoltageVolts()            << endl;
        cout << "SpeedKnots        : "<< (int)tb.GetSpeedKnots()         << endl;
        cout << "GpsIsValid        : "<< tb.GetGpsIsValid()              << endl;
    }

    return retVal;
}