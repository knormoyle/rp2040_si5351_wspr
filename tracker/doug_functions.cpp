// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "led_functions.h"
#include "debug_functions.h"
#include "print_functions.h"
#include <Adafruit_SleepyDog.h>
#include "WsprEncoded.h"

// #include <cstdint>

extern char _Band[3];
extern char _U4B_chan[4];
extern bool VERBY[10];

WsprMessageTelemetryBasic msg;

void encodeBasicTele(const char *id13, const char *grid56, uint32_t altitudeMeters, int8_t temperatureCelsius, double voltageVolts, uint8_t speedKnots, bool gpsIsValid) {
    V1_print(F("encodeBasicTele START" EOL));

    // Set the telemetry fields
    msg.SetGrid56(grid56);
    msg.SetAltitudeMeters(altitudeMeters);
    msg.SetTemperatureCelsius(temperatureCelsius);
    msg.SetVoltageVolts(voltageVolts);
    msg.SetSpeedKnots(speedKnots);
    msg.SetGpsIsValid(gpsIsValid);
    msg.SetId13(id13);

    // Report the parameters passed, and if they got automatically clamped
    V1_printf("id13 %s %s" EOL, id13, msg.GetId13());
    V1_printf("grid56 %s %s" EOL, grid56, msg.GetGrid56());
    V1_printf("altM %lu %d" EOL, altitudeMeters, msg.GetAltitudeMeters());
    V1_printf("tempC %u %u" EOL, temperatureCelsius, msg.GetTemperatureCelsius());
    V1_printf("voltage %.2f %.2f" EOL, voltageVolts, msg.GetVoltageVolts());
    V1_printf("speed %u %u" EOL, speedKnots, msg.GetSpeedKnots());
    V1_printf("gpsValid %u %u" EOL, gpsIsValid, msg.GetGpsIsValid());

    msg.Encode();

    // Extract the WSPR Type1 Message fields from the encoder
    const char *callsign = msg.GetCallsign();
    const char *grid4 = msg.GetGrid4();
    uint8_t powerDbm = msg.GetPowerDbm();

    // Report what the Type1 Message fields are
    V1_printf("callsign: %s" EOL, callsign);
    V1_printf("grid4: %s" EOL, grid4);
    V1_printf("pwrDbm: %u" EOL, powerDbm);

    V1_print(F("encodeBasicTele END" EOL));
}

//***************************************************************
WsprMessageTelemetryExtendedUserDefined<5> codecGpsMsg;
void defineExtendedUserTele(uint8_t slot) {
    V1_print(F("defineExtendedUserTele() START" EOL));


    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.

    // Define telemetry fields

    // Define counts of GPS satellites for each constellation type.
    // Values will be clamped between 0 - 100 inclusive.
    // Resolution will be in increments of 1.
    codecGpsMsg.DefineField("SatCountUSA", 0, 100, 1);
    codecGpsMsg.DefineField("SatCountChina", 0, 100, 1);
    codecGpsMsg.DefineField("SatCountRussia", 0, 100, 1);

    // Define a metric for GPS lock times, in seconds.
    // Values will be clamped between 0 - 180 inclusive.
    // Resolution will be in increments of 1.
    codecGpsMsg.DefineField("LockTimeSecs", 0, 180, 1);
    codecGpsMsg.DefineField("LockTimeSecsAvg", 0, 180, 1);


    V1_print(F("defineExtendedUserTele() END" EOL));
}

//***************************************************************
// FIX! have to add parameters to encode?
void encodeExtendedUserTele(char *hf_callsign, char *hf_grid4, uint8_t *hf_power, uint8_t slot) {

    V1_print(F("encodeExtendedUserTele() START" EOL));
    switch(slot) {
        case 0: {;}
        case 2: {;}
        case 4: {;}
        case 6: {;}
        case 8: break;
        default:
            V1_printf("ERROR: encodeExtendedUserTele illegal slot %u ..using 4" EOL, slot);
            slot = 4;
    }

    // const char *band = "20m";
    // uint16_t channel = 123;
    char band[4];
    // append 'm'
    snprintf(band, sizeof(band), "%sm", _Band);
    uint16_t channel = atoi(_U4B_chan);

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    // Encode the data in preparation to transmit
    codecGpsMsg.SetId13(cd.id13);
    // Note which transmission slot you will send in
    codecGpsMsg.SetHdrSlot(slot);

    if (true) {
        codecGpsMsg.Set("SatCountUSA", 12);
        codecGpsMsg.Set("SatCountChina", 10);
        codecGpsMsg.Set("SatCountRussia", 0);
        codecGpsMsg.Set("LockTimeSecs", 10.74);
        codecGpsMsg.Set("LockTimeSecsAvg", 12.76);
    }

    codecGpsMsg.Encode();

    // Extract the now-encoded WSPR message fields
    const char *GetCallsign = codecGpsMsg.GetCallsign();
    const char *GetGrid4 = codecGpsMsg.GetGrid4();
    uint8_t GetPowerDbm = (uint8_t) codecGpsMsg.GetPowerDbm();

    V1_print(F("WsprEncode encoded data:" EOL));
    V1_printf("GetCallsign: %s" EOL, GetCallsign);
    V1_printf("GetGrid4: %s" EOL, GetGrid4);
    V1_printf("GetPowerDbm: %u" EOL, GetPowerDbm);

    strncpy(hf_callsign, GetCallsign, sizeof(hf_callsign) - 1);
    strncpy(hf_grid4, GetGrid4, sizeof(hf_grid4) - 1);
    *hf_power = GetPowerDbm;

    V1_print(F("encodeExtendedUserTele() END" EOL));
}
