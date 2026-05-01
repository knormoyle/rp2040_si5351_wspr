// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include <WsprEncoded_CT.h>

#include "led_functions.h"
#include "debug_functions.h"
#include "print_functions.h"
#include "ct_functions.h"
#include "global_structs.h"

extern TeleStruct tt;
extern ConfigStruct cc;
extern bool VERBY[10];

WsprMessageTelemetryBasic msg;
// arg is the number of fields allowed
WsprMessageTelemetryExtendedUserDefined bmp_msg(3);
WsprMessageTelemetryExtendedUserDefined gps_msg(4);

// FIX! currently not using
void encodeBasicTele(char *hf_callsign, char *hf_grid4, char *hf_power,
    const char *id13, const char *grid56,
    uint32_t altitudeMeters, int8_t temperatureCelsius,
    double voltageVolts, uint8_t speedKnots, bool gps_msgIsValid) {

    V1_print(F("encodeBasicTele START" EOL));

    // Set the telemetry fields
    msg.SetGrid56(grid56);
    msg.SetAltitudeMeters(altitudeMeters);
    msg.SetTemperatureCelsius(temperatureCelsius);
    msg.SetVoltageVolts(voltageVolts);
    msg.SetSpeedKnots(speedKnots);
    msg.SetGpsIsValid(gps_msgIsValid);
    msg.SetId13(id13);

    V1_printf("id13 %s %s" EOL, id13, msg.GetId13());
    V1_printf("grid56 %s %s" EOL, grid56, msg.GetGrid56());
    V1_printf("altM %lu %d" EOL, altitudeMeters, msg.GetAltitudeMeters());
    V1_printf("tempC %u %u" EOL, temperatureCelsius, msg.GetTemperatureCelsius());
    V1_printf("voltage %.2f %.2f" EOL, voltageVolts, msg.GetVoltageVolts());
    V1_printf("speed %u %u" EOL, speedKnots, msg.GetSpeedKnots());
    V1_printf("gps_msgValid %u %u" EOL, gps_msgIsValid, msg.GetGpsIsValid());

    msg.Encode();

    const char *GetCallsign = msg.GetCallsign();
    const char *GetGrid4 = msg.GetGrid4();
    uint8_t GetPowerDbm = msg.GetPowerDbm();

    V1_printf("callsign: %s" EOL, GetCallsign);
    V1_printf("grid4: %s" EOL, GetGrid4);
    V1_printf("pwrDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encodeBasicTele END" EOL));
}

//***************************************************************
void define_gps_msg() {
    V1_print(F("define_gps_msg() START" EOL));

    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.

    // Define telemetry fields

    // Define counts of GPS satellites for each constellation type.
    // Values will be clamped between 0 - 100 inclusive.
    // Resolution will be in increments of 1.

    // Define a metric for GPS lock times, in seconds.
    // Values will be clamped between 0 - 180 inclusive.
    // Resolution will be in increments of 1.

    // FIX! does this go in order of "field 0" to "field N" for internal state?
    // "field 0" will be packed first ? or packed last to keep it on the "lsb side"
    bool accepted;
    accepted = gps_msg.DefineField("SatCountUSA", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: gps_msg.DefineField('SatCountUSA', 0, 40, 1) not accepted"));
    }
    accepted = gps_msg.DefineField("SatCountChina", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: gps_msg.DefineField('SatCountChina', 0, 40, 1) not accepted"));
    }
    accepted = gps_msg.DefineField("SatCountRussia", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: gps_msg.DefineField('SatCountRussia', 0, 40, 1) not accepted"));
    }
    accepted = gps_msg.DefineField("LockTimeSecs", 0, 600, 10);
    if (!accepted) {
        V1_println(F("ERROR: gps_msg.DefineField('LockTimeSecs', 0, 600, 10) not accepted"));
    }

    // for use in the traquito website
    // FIX! add wsprtv configurator info
    /* JSON
    { "name": "SatUSA",       "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
    { "name": "SatChina",     "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
    { "name": "SatRussia",    "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
    { "name": "LockTimeSecs",  "unit": "Seconds", "lowValue": 0, "highValue": 600, "stepSize": 10},
    */
    /*
    */

    // Returns true if field is accepted
    // Returns false if field is rejected
    //
    // A field will be rejected if:
    // - template-specified number of fields have already been configured
    // - field name is a nullptr
    // - field already exists
    // - lowValue, highValue, or stepSize is too precise (more than 3 decimal places of precision)
    // - lowValue >= highValue
    // - stepSize <= 0
    // - stepSize does not evenly divide the range between lowValue and highValue
    // - field size exceeds total capacity of 29.180 bits along with other fields, or by itself

    // how to form url
    // https://traquito.github.io/copilot/dashboard/#overview
    // FIX! add wsprtv url

    V1_print(F("define_gps_msg() END" EOL));
}

//***************************************************************
void encode_gps_msg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot) {
    V1_print(F("encode_gps_msg START" EOL));
    switch (slot) {
        case 0: {;}
        case 1: {;}
        case 2: {;}
        case 3: {;}
        case 4: break;
        default:
            // count 0,1,2,3,4 here
            V1_printf("ERROR: encode_gps_msg illegal slot %u ..using 2" EOL, slot);
            slot = 2;
    }

    // const char *band = "20m";
    // uint16_t channel = 123;
    char band[4];
    // append 'm'
    snprintf(band, sizeof(band), "%sm", cc._Band);
    uint16_t channel = atoi(cc._U4B_chan);

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    // Encode the data in preparation to transmit
    bool accepted = true;
    gps_msg.SetId13(cd.id13);
    // Note which transmission slot you will send in
    gps_msg.SetHdrSlot(slot);

    accepted &= gps_msg.Set("SatCountUSA", atoi(tt.gp_sats));
    accepted &= gps_msg.Set("SatCountChina", atoi(tt.gb_sats));
    accepted &= gps_msg.Set("SatCountRussia", atoi(tt.gl_sats));
    accepted &= gps_msg.Set("LockTimeSecs", atoi(tt.gpsLockSecs));
    if (!accepted) {
        V1_println("ERROR: Something didn't get accepted in gps_msg.Encode(). missing defines?");
    }
    gps_msg.Encode();

    const char *GetCallsign = gps_msg.GetCallsign();
    const char *GetGrid4 = gps_msg.GetGrid4();
    uint8_t GetPowerDbm = (uint8_t) gps_msg.GetPowerDbm();

    V1_print(F("WsprEncode encoded data:" EOL));
    V1_printf("GetCallsign: %s" EOL, GetCallsign);
    V1_printf("GetGrid4: %s" EOL, GetGrid4);
    V1_printf("GetPowerDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // https://stackoverflow.com/questions/77869711/returning-a-char-pointer-when-the-argument-is-a-constant-char-pointer

    // wants to be char array instead of uint8_t?
    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encode_gps_msg END" EOL));
}

//***************************************************************

void define_bmp_msg() {
    V1_print(F("define_bmp_msg() START" EOL));
    bool accepted;

    accepted = bmp_msg.DefineField("Pressure", 0.0, 80000.0, 5);
    if (!accepted) {
        V1_println(F("ERROR: bmp_msg.DefineField('Pressure', 0.0, 80000.0, 5) not accepted"));
    }

    accepted = bmp_msg.DefineField("Temperature", -60, 100, 2.5);
    if (!accepted) {
        V1_println(F("ERROR: bmp_msg.DefineField('Temperature', -60, 100, 2.5) not accepted"));
    }

    accepted = bmp_msg.DefineField("Altitude", 0, 55000, 100);
    if (!accepted) {
        V1_println(F("ERROR: bmp_msg.DefineField('Altitude', 0, 55000, 100) not accepted"));
    }

    // good expected values here
    // https://www.mide.com/air-pressure-at-altitude-calculator

    // size fields at sandbox https://traquito.github.io/pro/codec
    // programmed channel 391. comes out on 384
    /* JSON
    { "name": "Pressure",    "unit": "Pa", "lowValue": 0,   "highValue": 80000, "stepSize": 5 },
    { "name": "Temperature", "unit": "C",  "lowValue": -60, "highValue": 100,    "stepSize": 2.5 },
    { "name": "Altitude",    "unit": "M",  "lowValue": 0,   "highValue": 55000,  "stepSize": 100 },
    */

    // how to form url
    // https://traquito.github.io/copilot/dashboard/#overview

    V1_print(F("define_bmp_msg() END" EOL));
}

//***************************************************************
void encode_bmp_msg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot) {
    V1_print(F("encode_bmp_msg START" EOL));
    switch (slot) {
        case 0: {;}
        case 1: {;}
        case 2: {;}
        case 3: {;}
        case 4: break;
        default:
            // count 0,1,2,3,4 here
            V1_printf("ERROR: encode_bmp_msg illegal slot %u ..using 3" EOL, slot);
            slot = 3;
    }

    // const char *band = "20m";
    // uint16_t channel = 123;
    char band[4];
    // append 'm'
    snprintf(band, sizeof(band), "%sm", cc._Band);
    uint16_t channel = atoi(cc._U4B_chan);
    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    bool accepted = true;
    // Encode the data in preparation to transmit
    bmp_msg.SetId13(cd.id13);
    // Note which transmission slot you will send in
    bmp_msg.SetHdrSlot(slot);

    accepted &= bmp_msg.Set("Pressure", atof(tt.bmp_pressure));
    V1_printf("atof(tt.bmp_msg_pressure %f" EOL, atof(tt.bmp_pressure));
    accepted &= bmp_msg.Set("Temperature", atof(tt.bmp_temperature));
    V1_printf("atof(tt.bmp_msg_temperature %f" EOL, atof(tt.bmp_temperature));
    accepted &= bmp_msg.Set("Altitude", atof(tt.bmp_altitude));
    V1_printf("atof(tt.bmp_msg_altitude %f" EOL, atof(tt.bmp_altitude));

    bmp_msg.Encode();
    if (!accepted) {
        V1_println("Something didn't get accepted in encode_bmp_msg(). missing defines?");
    }

    const char *GetCallsign = bmp_msg.GetCallsign();
    const char *GetGrid4 = bmp_msg.GetGrid4();
    uint8_t GetPowerDbm = (uint8_t) bmp_msg.GetPowerDbm();

    V1_print(F("WsprEncode encoded data:" EOL));
    V1_printf("GetCallsign: %s" EOL, GetCallsign);
    V1_printf("GetGrid4: %s" EOL, GetGrid4);
    V1_printf("GetPowerDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // https://stackoverflow.com/questions/77869711/returning-a-char-pointer-when-the-argument-is-a-constant-char-pointer

    // wants to be char array instead of uint8_t?
    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encode_bmp_msg END" EOL));
}
