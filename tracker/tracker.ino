#include <Arduino.h>

// BOth in arduino-pico core?  yes, see https://github.com/earlephilhower/arduino-pico/tree/master/libraries
#include <SPI.h>
#include <Wire.h>

#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include <TinyGPS++.h>                      //https://github.com/mikalhart/TinyGPSPlus


// gets the head 1.1-beta? not release
// wget https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip

/* v1.0.a current A new, customizable Arduino NMEA parsing library A NEW Full-featured GPS/NMEA Parser for Arduino TinyGPSPlus is a new Arduino library for parsing NMEA data streams provided by GPS modules.

1.1-beta update: Several pull requests incorporated (or equiv)

Added Fix Quality and Fix Mode
Fix stringop truncation warning
Support for non-Arduino platforms
Slight change to earth radius
Support all satellite groups
Like its predecessor, TinyGPS, this library provides compact and easy-to-use methods for extracting position, date, time, altitude, speed, and course from consumer GPS devices.

However, TinyGPSPlus’s programmer interface is considerably simpler to use than TinyGPS, and the new library can extract arbitrary data from any of the myriad NMEA sentences out there, even proprietary ones.
*/

// got latest from https://github.com/kaduhi/LightAPRS-W-2.0
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence
// libraries/LightAPRS_Geofence:
// wget https://raw.githubusercontent.com/kaduhi/LightAPRS-W-2.0/refs/heads/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence/GEOFENCE.cpp
// wget https://raw.githubusercontent.com/kaduhi/LightAPRS-W-2.0/refs/heads/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence/GEOFENCE.h
// Modified version of https://github.com/TomasTT7/TT7F-Float-Tracker/blob/master/Software/ARM_GEOFENCE.c
// was used in https://github.com/TomasTT7/TT7F-Float-Tracker
// #include <GEOFENCE.h> 

// this is included in gps_functions.cpp? shouldn't be needed here or maybe need for Watchdog.* ?
// in libraries: wget https://github.com/adafruit/Adafruit_SleepyDog/archive/refs/heads/master.zip
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog

// needed for BMP085.h Uses Adafruit_I2CDevice.h and .cpp
// added Adafruit_BusIO to our repo 11_7_24 (libraries)
// in libraries: wget https://github.com/adafruit/Adafruit_BusIO/archive/refs/heads/master.zip
#include <Adafruit_I2CDevice.h> // https://github.com/adafruit/Adafruit_BusIO

// board uses the BMP280? (doesn't have temp)
// in libraries: wget https://github.com/adafruit/Adafruit_BMP280_Library/archive/refs/heads/master.zip
// can I put the BMP085 on there?
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library
// this needs the Adafruit_Sensor.h also
// in libraries: wget https://github.com/adafruit/Adafruit_Sensor/archive/refs/heads/master.zip
// https://github.com/adafruit/Adafruit_sensor
// they have accelerometer, gyroscope, magnetometer, humidity sensors, light?

// old
// Requires the https://github.com/adafruit/Adafruit_BusIO library for I2C abstraction
// #include <Adafruit_BMP085.h> // https://github.com/adafruit/Adafruit-BMP085-Library

// wget https://github.com/adafruit/Adafruit-BMP085-Library/archive/refs/heads/master.zip
#include <JTEncode.h> // https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
// in libraries: wget https://github.com/etherkit/JTEncode/archive/refs/heads/master.zip

// not used here any more. 
// setTime() use moved to gps_functions.cpp
// libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// do we use minute() here?
#include <TimeLib.h> //https://github.com/PaulStoffregen/Time

#include <MemoryFree.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
#include "hardware/pwm.h"

// lots of things used from Arduino-Pico core
// RP2040 Helper Class.
// https://arduino-pico.readthedocs.io/en/latest/rp2040.html

// const is typed, #define macros are not.
// const is scoped by C block, #define applies to a file (compilation unit)
// const is most useful with parameter passing.
// If you see const used on a prototype with pointers, you know it is safe to pass your array or struct because the function will not alter it. No const and it can.
// example: strcpy()
// Apply "const-ness" to function prototypes at the outset.

#define SerialUSB   Serial

const int Si5351Pwr=4;
const int BattPin=A3;


//macros
// FIX! does nothing?
#define Si5351ON

// used
#define Si5351OFF   vfo_turn_off()


//******************************  CONFIG **********************************
char    CallSign[7]="NOCALL";//DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=11; //11; //SSID http://www.aprs.org/aprs11/SSIDs.txt
// FIX! only have 46 bytes? (was 50)
char    comment[46] = "tracker 1.0";// Max 50 char
char    StatusMessage[50] = "tracker 1.0";

//*****************************************************************************

uint16_t  BeaconWait=50;  //seconds sleep for next beacon (HF or VHF). Optimized value, do not change this if possible.

uint16_t  BattWait=1;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel)
float     BattMin=0.0;    //min Volts to wake up.
float     GpsMinVolt=0.0; //min Volts for GPS to wake up. (important if power source is solar panel)
float     WsprBattMin=0.0;//min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
float     HighVolt=9.9;   //GPS is always on if the voltage exceeds this value to protect solar caps from overcharge

//******************************  HF (WSPR) CONFIG *************************************

char hf_call[7] = "NOCALL";// DO NOT FORGET TO CHANGE YOUR CALLSIGN

// const unsigned long WSPR_DEFAULT_FREQ=10140200UL; //30m band
const unsigned long WSPR_DEFAULT_FREQ=14097100UL; //20m band
//const unsigned long WSPR_DEFAULT_FREQ=18106100UL; //17M band
//const unsigned long WSPR_DEFAULT_FREQ=21096100UL; //15m band
//const unsigned long WSPR_DEFAULT_FREQ=24926100UL; //12M band
//const unsigned long WSPR_DEFAULT_FREQ=28126100UL; //10m band
//for all bands -> http://wsprnet.org/drupal/node/7352

// Supported modes, default HF mode is WSPR
enum mode {MODE_WSPR};
enum mode cur_mode = MODE_WSPR; //default HF mode

// #define DEVMODE // Development mode. Uncomment to enable for debugging.
boolean DEVMODE = true;
//*******************************************************************************
//******************************  APRS SETTINGS (old)****************************

boolean  aliveStatus = true; //for tx status message on first wake-up just once.
static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1; //increase +1 after every APRS transmission

//*******************************************************************************
//******************************  HF SETTINGS   *********************************

const int WSPR_TONE_SPACING=146; // ~1.46 Hz
const int WSPR_DELAY=683;        // Delay value for WSPR

// int HF_CORRECTION=-13000  // Change this for your ref osc
int HF_CORRECTION=0; 

// Global variables
unsigned long hf_freq;
char hf_message[13] = "NOCALL AA00";//for WSPR, updated by hf_call and GPS location
char hf_loc[] = "AA00";             //for WSPR, updated by GPS location. You don't have to change this.
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
volatile bool proceed = false;

//*******************************************************************************
//******************************  GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed

// FIX! removed all geofence, not used for wspr

boolean GpsFirstFix=false; //do not change this
boolean ublox_high_alt_mode_enabled = false; //do not change this
int16_t GpsInvalidTime=0; //do not change this

// gps_functions.cpp refers to this
TinyGPSPlus gps;

#include "debug_functions.h"

//*********************************
// in AdaFruit_I2CDevice.h
// theWire The I2C bus to use, defaults to &Wire
const int BMP280_I2C1_SDA_PIN=2;
const int BMP280_I2C1_SCL_PIN=3;
#include "bmp_functions.h"

Adafruit_BMP280 bmp;
// Adafruit_BMP805 bmp;

JTEncode jtencode;

//*********************************
extern const int STATUS_LED_PIN=25;

extern const int LED_STATUS_NO_GPS=1;
extern const int LED_STATUS_GPS_TIME=2;
extern const int LED_STATUS_GPS_FIX=3;
extern const int LED_STATUS_TX_WSPR=4;
extern const int LED_STATUS_TX_TELEMETRY=5;
extern const int LED_STATUS_TX_TELEN1=6;
extern const int LED_STATUS_TX_TELEN2=7;

#include "led_functions.h"

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// so it can be used in gps_functions.cpp
// extern is needed or the linker doesn't find it. see https://forum.arduino.cc/t/linker-problems-with-extern-const-struct/647136/2
extern const int GpsPwr=16;
// not used..GpsPwr is used
// const int GPS_VCC_ON_N_PIN=16;     // output ..this cuts VCC, leaves VBAT

// FIX! where are these used
extern const int GPS_NRESET_PIN=5; // output ..active low..should be 1?
extern const int GPS_ON_PIN=6;     // output ..this puts in sleep mode? should be 1?

// FIX! where is this used (calibration maybe)
extern const int GPS_1PPS_PIN=17;         // input

extern const int GPS_UART1_TX_PIN=8;
extern const int GPS_UART1_RX_PIN=9;

// stuff moved to functions from this .ino (not libraries)
#include "gps_functions.h"



//*********************************
// when we set both?
extern const int WSPR_TX_CLK_1_NUM=1;
// this is the other differential clock for wspr? (was aprs)
extern const int WSPR_TX_CLK_0_NUM=0;
extern const int WSPR_TX_CLK_NUM=0;


//*********************************
extern const int SI5351A_CLK_IDRV_8MA=(3 << 0);
extern const int SI5351A_CLK_IDRV_6MA=(2 << 0);
extern const int SI5351A_CLK_IDRV_4MA=(1 << 0);
extern const int SI5351A_CLK_IDRV_2MA=(0 << 0);

extern const int PLL_CALCULATION_PRECISION=4;

extern const int VFO_VDD_ON_N_PIN=4;
extern const int VFO_I2C0_SDA_PIN=12;
extern const int VFO_I2C0_SCL_PIN=13;
#include "si5351_functions.h"
//*********************************

void setup() {
  Watchdog.enable(30000);
  Watchdog.reset();
  // While the energy rises slowly with the solar panel,
  // using the analog reference low solves the analog measurement errors.

  initStatusLED();
  setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
  // FIX! why is this commented out?
  // pinMode(Si5351Pwr, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  // FIX! why is this commented out?
  // pinMode(BattPin, INPUT);
  analogReadResolution(12);

  GpsOFF();
  Si5351OFF;
  GpsINIT();
  bmp_init();

  //**********************
  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when serial is opened).

  // FIX! should I do this?
  while (!SerialUSB) { ; } // Serial is via USB; wait for enumeration

  if (SerialUSB.read() > 0) { // read and discard data
    Serial.println("SerialUSB.read() detected input");
  }
  //**********************

  Watchdog.reset();
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB){;}
  Watchdog.reset();

  SerialUSB.println(F("Starting"));

  Wire.begin(); // somehow this is necessary for Serial2 to work properly
  vfo_init();

  SerialUSB.print(F("WSPR (HF) CallSign: "));
  SerialUSB.println(hf_call);
  SerialUSB.println(F(""));

}

void loop() {
  Watchdog.reset();
  updateStatusLED();

  if (((readBatt() > BattMin) && GpsFirstFix) || ((readBatt() > GpsMinVolt) && !GpsFirstFix)) {

    if (aliveStatus) {

      sendStatus();
      aliveStatus = false;
      while (readBatt() < BattMin) sleepSeconds(BattWait);
    }

      updateGpsData(1000);
      gpsDebug();

      if(gps.location.isValid() && gps.location.age()<1000){
        GpsInvalidTime=0;
        setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
      }else{
        GpsInvalidTime++;
        if (gps.date.year() != 2000) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
        else setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
        if(GpsInvalidTime > GpsResetTime){
          GpsOFF();
          ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
          Watchdog.reset();
          delay(1000);
          GpsON();
          GpsInvalidTime=0;
        }
      }

      if ((gps.location.age() < 1000 || gps.location.isUpdated()) && gps.location.isValid()) {
        if (gps.satellites.isValid() && gps.satellites.value() > 3) {
          GpsFirstFix = true;
          if(readBatt() < HighVolt){
             GpsOFF();
             ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
          }
          GpsInvalidTime=0;

          // Checks if there is an HF (WSPR) TX window is soon
          if (!((minute() % 10 == 3 || minute() % 10 == 7) &&  second()>50 && readBatt() > WsprBattMin && timeStatus() == timeSet)){
            updateTelemetry();
            freeMem();
            SerialUSB.flush();

          }

          // FIX! it should depend on the channel starting minute - 1 (modulo 10)
          // preparations for HF starts one minute before TX time at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.
          printf("timeStatus():%u minute():%u\n", timeStatus(), minute());
          if (readBatt() > WsprBattMin && timeStatus() == timeSet && ((minute() % 10 == 3) || (minute() % 10 == 7)) ) {
            printf("start WSPR\n");
            GridLocator(hf_loc, gps.location.lat(), gps.location.lng());
            sprintf(hf_message,"%s %s",hf_call,hf_loc);

            if (DEVMODE) {
                SerialUSB.println(F("Digital HF Mode Preparing"));
                SerialUSB.print(F("Grid Locator: "));
                SerialUSB.println(hf_loc);
            }

            //HF transmission starts at minute 4, 8, 14, 18, 24, 28, 34, 38, 44, 48, 54 or 58
            // FIX! start on hte starting minute of the channel
            while (((minute() % 10 != 4) || (minute() % 10 != 8)) && second() != 0) {
              Watchdog.reset();
              delay(1);
              updateStatusLED();
            }
            if (DEVMODE) {
              SerialUSB.println(F("Digital HF Mode Sending..."));
            }
            setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);
            encode();
            //HFSent=true;

            if (DEVMODE) {
              SerialUSB.println(F("Digital HF Mode Sent"));
            }
            setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

          } else {
            sleepSeconds(BeaconWait);
          }
        }else {
          if (DEVMODE) {
            SerialUSB.println(F("Not enough satelites"));
          }
        }
      }

  } else {
    sleepSeconds(BattWait);
  }
}

void sleepSeconds(int sec) {
  Si5351OFF;
  SerialUSB.flush();
  for (int i = 0; i < sec; i++) {
    if (GpsFirstFix){ //sleep gps after first fix
      if (readBatt() < HighVolt){
        GpsOFF();
        ublox_high_alt_mode_enabled = false;
      }
    }else{
      if (readBatt() < BattMin){
        GpsOFF();
        ublox_high_alt_mode_enabled = false;
      }
    }

    Watchdog.reset();

    uint32_t usec = time_us_32();
    while ((time_us_32() - usec) < 1000000) {
      updateStatusLED();
    }

  }
  Watchdog.reset();
}


void updatePosition(int high_precision, char *dao) {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  RawDegrees rawDeg = gps.location.rawLat();
  uint32_t min_nnnnn;
  // not used
  // char lat_dao = 0;
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    // round up. Avoid overflow (59.999999 should never become 60.0 or more)
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(latStr, "%02u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 100), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'S' : 'N');
  if (dao)
    dao[0] = (char) ((min_nnnnn % 1000) / 11) + 33;

  // FIX! what is this
  // APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  rawDeg = gps.location.rawLng();
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(lonStr, "%03u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 1000), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'W' : 'E');
  if (dao) {
    dao[1] = (char) ((min_nnnnn % 1000) / 11) + 33;
    dao[2] = 0;
  }

  // FIX! what is this?
  // APRS_setLon(lonStr);
  // APRS_setTimeStamp(gps.time.hour(), gps.time.minute(),gps.time.second());
}


void updateTelemetry() {
  sprintf(telemetry_buff, "%03d", gps.course.isValid() ? (int)gps.course.deg() : 0);
  telemetry_buff[3] = '/';

  sprintf(telemetry_buff + 4, "%03d", gps.speed.isValid() ? (int)gps.speed.knots() : 0);
  telemetry_buff[7] = '/';

  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  //sprintf(telemetry_buff + 10, "%06lu", (long)gps.altitude.feet());

  //fixing negative altitude values causing display bug on aprs.fi
  float tempAltitude = gps.altitude.feet();

  if (tempAltitude>0){
    //for positive values
    sprintf(telemetry_buff + 10, "%06lu", (long)tempAltitude);
  } else{
    //for negative values
    sprintf(telemetry_buff + 10, "%06d", (int)tempAltitude);
    }

  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'x';
  telemetry_buff[22] = 'C';

  // FIX! don't need anymore?
  // Si5351ON; //little hack to prevent a BMP180 related issue (does nothing?)
  delay(1);

  telemetry_buff[23] = ' '; float tempC = bmp_read_temperature();
  // telemetry_buff[23] = ' '; float tempC = 0.f;
  dtostrf(tempC, 6, 2, telemetry_buff + 24);

  telemetry_buff[30] = 'C';
  telemetry_buff[31] = ' '; float pressure = bmp_read_pressure() / 100.0; //Pa to hPa
  // telemetry_buff[31] = ' '; float pressure = 0.f; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 32);

  // FIX! don't need anymore?
  // Si5351OFF;

  telemetry_buff[39] = 'h';
  telemetry_buff[40] = 'P';
  telemetry_buff[41] = 'a';
  telemetry_buff[42] = ' ';

  dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
  telemetry_buff[48] = 'V';
  telemetry_buff[49] = ' ';

  sprintf(telemetry_buff + 50, "%02d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  telemetry_buff[52] = 'S';
  telemetry_buff[53] = ' ';
  sprintf(telemetry_buff + 54, "%s", comment);
  // remove temperature and pressure info
  // memmove(&telemetry_buff[24], &telemetry_buff[43], (sizeof(telemetry_buff) - 43));

  if (DEVMODE) {
    SerialUSB.println(telemetry_buff);
  }

}

void sendLocation() {
  if (DEVMODE) {
    SerialUSB.println(F("Location sending with comment"));
  }

  // FIX! how do we end the telemetry_buff
  int GEOFENCE_APRS_frequency=0;

  // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
  // vfo_turn_on(APRS_TX_CLK_NUM);
  // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
  {
    delay(500);
    // FIX! how do we end the telemetry_buff
    // APRS_sendLoc(telemetry_buff);
    delay(10);
    vfo_turn_off();
    SerialUSB.print(F("APRS Location sent (Freq: "));
    SerialUSB.print(GEOFENCE_APRS_frequency);
    SerialUSB.print(F(") - "));
    SerialUSB.println(TxCount);
    TxCount++;
  }

}

void sendStatus() {
  // FIX! how do we end the telemetry_buff
  int GEOFENCE_APRS_frequency=0;

  // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
  // vfo_turn_on(APRS_TX_CLK_NUM);
  // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
  {
    delay(500);
    // APRS_sendStatus(StatusMessage);
    delay(10);
    vfo_turn_off();
    SerialUSB.print(F("Status sent (Freq: "));
    SerialUSB.print(GEOFENCE_APRS_frequency);
    SerialUSB.print(F(") - "));
    SerialUSB.println(TxCount);
    TxCount++;
  }

}

float readBatt() {
  int adc_val = 0;
  adc_val = analogRead(BattPin);
  adc_val += analogRead(BattPin);
  adc_val += analogRead(BattPin);
  // The Raspberry Pi Pico's analog to digital converter (ADC) can measure voltages between 0 and 3.3 volts.
  // The ADC uses a 3.3V reference voltage,
  // and a read operation returns a number between 0 and 4095.
  // The ADC's resolution is 3.3/4096, or roughly 0.8 millivolts.
  // is the precision set to 4096? (12 not 16 bits resolution)
  // 4096/3.3 = 1241
  // 1241 / 3 = 413.66

  // FIX! this doesn't seem right. should I just multiply by the conversion factor
  // he's got some special 1/3 voltage divider for VBUS to BATT_V
  // you leave it open than the ADC converter voltage reference is the 3.3V .
  // In reality it is the voltage of the pin 3V3 - ( ~150uA * 200) which is roughly a 30mv drop. (0.8mv * 30 = 24 steps)

  // this must be a calibrated linear equation? only need to calibrate between 2.8v and 5v?
  float solar_voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;
  // there is a 200 ohm resistor between 3V3 and ADC_AVDD
  // we did 3 reads above ..averaging? so don't need the 3x because of onboard voltage divider
  // pico-WSPRer does this (no use of ADC_AVDD) ?
  // const float conversionFactor = 3.3f / (1 << 12);
  // float solar_voltage = 3 * (float)adc_read() * conversionFactor;

  // if (solar_voltage < 0.0f) solar_voltage = 0.0f;
  // if (solar_voltage > 9.9f) solar_voltage = 9.9f;
  return solar_voltage;
}

#define WSPR_PWM_SLICE_NUM  4
static pwm_config wspr_pwm_config;
void PWM4_Handler(void) {
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  static int cnt = 0;
  if (++cnt >= 500) {
    cnt = 0;
    proceed = true;
  }
}

void zeroTimerSetPeriodMs(float ms){
  wspr_pwm_config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&wspr_pwm_config, 250); // 2uS
  pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)ms - 1));
  pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

  irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
  pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
}

void encode()
{
  Watchdog.reset();
  vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
  vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
  vfo_turn_on(WSPR_TX_CLK_NUM);
  // do we have to turn on the differential clock?
  // they share a pll ? so the differential enable ifdef should handle that?
  // vfo_turn_on(WSPR_TX_CLK_1_NUM);
  uint8_t i;

  if (DEVMODE) {
    printf("cur_mode:%u\n", cur_mode);
  }
  switch(cur_mode)
  {
  case MODE_WSPR:
    hf_freq = WSPR_DEFAULT_FREQ - 100 + (rand() % 200);

    if (DEVMODE) {
      printf("WSPR freq: %lu\n", hf_freq);
    }

    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    break;
  }
  set_tx_buffer();
  zeroTimerSetPeriodMs(tone_delay);

  for(i = 0; i < symbol_count; i++)
  {
      uint32_t freq_x16 = (hf_freq << PLL_CALCULATION_PRECISION) + (tx_buffer[i] * (12000L << PLL_CALCULATION_PRECISION) + 4096) / 8192L;
      // printf("%s vfo_set_freq_x16(%u)\n", __func__, (freq_x16 >> PLL_CALCULATION_PRECISION));
      vfo_set_freq_x16(WSPR_TX_CLK_NUM, freq_x16);
      proceed = false;

      while (!proceed) {
        updateStatusLED();
      }
      Watchdog.reset();
  }
  pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
  pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  irq_set_enabled(PWM_IRQ_WRAP, false);
  irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);
  vfo_turn_off();
  Watchdog.reset();
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_WSPR:
    jtencode.wspr_encode(hf_call, hf_loc, dbm, tx_buffer);
    break;
  }
}

void freeMem() {
  if (DEVMODE) return;
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
}
