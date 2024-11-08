/* 
 * gps_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#define SerialUSB Serial

// In the Arduino IDE, to reference a variable declared within a function from another part of 
// your code, you need to declare the variable as "global" by defining it outside of any specific function, 
// allowing access to it from anywhere in your sketch

// It is a special feature of the Arduino IDE that you can define functions after the point 
// at which they are used without needing an explicit function prototype before hand. 
// The Arduino build routine creates these prototypes but, unhappily, not always correctly, 
// leading to errors which are not obvious. 
// This is especially so if the function argument list contains user defined data types and 
// the automatically created function prototype is placed before the declaration of that data type.


// A CPP file isn't an Arduino file. 
// It doesn't know about anything Arduino-esque unless you tell it
// The simplest way is to just add, at the top of the file:
// need this for Serial2.* ?? otherwise says no type
// #include <Arduino.h>

#include <Arduino.h>
#include "gps_functions.h"
#include "debug_functions.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h> //https://github.com/PaulStoffregen/Time

// need this file to be .cpp because this uses type class
// for Watchdog.*
#include <Adafruit_SleepyDog.h>             //https://github.com/adafruit/Adafruit_SleepyDog

#include <TinyGPS++.h>                      //https://github.com/mikalhart/TinyGPSPlus

// what about the tiny gps library? already included in the .ino?

// FIX! can't say extern boolean? why does the *.ino have boolean
// we don't really need this if the variable is created before we include this?
extern bool ublox_high_alt_mode_enabled;
extern const int GpsPwr;
// in the *ino
// how to reference?
// should we declare it here?
// refer to gps.* stuff?
// doesn't work
extern TinyGPSPlus gps;
extern bool DEVMODE;

// extern void updateStatusLED(void)
// instead, should we have
#include "led_functions.h"

// access library variables
// it's declared in a class only if the library uses a class. if it is declared in a class, it must be public for you to access it. 

// inside your own sketch
// if it's not inside a class, then you should be able to access it by using "extern" inside your sketch
// for .c or .cpp

// FIX! a bunch of things work because includes where done before gps_functions.h was included?

void GpsON()
{
  do {
    Serial2.begin(9600);
    digitalWrite(GpsPwr, LOW);
    /*printf("GpsON\n");*/
  } while (false);
}

/*
This used to be in the LightAPRS version of TinyGPSPlus-0.95
< #if defined(ARDUINO_ARCH_RP2040)
< void TinyGPSDate::clear()
< {
<    valid = updated = false;
<    date = 0;
< }
< #endif
*/

void GpsOFF()
{
  do {
    digitalWrite(GpsPwr, HIGH);
    Serial2.end();
    // gps.date.clear();
    // are these declared private?
    // FIX! how can we clear these? Do we change the library to make them public?
    
    /* from TinyGPS++.h in libraries..modify it and move to public? (in struct TinyGPSDate
    what about location etc? they have valid and updated
    
    private:
       bool valid, updated;
       uint32_t date, newDate;
       uint32_t lastCommitTime;
       void commit();
       void setDate(const char *term);
TinyGPS++.h:   bool valid, updated;
    };
    */

    // these three are the initial value
    gps.date.valid = false;
    gps.date.updated = false;
    gps.date.date = 0; 

    // also has lastCommitTime = millis()
    // we don't change that?

    /*printf("GpsOFF\n");*/
  } while (false);
}

// FIX! why was this static void before?
void updateGpsData(int ms)
// ms has to be positive?
{
  Watchdog.reset();
  GpsON();
  while (!SerialUSB) {delay(1);} // wait for serial port to connect.  

  // FIX! do we need any config of the ATGM336?
  if(!ublox_high_alt_mode_enabled){
    //enable ublox high altitude mode
    setGPS_DynamicModel6();
    if (DEVMODE) {
      SerialUSB.println(F("ublox DynamicModel6 enabled..."));
    }
    ublox_high_alt_mode_enabled = true;      
  }
  
  unsigned long start = millis();
  unsigned long bekle=0;
  do
  {

    while (Serial2.available()>0) {
      char c;
      c=Serial2.read();
      gps.encode(c);
      bekle= millis();
    }
    
    if (bekle!=0 && bekle+10<millis())break;
    updateStatusLED();
  } while ( (millis() - start) < (unsigned long) ms);

  if (DEVMODE) {
    printf("gps.time.isValid():%u\n", gps.time.isValid());
  }
  if (gps.time.isValid())
  {
    // kevin 11_6_24 0 instead of NULL (not pointer)
    // causes problems with the routines declares?
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);     
    if (DEVMODE) {
      printf("setTime(%02u:%02u:%02u)\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    }
  }
}

//following GPS code from : https://github.com/HABduino/HABduino/blob/master/Software/habduino_v4/habduino_v4.ino
void setGPS_DynamicModel6()
{  
  int gps_set_success=0;
  uint8_t setdm6[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

  while(!gps_set_success)
  {
    if (DEVMODE) {
      SerialUSB.println(F("ublox DynamicModel6 try..."));
    }
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_success=getUBX_ACK(setdm6);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial2.write(0xFF);
  delay(500);
  for(int i=0; i<len; i++) {
    Serial2.write(MSG[i]);
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  boolean status =false;

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      status= true;
      break;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      status= false;
      break;
    }

    // Make sure data is available to read
    if (Serial2.available()) {
      b = Serial2.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
  return status;
}

void gpsDebug() {
  if (! DEVMODE) return;

  SerialUSB.println();
  SerialUSB.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
  SerialUSB.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
  SerialUSB.println(F("-----------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  SerialUSB.println();

}

void GridLocator(char *dst, float latt, float lon) {
  int o1, o2;
  int a1, a2;
  float remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (float)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  // latitude
  remainder = latt + 90.0;
  a1 = (int)(remainder / 10.0); 
  remainder = remainder - (float)a1 * 10.0;
  a2 = (int)(remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)0;
}


