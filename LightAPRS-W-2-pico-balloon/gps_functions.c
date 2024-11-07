/* 
 * gps_functions.c
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#include "gps_functions.h"

// FIX! why was this static void before?
void updateGpsData(int ms)
{
  Watchdog.reset();
  GpsON;
  while (!Serial) {delay(1);} // wait for serial port to connect.  

  // FIX! do we need any config of the ATGM336?
  if(!ublox_high_alt_mode_enabled){
    //enable ublox high altitude mode
    setGPS_DynamicModel6();
    #if defined(DEVMODE)
      SerialUSB.println(F("ublox DynamicModel6 enabled..."));
    #endif      
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
  } while (millis() - start < ms);

  #if defined(DEVMODE2)
  printf("gps.time.isValid():%u\n", gps.time.isValid());
  #endif
  if (gps.time.isValid())
  {
    // kevin 11_6_24 0 instead of NULL (not pointer)
    // causes problems with the routines declares?
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);     
    #if defined(DEVMODE2)
    printf("setTime(%02u:%02u:%02u)\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    #endif
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
    #if defined(DEVMODE)
      SerialUSB.println(F("ublox DynamicModel6 try..."));
    #endif 
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
#if defined(DEVMODE)
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

#endif
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


