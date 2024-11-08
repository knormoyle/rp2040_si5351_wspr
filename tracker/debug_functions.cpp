/* 
 * debug_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#define SerialUSB   Serial
extern bool DEVMODE;

// for TinyGPSDate definition
#include <TinyGPS++.h>   

// why was this static?
void printInt(unsigned long val, bool valid, int len)
{
  if (! DEVMODE) return;

  char sz[32] = "*****************";
  if (valid) sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i) sz[i] = ' ';

  if (len > 0) sz[len - 1] = ' ';
  SerialUSB.print(sz);
}

// why was this static
void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (! DEVMODE) return;

  if (!d.isValid()) {
    SerialUSB.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    SerialUSB.print(sz);
  }

  if (!t.isValid()) {
    SerialUSB.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    SerialUSB.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
}

// why was this static
void printStr(const char *str, int len)
{
  if (! DEVMODE) return;
  int slen = strlen(str);
  for (int i = 0; i < len; ++i) SerialUSB.print(i < slen ? str[i] : ' ');
}


// why was this static?
void printFloat(float val, bool valid, int len, int prec)
{
  if (! DEVMODE) return;

  if (!valid) {
    while (len-- > 1) SerialUSB.print('*');
    SerialUSB.print(' ');

  } else {
    SerialUSB.print(val, prec);
    int vi = abs((int)val);

    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i) SerialUSB.print(' ');
  }
}

