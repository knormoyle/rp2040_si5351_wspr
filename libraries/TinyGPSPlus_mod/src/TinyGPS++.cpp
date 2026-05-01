/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2024 Mikal Hart
All rights reserved.
... (LGPL license unchanged) ...
*/

#include "TinyGPS++.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define _RMCterm "RMC"
#define _GGAterm "GGA"
#define _ZDAterm "ZDA"
#define _GSTterm "GST"

// Speed: fast 3-char + null suffix compare — avoids strcmp() call overhead
#define TERM3EQ(t, a, b, c) ((t)[0]==(a) && (t)[1]==(b) && (t)[2]==(c) && (t)[3]=='\0')

#if !defined(ARDUINO) && !defined(__AVR__)
unsigned long millis()
{
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    return static_cast<unsigned long>(duration.count());
}
#endif

TinyGPSPlus::TinyGPSPlus()
  :  parity(0), isChecksumTerm(false), curSentenceType(GPS_SENTENCE_OTHER)
  ,  curTermNumber(0), curTermOffset(0), sentenceHasFix(false)
  ,  customElts(0), customCandidates(0), encodedCharCount(0)
  ,  sentencesWithFixCount(0), failedChecksumCount(0), passedChecksumCount(0)
{ term[0] = '\0'; }

// public methods

bool TinyGPSPlus::encode(char c)
{
  ++encodedCharCount;
  switch(c)
  {
  case ',':  // kbn doesn't work if we just return false here, after parity
  case '\r':
  case '\n':
  case '*':
    {
      // kbn: to avoid compiler warnings on fallthrough
      if (c == ',') parity ^= (uint8_t)c;
      bool isValidSentence = false;
      if (curTermOffset < sizeof(term))
      {
        term[curTermOffset] = 0;
        isValidSentence = endOfTermHandler();
      }
      ++curTermNumber;
      curTermOffset = 0;
      isChecksumTerm = c == '*';
      return isValidSentence;
    }
    break;

  case '$': // sentence begin
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  default: // ordinary characters
    if (curTermOffset < sizeof(term) - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    return false;
  }
  return false;
}

// internal utilities

int TinyGPSPlus::fromHex(char a)
{
  if (a >= 'A' && a <= 'F') return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f') return a - 'a' + 10;
  else return a - '0';
}

// static
// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
// Speed: replaced atol() + redundant digit re-scan with a single inline parse loop
int32_t TinyGPSPlus::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative) ++term;

  // Inline integer parse — one forward pass, no atol() overhead
  int32_t whole = 0;
  while (isdigit(*term)) whole = whole * 10 + (*term++ - '0');
  int32_t ret = whole * 100;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
// Speed: replaced atol() + duplicate digit scan with a single forward pass
void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
{
  // Parse integer part (DDMM) inline — one pass replaces atol() + re-scan
  uint32_t leftOfDecimal = 0;
  while (isdigit(*term)) leftOfDecimal = leftOfDecimal * 10 + (*term++ - '0');

  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  deg.deg = (int16_t)(leftOfDecimal / 100);

  if (*term == '.')
    while (isdigit(*++term))
    {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }

  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}

// This macro combines two values into a single unsigned integer by packing them into different bit fields
#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// (unsigned)(sentence_type) — casts sentence_type to an unsigned integer

// << 5 — shifts it left by 5 bits (multiplies by 32), moving it into the upper bits
// | term_number — ORs in term_number, which occupies the lower 5 bits

// Resulting bit layout:
// [ sentence_type bits ][ term_number bits ]
//        ...              b4 b3 b2 b1 b0
//                        ^--- 5 bits ---^

// term_number can hold values 0–31 (5 bits)
// sentence_type occupies all remaining upper bits
// The two values are stored together in one integer, with no overlap — as long as term_number stays within 0–31

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    byte checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity)
    {
      passedChecksumCount++;
      if (sentenceHasFix) ++sentencesWithFixCount;

      switch(curSentenceType)
      {
      case GPS_SENTENCE_ZDA: // kbn new. get time from ZDA, last in SIM65M burst
        // kbn 2/13/25 just use GGA for consistency early in burst
        // time.commit();
        break;
      case GPS_SENTENCE_GST: // kbn new. get time from GST, last in ATGM336H burst
        // kbn 2/13/25 just use GGA for consistency early in burst
        // time.commit();
        break;

      case GPS_SENTENCE_RMC:
        // kbn 2/13/25 just use GGA for time consistency early in burst
        // FixMode now lives on TinyGPSFix; commit it alongside date
        date.commit();
        fix.commit();
        // time.commit();
        if (sentenceHasFix)
        {
           // only want location from GGA?
           // location.commit();
           speed.commit();
           course.commit();
        }
        break;

      case GPS_SENTENCE_GGA:
        // kbn 2/13/25 just use GGA for time consistency early in burst
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          altitude.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
      }
      // Commit all custom listeners of this sentence type
      // Extracted customCandidates->sentenceName into referenceSentenceName — same reasoning as below
      // Moved the strcmp out of the loop condition and into an explicit break — 
      // the loop condition was doing double duty (null check + string comparison), 
      // Separating them makes the early-exit logic stand on its own.
      const char *referenceSentenceName = customCandidates->sentenceName;
      for (TinyGPSCustom *p = customCandidates; p != NULL; p = p->next)
      {
        if (strcmp(p->sentenceName, referenceSentenceName) != 0) { break; }
        p->commit();
      }

      return true;
    }
    else { ++failedChecksumCount; }
    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    // add BDRMC/BDGGA/BDGSV single constellation possibilities. GLRMC/GLGGA/GLGSV were already there
    // allow D in the 2nd char
    // Speed: replaced 4x strchr()+strcmp() with direct char compares + TERM3EQ macro
    const char c0  = term[0];
    const char c1  = term[1];
    const char *sfx = term + 2;  // 3-char suffix: RMC / GGA / ZDA / GST

    // First char must be 'G' or 'B'; second char must be one of D,P,N,A,B,L
    if ((c0=='G' || c0=='B') &&
        (c1=='D' || c1=='P'|| c1=='N'|| c1=='A'|| c1=='B'|| c1=='L'))
    {
      if      (TERM3EQ(sfx, 'R','M','C')) curSentenceType = GPS_SENTENCE_RMC;
      else if (TERM3EQ(sfx, 'G','G','A')) curSentenceType = GPS_SENTENCE_GGA;
      // don't need to case on ZDA and GST anymore
      else if (TERM3EQ(sfx, 'Z','D','A')) curSentenceType = GPS_SENTENCE_ZDA;
      else if (TERM3EQ(sfx, 'G','S','T')) curSentenceType = GPS_SENTENCE_GST;
      else                                curSentenceType = GPS_SENTENCE_OTHER;
    }
    else { curSentenceType = GPS_SENTENCE_OTHER; }


    // for with empty body → while to make the traversal explicit.
    // > 0 → != 0 since < 0 was already ruled out by the loop.
    // Comments explain the sorted-list assumption and the overshoot case.
    // Advance through the sorted custom elements list to find a matching sentence name

    // Any custom candidates of this sentence type?
    while (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) < 0)
    {
      customCandidates = customCandidates->next;
    }

    // If we overshot (no exact match exists), clear the candidates pointer
    if (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) != 0)
    {
      customCandidates = NULL;
    }

    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch(COMBINE(curSentenceType, curTermNumber))
  {
    case COMBINE(GPS_SENTENCE_GGA, 1):
      time.setTime(term); break;
    case COMBINE(GPS_SENTENCE_GGA, 2):
      location.setLatitude(term); break;
    case COMBINE(GPS_SENTENCE_GGA, 3):
      location.rawNewLatData.negative = term[0] == 'S'; break;
    case COMBINE(GPS_SENTENCE_GGA, 4):
      location.setLongitude(term); break;
    case COMBINE(GPS_SENTENCE_GGA, 5):
      location.rawNewLngData.negative = term[0] == 'W'; break;
    case COMBINE(GPS_SENTENCE_GGA, 6): // Fix data (GGA)
      // both GGA and RMC can have this? includes type 6? 
      sentenceHasFix = term[0] > '0';
      location.newFixQuality = (TinyGPSLocation::Quality)term[0]; break;
    case COMBINE(GPS_SENTENCE_GGA, 7): // Satellites used (GGA)
      satellites.set(term); break;
    case COMBINE(GPS_SENTENCE_GGA, 8): // HDOP
      hdop.set(term); break;
    case COMBINE(GPS_SENTENCE_GGA, 9): // Altitude (GGA)
      altitude.set(term); break;

    // RMC comes after GGA. so can't have FixMode in location
    case COMBINE(GPS_SENTENCE_RMC, 2): // RMC validity
      // both GGA and RMC can have this?
      sentenceHasFix = term[0] == 'A'; break;
    case COMBINE(GPS_SENTENCE_RMC, 7): // Speed (RMC)
      speed.set(term); break;
    case COMBINE(GPS_SENTENCE_RMC, 8): // Course (RMC)
      course.set(term); break;
    case COMBINE(GPS_SENTENCE_RMC, 9): // Date (RMC)
      date.setDate(term); break;
    case COMBINE(GPS_SENTENCE_RMC, 12):
      // if it's not a legal enum, it will just put the numeric value of the char?
      // could it be non-printable (others could also?)
      // FixMode lives on TinyGPSFix now (committed together with date in RMC)
      fix.newFixMode = (TinyGPSFix::Mode)term[0]; break;

    // these aren't used any more
    /*
    case COMBINE(GPS_SENTENCE_ZDA, 1): // kbn Add ZDA (last sentence in burst) for SIM65M
    case COMBINE(GPS_SENTENCE_GST, 1): // kbn Add GST (last sentence in burst) for ATGM336
    case COMBINE(GPS_SENTENCE_RMC, 1): // Time in both sentences
    case COMBINE(GPS_SENTENCE_RMC, 3): // Latitude
    case COMBINE(GPS_SENTENCE_RMC, 4): // N/S
    case COMBINE(GPS_SENTENCE_RMC, 5): // Longitude
    case COMBINE(GPS_SENTENCE_RMC, 6): // E/W
    */
    // kbn ZDA also has date but in 3 fields. so don't use.
    // RMC has 120225 (not 2025 ..just 2 digits)
  }

  // Set custom values as needed
  // this is kind of slow traversal for the GSV and GSA stuff I use custom for?

  // Extracted customCandidates->sentenceName into referenceSentenceName
  // makes it clear the loop is comparing against a fixed reference, not a moving target, 
  // avoids re-dereferencing on every iteration.
  // Converted for to while — the loop advances p as a side effect with no init expression, 
  // which is a cleaner fit for while. 
  // Separated the iterator advance (p = p->next) from the loop header into the body
  TinyGPSCustom *p = customCandidates;
  const char *referenceSentenceName = customCandidates->sentenceName;
  while (p != NULL &&
         strcmp(p->sentenceName, referenceSentenceName) == 0 &&
         p->termNumber <= curTermNumber)
  {
    if (p->termNumber == curTermNumber)
    {
        p->set(term);
    }
    p = p->next;
  }

  return false;
}

/* static */
double TinyGPSPlus::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6371009 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1); lat2 = radians(lat2);
  double slat1 = sin(lat1), clat1 = cos(lat1);
  double slat2 = sin(lat2), clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * _GPS_EARTH_MEAN_RADIUS;
}

double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1); lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) a2 += TWO_PI;
  return degrees(a2);
}

const char *TinyGPSPlus::cardinal(double course)
{
  static const char* directions[] = {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

void TinyGPSLocation::commit()
{
   rawLatData = rawNewLatData; 
   rawLngData = rawNewLngData;
   fixQuality = newFixQuality; 
   lastCommitTime = millis(); 
   valid = updated = true;
}
void TinyGPSLocation::setLatitude(const char *term)  { TinyGPSPlus::parseDegrees(term, rawNewLatData); }
void TinyGPSLocation::setLongitude(const char *term) { TinyGPSPlus::parseDegrees(term, rawNewLngData); }

double TinyGPSLocation::lat()
{
   updated = false;
   double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
   return rawLatData.negative ? -ret : ret;
}
double TinyGPSLocation::lng()
{
   updated = false;
   double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
   return rawLngData.negative ? -ret : ret;
}

void TinyGPSTime::commit() { time = newTime; lastCommitTime = millis(); valid = updated = true; }
void TinyGPSTime::setTime(const char *term) { newTime = (uint32_t)TinyGPSPlus::parseDecimal(term); }

// Speed: replaced atol() with inline digit accumulator
void TinyGPSDate::commit() 
{ 
    date = newDate; 
    lastCommitTime = millis(); 
    valid = updated = true; 
}

void TinyGPSDate::setDate(const char *term)
{
   uint32_t v = 0;
   while (isdigit(*term)) v = v * 10 + (*term++ - '0');
   newDate = v;
}

void TinyGPSFix::commit()
{
    fixMode = newFixMode;
    lastCommitTime = millis();
    valid = updated = true;
}

void TinyGPSFix::set(const char *term)
{
    // Single-char NMEA mode indicator (N/A/D/E). If absent or unknown, fall back to N.
    newFixMode = (term && term[0]) ? (Mode)term[0] : N;
}

uint16_t TinyGPSDate::year()        { updated = false; return (date % 100) + 2000; }
uint8_t  TinyGPSDate::month()       { updated = false; return (date / 100) % 100; }
uint8_t  TinyGPSDate::day()         { updated = false; return date / 10000; }
uint8_t  TinyGPSTime::hour()        { updated = false; return time / 1000000; }
uint8_t  TinyGPSTime::minute()      { updated = false; return (time / 10000) % 100; }
uint8_t  TinyGPSTime::second()      { updated = false; return (time / 100) % 100; }
uint8_t  TinyGPSTime::centisecond() { updated = false; return time % 100; }

void TinyGPSDecimal::commit() { val = newval; lastCommitTime = millis(); valid = updated = true; }
void TinyGPSDecimal::set(const char *term) { newval = TinyGPSPlus::parseDecimal(term); }
void TinyGPSInteger::commit() { val = newval; lastCommitTime = millis(); valid = updated = true; }

// Speed: replaced atol() with inline digit accumulator
void TinyGPSInteger::set(const char *term)
{
   uint32_t v = 0;
   while (isdigit(*term)) v = v * 10 + (*term++ - '0');
   newval = v;
}

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   lastCommitTime = 0; updated = valid = false;
   sentenceName = _sentenceName; termNumber = _termNumber;
   memset(stagingBuffer, '\0', sizeof(stagingBuffer));
   memset(buffer, '\0', sizeof(buffer));
   // Insert this item into the GPS tree
   gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
   strcpy(this->buffer, this->stagingBuffer);
   lastCommitTime = millis(); valid = updated = true;
}
void TinyGPSCustom::set(const char *term)
{
   strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer) - 1);
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int termNumber)
{
   TinyGPSCustom **ppelt;
   for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
   {
      int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
      if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
         break;
   }
   pElt->next = *ppelt;
   *ppelt = pElt;
}
