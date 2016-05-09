/***********************************
This is the Adafruit GPS library - the ultimate GPS library
for the ultimate GPS module!

Tested and works great with the Adafruit Ultimate GPS module
using MTK33x9 chipset
    ------> http://www.adafruit.com/products/746
Pick one up today at the Adafruit electronics shop 
and help support open source hardware & software! -ada

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/
// Fllybob added lines 34,35 and 40,41 to add 100mHz logging capability 

#ifndef _ADAFRUIT_GPS_H
#define _ADAFRUIT_GPS_H

#ifdef __AVR__
  #if ARDUINO >= 100
    #include <SoftwareSerial.h>
  #else
    #include <NewSoftSerial.h>
  #endif
#endif

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status 
#define PGCMD_ANTENNA "$PGCMD,33,1*6C" 
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 

// how long to wait when we're looking for a response
#define MAXWAITSENTENCE 5

#if ARDUINO >= 100
 #include "Arduino.h"
#if defined (__AVR__) && !defined(__AVR_ATmega32U4__)
 #include "SoftwareSerial.h"
#endif
#else
 #include "WProgram.h"
 #include "NewSoftSerial.h"
#endif


// how long are max NMEA lines to parse?
#define ADAGPS_MAXLINELENGTH 120


/*******************************
 * Adafruit_GPS class          *
 *******************************/


template <typename Serial>
class Adafruit_GPS {
  public:
    
    
    /************************
     * Constructor          *
     ************************/
    
    
    Adafruit_GPS(Serial* serial) {
      recvdflag   = false;
      paused      = false;
      lineidx     = 0;
      currentline = line1;
      lastline    = line2;

      hour = minute = seconds = year = month = day =
        fixquality = satellites = 0; // uint8_t
      lat = lon = mag = 0; // char
      fix = false; // boolean
      milliseconds = 0; // uint16_t
      latitude = longitude = geoidheight = altitude =
        speed = angle = magvariation = HDOP = 0.0; // float
      gpsSerial = serial;
    }
    
    
    /************************
     * Member functions     *
     ************************/
    
    
    void begin(uint16_t baud) {
      gpsSerial->begin(baud);
      delay(10);
    }
    
    
    char* lastNMEA(void) {
      recvdflag = false;
      return (char *)lastline;
    }
    
    
    boolean newNMEAreceived() {
      return recvdflag;
    }
    
    void sendCommand(const char* str) {
      gpsSerial->println(str);
    }
    
    
    void pause(boolean p) {
      paused = p;
    }
    
    
    uint8_t parseHex(char c) {
      if (c < '0')
        return 0;
      if (c <= '9')
        return c - '0';
      if (c < 'A')
         return 0;
      if (c <= 'F')
         return (c - 'A')+10;
      // if (c > 'F')
      return 0;
    }
    
    
    char read(void) {
      char c = 0;
      
      if (paused) return c;
      
      if (!gpsSerial->available()) return c;
      c = gpsSerial->read();
      
      if (c == '\n') {
        currentline[lineidx] = 0;

        if (currentline == line1) {
          currentline = line2;
          lastline = line1;
        } else {
          currentline = line1;
          lastline = line2;
        }
        
        lineidx = 0;
        recvdflag = true;
      }

      currentline[lineidx++] = c;
      if (lineidx >= ADAGPS_MAXLINELENGTH)
        lineidx = ADAGPS_MAXLINELENGTH-1;

      return c;
    }
    
    
    boolean parse(char* nmea) {
      // do checksum check
      
      // first look if we even have one
      if (nmea[strlen(nmea)-4] == '*') {
        uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
        sum += parseHex(nmea[strlen(nmea)-2]);
        
        // check checksum 
        for (uint8_t i=2; i < (strlen(nmea)-4); i++) {
          sum ^= nmea[i];
        }
        if (sum != 0) {
          // bad checksum :(
          return false;
        }
      }
      
      int32_t degree;
      long minutes;
      char degreebuff[10];
      // look for a few common sentences
      if (strstr(nmea, "$GPGGA")) {
        // found GGA
        char *p = nmea;
        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        hour = time / 10000;
        minute = (time % 10000) / 100;
        seconds = (time % 100);
        
        milliseconds = fmod(timef, 1.0) * 1000;
        
        // parse out latitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 2);
          p += 2;
          degreebuff[2] = '\0';
          degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          minutes = 50 * atol(degreebuff) / 3;
          latitude_fixed = degree + minutes;
          latitude = degree / 100000 + minutes * 0.000006F;
          latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
          latitudeDegrees += int(latitude/100);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'S') latitudeDegrees *= -1.0;
          if (p[0] == 'N') lat = 'N';
          else if (p[0] == 'S') lat = 'S';
          else if (p[0] == ',') lat = 0;
          else return false;
        }
        
        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 3);
          p += 3;
          degreebuff[3] = '\0';
          degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          minutes = 50 * atol(degreebuff) / 3;
          longitude_fixed = degree + minutes;
          longitude = degree / 100000 + minutes * 0.000006F;
          longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
          longitudeDegrees += int(longitude/100);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'W') longitudeDegrees *= -1.0;
          if (p[0] == 'W') lon = 'W';
          else if (p[0] == 'E') lon = 'E';
          else if (p[0] == ',') lon = 0;
          else return false;
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          fixquality = atoi(p);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          satellites = atoi(p);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          HDOP = atof(p);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          altitude = atof(p);
        }
        
        p = strchr(p, ',')+1;
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          geoidheight = atof(p);
        }
        return true;
      }
      if (strstr(nmea, "$GPRMC")) {
       // found RMC
        char *p = nmea;

        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        hour = time / 10000;
        minute = (time % 10000) / 100;
        seconds = (time % 100);

        milliseconds = fmod(timef, 1.0) * 1000;

        p = strchr(p, ',')+1;
        
        if (p[0] == 'A') 
          fix = true;
        else if (p[0] == 'V')
          fix = false;
        else
          return false;

        // parse out latitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 2);
          p += 2;
          degreebuff[2] = '\0';
          long degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          long minutes = 50 * atol(degreebuff) / 3;
          latitude_fixed = degree + minutes;
          latitude = degree / 100000 + minutes * 0.000006F;
          latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
          latitudeDegrees += int(latitude/100);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'S') latitudeDegrees *= -1.0;
          if (p[0] == 'N') lat = 'N';
          else if (p[0] == 'S') lat = 'S';
          else if (p[0] == ',') lat = 0;
          else return false;
        }
        
        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 3);
          p += 3;
          degreebuff[3] = '\0';
          degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          minutes = 50 * atol(degreebuff) / 3;
          longitude_fixed = degree + minutes;
          longitude = degree / 100000 + minutes * 0.000006F;
          longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
          longitudeDegrees += int(longitude/100);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'W') longitudeDegrees *= -1.0;
          if (p[0] == 'W') lon = 'W';
          else if (p[0] == 'E') lon = 'E';
          else if (p[0] == ',') lon = 0;
          else return false;
        }
        // speed
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          speed = atof(p);
        }
        
        // angle
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          angle = atof(p);
        }
        
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          uint32_t fulldate = atof(p);
          day = fulldate / 10000;
          month = (fulldate % 10000) / 100;
          year = (fulldate % 100);
        }
        // we dont parse the remaining, yet!
        return true;
      }

      return false;
    }
    
    
    boolean wakeup(void)  {
      if (inStandbyMode) {
       inStandbyMode = false;
        sendCommand("");  // send byte to wake it up
        return waitForSentence(PMTK_AWAKE);
      }
      else {
          return false;  // Returns false if not in standby mode, nothing to wakeup
      }
    }
    
    
    boolean standby(void) {
      if (inStandbyMode) {
        return false;  // Returns false if already in standby mode, so that you do not wake it up by sending commands to GPS
      }
      else {
        inStandbyMode = true;
        sendCommand(PMTK_STANDBY);
        //return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast enough to catch the message, or something else just is not working
        return true;
      }
    }
    
    
    boolean waitForSentence(const char* wait4me, uint8_t max = MAXWAITSENTENCE) {
      char str[20];

      uint8_t i=0;
      while (i < max) {
        if (newNMEAreceived()) { 
          char *nmea = lastNMEA();
          strncpy(str, nmea, 20);
          str[19] = 0;
          i++;

          if (strstr(str, wait4me))
      return true;
        }
      }

      return false;
    }
    
    
    boolean LOCUS_StartLogger(void) {
      sendCommand(PMTK_LOCUS_STARTLOG);
      recvdflag = false;
      return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
    }


    boolean LOCUS_StopLogger(void) {
      sendCommand(PMTK_LOCUS_STOPLOG);
      recvdflag = false;
      return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
    }


    boolean LOCUS_ReadStatus(void) {
      sendCommand(PMTK_LOCUS_QUERY_STATUS);
      
      if (! waitForSentence("$PMTKLOG"))
        return false;

      char *response = lastNMEA();
      uint16_t parsed[10];
      uint8_t i;
      
      for (i=0; i<10; i++) parsed[i] = -1;
      
      response = strchr(response, ',');
      for (i=0; i<10; i++) {
        if (!response || (response[0] == 0) || (response[0] == '*')) 
          break;
        response++;
        parsed[i]=0;
        while ((response[0] != ',') && 
         (response[0] != '*') && (response[0] != 0)) {
          parsed[i] *= 10;
          char c = response[0];
          if (isDigit(c))
            parsed[i] += c - '0';
          else
            parsed[i] = c;
          response++;
        }
      }
      LOCUS_serial = parsed[0];
      LOCUS_type = parsed[1];
      if (isAlpha(parsed[2])) {
        parsed[2] = parsed[2] - 'a' + 10; 
      }
      LOCUS_mode = parsed[2];
      LOCUS_config = parsed[3];
      LOCUS_interval = parsed[4];
      LOCUS_distance = parsed[5];
      LOCUS_speed = parsed[6];
      LOCUS_status = !parsed[7];
      LOCUS_records = parsed[8];
      LOCUS_percent = parsed[9];

      return true;
    }
    
    
    /************************
     * Member variables     *
     ************************/


    uint8_t  hour; 
    uint8_t  minute; 
    uint8_t  seconds; 
    uint8_t  year; 
    uint8_t  month; 
    uint8_t  day;
    uint16_t milliseconds;
    
    // Floating point latitude and longitude value in degrees.
    float latitude;
    float longitude;
    
    // Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
    // and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
    //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
    int32_t latitude_fixed; 
    int32_t longitude_fixed;
    
    float latitudeDegrees;
    float longitudeDegrees;
      
    float geoidheight; 
    float altitude;
    float speed;
    float angle;
    float magvariation;
    float HDOP;
    
    char lat;
    char lon;
    char mag;
    
    boolean fix;
    uint8_t fixquality;
    uint8_t satellites;

    uint16_t LOCUS_serial;
    uint16_t LOCUS_records;
    uint8_t  LOCUS_type;
    uint8_t  LOCUS_mode;
    uint8_t  LOCUS_config;
    uint8_t  LOCUS_interval;
    uint8_t  LOCUS_distance;
    uint8_t  LOCUS_speed;
    uint8_t  LOCUS_status;
    uint8_t  LOCUS_percent;
    
   private:
    boolean paused;
    Serial* gpsSerial;
    
    // parsing buffers

    // we double buffer: read one line in and leave one for the main program
    volatile char line1[ADAGPS_MAXLINELENGTH];
    volatile char line2[ADAGPS_MAXLINELENGTH];
    // our index into filling the current line
    volatile uint8_t lineidx;
    // pointers to the double buffers
    volatile char* currentline;
    volatile char* lastline;
    volatile boolean recvdflag;
    volatile boolean inStandbyMode;
    
};


#endif
