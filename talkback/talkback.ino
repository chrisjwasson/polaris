#include "Wire.h"
// I2Cdev and MPU6050 must be installlibraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Matrix.h"
#include "Quat.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Define software serial object, used to communicate with gps.
// NOTE: this sketch is written assuming the xbee shield uses
// the UART. As long as this is true, we don't need to worry 
// swserial collisions.
SoftwareSerial swserial(5,4);

// Define GPS object
Adafruit_GPS<SoftwareSerial> gps(&swserial);

#define GYRO_SCALE_FACTOR 2000.0/32768.0
#define ACCEL_SCALE_FACTOR 16.0/32768.0*9.81;

// Define telemetry structure
#pragma pack(1)
typedef struct {
//  float timeElapsedSeconds;
//  char message[32];
  float ax, ay, az;
  float gx, gy, gz;
  float q1, q2, q3, q4;
  float lat, lon, alt;
  float deltaTimeSec;
//  uint32_t loopCounter;
} TmMessageStruct;
#pragma pack()

TmMessageStruct tmMessage;

// declare IMU class
MPU6050 imu;

// declare variables for sensor measurements
int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;
float mx,my,mz;
float q1,q2,q3,q4;
float deltaTSec;
float tmDeltaTSec;
uint32_t lastMicroSec;
uint32_t currMicroSec;
uint32_t lastTmMicroSec;
Quat attQuat;
Quat attQuatDeriv;
Matrix<double,3,1> rateDegSec;
uint32_t gyroBiasCounter = 0;
float gyroBias1 = -1.459;
float gyroBias2 = 0.794;
float gyroBias3 = -0.597;

Matrix<double,3,3> DCM_gyroAccelToBody;


// computes the checksum of the telemetry struct by computing a 1-byte parity word (XOR of all 1-byte words in frame)
uint8_t computeChecksum(void* ptr, int sizeBytes) {
  
  uint8_t checksum = 0;
  for (int ii = 0; ii < sizeBytes; ii++) {
    checksum = checksum^(((uint8_t*) ptr)[ii]);
  }
  
  return checksum;
  
}

void sendTmMessage(void* tmMessagePtr, int sizeBytes) {
  
  uint8_t checksum = computeChecksum(tmMessagePtr,sizeBytes);
  uint16_t preAmble = 0xABCD;
  Serial.write((uint8_t*) (&preAmble),2);
  Serial.write((uint8_t*) tmMessagePtr,sizeBytes);
  Serial.write(checksum);
  
}

void setup() {

  Wire.begin();
  
  Serial.begin(57600);

  // initialize GPS
  // RMCGGA = 
  gps.begin(9600);
  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // Request updates on antenna status, comment out to keep quiet
//  GPS.sendCommand(PGCMD_ANTENNA);
  
  // initialize IMU
  imu.initialize();
  
  // verify connection
  Serial.print("Testing device connection...");
  Serial.print(imu.testConnection() ? F("Success") : F("Failed"));
  
  // set accel, gyro, and magneto sampling rates
  imu.setRate(7); // gyro rate 1 kHz, accel is 1 kHz
  
  // initialize loop counter
  //tmMessage.loopCounter = 0;
  
  // initialize timing variables
  lastMicroSec = micros();
  lastTmMicroSec = micros();
  
  // define sensor to body frame
  DCM_gyroAccelToBody(0,0) = -1;
  DCM_gyroAccelToBody(0,1) = 0;
  DCM_gyroAccelToBody(0,2) = 0;
  DCM_gyroAccelToBody(1,0) = 0;
  DCM_gyroAccelToBody(1,1) = 1;
  DCM_gyroAccelToBody(1,2) = 0;
  DCM_gyroAccelToBody(2,0) = 0;
  DCM_gyroAccelToBody(2,1) = 0;
  DCM_gyroAccelToBody(2,2) = -1;
  

}

// passes gps messages directly to hard serial
void debugGPS()
{
  while (char c = gps.read())
      Serial.print(c);

  if (gps.newNMEAreceived())
  {
    Serial.println("new nmea message recieved");
    if (gps.parse(gps.lastNMEA()))
      Serial.println("message parsed");
  }
}

uint32_t debug_timer = millis();
bool handleGPS()
{

  // if millis() or timer wraps around, we'll just reset it
  // (should only happen once every 50 days)
  if (debug_timer > millis())  debug_timer = millis();

//  int32_t t0 = millis();
  while (char c = gps.read())
  {
//      Serial.print(c);
  }
//  int32_t t1 = millis();
//  if (t1 - t0 > 2)
//    Serial.println(t1 - t0);

  bool newdata = false;
  if (gps.newNMEAreceived())
  {
    if (gps.parse(gps.lastNMEA()))
    {
      newdata = true;
    }
  }

  // approximately every 2 seconds or so, print out the current stats
  bool print_gps_debug = false;
  if (   newdata
      && (millis() - debug_timer > 2000)
      && print_gps_debug)
  {
    debug_timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(gps.hour, DEC); Serial.print(':');
    Serial.print(gps.minute, DEC); Serial.print(':');
    Serial.print(gps.seconds, DEC); Serial.print('.');
    Serial.println(gps.milliseconds);
    Serial.print("Date: ");
    Serial.print(gps.day, DEC); Serial.print('/');
    Serial.print(gps.month, DEC); Serial.print("/20");
    Serial.println(gps.year, DEC);
    Serial.print("Fix: "); Serial.print((int)gps.fix);
    Serial.print(" quality: "); Serial.println((int)gps.fixquality); 
    if (gps.fix) {
      Serial.print("Location: ");
      Serial.print(gps.latitude, 4); Serial.print(gps.lat);
      Serial.print(", "); 
      Serial.print(gps.longitude, 4); Serial.println(gps.lon);
      
      Serial.print("Speed (knots): "); Serial.println(gps.speed);
      Serial.print("Angle: "); Serial.println(gps.angle);
      Serial.print("Altitude: "); Serial.println(gps.altitude);
      Serial.print("Satellites: "); Serial.println((int)gps.satellites);
    }
  }

  // return
  return newdata;
}

bool handleIMU()
{
  if (imu.getIntDataReadyStatus() == 1) {

    // read data
    imu.getAcceleration(&a1,&a2,&a3);
    imu.getRotation(&g1,&g2,&g3);
    
    // 250 deg/sec full rate range
    rateDegSec(0) = g1*GYRO_SCALE_FACTOR - gyroBias1;
    rateDegSec(1) = g2*GYRO_SCALE_FACTOR - gyroBias2;
    rateDegSec(2) = g3*GYRO_SCALE_FACTOR - gyroBias3;
    
    // transform to plane body frame
    rateDegSec = DCM_gyroAccelToBody*rateDegSec;
    
    // get time since last imu update
    if (currMicroSec < lastMicroSec) {
      deltaTSec = ((4294967295 - lastMicroSec) + currMicroSec)/1000000.0;
    }
    else {
      deltaTSec = (currMicroSec - lastMicroSec)/1000000.0;
    }
    lastMicroSec = currMicroSec; // reset timer
        
    // get attitude derivative quaternion
    attQuatDeriv = attQuat.getQuatDerivative(rateDegSec*M_PI/180.0);
    
    // perform integration for this step
    attQuat = attQuat + (attQuatDeriv*deltaTSec);
    
    // normalize attitude quaternion
    attQuat = attQuat.normalize();
    
    /*
    // compute gyro bias (continue averaging)
    gyroBias1 *= gyroBiasCounter;
    gyroBias2 *= gyroBiasCounter;
    gyroBias3 *= gyroBiasCounter;
    gyroBias1 += rateDegSec(0);
    gyroBias2 += rateDegSec(1);
    gyroBias3 += rateDegSec(2);
    gyroBiasCounter++;
    gyroBias1 /= gyroBiasCounter;
    gyroBias2 /= gyroBiasCounter;
    gyroBias3 /= gyroBiasCounter;
    tmMessage.q1 = gyroBias1;
    tmMessage.q2 = gyroBias2;
    tmMessage.q3 = gyroBias3;
    */
  }

  // print in console-friendly format for direct usb debugging
  if (false){
      char buf [7];
      Serial.print("rate: [ ");
      dtostrf(rateDegSec(0),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(rateDegSec(1),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(rateDegSec(2),5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
  }
  if (false){
      char buf [7];
      Serial.print("quat: [ ");
      dtostrf(attQuat(0),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(attQuat(1),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(attQuat(2),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(attQuat(3),5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
  }
}

bool handleTelem()
{ 
  // increment loop counter
  //tmMessage.loopCounter++;

  // 2.0 g full acceleration range with 16 bits
  tmMessage.ax = a1*ACCEL_SCALE_FACTOR;
  tmMessage.ay = a2*ACCEL_SCALE_FACTOR;
  tmMessage.az = a3*ACCEL_SCALE_FACTOR;

  // rate data
  tmMessage.gx = rateDegSec(0);
  tmMessage.gy = rateDegSec(1);
  tmMessage.gz = rateDegSec(2);
  
  // attitude estimate
  tmMessage.q1 = attQuat(0);
  tmMessage.q2 = attQuat(1);
  tmMessage.q3 = attQuat(2);
  tmMessage.q4 = attQuat(3);

  // translation state
  tmMessage.lat = gps.latitude;
  tmMessage.lon = gps.longitude;
  tmMessage.alt = gps.altitude;

  // get time since last TM frame send
  if (currMicroSec < lastTmMicroSec) {
    tmDeltaTSec = ((4294967295 - lastTmMicroSec) + currMicroSec)/1000000.0;
  }
  else {
    tmDeltaTSec = (currMicroSec - lastTmMicroSec)/1000000.0;
  }

  // timestep
  tmMessage.deltaTimeSec = deltaTSec;
  //tmMessage.deltaTimeSec = tmDeltaTSec;

  // send TM message if TM interval is reached
  if (tmDeltaTSec > 0.05) {
    lastTmMicroSec = currMicroSec;
    sendTmMessage(&tmMessage,sizeof(tmMessage));
  }
}

void loop() {

  // main timer
  // TODO: handle rollover
  currMicroSec = micros();

  // handle gps
  handleGPS();

  // handle imu
  handleIMU();

  // handle telem
  handleTelem();

  // debug...
//  debugGPS();
    
  //delay(20);

}
