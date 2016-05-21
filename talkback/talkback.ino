#include "Wire.h"
// I2Cdev and MPU6050 must be installlibraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Matrix.h"
#include "Quat.h"
#include <Adafruit_GPS.h>
#include <AltSoftSerial.h>

// Define software serial object, used to communicate with gps.
// NOTE: this sketch is written assuming the xbee shield uses
// the UART. As long as this is true, we don't need to worry 
// swserial collisions.
AltSoftSerial swserial(5,4);

// Define GPS object
Adafruit_GPS<AltSoftSerial> gps(&swserial);

#define GYRO_SCALE_FACTOR 2000.0/32768.0
#define ACCEL_SCALE_FACTOR 16.0/32768.0*9.81;

// Define telemetry structure
#pragma pack(1)
typedef struct {
//  float timeElapsedSeconds;
//  char message[32];
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
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
float q1,q2,q3,q4;
float deltaTSec;
float tmDeltaTSec;
uint32_t lastMicroSec;
uint32_t currMicroSec;
uint32_t lastTmMicroSec;
Quat q_body2ned;
Quat qdot_body2ned;
Vec3d a_body;
Vec3d w_body;
Vec3d m_body;
Vec3d g_ned_true;
Vec3d mhat_ned_true;
uint32_t gyroBiasCounter = 0;

// ===============================================================
// declare local configuration values
// TODO: shouldn't have to hard code these

// determined by averaging over long time
float gyroBias1 = -2.843;
float gyroBias2 = 1.484;
float gyroBias3 = -1.207;

// rotated each axis to point toward and away magnetic north
// max:  [  148  235   90 ]
// min:  [ -112  -30 -155 ]
// bias: [   18  102 -32.5 ]
// scale:[ 260   265  245 ]
float magnetBias1 = 20;
float magnetBias2 = 105;
float magnetBias3 = -30;
float magnetScaleMod1 = 2.0/260;
float magnetScaleMod2 = 2.0/265;
float magnetScaleMod3 = 2.0/245;

// from http://www.magnetic-declination.com/
float mag_dec =  (12.0 + 7.0/60.0);
float mag_inc =  (58.0 + 38.0/60.0);
// ===============================================================


// ===============================================================
// filter coefficients, gains, guidance parameters

// made up. seem to work okay.
float k_magnet = 0.1; // fraction per second
float k_grav = 0.1; // fraction per second

// ===============================================================



Matrix<double,3,3> DCM_gyroAccelToBody;
Matrix<double,3,3> DCM_MagToBody;


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
  
  // define gyro+accel to body frame
  DCM_gyroAccelToBody(0,0) = -1;
  DCM_gyroAccelToBody(0,1) = 0;
  DCM_gyroAccelToBody(0,2) = 0;
  DCM_gyroAccelToBody(1,0) = 0;
  DCM_gyroAccelToBody(1,1) = 1;
  DCM_gyroAccelToBody(1,2) = 0;
  DCM_gyroAccelToBody(2,0) = 0;
  DCM_gyroAccelToBody(2,1) = 0;
  DCM_gyroAccelToBody(2,2) = -1;

  // define magnetometer to body frame
  DCM_MagToBody(0,0) = 0;
  DCM_MagToBody(0,1) = -1;
  DCM_MagToBody(0,2) = 0;
  DCM_MagToBody(1,0) = 1;
  DCM_MagToBody(1,1) = 0;
  DCM_MagToBody(1,2) = 0;
  DCM_MagToBody(2,0) = 0;
  DCM_MagToBody(2,1) = 0;
  DCM_MagToBody(2,2) = 1;

  // initialize nav stuff
  g_ned_true(0) = 0.0;
  g_ned_true(1) = 0.0;
  g_ned_true(2) = 9.80665;

  mhat_ned_true(0) = cos(mag_dec * PI/180) * cos(mag_inc * PI/180);
  mhat_ned_true(1) = sin(mag_dec * PI/180) * cos(mag_inc * PI/180);
  mhat_ned_true(2) = sin(mag_inc * PI/180);
  
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

  // return if new data received
  return newdata;
}

bool handleIMU()
{
  if (imu.getIntDataReadyStatus() == 1) {

    // read data
    imu.getAcceleration(&a1,&a2,&a3);
    imu.getRotation(&g1,&g2,&g3);
    imu.getMag(&m1,&m2,&m3);

    // stuff accels into vector
    a_body(0) = a1*ACCEL_SCALE_FACTOR;
    a_body(1) = a2*ACCEL_SCALE_FACTOR;
    a_body(2) = a3*ACCEL_SCALE_FACTOR;
    
    // 250 deg/sec full rate range
    w_body(0) = g1*GYRO_SCALE_FACTOR - gyroBias1;
    w_body(1) = g2*GYRO_SCALE_FACTOR - gyroBias2;
    w_body(2) = g3*GYRO_SCALE_FACTOR - gyroBias3;

    // stuff magnets into vector
    m_body(0) = (m1 - magnetBias1)*magnetScaleMod1;
    m_body(1) = (m2 - magnetBias2)*magnetScaleMod2;
    m_body(2) = (m3 - magnetBias3)*magnetScaleMod3;
    
    // transform stuff plane body frame
    a_body = DCM_gyroAccelToBody*a_body;
    w_body = DCM_gyroAccelToBody*w_body;
    m_body = DCM_MagToBody*m_body;

    return true;
  }
  else
  {
    return false;
  }
}

bool hackyNav()
{      
  // get time since last imu update
  if (currMicroSec < lastMicroSec) {
    deltaTSec = ((4294967295 - lastMicroSec) + currMicroSec)/1000000.0;
  }
  else {
    deltaTSec = (currMicroSec - lastMicroSec)/1000000.0;
  }
  lastMicroSec = currMicroSec; // reset timer


  // -----------------------------------------------------------------
  // Integrate angular velocity
  // -----------------------------------------------------------------
  // get attitude derivative quaternion
  qdot_body2ned = q_body2ned.getQuatDerivative(w_body*M_PI/180.0);
  
  // perform integration for this step
  q_body2ned = q_body2ned + (qdot_body2ned*deltaTSec);
  
  // normalize attitude quaternion
  q_body2ned = q_body2ned.normalize();

  // -----------------------------------------------------------------
  // Absolute attitude correction
  // -----------------------------------------------------------------

  // OH GOD THIS IS ALL SO HACKY
  
  // Correct towards north
  Vec3d mhat_body_calc = q_body2ned.inverse().rotateCsys(mhat_ned_true);
  Vec3d mhat_body_meas = m_body/m_body.norm();
  double theta_mhat = angle_between(mhat_body_calc, mhat_body_meas);
  Vec3d rvec_mhat = cross(mhat_body_meas, mhat_body_calc).unit();
  
  Quat q_mbody2body = Quat(rvec_mhat, 
                               theta_mhat * k_magnet * deltaTSec);

  q_body2ned = q_body2ned * q_mbody2body;

  // Correct towards down
  Vec3d ghat_body_calc = q_body2ned.inverse().rotateCsys(g_ned_true.unit());
  Vec3d ghat_body_meas = a_body/a_body.norm() * -1;
  double theta_ghat = angle_between(ghat_body_calc, ghat_body_meas);
  Vec3d rvec_ghat = cross(ghat_body_meas, ghat_body_calc).unit();
  
  Quat q_gbody2body = Quat(rvec_ghat, 
                               theta_ghat * k_grav * deltaTSec);

  q_body2ned = q_body2ned * q_gbody2body;

//  Serial.print("theta_mhat: ");
//  Serial.print(theta_mhat*180/PI);
//  Serial.print("   theta_ghat: ");
//  Serial.print(theta_ghat*180/PI);
//  Serial.println();

  // -----------------------------------------------------------------
  // Integrate accel
  // -----------------------------------------------------------------

  
}

bool handleTelem()
{ 
  // increment loop counter
  //tmMessage.loopCounter++;

  // magnets!
  tmMessage.mx = m_body(0);
  tmMessage.my = m_body(1);
  tmMessage.mz = m_body(2);

  // 2.0 g full acceleration range with 16 bits
  tmMessage.ax = a_body(0);
  tmMessage.ay = a_body(1);
  tmMessage.az = a_body(2);

  // rate data
  tmMessage.gx = w_body(0);
  tmMessage.gy = w_body(1);
  tmMessage.gz = w_body(2);
  
  // attitude estimate
  tmMessage.q1 = q_body2ned(0);
  tmMessage.q2 = q_body2ned(1);
  tmMessage.q3 = q_body2ned(2);
  tmMessage.q4 = q_body2ned(3);

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

void debugBias()
{
    // compute gyro bias (continue averaging)
    gyroBias1 *= gyroBiasCounter;
    gyroBias2 *= gyroBiasCounter;
    gyroBias3 *= gyroBiasCounter;
    gyroBias1 += g1*GYRO_SCALE_FACTOR;
    gyroBias2 += g2*GYRO_SCALE_FACTOR;
    gyroBias3 += g3*GYRO_SCALE_FACTOR;
    gyroBiasCounter++;
    gyroBias1 /= gyroBiasCounter;
    gyroBias2 /= gyroBiasCounter;
    gyroBias3 /= gyroBiasCounter;
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.print("X: ");
    Serial.println(gyroBias1,3);
    Serial.print("Y: ");
    Serial.println(gyroBias2,3);
    Serial.print("Z: ");
    Serial.println(gyroBias3,3);
}

// print in console-friendly format for direct usb debugging
void debugIMU()
{
  // print accels all o'er
  if (false){
      char buf [7];
      Serial.print("accels: [ ");
      dtostrf(a_body(0),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(a_body(1),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(a_body(2),5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
  }
  
  // print rate
  if (false){
      char buf [7];
      Serial.print("rate: [ ");
      dtostrf(w_body(0),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(w_body(1),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(w_body(2),5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
  }

  // print magnets all o'er
  if (true){
      char buf [7];
      Serial.print("magnets: [ ");
//      dtostrf(m_body(0),5,1,buf);
      dtostrf(m1,5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
//      dtostrf(m_body(1),5,1,buf);
      dtostrf(m2,5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
//      dtostrf(m_body(2),5,1,buf);
      dtostrf(m3,5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
  }
  
  // print quat att
  if (false){
      char buf [7];
      Serial.print("quat: [ ");
      dtostrf(q_body2ned(0),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(q_body2ned(1),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(q_body2ned(2),5,1,buf);
      Serial.print(buf);
      Serial.print(" ");
      dtostrf(q_body2ned(3),5,1,buf);
      Serial.print(buf);
      Serial.println(" ]");
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

  // navigate
  hackyNav();

  // handle telem
  handleTelem();

  // debug...
//  debugGPS();
//  debugBias();
//  debugIMU();
    
  //delay(20);

}
