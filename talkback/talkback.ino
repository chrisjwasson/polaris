#include "Wire.h"
// I2Cdev and MPU6050 must be installlibraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Matrix.h"
#include "Quat.h"

unsigned long timerSec = 0;
uint8_t msgBuffer[256];
char secondsElapsedString[256];

#define GYRO_SCALE_FACTOR 2000.0/32768.0
#define ACCEL_SCALE_FACTOR 16.0/32768.0*9.81;

#pragma pack(1)
typedef struct {
//  float timeElapsedSeconds;
//  char message[32];
  float ax, ay, az;
  float gx, gy, gz;
  float q1, q2, q3, q4;
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
float gyroBias1 = -2.70354652405;
float gyroBias2 = -0.267625242472;
float gyroBias3 = -0.714644670486;

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
  

void loop() {
  
  // increment loop counter
  //tmMessage.loopCounter++;
  
  // update sensor data if ready
  if (imu.getIntDataReadyStatus() == 1) {
    
    imu.getAcceleration(&a1, &a2, &a3);
    imu.getRotation(&g1,&g2,&g3);

    // 2.0 g full acceleration range with 16 bits
    tmMessage.ax = a1*ACCEL_SCALE_FACTOR;
    tmMessage.ay = a2*ACCEL_SCALE_FACTOR;
    tmMessage.az = a3*ACCEL_SCALE_FACTOR;
    
    // 250 deg/sec full rate range
    rateDegSec(0) = g1*GYRO_SCALE_FACTOR - gyroBias1;
    rateDegSec(1) = g2*GYRO_SCALE_FACTOR - gyroBias2;
    rateDegSec(2) = g3*GYRO_SCALE_FACTOR - gyroBias3;
    
    // transform to plane body frame
    rateDegSec = DCM_gyroAccelToBody*rateDegSec;
    
    // get time since last update
    currMicroSec = micros();
    if (currMicroSec < lastMicroSec) {
      deltaTSec = ((4294967295 - lastMicroSec) + currMicroSec)/1000000.0;
    }
    else {
      deltaTSec = (currMicroSec - lastMicroSec)/1000000.0;
    }
    lastMicroSec = currMicroSec; // reset timer
    tmMessage.deltaTimeSec = deltaTSec;
    
    // get time since last TM frame send
    if (currMicroSec < lastTmMicroSec) {
      tmDeltaTSec = ((4294967295 - lastTmMicroSec) + currMicroSec)/1000000.0;
    }
    else {
      tmDeltaTSec = (currMicroSec - lastTmMicroSec)/1000000.0;
    }

    
    
        
    // get attitude derivative quaternion
    attQuatDeriv = attQuat.getQuatDerivative(rateDegSec*M_PI/180.0);
    
    // perform integration for this step
    attQuat = attQuat + (attQuatDeriv*deltaTSec);
    
    // normalize attitude quaternion
    attQuat = attQuat.normalize();
    
    // save rate and quaternion information in TM message
    tmMessage.gx = rateDegSec(0);
    tmMessage.gy = rateDegSec(1);
    tmMessage.gz = rateDegSec(2);
    
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
    
    tmMessage.q1 = attQuat(0);
    tmMessage.q2 = attQuat(1);
    tmMessage.q3 = attQuat(2);
    tmMessage.q4 = attQuat(3);
    
    //tmMessage.deltaTimeSec = deltaTSec;
    
    // populate TM message
    //tmMessage.timeElapsedSeconds = millis()/1000.0;
    //strcpy(tmMessage.message,"hey chris this is hapnin\0");
  
    // send TM message if TM interval is reached
    if (tmDeltaTSec > 0.05) {
      lastTmMicroSec = currMicroSec;
      //tmMessage.deltaTimeSec = tmDeltaTSec;
      sendTmMessage(&tmMessage,sizeof(tmMessage));
    }
    
  }
  //delay(20);

}
