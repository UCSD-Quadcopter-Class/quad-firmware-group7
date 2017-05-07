#include <signals.h>

#include "radio.h"

// For the IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>

const float COMPLIMENTARY_CONST = .65;

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro() );

int count = 0;
bool done = false;
bool first = true;
unsigned long start;
byte data[32];
int idx = -1;

const int FR_PIN = 5;
const int FL_PIN = 3;
const int BR_PIN = 4;
const int BL_PIN = 8;

const int PITCH = 0;
const int PITCH_GYRO = 1;
const int ROLL = 2;
const int YAW = 3;

const int FL = 0;
const int FR = 1;
const int BL = 2;
const int BR = 3;

int value_to_read = -1;
int values[4] = {0, 0, 0, 0};
bool armable = false;
bool armed = false;

//PID VALS
const float Kp = .5;
const float Ki = 0;
const float Kd = .1;
float prev_error = 0;
float cur_error = 0;
//float errors[3][3];
float IMUvals[3];
int p_adj = 0;

void throttle(int speed) {
  int fr = speed + p_adj;
  int fl = speed + p_adj;
  int br = speed - p_adj + 2;
  int bl = speed - p_adj - 3;

  if ( fr < 0 ) fr = 0;
  if ( fl < 0 ) fl = 0;
  if ( br < 0 ) br = 0;
  if ( bl < 0 ) bl = 0;

  if ( fr > 255 ) fr = 255;
  if ( fl > 255 ) fl = 255;
  if ( br > 255 ) br = 255;
  if ( bl > 255 ) bl = 255;
  
  analogWrite(FR_PIN, fr);
  analogWrite(FL_PIN, fl);
  analogWrite(BR_PIN, br);
  analogWrite(BL_PIN, bl);
}

struct quad_values {
  float pitch;
  float roll;
  float gyro;
};

struct quad_values calibrated;

void adjust_imu(sensors_vec_t * vals) {
  vals->pitch -= calibrated.pitch;
  vals->roll -= calibrated.roll;
  vals->gyro_y -= calibrated.gyro;
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void readIMU() {
  sensors_vec_t   orientation;

  if (ahrs.getQuad(&orientation))
  {
    adjust_imu(&orientation);
    
    IMUvals[PITCH] = orientation.pitch;
    IMUvals[ROLL] = orientation.roll;
    IMUvals[PITCH_GYRO] = orientation.gyro_y;
  }

  //float new_angle = (1-COMPLIMENTARY_CONST) * (IMUvals[PITCH_GYRO]) + (COMPLIMENTARY_CONST)*IMUvals[PITCH];
  //Serial.print(new_angle);
  //Serial.print(" ");
  //Serial.print(IMUvals[PITCH]);
  //Serial.print(" ");
  //Serial.print(IMUvals[PITCH_GYRO]);
  //Serial.println(" ");
  

}



float decaying_error = 0;

void PID(struct signals* rvals) {
  float time_ms = 10.0;
  
  prev_error = cur_error;
  cur_error = rvals->pitch - IMUvals[PITCH];
  decaying_error /= 2;
  decaying_error += cur_error;
  
  float P = cur_error;
  float I = decaying_error;
  float D = (prev_error - cur_error);

  
  
  p_adj = Kp*P + Ki*I + Kd*D;

  Serial.print(p_adj);
  Serial.print(" ");
  Serial.print(Kp*P);
  Serial.print(" ");
  Serial.print(Ki*I);
  Serial.print(" ");
  Serial.print(Kd*D);
  Serial.println(" ");
}
 
void setup()
{
//  Serial.begin(9600);  // Start up serial
  Serial.begin(115200);
//  Serial1.begin(115200);
  rfBegin(19);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
    
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  setupSensor();
    
  Serial.println("Initiliazed");
}

void calibrate_values() {
  sensors_vec_t   orientation;
  
  if ( armable ) {
    // already zeroed
    return;
  } else {
      float pitches[10];
      float rolls[10];
      float gyros[10];
      for ( int i = 0; i < 10; i++ ) {
        delay(100);
        if (ahrs.getQuad(&orientation))
        {
          pitches[i] = orientation.pitch;
          rolls[i] = orientation.roll;
          gyros[i] = orientation.gyro_y;
        }
      }

      for ( int i = 0; i < 10; i++ ) {
        calibrated.pitch += pitches[i];
        calibrated.roll += rolls[i];
        calibrated.gyro += gyros[i];
      }
      
      calibrated.pitch /= 10;
      calibrated.roll /= 10;
      calibrated.gyro /= 10;

      armable = true;
  }
}

void loop()
{
  calibrate_values();
  readIMU();
  if ( rfAvailable() ) {
    struct signals remote_values;
    rfRead( (uint8_t*) (&remote_values), sizeof(struct signals));
    PID(&remote_values);
    if ( remote_values.magic != MAGIC_NUMBER ) {
      return;
    }

    if ( remote_values.button_flags & BUTTON1_MASK > 0 && armable) {
      armed = true;
    }
    
    if ( armed ) {
      throttle(remote_values.throttle);
    }
  }

}

