#include <signals.h>

#include "radio.h"

// For the IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>

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
const float Kp = 1;
const float Ki = 1;
const float Kd = 0.5;
float prev_error = 0;
float cur_errpr = 0;
//float errors[3][3];
float IMUvals[3];
int p_adj = 0;

void throttle(int speed) {
  analogWrite(FR_PIN, speed + p_adj);
  analogWrite(FL_PIN, speed + p_adj);
  analogWrite(BR_PIN, speed - p_adj);
  analogWrite(BL_PIN, speed - p_adj);
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
  vals->gyro_z -= calibrated.gyro;
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
    IMUvals[PITCH_GYRO] = orientation.gyro_z;
  }
}

void PID(struct signals* rvals) {
  prev_error = cur_error;
  cur_error = rvals->pitch - IMUvals[PITCH];
  float time_ms = 100.0;
  float P = cur_error;
  float I = cur_error * time_ms/1000;
  float D = (prev_error - cur_error) / (time_ms/1000);
  
  p_adj = Kp*P + Ki*I + Kd*D;
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
        if (ahrs.getQuad(&orientation))
        {
          pitches[i] = orientation.pitch;
          rolls[i] = orientation.roll;
          gyros[i] = orientation.gyro_z;
        }
        delay(100);
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

