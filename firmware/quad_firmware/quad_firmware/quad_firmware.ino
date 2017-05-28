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

const int TWEAK_COMP_CONST = 1;
const int TWEAK_KP = 2;
const int TWEAK_KD = 3;
const int TWEAK_KI = 4;

bool tweak = true;

int tweak_pot1 = TWEAK_KD;
int tweak_pot2 = TWEAK_KI;

float pot1_factor = 1000.0;
float pot2_factor = 10.0;

// MOTOR PIN CONSTANTS
const int FR_PIN = 5;
const int BR_PIN = 4;
const int FL_PIN = 3;
const int BL_PIN = 8;

// VALUES
const int PITCH = 0;
const int PITCH_GYRO = 1;
const int ROLL = 2;
const int YAW = 3;

int value_to_read = -1;
int values[4] = {0, 0, 0, 0};
struct signals remote_values;

//PID VALS
//  P      D       I
//  .19   .02     .01
//  .21   .13     .007
// .19    .07
//  .19   .085     .1
float Kp = 0.23;
float Kd = 0.09;
float Ki = 0.70;

//float COMP_CONST = .40;
float COMP_CONST = .2;

float prev_pitch = 0.0;
float prev_error = 0;
float prev_time = 0;
float cur_error = 0;
float cur_time = 0;
float dt = 0;

float IMUvals[3];
int p_adj = 0;

void throttle() {
  int speed = remote_values.throttle;
  int fr = speed + p_adj - 8;
  int fl = speed + p_adj - 8;

  int br = speed - p_adj + 8;
  int bl = speed - p_adj + 8;

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
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void readIMU() {
  sensors_vec_t   orientation;

  if (ahrs.getQuad(&orientation))
  {
    adjust_imu(&orientation);
    
    IMUvals[PITCH] = orientation.pitch;
    IMUvals[ROLL] = orientation.roll;
    IMUvals[PITCH_GYRO] = -orientation.gyro_y;
  }

  prev_time = cur_time;
  cur_time = millis();
  dt = cur_time - prev_time;

  // complemetary filter
  // add in previous value to first term
  float new_angle = ((1.0-COMP_CONST) * (IMUvals[PITCH_GYRO] * dt/1000.0 + prev_pitch)) + (COMP_CONST * IMUvals[PITCH]);
  prev_pitch = IMUvals[PITCH];
  IMUvals[PITCH] = new_angle;
}

float decaying_error = 0;

void PID() { 

  prev_error = cur_error;
  cur_error = 0.0 - IMUvals[PITCH];
  decaying_error += cur_error/16.0;
  
  // Caps decaying error
  if(decaying_error > 100.0) decaying_error = 100.0;
  if(decaying_error < -100.0) decaying_error = -100.0;
  
  float P = cur_error;
  float D = (cur_error - prev_error)/ dt * 1000.0;

  // cap from -100 - 100 after multiplying in dt
  float I = Ki * decaying_error * dt / 1000.0;

 

  p_adj = Kp*P + I + Kd*D;
  Serial.print(Kp*P);
  Serial.print(" ");
  Serial.print(I);
  Serial.print(" ");
  Serial.print(Kd*D);
  Serial.print(" ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Kd);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.println(" ");
}
 
void setup()
{
//  Serial.begin(9600);  // Start up serial
  Serial.begin(9600);
//  Serial1.begin(115200);
  rfBegin(18);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
    
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  setupSensor();
    
  //Serial.println("Initiliazed");
  delay(1000);
  calibrate_values();
  cur_time = millis();
}

void calibrate_values() {
  sensors_vec_t   orientation;
  float pitches[10];
  float rolls[10];
  float gyros[10];
  
  for ( int i = 0; i < 10; i++ ) {
    delay(100);
    if (ahrs.getQuad(&orientation))
    {
      pitches[i] = orientation.pitch;
      rolls[i] = orientation.roll;
      gyros[i] = -orientation.gyro_y;
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
}

void radio() {
  if ( rfAvailable() ) {
    rfRead( (uint8_t*) (&remote_values), sizeof(struct signals));
    if ( remote_values.magic != MAGIC_NUMBER ) {
      return;
    }
    
    tweak_values();
  }
}

// things to tweak:
// 1. complimentary const
// 2. Kp
// 3. Kd
// 4. Ki
// other values:
// pot1 reduction, pot2 reduction


void tweak_values()
{
  if (!tweak) return;
  if ( tweak_pot1 == TWEAK_COMP_CONST ) {
    COMP_CONST = ((float)remote_values.pot1 / 100.0);
  } else if ( tweak_pot1 == TWEAK_KP ) {
    Kp = ((float)remote_values.pot1 - 1.0) / pot1_factor;
  } else if ( tweak_pot1 == TWEAK_KD ) {
    Kd = ((float)remote_values.pot1 - 1.0) / pot1_factor;
  } else if ( tweak_pot1 == TWEAK_KI ) {
    Ki = ((float)remote_values.pot1 - 1.0) / pot1_factor;
  }

  if ( tweak_pot2 == TWEAK_COMP_CONST ) {
    COMP_CONST = ((float)remote_values.pot2 / 100.0);
  } else if ( tweak_pot2 == TWEAK_KP ) {
    Kp = ((float)remote_values.pot2 - 2.0) / pot2_factor;
  } else if ( tweak_pot2 == TWEAK_KD ) {
    Kd = ((float)remote_values.pot2 - 2.0) / pot2_factor;
  } else if ( tweak_pot2 == TWEAK_KI ) {
    Ki = ((float)remote_values.pot2 - 2.0) / pot2_factor;
  }
}

void print_values()
{

}

void loop()
{  
  readIMU();
  radio();
  PID();
  throttle();
}

