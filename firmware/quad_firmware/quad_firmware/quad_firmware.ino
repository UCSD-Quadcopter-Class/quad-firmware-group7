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
const int FL = 1;
const int FR = 2;
const int BL = 3;
const int BR = 4;

int value_to_read = -1;
int values[4] = {0, 0, 0, 0};

bool armable = false;
bool armed = false;

void throttle(int speed) {
  analogWrite(FR_PIN, speed);
  analogWrite(FL_PIN, speed);
  analogWrite(BR_PIN, speed);
  analogWrite(BL_PIN, speed);
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

void test_getQuad() {
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getQuad(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
//    Serial.print(F("Orientation: "));
//    Serial.print(orientation.roll);
//    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.gyro_z);
    Serial.println(F(""));
  }
  
  delay(100);
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

void loop()
{
//  test_eulerAngles();

  test_getQuad();
  
  if ( rfAvailable() ) {
    struct signals remote_values;
    rfRead( (uint8_t*) (&remote_values), sizeof(struct signals));
    if ( remote_values.magic != MAGIC_NUMBER ) {
      return;
    }

    if ( remote_values.button_flags & BUTTON1_MASK > 0 ) {
      armed = true;
    }
    
    if ( armed ) {
      throttle(remote_values.throttle);
    }
    
    char str[64];
    sprintf(str,"t%d\n",remote_values.button_flags);
//    Serial.print(str);
  }

}

