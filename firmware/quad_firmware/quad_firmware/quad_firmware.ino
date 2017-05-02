#include <signals.h>

#include "radio.h"

// For the IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

int count = 0;
bool done = false;
bool first = true;
unsigned long start;
byte data[32];
int idx = -1;

const int FR_PROP = 5;
const int FL_PROP = 3;
const int BR_PROP = 4;
const int BL_PROP = 8;

int value_to_read = -1;
int values[4] = {0, 0, 0, 0};

void throttle(int speed) {
  analogWrite(FR_PROP, speed);
  analogWrite(FL_PROP, speed);
  analogWrite(BR_PROP, speed);
  analogWrite(BL_PROP, speed);
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void test_imu()
{
  lsm.read();  /* ask it to read in the data */ 
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
  delay(200);
}

void test_eulerAngles() {
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
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
  test_eulerAngles();
  if ( rfAvailable() ) {
    struct signals remote_values;
    rfRead( (uint8_t*) (&remote_values), sizeof(struct signals));
    if ( remote_values.magic != MAGIC_NUMBER ) {
      return;
    }
    
    throttle(remote_values.throttle);
    char str[64];
    sprintf(str,"t%dy%dp%dr%d %d%d%d%d\0\n",remote_values.throttle,remote_values.yaw,remote_values.pitch,remote_values.roll,remote_values.pot1,remote_values.pot2);
    Serial.print(str);
  }
//  if (rfAvailable()) {
//    byte b = rfRead();
//    if ( (char)b == ' ' ) {
//      char str[32];
//      sprintf(str, "t %d, y %d, p %d, r %d\n\0", values[0], values[1], values[2], values[3]);
//      Serial.print(str);
//      memset(values, 0, 16);
//    }
//    
//    if ( value_to_read == -1 ) {
//      if ( (char)b == 't' ) {
//        value_to_read = 0;
//      } else if ( (char)b == 'y' ) {
//        value_to_read = 1;
//      } else if ( (char)b == 'p' ) {
//        value_to_read = 2;
//      } else if ( (char)b == 'r' ) {
//        value_to_read = 3;
//      } else {
//        value_to_read = -1;
//      }
//    } else {
//      if ( b >='0' && b <= '9' ) {
//        values[value_to_read] *= 10;
//        values[value_to_read] += (b-'0');
//      } else {
//        if ( (char)b == 't' ) {
//        value_to_read = 0;
//      } else if ( (char)b == 'y' ) {
//        throttle(values[0]);
//        value_to_read = 1;
//      } else if ( (char)b == 'p' ) {
//        value_to_read = 2;
//      } else if ( (char)b == 'r' ) {
//        value_to_read = 3;
//      } else {
//        value_to_read = -1;
//      }
//      }
//    }
//  }

}
