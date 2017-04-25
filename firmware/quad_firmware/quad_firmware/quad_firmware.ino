#include "radio.h"

int count = 0;
bool done = false;
bool first = true;
unsigned long start;
byte data[32];
int idx = -1;

int value_to_read = -1;
int values[4] = {0, 0, 0, 0};

void setup()
{
  Serial.begin(9600);  // Start up serial
  Serial1.begin(115200);
  rfBegin(19);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

  // Send a message to other RF boards on this channel
  Serial.println("Initiliazed");
}

void loop()
{
  if (rfAvailable()) {
    byte b = rfRead();
    if ( (char)b == ' ' ) {
      char str[32];
      sprintf(str, "t %d, y %d, p %d, r %d\n\0", values[0], values[1], values[2], values[3]);
      Serial.print(str);
      memset(values, 0, 16);
    }
    
    if ( value_to_read == -1 ) {
      if ( (char)b == 't' ) {
        value_to_read = 0;
      } else if ( (char)b == 'y' ) {
        value_to_read = 1;
      } else if ( (char)b == 'p' ) {
        value_to_read = 2;
      } else if ( (char)b == 'r' ) {
        value_to_read = 3;
      } else {
        value_to_read = -1;
      }
    } else {
      if ( b >='0' && b <= '9' ) {
        values[value_to_read] *= 10;
        values[value_to_read] += (b-'0');
      } else {
        if ( (char)b == 't' ) {
        value_to_read = 0;
      } else if ( (char)b == 'y' ) {
        analogWrite(8, values[0]);
        value_to_read = 1;
      } else if ( (char)b == 'p' ) {
        value_to_read = 2;
      } else if ( (char)b == 'r' ) {
        value_to_read = 3;
      } else {
        value_to_read = -1;
      }
      }
    }
  }
  
}
