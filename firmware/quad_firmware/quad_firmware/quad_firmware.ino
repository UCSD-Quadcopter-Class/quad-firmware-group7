/*
  ATmega128RFA1 Dev Board Basic Chat
  by: Jim Lindblom
      SparkFun Electronics
  date: July 3, 2012
  License: Beerware. Feel free to use, reuse, and modify this code
  as you please. If you find it useful, you can buy me a beer.

  This code sets up the ATmega128RFA1's wireless transciever in
  the most basic way possible to serve as a serial gateway.
  Serial into the ATmega128RFA1's UART0 will go out the RF radio.
  Data into the RF radio will go out the MCU's UART0.
*/

#include "radio.h"

int count = 0;
bool done = false;
bool first = true;
unsigned long start;
byte data[32];
int idx = -1;

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

  if (rfAvailable())  // If data receievd on radio...
  {
    byte b = rfRead();
    if ((char)b == 'x') {
      // about to receive new data, so reset the buffer
      idx = 0;
      if (first) {
        Serial.println("recieved first x");
      }

      Serial.println("resetting buffer");

    }
    else if(idx >= 0){
      data[idx] = b;
      
      if (idx == 3) {
      // we filled buffer with bytes
      int *intArray = (int)data;
     
      Serial.println("Printing data");
      Serial.print(intArray[0]);
      Serial.println("");
    }
    if(idx == 3){
      idx = 0;
    }
    else {
      idx++;
    }
//      Serial.print(idx);
//      Serial.println("");
    }
    

    
  }
}
