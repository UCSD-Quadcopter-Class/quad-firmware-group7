/* signals.h
 * 
 * Contains the signals struct that contains the values from the remote.
 * A magic number is used to prevent reading noise in the ether--this 
 * value will be 0xBEEF
 *
 */

const uint16_t MAGIC_NUMBER = 0xBEEF;
const uint8_t BUTTON1_MASK = 0x01;
const uint8_t BUTTON2_MASK = 0x02;

struct signals {
  uint16_t magic;
  uint8_t throttle;
  uint8_t yaw;
  uint8_t roll;
  uint8_t pitch;
  uint8_t pot1;
  uint8_t pot2;
  uint8_t button_flags;
} signals_t;


