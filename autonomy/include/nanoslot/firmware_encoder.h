#ifndef NANOSLOT_FIRMWARE_ENCODER_H
#define NANOSLOT_FIRMWARE_ENCODER_H

#include "firmware_milli.h"
#include "Arduino.h"

/**
  Read an optical encoder value, and reconstruct the
  speed of the changes being seen.
*/
class encoder_t{
  public:
   int pin; // Arduino analog pin to read light sensor
   int value; // value from sensor
   int old_value; // Value from last read 
   uint16_t count_mono; // total number of changes seen (monotonic)
   uint16_t count_dir;  // encoder count including up/down directions
   int16_t last_dir; // +1 for counting upward, -1 for counting downward, 0 for no direction
   
   encoder_t(int pin_)
     :pin(pin_)
   {
     pinMode(pin_,INPUT_PULLUP);
     count_mono=0;
     count_dir=0;
     last_dir=0;
     old_value=0;
     value=0;
   }

   void read()
   {
      old_value = value;
      //if (pin!=A6) {
        value = digitalRead(pin);
      //} else {
      //  value = analogRead(pin)>100; // A6 is digital only
      //}
      
      if (value!=old_value)
      {
        // edge detected!
        ++count_mono;
        count_dir+=last_dir;
      }
    }
};

#endif
