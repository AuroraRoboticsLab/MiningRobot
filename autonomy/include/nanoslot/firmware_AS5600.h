/*
 Talk to the AS5600 magnetic angle sensor, over I2C
  https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf

 Hookup:
 AS5600 connected to:
  Arduino Uno/Nano pins A4 (SDA) & A5 (SCL), power (3.3VDC), ground.
 Motor control servo connected to:
  Arduino pin 9 (servo signal) and ground.

 Limitations:
  Gets weird around the 0-4096 wraparound, so face the magnet the other way.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2025-02-20 (Public Domain)
*/
#ifndef __NANOSLOT_AS5600_H
#define __NANOSLOT_AS5600_H
#include <Wire.h>
#include <Servo.h>


const int AS5600_I2Caddr = 0x36; // AS5600 I2C address

void AS5600_begin() 
{
  Wire.begin();
  Wire.setWireTimeout(1000,true);
}

// Read a 16-bit big endian values from this I2C register.
unsigned int AS5600_readHex(int reg)
{
  Wire.beginTransmission(AS5600_I2Caddr);
  Wire.write(reg); 
  Wire.endTransmission();

  unsigned char data[2] = {0,0};
  int n = Wire.requestFrom(AS5600_I2Caddr,2);
  for (int i=0;i<n;i++) data[i] = Wire.read();
  return (data[0]<<8)|(data[1]);
}


// PID algorithm for angle control
class PIDcontroller
{
public:

    int last_error = 0; // for rate term
    float smooth_rate = 0; // smoothed version, less noisy
    int total_error = 0; // for integral term

    // Run arm actuator PID algorithm to produce a motor command, in microseconds
    //   ang is the current raw magnetic angle reading
    //   target is the target angle
    int get_command(int ang,int target)
    {
      int error = target - ang; // in raw 4096th of rotation
      
      int cur_rate = error - last_error;
      last_error = error;
      smooth_rate = 0.75*smooth_rate + 0.25*cur_rate;

      if (error>30 || error<-30) { // big error, history corrupted
        total_error=0; 
      } else { // small error, add to history
        total_error += error;

        // Limit total windup to avoid oscillations
        int windup = 200; 
        if (total_error>+windup) total_error=+windup;
        if (total_error<-windup) total_error=-windup;
      }

      float Kp = 1.5; // microseconds of servo command per angle error
      float Kd = 10; // microseconds of command per angle per tick
      float Ki = 0.1; // correct total error
      float command = Kp * error + Kd*smooth_rate + Ki*total_error; // zero centered, units microseconds

      // Set power limit to 'tepid'
      int power=100;
      if (command>power) command=power;
      if (command<-power) command=-power;

      return command;
    }
    
    // Center and limit this command value (microsecond servo command)
    int get_centered(int command,int center) const {
        // Convert command to microseconds
        command += center; // add zero point for servo

        // Limit microseconds to plausible values
        if (command<800) command=800;
        if (command>2200) command=2200;

        return command;
    }
};



#endif


