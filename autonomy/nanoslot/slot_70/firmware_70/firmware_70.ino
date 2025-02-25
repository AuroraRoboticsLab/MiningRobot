/*
 Firmware that runs on robot arm joint
*/
#define NANOSLOT_MY_ID 0x70
#include "nanoslot/firmware.h"
#include "nanoslot/firmware_AS5600.h"

const int motorPin=9; // motor controller PWM pin

#define USE_SERVO_H 1 /* generate motor PWM pulses with Servo.h (0 = use delays) */
#if USE_SERVO_H
#include <Servo.h>
Servo motor;
#endif

const int pwmStop=1010; // microsecond RC PWM width for stop
const int pwmFull=150; // microsecond RC PWM difference for full power


void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  
  unsigned int ang=AS5600_readHex(0x0E);
  unsigned int mag=AS5600_readHex(0x1B);
  
  my_sensor.mag[0]=(mag>>4);
  if (mag>100) {
      my_sensor.angle[0]=ang;
  }
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.autonomy.mode=0;
  
  int cmd=pwmStop;
  if (my_command.autonomy.mode!=0) { // torque control
    cmd = pwmStop + pwmFull*(long)my_command.torque[0]/100;
  }  
  if (0) { // PID angle control
    static PIDcontroller ctrl;
    cmd = ctrl.get_centered(ctrl.get_command(my_sensor.angle[0],my_command.target[0]),pwmStop);
  }

#if USE_SERVO_H
  motor.writeMicroseconds(cmd);
#else // busywait: allows faster PWM rate, but less reliable
  digitalWrite(motorPin,1); // RC PWM pulse
  delayMicroseconds(cmd); 
  digitalWrite(motorPin,0); // end pulse
#endif

  digitalWrite(13,(abs(cmd-pwmStop)>20)?1:0);
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  pinMode(13,OUTPUT); // blink pin

#if USE_SERVO_H
  motor.attach(motorPin); // motor controller command pin
  motor.write(pwmStop);
#else
  pinMode(motorPin,OUTPUT); digitalWrite(motorPin,0);
#endif

  AS5600_begin();
  
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop(10); //<- 10ms cycle target -> 100Hz motor updates
}
