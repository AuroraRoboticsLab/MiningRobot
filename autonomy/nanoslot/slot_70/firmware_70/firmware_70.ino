/*
 Firmware that runs on robot arm joint
*/
#define NANOSLOT_MY_ID 0x70
#include "nanoslot/firmware.h"
#include "nanoslot/firmware_AS5600.h"
#include <Servo.h>

const int motorPin=9; // motor controller PWM pin
Servo motor;

const int pwmStop=1000; // microsecond RC PWM width for stop
const int pwmFull=200; // microsecond RC PWM difference for full power


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
  motor.writeMicroseconds(cmd);

  digitalWrite(13,(abs(cmd-pwmStop)>100)?1:0);
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  pinMode(13,OUTPUT); // blink pin

  motor.attach(motorPin); // motor controller command pin
  motor.write(pwmStop);
  
  AS5600_begin();
  
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
