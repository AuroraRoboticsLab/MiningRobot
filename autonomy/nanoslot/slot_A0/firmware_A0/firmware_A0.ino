/*
 Firmware that runs on robot arm board 0
*/
#define NANOSLOT_MY_ID 0xA0 /* robot Arm board 0 */
#include "nanoslot/firmware.h"

#define NeoPIN        A1 /* neopixel chain is on a 3-pin header */
#include "nanoslot/firmware_neopixel.h"

#include "nanoslot/firmware_un178.h"
/* Hardware-connected motor drivers, UN178 green brushed boards */
un178_motor_single_t hardware_motor[NANOSLOT_COMMAND_MY::n_motors]={
    //un178_motor_single_t(11,12,A0),
    //un178_motor_single_t(10,9,8),
    un178_motor_single_t(3,2,4),
    un178_motor_single_t(6,5,7),
  };

void firmware_read_encoders(void)
{
  my_sensor.feedback=my_command.motor[0];
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.autonomy.mode=0;
  
  NANOSLOT_MOTOR_SEND_POWER();

  updateNeopixels(my_command.autonomy.mode);
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  NANOSLOT_MOTOR_SETUP();
  neopixels.begin();
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
