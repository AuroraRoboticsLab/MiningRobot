/*
 Interface the lunatic data exchange with slot 70 arm actuator.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2025-02-21 (Public Domain)
*/
#define NANOSLOT_MY_ID 0x70 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_70  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p,comm.my_sensor);

            if (comm.got_sensor) 
            {
                comm.my_state.angle[0] = comm.my_sensor.angle[0]*(360.0/4096);
                if (comm.verbose) {
                    printf("  70 sensed angle: %4d\n",comm.my_sensor.angle[0]); fflush(stdout);
                }
            }
            
            if (comm.lunatic_post_packet(p))
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  70 torque command: %3d\n",comm.my_command.torque[0]); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(20);
    }
    
    return 0;
}

