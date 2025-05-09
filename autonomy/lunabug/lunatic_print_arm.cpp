/* Debug print the backend's state variables */
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_nanoslot();
    
    nanoslot_exchange nano;
    while (true) {
        if (exchange_nanoslot.updated()) {
            nano=exchange_nanoslot.read();
            
            printf("Arm_angles: %.1f %.1f %.1f %1f\n", 
                nano.slot_70.state.angle[0],
                nano.slot_71.state.angle[0],
                nano.slot_72.state.angle[0],
                nano.slot_73.state.angle[0]
                );
        }
        
        aurora::data_exchange_sleep(100);
    }
}


