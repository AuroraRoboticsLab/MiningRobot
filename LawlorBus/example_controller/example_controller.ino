/* Example being a lawlorbus controller */
#define LAWLORBUS_CONTROLLER 1 // we're a controller
#include "lawlorbus.h"

// This is the data we send out
struct mydata_command {
    unsigned char count;
    unsigned char LED; // 0 : LED off.  1 : LED on
    unsigned char fencepost;
};

mydata_command command;


// This is the data got reported back
struct mydata_report {
    unsigned char count;
    unsigned char happy;
};

mydata_report report;


void setup()
{
    lawlorbus_begin();
    Serial.begin(115200);
    Serial.println("LawlorBus controller test v0.1");
}

void loop()
{
    command.count++;
    command.LED=command.count>128;
    command.fencepost = 0xA5;

    int ret=lawlorbus_send(7,
      &command,sizeof(command),
      &report,sizeof(report));
    
    if (ret==LAWLORBUS_RET_TIMEOUT) Serial.println("timeout");

    delay(20); //<- wait for serial output on other side
}


