/* Example being a lawlorbus peripheral */
#define LAWLORBUS_PERIPHERAL 1 // we're a peripheral
#include "lawlorbus.h"

// This is the data the controller sends us
struct mydata_command {
    unsigned char count;
    unsigned char LED; // 0 : LED off.  1 : LED on
    unsigned char fencepost;
};

mydata_command command;


// This is the data we want to report back (keep it short!)
struct mydata_report {
    unsigned char count;
    unsigned char happy;
};

mydata_report report;


void setup()
{
    lawlorbus_begin();
    Serial.begin(115200);
    Serial.println("LawlorBus peripheral test v0.1");

    pinMode(13,OUTPUT);
}

long total=0; // total communicated
long errs=0; // error count
unsigned char last=0;

void loop()
{
    int addr=7; // our address

    report.count++;
    report.happy=1;
    
    // Need to call this at least every 40 microseconds:
    //  it will send the report when the controller asks for it.
    int ret=lawlorbus_listen(addr,
        &command, sizeof(command), 
        &report, sizeof(report));
    
    if (ret==LAWLORBUS_RET_SUCCESS) {
      digitalWrite(13,command.LED);
      if (++total%100==0) {
        Serial.print("C ");
        Serial.println(command.count);
      }
      //Serial.print(" ");
      //Serial.print(command.LED);
      //Serial.println();
      if (command.count!=(unsigned char)(last+1)) {
        errs++;
        Serial.print(command.count,HEX);
        Serial.println(" XC");
      }
      last = command.count;
      if (command.fencepost!=0xA5) {
        errs++;
        Serial.print(command.fencepost,HEX);
        Serial.println(" XF");
      } 
      if (command.count==10) {
          Serial.print(" E:");
          Serial.print(errs);
          Serial.print(" rate ");
          Serial.println(100.0*errs/total);
      }
    }
    if (ret==LAWLORBUS_RET_TIMEOUT) {
      Serial.println("       timeout");
      errs++;
    }
}


