// Basic skeleton file for stepper data exchange.
//stepper talks to the camera pointing stepper motor
// Reads requested view angle from backend
// Publishes current view angle to the localizer

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "../include/serial.h"
#include "../include/serial.cpp"
#include "../include/aurora/data_exchange.h"
#include "../include/aurora/stepper.h"
#include "../include/aurora/lunatic.h"

//SerialPort Serial;

bool nanoComm(float loc)
{
    static int MIL = 1000000; // to convert microseconds to seconds

    int count = 0;
    bool leave = false, once = false, ret = false;
    std::string received;
    std::string received_str = "";

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << loc;
    std::string tmp = ss.str(); // float to string, precision 2
    char cstr[tmp.length() + 1];
    strcpy(cstr, tmp.c_str()); // string to char array

    while(Serial.Is_open())
    {
        if(!once)
        {
            usleep(1 * MIL); // must wait for nano
            Serial.write(cstr);
            once = true;
        }

        while(Serial.available() > 0)
        {
            received = Serial.read(); // one byte at a time

            if(received == "#")
            {
                std::cout << received_str << std::endl;

                if(received_str == "<Arduino Nano Ready>")
                {
                    std::cout << std::endl;
                }
                else if(received_str == "RECEIVED: New Angle (" + tmp + ")")
                {
                    leave = true;
                }
                else if(received_str.substr(0,24) == "CONFIRM: Current Angle (")
                {
                    std::cout << "CONFIRMED UPDATE" << std::endl;
                    // grab float from string
                    leave = true;
                }

                received_str = "";
            }
            else // end of packet
            {
                received_str += received;
            }
        }

        ++count;

        if(leave)
        {
            ret = true;
            break;
        }
        else if(count == 10 * MIL)
        {
            break;
        }
    }
    std::cout << std::endl;

    return ret;
}

int main()
{
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_report();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_request();

    Stepper spyglass;
    bool res;

    Serial.begin(115200);
    if(Serial.Is_open())
    {
        std::cout << "<Serial  Port Ready>" << std::endl;
    }

    while (true)
    {
        aurora::stepper_pointing oldDir = exchange_stepper_request.read();

        aurora::stepper_pointing newDir;
        //writing new data to files:
        // newDir.angle = ?
        // newDir.stable = ?

        exchange_stepper_report.write_begin() = newDir;
        exchange_stepper_report.write_end();

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);

        spyglass.loc += 10.5;
        res = nanoComm(spyglass.loc);

        if(res)
        {
            std::cout << "SUCCESS\n\n" << std::endl;
        }
        else
        {
            std::cout << "FAILED\n\n" << std::endl;
        }
    }

    return 0;
}