# UAF Robot Control Architecture, 2023 Version
## Codebase
The autonomy/ directory contains our robot control stack, written in C++ for Linux and running happily on x86 and ARM based robots including the Raspberry Pi or NVIDIA Jetson.  See [the README in that directory](https://github.com/AuroraRoboticsLab/MiningRobot/blob/master/autonomy/README.txt) for build directions.


## LUNATIC
The "Logically UNcoupled Architecture Technology for Intra-robot Communication" or LUNATIC (pronounced lun-AT-ic, like automatic) is our robot control system, built as a loosely coupled set of microservices.

![Box diagram showing the parts of LUNATIC: frontend, backend, and the localization and autonomy services](documentation/autonomy_stack.png?raw=true "Microservices in LUNATIC")

Using microservices means you can compile, run, and test a single service independently, unlike our previous single monolithic executable the [backend](autonomy/backend/main.cpp).  

Due to the nature of a robotics control stack, being able to efficiently exchange quickly changing data between modules is crucial. We do this using the new MMAP-based shared file data exchange system, the details of which can be found in the [data_exchange.h](autonomy/include/aurora/data_exchange.h) header. 

The data being exchanged is listed in the [lunatic.h](autonomy/include/aurora/lunatic.h) header for the microservices, and [nanoslot_exchange.h](autonomy/nanoslot/include/nanoslot/nanoslot_exchange.h) for the Arduino data.

![Diagram with arrows showing how the services communicate, a visual version of the lunatic.h header](documentation/autonomy_stack_full.png?raw=true "Data exchange in LUNATIC")

## Control Software Stack
A basic manual control setup has autonomy/frontend running on the "pilot" machine, and sending commands to the autonomy/backend running onboard the robot.  These commands are sent via UDP broadcast, so they should work anywhere on the same subnet.  The backend talks to the Arduinos directly via USB--each Arduino has a "nanoslot ID" that lets nanoboot connect the appropriate slot program to communicate with it.  To control the robot, all you need is a backend and the slot programs: put the backend into "backend driver" state, and use WASD to drive the robot.

For autonomous robot operations, we rely on a huge variety of microservices, such as the localizer (where is the robot?), path planner (how do we get to the target?), andretti (how do we drive along the path?), vision (what obstacles can we see?), and several other debug tools.

This architecture has been incrementally built up since 2013 by a variety of UAF students for the NASA Robotic Mining Contest.  Unless otherwise noted, everything here is public domain. 

For details on how build and run this software, see autonomy/README.
Send a pull request or email lawlor@alaska.edu if you have comments or suggestions!

