UPDATED 2023-03

The "Autonomy" system is separated into two halves:

The "Front End" runs on a control booth PC, and it:
	- Accepts keyboard presses, joystick events, etc in manual mode.  
	- Connects to the back end over the network (probably TCP to start with; should also try UDP subnet broadcast).
	
The "Backend" runs on an on-robot PC,
The System is divided into 5 parts, these parts include the Bac-Kend, Pathplanner, Vision, Stepper, and Localizer.
These systems are designed to have comms via MMap. With dedicated files to communicate between systems. The files defined are located in the lunatic.h. This style of system is replacing the monolithic backend from previous iterations. 

Logically UNcoupled Architecture Technology for Intra-robot Communication

Lunatic manages data exchange between all on-robot 
software components, including these programs:

Vision manages the realsense camera
    -Reads color and depth frames from the camera
    -Passes color to aruco to look for computer vision markers
        -Any detected markers create position estimates for the localizer
    -Passes depth to obstacle detector subsystem to look for rocks
        -Any detected obstacles get written to the obstacle grid

Localizer integrates the robot position
    -Reads drive encoder counts from the backend
    -Reads aruco marker data from the vision subsystem
    -Publishes coordinates used by all other components

Path planner computes an obstacle-free drive path
    -Reads the robot location from the localizer
    -Reads the target location from the backend
    -Reads obstacle grid from the obstacle detector
    -Publishes drive commands to the backend
    -Uses an A* to plan paths. 
    	-A* is a goal area rather than a point.

backend is the main autonomy controller
    -Manages the autonomy state machine
    -Manages pilot comms over UDP broadcast
    -Communicates with nanoslot Arduinos

nanoslot is how we talk to the Arduino nano microcontrollers
    -Each Arduino knows its "ID", an 8-bit hex number
        - slot_D0 is the main drive motor controller
        - slot_A0 is the main arm motor controller
        - slot_A1 is the arm IMU and strain gauge interface
        - slot_F0 is the front linear motor controller
        - slot_F1 is the front IMU and strain gauge interface
    -Arduinos talk to slot programs, which are started by nanoboot
    -Nanoboot gets run on Arduino hotplug by udev / systemd



Build instructions:
	sudo apt-get install freeglut3-dev g++ make libopencv-dev
	cd backend
	make
	./backend --sim
(The backend without --sim tries to connect to the nanoslot data exchange.)

Make Arduino comms work:
	sudo vipw -g
	... add your account to the dialout group and re-login ...
	sudo apt-get remove modemmanager brltty
	sudo apt-get install libfuse2
	... now open autonomy/nanoslot/slot_F0/firmware_F0/firmware_F0.ino in the Arduino IDE 2.0 ...
	
On a Raspberry Pi:
	A Pi compatible Arduino IDE 2.0 build is here: 
		https://github.com/koendv/arduino-ide-raspberrypi
	Decrease bloat with:
		sudo apt-get remove cups avahi-daemon update-notifier

For the aruco marker viewer:
	sudo apt-get install cmake libopencv-dev
	cd aruco/aruco-3.0.11
	cmake .
	make
	sudo make install
	sudo ldconfig
	cd ../../vision
	make

For the realsense viewer:
  sudo apt-get install libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libzmq3-dev cmake libssl-dev
  git clone https://github.com/IntelRealSense/librealsense
  cd librealsense/
# ./scripts/patch-realsense-ubuntu-lts.sh 
  mkdir build
  cd build
  cmake .. -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=false -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release
  make
  sudo make install
  sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && sudo udevadm trigger


For debug operation of the nanoslot programs, you can start them 
manually.  For full on-robot operation, you want them reliably started
automatically on hotplug.  There is a script to do this here:
  cd nanoslot/install
  sudo ./install.sh
This requires a user named "robot", and creates a directory "/nanoslot".
Watch nanoslot operations and slot program print output with:
  tail -f /nanoslot/log


ROS setup for Ubuntu 22.04 (ROS2 humble) from:
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo add-apt-repository universe
sudo apt update
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools






------- Obsolete ------

For the vive viewer:
	sudo cp vive/91-vive.rules /etc/udev/rules.d/
	cd vive
	make

For the kinect classifier:
	sudo cp kinect/66-kinect.rules /etc/udev/rules.d/
	sudo apt-get install libfreenect-dev libusb-1.0-0-dev
	cd kinect
	make


