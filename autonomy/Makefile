PROGS=nanoslot frontend localizer pathplanner lunabug andretti cartographer vision

all: 
	for dir in $(PROGS); do make -C $$dir; done

clean: 
	for dir in $(PROGS); do make -C $$dir clean; done

install:
	sudo apt-get install freeglut3-dev g++ make cmake libopencv-dev 
	cd aruco/aruco-3.0.11; cmake .; make; sudo make install

