/*****************************************************************************************
Locates ArUco computer vision markers in a webcam image. 

    - Grabs frames using OpenCV webcam capture
	- Finds markers in the webcam image frames
	- Reconstructs the camera's location relative to the marker
	- Writes the camera location and orientation for use by navigation
	- Periodically saves an image to vidcap.jpg and vidcaps/<date>.jpg


With Logitech C920, at 640x480, a 305mm marker at 7m distance detects reliably, but does flip inside and out.
    ./camera --gui --cam 0 --res 640x480
Time detection=11.5047 milliseconds
Marker 2: Camera -2.554 5.488 -0.271 meters, heading 149.7 degrees
Time detection=11.5054 milliseconds
Marker 2: Camera 4.570 5.474 0.255 meters, heading -144.2 degrees



With Genius 120 FOV wide angle, at 720p, a 305mm marker at 5m distance is not readable (blurred).
    ./camera --gui --cam 0 --res 1280x720


Originally based on:
ArUco example Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "aruco.h"
#include "cvdrawingutils.h"
#include "errno.h"

#include "aurora/lunatic.h"
#include "vision/aruco_detector.hpp"
#include "vision/aruco_detector.cpp"
#include "vision/aruco_watcher.hpp"

using namespace cv;
using namespace aruco;
using namespace std;


bool show_GUI=false;
Mat TheInputImage;


/* Keep the webcam from locking up when you interrupt a frame capture.
http://lawlorcode.wordpress.com/2014/04/08/opencv-fix-for-v4l-vidioc_s_crop-error/ */
volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum)
{
	if (quit_signal!=0) exit(0); // just exit already
	quit_signal=1;
	printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif


/**
  Silly hack to detect camera disconnections.
  When you unplug a video camera, it's actually detected right in:
    opencv/modules/highgui/src/cap_libv4l.cpp
  when the VIDIOC_DQBUF ioctl fails.  However, this function 
  somehow fails to correctly report the problem via the error reporting
  return code.  It does log it using perror, so I'm hooking the global perror
  to figure out if this is what went wrong, and exit appropriately.
*/
extern "C" void perror(const char *str) {
	int e=errno;
	std::cout<<"perror errno="<<e<<": "<<str<<"\n";
	if (e==ENODEV) {
		std::cout<<"ERROR!  Camera no longer connected!\n";
		std::cerr<<"ERROR!  Camera no longer connected!\n";
		exit(1);
	}
}


int main(int argc,char **argv)
{
	try {
	string TheInputVideo;
	int camNo=1;
	float TheMarkerSize=-1;
	int ThePyrDownLevel=0;
	VideoCapture vidcap;
	vector<Marker> TheMarkers;
	CameraParameters cam_param;
	bool aruco = true;
	float minSize=0.02; // fraction of frame, minimum size of rectangle
	pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
	int skipCount=1; // only process frames ==0 mod this
	int skipPhase=0;
        int downscale=2;
    
	int wid=1280, ht=720;
	const char *dictionary="TAG25h9";
	for (int argi=1; argi<argc; argi++) {
		if (0==strcmp(argv[argi],"--gui")) show_GUI=true;
		else if (0==strcmp(argv[argi],"--res")) sscanf(argv[++argi],"%dx%d",&wid,&ht);
		else if (0==strcmp(argv[argi],"--skip")) sscanf(argv[++argi],"%d",&skipCount);
		else if (0==strcmp(argv[argi],"--cam")) camNo=atoi(argv[++argi]);		
		else if (0==strcmp(argv[argi],"--min")) sscanf(argv[++argi],"%f",&minSize);
		else printf("Unrecognized argument %s\n",argv[argi]);
	}

	//read from camera
	// Gemini 3.6 suggested these fixes: use CAP_V4L2 to avoid gstreamer induced errors.
	vidcap.open(camNo, cv::CAP_V4L2);
	// Set format to MJPEG before setting resolution (uses much less USB bandwidth, higher res enabled)
    vidcap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
	if (wid) vidcap.set(cv::CAP_PROP_FRAME_WIDTH, wid);
	if (ht)  vidcap.set(cv::CAP_PROP_FRAME_HEIGHT, ht);

	//check video is open
	if (!vidcap.isOpened()) {
		cerr<<"Could not open video"<<endl;
		return -1;
	}
	
	MAKE_exchange_marker_reports_webcam();
	aruco_detector *detector=0;
    if (aruco) {
        detector = new aruco_detector(minSize,"camera.yml");
    }

#ifdef __unix__
	signal(SIGINT,quit_signal_handler); // listen for ctrl-C
#endif
	unsigned int framecount=0;
	uint32_t vidcap_count=0;

	//capture until press ESC or until the end of the video
	while (vidcap.grab()) {
		if (!vidcap.retrieve( TheInputImage) || !vidcap.isOpened()) {
			std::cout<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
			std::cerr<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
			exit(1);
		}
		if (quit_signal) exit(0);

		// Skip frames (do no processing) to keep up with real time
		skipPhase=(skipPhase+1)%skipCount;
		if (skipPhase!=0) continue;

        // Run aruco marker detection on color image
        if (aruco)
        {
            vision_marker_watcher watcher;
            detector->find_markers(TheInputImage,watcher,show_GUI);
            if (watcher.found_markers()>0) { // only write if we actually saw something.
                exchange_marker_reports_webcam.write_begin()=watcher.reports;
                exchange_marker_reports_webcam.write_end();
            }
        }
        
        // Draw detected markers and stash frames
		bool vidcap=false;
		// if ((framecount++%32) == 0) vidcap=true;
		if (vidcap) { // write to disk
			std::vector<int> params;
			params.push_back(cv::IMWRITE_JPEG_QUALITY);
			params.push_back(30); // <- low quality, save disk space and network bandwidth
			cv::imwrite("vidcap_next.jpg",TheInputImage,params); // dump JPEG
			int ignore;
			ignore=system("mv -f vidcap_next.jpg vidcap.jpg"); // atomic(?) file replace
			ignore=system("cp vidcap.jpg vidcaps/`date '+%F__%H_%M_%S__%N'`.jpg"); // telemetry log
			vidcap_count++;
		}
		if (show_GUI) {
                    const cv::Mat &src=TheInputImage; // cap.color_image;
           	    cv::Mat img;
        	    cv::resize(src,img,cv::Size(src.cols/downscale,src.rows/downscale));
	            imshow("Color image",img);

		    //show input with augmented information and  the thresholded image
		    //cv::imshow("in",TheInputImage);
		    // cv::imshow("thres",MDetector.getThresholdedImage());

		    char key=cv::waitKey(1);//wait for key to be pressed
		    if (key=='q' || key=='x' || key==0x13) exit(0);
		} /* end show_GUI */
	} /* end frame loop */

	} catch (std::exception &ex) {
		cout<<"Vision/ArUco exception: "<<ex.what()<<endl;
	}

}

