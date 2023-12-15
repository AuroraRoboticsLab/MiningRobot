/*
This is the computer vision system, reading color and depth data
from a realsense camera, and writing data used by the localizer.

From the color images, we extract aruco marker locations.

From the depth images, we extract drivable / non-drivable areas.
*/
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

#include "vision/realsense_camera.hpp"
#include "vision/realsense_camera.cpp"

#include "vision/aruco_detector.hpp"
#include "vision/aruco_detector.cpp"
#include "vision/aruco_watcher.hpp"

// Obstacle detection
#include "vision/grid.hpp"
#include "vision/grid.cpp"
#include "vision/erode.hpp"

#include "aurora/kinematics.h"


/* Project current depth data onto 2D map */
void project_depth_to_2D(const realsense_camera_capture &cap,
    const aurora::robot_coord3D &view3D,
    obstacle_grid &map2D)
{
    printf("Camera view: "); view3D.print();
    const float depth_calibration_scale_factor=1.0f; // fudge factor to match real distances
    const float sanity_distance_min = 60.0; // mostly parts of robot if they're too close
    const float sanity_distance_max = 550.0; // depth gets ratty if it's too far out
    const float sanity_Z_max = 300.0; // ignore ceiling (with wide error band for tilt)
    const float sanity_Z_min = -200.0; // ignore invalid too-low
    const int realsense_left_start=30; // invalid data left of here
    for (int y = 0; y < cap.depth_h; y++)
    for (int x = realsense_left_start; x < cap.depth_w; x++)
    {
        float depth=cap.get_depth_cm(x,y);
        if (depth<=sanity_distance_min || depth>sanity_distance_max) 
            continue; // out of range value
        
        depth *= depth_calibration_scale_factor;
        
        vec3 cam = cap.project_3D(depth,x,y);
        vec3 world = view3D.world_from_local(cam);
        
        if (world.z<sanity_Z_max && world.z>sanity_Z_min)
        {
            map2D.add(world);
        }
    }   
}

/* Mark grid cells as driveable or non-driveable */


int main(int argc,const char *argv[]) {
    int show_GUI=0;
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_marker_reports_depth();
    MAKE_exchange_field_raw();
    MAKE_exchange_obstacle_view();

    // res=720; fps=30; // <- 200% of gaming laptop CPU
    // res=540; fps=60; // <- 220% of gaming laptop CPU
    // res=540; fps=30; // <- 100% of gaming laptop CPU
    
    int res=540; // camera's requested vertical resolution
    int fps=30; // camera's frames per second 
    bool aruco=true; // look for computer vision markers in RGB data
    bool obstacle=true; // look for obstacles/driveable areas in depth data
    bool tabletop=false; // tabletop testing mode: build map from fixed location, no robot
    int erode=4; // image erosion passes / pixels (reduces border bleed)
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--gui") show_GUI++;
      else if (arg=="--res") res=atoi(argv[++argi]); // manual resolution
      else if (arg=="--fps") fps=atoi(argv[++argi]); // manual framerate
      else if (arg=="--no-aruco") aruco=false; 
      else if (arg=="--no-obstacle") obstacle=false; 
      else if (arg=="--tabletop") tabletop=true; 
      else if (arg=="--erode") erode=atoi(argv[++argi]);
      
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }

    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    
    printf("Connecting to realsense camera...\n");
    realsense_camera cam(res,fps);
    printf("Connected.\n");
    
    aruco_detector *detector=0;
    if (aruco) {
        detector = new aruco_detector();
    }

    while (true) {
        // Grab data from realsense
        realsense_camera_capture cap(cam);
        // If the two captures dont have the same data do not draw the obsticles.
        // Maybe solution is to iterate over the two realsense scene.
        // Helper script maybe define an way to compare in realsense.h
        // ex: cap.blend(last);

        
        // Run aruco marker detection on color image
        if (aruco)
        {
            vision_marker_watcher watcher;
            detector->find_markers(cap.color_image,watcher,show_GUI);
            if (watcher.found_markers()>0) { // only write if we actually saw something.
                exchange_marker_reports_depth.write_begin()=watcher.reports;
                exchange_marker_reports_depth.write_end();
            }
        }
        
        // Run obstacle detection on depth image
        if (obstacle) {
            if (erode) erode_depth(cap,erode);
            
            // Project to 2D map
            obstacle_grid map2D;
            aurora::robot_coord3D view3D = exchange_obstacle_view.read();
            
            if (tabletop) { // tabletop hardcoded depth camera testing: 
                aurora::robot_coord3D camera;
                camera.reset();
                camera.percent=101.0;
                camera.origin=vec3(field_x_size/2,field_y_size,100);
                float scale=4.0;
                camera.X=vec3(-scale,0,0);
                camera.Y=vec3(0,0,-1.0); 
                camera.Z=vec3(0,-scale,0); 
                view3D = camera;
                aurora::robot_link_coords::rotate_link(view3D,camera,aurora::axisX,-20.0);
            }
            std::cout << "current percent: " << view3D.percent << "\n";
            
            
            if(view3D.percent > 0.0)
            {
                project_depth_to_2D(cap,view3D,map2D);
                exchange_field_raw.write_begin() = map2D;
                exchange_field_raw.write_end();
            }
            
            
            
            if (show_GUI) {
                cv::Mat debug=map2D.get_debug_2D(2);
                imshow("Top-down 2D Map",debug);
            }
        }
        
        // Show debug GUI
        if (show_GUI) {
            imshow("Color image",cap.color_image);
            imshow("Depth image",8.0f*cap.depth_image); // scale up brightness
        }

        
        if (show_GUI) {
            int key = cv::waitKey(1);  
            if (key == 27 || key=='q')  
                break;  
        }
        // Store The previous copy of the data before grabbing new
        realsense_camera_capture last(cap);
    }
    return 0;
}
