/**
 Aurora Robotics OpenGL display code.
 Shared between front end and back end.
 
 This draws the robot state and telemetry onscreen using OpenGL calls.
 
*/
#ifndef __AURORA_ROBOTICS__DISPLAY_H
#define __AURORA_ROBOTICS__DISPLAY_H

#include <GL/glut.h> /* OpenGL Utilities Toolkit, for GUI tools */

/* robotPrintln support code: */
#include <stdio.h>
#include <stdarg.h>  /* for varargs stuff below */
#include "osl/vec2.h"
#include "aurora/network.h"
#include "aurora/coords.h"

// Needed to draw the robot:
#include "aurora/model.h" // OBJ model file loader
#include "aurora/kinematics.h" // joint orientations

#include <string>

robot_state_t robotState_requested=state_last;
vec2 robotMouse_pixel; // pixel position of robot mouse
vec2 robotMouse_cm; // field-coordinates position of mouse
bool robotMouse_down=false;

double robotPrintf_x=field_x_GUI, robotPrintf_y=0.0;
double robotPrintf_line=-75.0; // size, in field cm, between lines of displayed text

/* Return the current time, in seconds */
double robotTime(void) {
	return 0.001*glutGet(GLUT_ELAPSED_TIME);
}


bool robotPrintf_enable=true;
bool robotPrintgl_enable=true;

/* Render this string at this X,Y location */
void robotPrint(float x,float y,const char *str)
{
	if (robotPrintf_enable) 
	{
        // Dump everything to the console, and log it too
        fprintf(stdout,"%.3f %s\n",robotTime(),str);
        fflush(stdout);

        static FILE *flog=fopen("log.txt","w");
        if (flog) {
	        fprintf(flog,"%.3f %s\n",robotTime(),str);
            fflush(flog);
        }
    }

    if (robotPrintgl_enable) 
    {
        // Draw it onscreen via OpenGL / glut
        void *font=GLUT_BITMAP_HELVETICA_12;
        glRasterPos2f(x,y);
        while (*str!=0) {
        	glutBitmapCharacter(font,*str++);
        	if (robotPrintf_enable && (*str=='\n' || *str==0)) {
        		robotPrintf_x=field_x_GUI;
        		robotPrintf_y+=robotPrintf_line;
        	}
        }
        glPopAttrib();
    }
}

/** Render this string onscreen, followed by a newline. */
void robotPrintln(const char *fmt,...) {
        va_list p; va_start(p,fmt);
        char dest[1000];
        vsnprintf(dest,sizeof(dest),fmt,p);
        robotPrint(robotPrintf_x,robotPrintf_y,dest);
        va_end(p);
}

void robotPrintLines(const std::string& text)
{
    /// Why doesn't this work?
    //robotPrint(robotPrintf_x,robotPrintf_y,text.c_str());
    
	std::string temp;

	for(size_t ii=0;ii<text.size();++ii)
	{
		if(text[ii]=='\n'||ii+1>=text.size())
		{
			//robotPrintln(temp.c_str());
            robotPrint(robotPrintf_x,robotPrintf_y,temp.c_str());
			temp="";
			continue;
		}

		temp+=text[ii];
	}
}

/******************************** GUI ****************************/

// Rotate vector src around the origin by this many degrees
inline vec2 rotate(const vec2 &src,float ang_deg) {
	double ang_rad=ang_deg*M_PI/180.0;
	double s=sin(ang_rad), c=cos(ang_rad);
	return vec2( c*src.x-s*src.y, s*src.x+c*src.y);
}

inline float state_to_Y(int state) {
	return field_y_size*(state_last-state)*(1.0/state_last);
}

/* Set up for 3D side view of robot */
void robot_3D_setup(float Xrot=0, float Zrot=-90.0f) {
    // Incoming coordinate system: centimeters field coordinates
    //    X+ screen right, Y+ screen up, Z+ toward camera
    // Intermediate coordinate system: robot frame coordinates for front view
    //    X+ screen right, Y+ toward camera, Z+ screen up
    // Outgoing coordinate system: robot frame coordinates for side view
    //    X+ toward camera, Y+ screen right, Z+ screen up
    glPushMatrix();
    glScalef(100.0f,100.0f,0.1f); // from cm to meters (and squish Z so it doesn't clip)
    glTranslatef(3.0f,0.0f,0.0f); // meters field coordinates to origin of side view of robot
    float zoom=12.0; // Zoom from field coords to robot 3D display
    glScalef(zoom,zoom,1.0f);
    glRotatef(-90.0f,1.0f,0.0f,0.0f); // rotate around X to front view
    glRotatef(Zrot,0.0f,0.0f,1.0f); // rotate around Z to side view
    glRotatef(Xrot,1.0f,0.0f,0.0f); // pitch rotate
    
    //glRotatef(-5.0f,0.0f,1.0f,0.0f); // rotate around Y to gently tip the view  
}

/* Draw robot in this joint state */
void robot_3D_draw(const robot_joint_state &jointstate,tool_type tool=tool_none,float alpha=1.0)
{
    using namespace aurora;

    float colorPrint[4]={0.2,0.2,0.3,alpha}; // black 3D printed parts
    float colorBox[4]={0.7,0.7,0.6,alpha}; // electronics boxes
    float colorFrame[4]={0.8,0.8,0.7,alpha}; // white painted steel frame

    static bool meshes_OK=true;
    if (meshes_OK) {
        try {
            static mesh mesh_frameBox=readOBJmesh("../../documentation/obj/frameBox.obj");
            static mesh mesh_frame=readOBJmesh("../../documentation/obj/frame.obj");

            static mesh mesh_fork=readOBJmesh("../../documentation/obj/fork.obj");
            static mesh mesh_dump=readOBJmesh("../../documentation/obj/dump.obj");

            static mesh mesh_boom=readOBJmesh("../../documentation/obj/boom.obj");
            static mesh mesh_stick=readOBJmesh("../../documentation/obj/stick.obj");
            static mesh mesh_stickCamera=readOBJmesh("../../documentation/obj/stickCamera.obj");
            static mesh mesh_stickBox=readOBJmesh("../../documentation/obj/stickBox.obj");

            static mesh mesh_tilt=readOBJmesh("../../documentation/obj/tilt.obj");
            static mesh mesh_tool=readOBJmesh("../../documentation/obj/tool.obj");

            static mesh mesh_grinder=readOBJmesh("../../documentation/obj/Rockgrinder.obj");

            static mesh mesh_wheel=readOBJmesh("../../documentation/obj/wheel.obj");
            
            glPushMatrix();
            robot_link_coords::glTransform(link_frame,jointstate);
                        
            // The scoop
            glColor4fv(colorBox);
            glPushMatrix();
            robot_link_coords::glTransform(link_fork,jointstate);
            mesh_fork.draw();
            robot_link_coords::glTransform(link_dump,jointstate);
            mesh_dump.draw();
            glPopMatrix();

            // The frame
            glColor4fv(colorBox);
            mesh_frameBox.draw();
            glColor4fv(colorFrame);
            mesh_frame.draw();

            glPushMatrix();
            // The boom      
            robot_link_coords::glTransform(link_boom,jointstate);
            mesh_boom.draw();

            // The stick
            robot_link_coords::glTransform(link_stick,jointstate);
            glColor4fv(colorBox);
            mesh_stickBox.draw();
            glColor4fv(colorFrame);
            mesh_stick.draw();
            glColor4fv(colorPrint);
            mesh_stickCamera.draw();

            // The tool coupler
            robot_link_coords::glTransform(link_tilt,jointstate);
            glColor4fv(colorFrame);
            mesh_tilt.draw();
            robot_link_coords::glTransform(link_spin,jointstate);
            robot_link_coords::glTransform(link_coupler,jointstate);
            glColor4fv(colorPrint);
            mesh_tool.draw();

	    
            // The held tool
            glColor4fv(colorBox);
            if (tool==tool_rockgrinder) {
                mesh_grinder.draw();
                //robot_link_coords::glTransform(link_grinder,jointstate);
                // draw actual cutting point?
            }

            glPopMatrix(); // back to frame coords

            // Wheels
            glColor4fv(colorPrint);
            float axleX = 0.400;
            for (float side : {-1.0f, +1.0f})
            for (float axleY : { -0.455f, 0.0f, +0.455f})
            {
                glPushMatrix();
                glScalef(side,1.0f,1.0f);
                glTranslatef(axleX,axleY,0.150f);
                glRotatef(90.0f,0.0f,1.0f,0.0f); // wheel has Z up, should have X up
                mesh_wheel.draw();
                glPopMatrix();
            }
            
            // Draw the idealized axes for each link
            robot_link_coords links(jointstate);
            glLineWidth(2.0);
            float len=0.1; // length of axis lines in meters
            glBegin(GL_LINES);
            for (int i=link_frame;i<link_count;i++) {
                const robot_coord3D &coord=links.coord3D(robot_link_index(i));
                glColor3f(0,0,1);
                glVertex3fv(coord.origin); glVertex3fv(coord.origin+len*coord.Z);
                glColor3f(0,1,0);
                glVertex3fv(coord.origin); glVertex3fv(coord.origin+len*coord.Y);
                glColor3f(1,0,0);
                glVertex3fv(coord.origin); glVertex3fv(coord.origin+len*coord.X);
            }
            glEnd();

            
            glPopMatrix(); // back to field coords
            
        }
        catch (std::runtime_error &e) {
            std::cout<<"Mesh load exception: "<<e.what()<<"\n";
            meshes_OK=false;
        }
    }  
    // back to normal vertex color (so text is white)
    glColor4f(1,1,1,1);
}

/* Clean up after 3D view of robot */
void robot_3D_cleanup() {
    glPopMatrix();
}


/* Called at start of user's OpenGL display function */
void robot_display_setup(const robot_base &robot) {

	glDisable(GL_DEPTH_TEST); // draw stuff back-to-front
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int wid=glutGet(GLUT_WINDOW_WIDTH), ht=glutGet(GLUT_WINDOW_HEIGHT);
	glViewport(0,0,wid,ht);

	// Encode current robot state in background color:
	if (robot.state==state_STOP) {
		glClearColor(0.0,0.6,0.9,0.0); // peaceful sky blue-green (safe to approach)
	}
	else if (robot.state==state_backend_driver) {
		glClearColor(0.4,0.3,0.1,0.0); // backend drive: dim yellow
	}
	else if (robot.state==state_drive) {
		glClearColor(0.4,0.5,0.1,0.0); // drive: yellow
	}
	else {
		glClearColor(0.6,0.1,0.0,0.0); // danger red: full autonomy
	}

	glClear(GL_COLOR_BUFFER_BIT+GL_DEPTH_BUFFER_BIT);

	// Scale to showing the whole field, in centimeter units
	float xShift=-0.8, yShift=-0.8; // GL-coordinates origin of field
	glTranslatef(xShift,yShift,0.0);
	float yScale=1.6/field_y_size;
	float xScale=yScale*ht/wid;
	glScalef(xScale, yScale, 0.1);
	robotPrintf_y=(1.0-yShift)/yScale+robotPrintf_line;

	// Read back the matrix to get from cm to onscreen pixels
	float mat[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,mat);
	int w=glutGet(GLUT_WINDOW_WIDTH), h=glutGet(GLUT_WINDOW_HEIGHT);
	vec2 mat_scale(1.0/mat[0],1.0/mat[5]);
	vec2 mat_offset(mat[12],mat[13]);

	// coordinate-convert mouse to cm coords
	vec2 m=vec2(robotMouse_pixel.x*2.0/w-1.0,(h-robotMouse_pixel.y)*2.0/h-1.0)-mat_offset;
	m.x*=mat_scale.x;
	m.y*=mat_scale.y;
	robotMouse_cm=m;

	glLineWidth(1+3*wid/1000);

	/*
	glBegin(GL_LINES); // to verify mouse position
	glColor4f(0.0,0.6,0.0,1.0);
	glVertex2fv(robotMouse_cm);
	glVertex2fv(robotMouse_cm+vec2(10,20));
	glEnd();
	*/

// Delineate the start and mine bays
	glBegin(GL_LINES);
	glColor4f(0.3,0.3,0.5,1.0);
	glVertex2i(0,field_y_start_zone);
	glVertex2i(field_x_size,field_y_start_zone);
	glVertex2i(0,field_y_mine_zone);
	glVertex2i(field_x_size,field_y_mine_zone);

// Draw the scoring trough
	glColor4f(0.3,1.0,1.0,1.0);
	glVertex2i(field_x_trough_start,field_y_trough_edge);
	glVertex2i(field_x_trough_end,field_y_trough_edge);
	glEnd();

// Outline the field
	glBegin(GL_LINE_LOOP);
	glColor4f(0.0,0.0,0.8,1.0);
	glVertex2i(0,0);
	glVertex2i(field_x_size,0);
	glVertex2i(field_x_size,field_y_size);
	glVertex2i(0,field_y_size);
	glEnd();
	
	// Leave text white
	glColor3f(1.0,1.0,1.0);
	
	glEnable(GL_ALPHA_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}


// Use robotPrint to show this robot's telemetry state
//  (no other OpenGL calls, safe for nogui version)
void robot_display_telemetry(const robot_base &robot)
{
	for (int bit=0;bit<robot_sensors_arduino::connected_C0;bit++) {
	    const static char *nameFromBit[robot_sensors_arduino::connected_C0]=
	        {"D0","F0","F1","A0","A1"};
	    bool conn = (robot.sensor.connected>>bit)&1;
	    if (conn==0) robotPrintln("Missing nanoslot %s",nameFromBit[bit]);
	}
	if (0==(robot.sensor.connected & robot_sensors_arduino::connected_C0))
	{
        robotPrintln("Tool not connected");
        robotPrintln("Batteries: - - (-V), drive %.0f%% (%.2fV)",
	    robot.sensor.charge_D, robot.sensor.cell_D);
	}
	else 
	{
		robotPrintln("Batteries: mine %.0f%% (%.2fV), drive %.0f%% (%.2fV)",
	    robot.sensor.charge_M, robot.sensor.cell_M,
	    robot.sensor.charge_D, robot.sensor.cell_D);
	}	

    robotPrintln("Load cells: tool %.1f %.1f  scoop %.1f %.1f (%s)",
        robot.sensor.load_TL,robot.sensor.load_TR,
        robot.sensor.load_SL,robot.sensor.load_SR,
        robot.power.read_L?"L":"R");
	robotPrintln("Accum: scoop %.1f total %.1f kg, drive %.1f total %.0f m, op %.2f hr",
	    robot.accum.scoop, robot.accum.scoop_total,
	    robot.accum.drive, robot.accum.drive_total,
	    robot.accum.op_total/3600.0);

	robotPrintln("Mining rate %.0f (%d)",robot.sensor.minerate, robot.sensor.Mcount);
	robotPrintln("Drive encoder %d L %d R", robot.sensor.DLcount, robot.sensor.DRcount);
	robotPrintln("Frame PRY: %.1f %.0f %.0f deg", robot.sensor.frame_pitch, robot.sensor.frame_roll, robot.sensor.frame_yaw);

	std::string encoder_str("Encoder Raw ");
	for(int ii=12-1;ii>=0;--ii)
	{
		if((robot.sensor.encoder_raw&(1<<ii))!=0)
			encoder_str+="1";
		else
			encoder_str+="0";
		if(ii==6)
			encoder_str += " ";
	}
	robotPrintln(encoder_str.c_str());

	std::string str("Stall Raw ");
	for(int ii=12-1;ii>=0;--ii)
	{
		if((robot.sensor.stall_raw&(1<<ii))!=0)
			str+="1";
		else
			str+="0";
		if(ii==6)
			str += " ";
	}
	str += "\n"; //<- leaves blank line, to space out info
	robotPrintln(str.c_str());

	if (robot.loc.percent>50.0) {
		robotPrintln("Location:  X %.1f   Y %.1f   angle %.0f",
			robot.loc.x,robot.loc.y,
			robot.loc.angle);
	}
}


/* Called near end of OpenGL display function */
void robot_display_finish(const robot_base &robot)
{
// Draw the current autonomy state
	robotPrintf_enable=false;
	double state_display_x=field_x_size*2.6; // 3*field_x_hsize;
	for (robot_state_t state=state_STOP;state<state_last;state=(robot_state_t)(state+1))
	{
		glColor4f(0.0,0.0,0.0,1.0); // black inactive

		if (state==robotState_requested || (
		    robotMouse_cm.x>state_display_x &&
		    robotMouse_cm.y<state_to_Y(state) &&
		    robotMouse_cm.y>state_to_Y(state+1)
		    ))
		{ // red mouse hover
			glColor4f(1.0,0.0,0.0,1.0);
			if (robotMouse_down==true)
			{ // request new state
				robotState_requested=state;
			}
		}

		if (state==robot.state) {
			glColor4f(1.0,1.0,1.0,1.0); // white when active
		}
		robotPrint(state_display_x,
			0.5*(state_to_Y(state)+state_to_Y(state+1)), // average height
			state_to_string(state));
	}
	robotPrintf_enable=true;

// Draw current robot power values as weird triangles
	float *powers=(float *)&robot.power.left; // HACK: want array of powers
	glBegin(GL_TRIANGLES);
	for (unsigned int i=0;i<12;i++) {
		float pow=powers[i];
		float cenx=60*(0.5+i); // +field_x_GUI;
		float ceny=0.90*field_y_size;
		glColor3ub(128+100*pow,128,255);
		glVertex2f(cenx-20,ceny);
		glVertex2f(cenx+20,ceny);
		glVertex2f(cenx,ceny+100*(pow+0.01));
	}
	glEnd();

// Output telemetry as text (for log, mostly)
	glColor3f(1.0,1.0,1.0);
	
	robot_display_telemetry(robot);
}

// Top-down view of robot
void robot_2D_display(const robot_localization &loc,double alpha=1.0)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

// Draw the robot
	float conf=loc.percent*0.01;
	glColor4f(0.8,0.8*conf,0.8*conf,alpha);
	glBegin(GL_TRIANGLE_FAN);
	//float ang=loc.angle*M_PI/180.0;
	float m2cm=100.0; // convert meters to centimeters
	vec2 C=m2cm*loc.center(); // (loc.x,loc.y); // center of robot
	vec2 f=loc.forward();
	vec2 r=loc.right();
	vec2 F=robot_x*f; // (+30.0*sin(ang), +30.0*cos(ang)); // robot forward direction
	vec2 R=robot_y*r; // (+70.0*cos(ang), -70.0*sin(ang)); // robot right side
	vec2 MF=robot_mine_x*f;
	vec2 MR=robot_mine_width*0.5*r;

	glColor4f(0.0,0.0,0.0,alpha); // black back wheels
	glVertex2fv(C-F); // start at back center
	glVertex2fv(C+R-F);

	glColor4f(0.8*conf,0.0,0.0,alpha); // red front right wheel
	glVertex2fv(C+R+F);
	glVertex2fv(C+MR+F);
	
	glColor4f(0.0,0.8*conf,0.0,alpha); // green mining tool
	glVertex2fv(C+MR+MF);
	glVertex2fv(C-MR+MF);

	glColor4f(0.8*conf,0.0,0.0,alpha); // red front left wheel
	glVertex2fv(C-MR+F);
	glVertex2fv(C-R+F);
	
	glColor4f(0.0,0.0,0.0,alpha); // black back left wheel
	glVertex2fv(C-R-F);
	
	glEnd();

	glColor4f(1.0,1.0,1.0,1.0);
}

void robot_display_autonomy(const robot_autonomy_state &a)
{
  // robot_display_markers(a.markers);
  glBegin(GL_LINE_STRIP);
    glColor3f(0.0,1.0,0.0); // green path to target
    for (int i=0;i<(int)a.plan_len;i++)
      glVertex2f(a.path_plan[i].v.x,a.path_plan[i].v.y);
    
    if (a.target.v.y!=0.0) {
      glColor3f(0.0,1.0,1.0); // cyan target
      glVertex2f(a.target.v.x,a.target.v.y);
    }
  glEnd();
  
  glPointSize(4.0f);
  glBegin(GL_POINTS);
    for (int i=0;i<(int)a.obstacle_len;i++) {
      int z=a.obstacles[i].height;
      float badness=z*(1.0/25.0);
      if (badness>1.0) glColor3f(0.0,0.0,0.0); // black == can't even straddle
      else glColor3f(1.0,1.0-badness,1.0-badness);
      glVertex2f(a.obstacles[i].x,a.obstacles[i].y);
    }
  glEnd();
  
  glColor3f(1.0,1.0,1.0);
}

/*************************** Keyboard **********************************/
/** Handle keyboard presses */
#include "../ogl/event.h"

extern "C" void ogl_main_keyboard(unsigned char key, int x, int y)
{
        // flip toggles on keypress
        oglToggles[key]=!oglToggles[key];
        oglKeyMap[key]=1;
}
extern "C" void ogl_main_keyboard_up(unsigned char key, int x, int y)
{
        oglKeyMap[key]=0;
}
extern "C" void ogl_main_special(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=1;
}
extern "C" void ogl_main_special_up(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=0;
}

void ogl_mouse_motion(int x, int y) {
	robotMouse_pixel=vec2(x,y);
}

void ogl_mouse(int button,int state,int x,int y)
{ /* mouse being pressed or released--save position for motion */
	ogl_mouse_motion(x,y);
	if (state==GLUT_DOWN) {
	       robotMouse_down=true;
	} else {
	       robotMouse_down=false;
	}
}

void robotMainSetup(void) {
	glutKeyboardFunc (ogl_main_keyboard);
	glutKeyboardUpFunc (ogl_main_keyboard_up); /* "up" version for KeyMap */
	glutSpecialFunc (ogl_main_special); /* for arrow keys */
	glutSpecialUpFunc (ogl_main_special_up);
	glutMouseFunc(ogl_mouse);
	glutMotionFunc(ogl_mouse_motion);
	glutPassiveMotionFunc(ogl_mouse_motion);

}



#endif

