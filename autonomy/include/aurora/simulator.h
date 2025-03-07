/**
 Robot simulator, used for path planning, software development, and pilot training.
 
 Orion Lawlor, lawlor@alaska.edu, 2014-04-18 (public domain)
*/
#ifndef __AURORA_SIMULATOR_H
#define __AURORA_SIMULATOR_H

#include "../aurora/kinematics.h"
#include "../osl/vec4.h"
#include "../osl/vec2.h"

void blend_angles(float &dest,const float &src,float weight) {
	if(fabs(src - dest) > 180) { // must reduce source to match dest
		float sa=src;
		if (sa>dest+180) sa-=360;
		if (sa<dest-180) sa+=360;
		dest=sa*weight+dest*(1.0-weight);
		if (dest>180) dest-=360;
		if (dest<-180) dest+=360;
	} else {
		dest=src*weight+dest*(1.0-weight);
	}
}

void blend(robot_localization &dest, const robot_localization &src, float weight) {
	dest.x=src.x*weight+dest.x*(1.0-weight);
	dest.y=src.y*weight+dest.y*(1.0-weight);
	blend_angles(dest.angle,src.angle,weight);
        dest.percent = src.percent*weight + dest.percent*(1.0-0.5*weight);
	if (dest.percent>100.0) dest.percent=100.0;
}

class robot_simulator {
public:
	// Actuators:
	robot_joint_state joint; // simulated joint angles (degrees)
	
	double DLcount, DRcount; // driving left/right track counts
	double Mcount; // mining head counter
	double Rcount; // roll motor
	double bucket; // linear actuators, 0-1 range
	robot_localization loc; // current location of robot
	
	robot_simulator() {
        joint.angle.boom=0.0f;
        joint.angle.stick=0.0f;
        joint.angle.dump=0.0f;
		bucket=0.6; // lowered
		DLcount=DRcount=0;
		Mcount=0;
		Rcount=0;
	}

/* Coordinate systems */
	/** Return the robot's orientation angle, in radians.  
	    0 is facing the lunabin.  + is clockwise; - is counterclockwise.
	*/
	double angle_rad() const {
		return loc.angle*M_PI/180.0;
	}
	/** Return the robot's forward (+y) unit direction vector. */
	vec2 forward() const {
		double a=angle_rad();
		return vec2(sin(a),cos(a));
	}
	/** Return the robot's right (+x) unit direction vector. */
	vec2 right() const {
		double a=angle_rad();
		return vec2(cos(a),-sin(a));
	}
	/** Convert this robot-coordinates location to world coordinates (in cm) */
	vec2 world_from_robot(const vec2 &robot_coords) const {
		return vec2(loc.x,loc.y)+forward()*robot_coords.y+right()*robot_coords.x;
	}
	/** Convert this world-coordinates location to robot coordinates (in cm) */
	vec2 robot_from_world(const vec2 &world_coords) const {
		vec2 rel=world_coords-vec2(loc.x,loc.y);
		return vec2(dot(right(),rel),dot(forward(),rel));
	}

	enum {wheelbase=65}; // left-right centimeters from centerline to wheel drive point (X)
	enum {wheelfront=45}; // front-back centimeters between axles (Y)
	enum {wheelforward=0}; // centimeters from center of mass to drive center (Y)

/* Return the world-coordinates location of this corner of the robot. */
	vec2 corner(bool right,bool front) {
		return world_from_robot(vec2(right?+wheelbase:-wheelbase, front?+wheelfront:-wheelfront));
	}

/* Move this kinematic link's angle according to this much simulated power */
    void move_joint(aurora::robot_link_index L,float &angle,float power) {
        // Move under power
        angle += power;
        
        // Limit the angle motion
        const aurora::robot_link_geometry &G=aurora::link_geometry(L);        
        if (angle>G.angle_max) angle=G.angle_max;
        if (angle<G.angle_min) angle=G.angle_min;
    }

/* Simulate these robot power values, for this timestep (seconds) */
	void simulate(const robot_power &power, double dt) {
	// Move the linear actuators
	    float linear_speed = dt*15.0; // degrees/sec at full power
	    
	    move_joint(aurora::link_fork, joint.angle.fork, power.fork*linear_speed);
	    move_joint(aurora::link_dump, joint.angle.dump, power.dump*linear_speed);

	    move_joint(aurora::link_boom, joint.angle.boom, -0.6f*power.boom*linear_speed); // boom is a little slower
	    move_joint(aurora::link_stick, joint.angle.stick, power.stick*linear_speed);
	    move_joint(aurora::link_tilt, joint.angle.tilt, power.tilt*linear_speed);
	    //move_joint(aurora::link_spin, joint.angle.spin, power.spin*linear_speed);
	
	// Move both wheels
		vec2 side[2];  // Location of wheels:  0: Left; 1:Right
		side[0]=world_from_robot(vec2(-wheelbase,wheelforward));
		side[1]=world_from_robot(vec2(+wheelbase,wheelforward));

		float sidepower[2]={0.0,0.0};
		float sideticks[2]={0.0,0.0};
		float topspeed=20.0; // <- drive speed in ticks/sec at 100% power
		sidepower[0]=power.left;
		sidepower[1]=power.right;

		for (int s=0;s<2;s++) {
			double torque=sidepower[s]; // wheel torque command
			if (fabs(torque)>0.001) { // friction
				double distance=torque*topspeed*dt;
				sideticks[s]=distance;
				side[s]+=distance*forward();
			}
		}
		DLcount+=fabs(sideticks[0]); //<- non-quadrature encoders always count up
		DRcount+=fabs(sideticks[1]);
		
	// Set robot position and orientation from wheel positions
		vec2 center=(side[0]+side[1])*0.5;
		loc.x=center.x; loc.y=center.y;
		vec2 right=side[1]-side[0];
		loc.angle=atan2(-right.y,right.x)*180.0/M_PI;
	
	/*
	// Update bag roll counter
	  float Rcount_per_sec=72*220.0/60.0; // roll motor counts/sec at full speed
  	Rcount+=dt*Rcount_per_sec*(power.roll)/100.0;
  	
	// Update mining head counter
		float Mcount_per_sec=100.0; // mining head counts/sec at max speed
		float Mpower=power.mine/100.0;
		if (power.mineDump) Mpower=-0.3;
		if (power.mineMode) Mpower=0.6;
		Mcount+=dt*Mpower*Mcount_per_sec;
		while (Mcount>120.0) Mcount-=120.0;
		while (Mcount<0.0) Mcount+=120.0;
	*/
	
	// Update linear actuators
		double linear_scale=1.0/7.0/100.0; // seconds to full deploy, and power scale factor
		
		bucket+=dt*(power.dump)*linear_scale;
		if (bucket<0.0) bucket=0.0; 
		if (bucket>1.0) bucket=1.0;
	}
};




#endif


