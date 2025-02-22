/**
 Robot-specific kinematics: inverse kinematic solver, and 
 exact numeric details on each of the robot's links.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03 (Public Domain)
*/
#include <stdexcept>
#include <stdio.h>
#include "kinematics.h"
#include "excahaul_collision.h"

namespace aurora {

/** List of all robot links that include joints with angles */
const static std::vector<robot_link_index> links_with_revolute_joints = {
    link_fork, link_dump,
    link_boom, link_stick, link_tilt, link_spin
};


/** Coarse sanity-check this set of joint angles (check limits on angles) */
bool joint_state_sane(const robot_joint_state &joint)
{
    for (robot_link_index L : links_with_revolute_joints) {
        const robot_link_geometry &G=link_geometry(L);
        if (G.joint_index>=0) {
            float angle = joint.array[G.joint_index];
            if (angle<G.angle_min || angle>G.angle_max) return false;
        }
    }
    return true;
}

/** Detailed sanity-check this motion with these joint angles: 
  Return NULL if these power commands keep the robot in a safe configuration,
  or a short human-readable description string of the hazards if unsafe.
*/
const char* joint_move_hazards(const robot_joint_state &joint,const robot_power &power)
{
// (1) Simple crude angle tests:
    float small=0.01; // nominal 1% power
    
    // Check for scoop down while driving
    bool scoop_down = joint.angle.fork<-10 || joint.angle.dump<-70;
    bool driving = fabs(power.left)>small || fabs(power.right)>small;
    if (scoop_down && driving) return "scoop dragging on ground";
    
    // Check for arm elbow mashing back electronics box
    bool back_tilted = joint.angle.boom>40 && joint.angle.stick>20;
    bool back_move = power.boom<-small || power.stick>small; 
    if (back_tilted && back_move) return "hitting back ebox";
    
// (2) Fancy coordinate system calculations
    robot_link_coords links(joint);
    
    // Get frame-relative orientations of major parts
    const robot_coord3D &tool = links.coord3D(link_grinder);
    const robot_coord3D &scoop = links.coord3D(link_dump);
    const robot_coord3D &boom = links.coord3D(link_boom);

    // Fix the 45 degree scoop offset
    robot_coord3D mod_scoop = scoop;
    float scoop_Y_angle_old = atan(scoop.Y.z/scoop.Y.y);
    float scoop_Y_angle_new = scoop_Y_angle_old + (3.1416f/4.0f);
    float scoop_Z_angle_old = atan(scoop.Z.z/scoop.Z.y);
    float scoop_Z_angle_new = scoop_Z_angle_old + (3.1416f/4.0f);

    mod_scoop.Y = vec3(0, cos(scoop_Y_angle_new), sin(scoop_Y_angle_new));
    mod_scoop.Z = vec3(0, cos(scoop_Z_angle_new), sin(scoop_Z_angle_new));
    
    // vec3 point_finder = mod_scoop.local_from_world(tool.world_from_local(vec3(0,0,0))); // debugging
    
    // Is tool in scoop?
    vec3 tip = mod_scoop.local_from_world(tool.world_from_local(MINING_HEAD_MID));
    vec3 tool_back_lower = mod_scoop.local_from_world(tool.world_from_local(TOOL_BACK_LOWER));
    vec3 tool_back_upper = mod_scoop.local_from_world(tool.world_from_local(TOOL_BACK_UPPER));
    
    bool head_in_scoop = (tip.y+MINING_HEAD_R+SAFE_DIST>SCOOP_HAZ_UPPER.y) &&
                         (tip.z-(MINING_HEAD_R+SAFE_DIST)<SCOOP_HAZ_UPPER.z) && 
                         (tip.y-(MINING_HEAD_R+SAFE_DIST)<SCOOP_HAZ_LOWER.y) && 
                         (tip.z+MINING_HEAD_R+SAFE_DIST>SCOOP_HAZ_LOWER.z);
    bool tool_back_in_scoop = (tool_back_lower.y+SAFE_DIST>SCOOP_HAZ_UPPER.y) &&
                              (tool_back_lower.z-SAFE_DIST<SCOOP_HAZ_UPPER.z) && 
                              (tool_back_lower.y-SAFE_DIST<SCOOP_HAZ_LOWER.y) && 
                              (tool_back_lower.z+SAFE_DIST>SCOOP_HAZ_LOWER.z);
    
    bool in_scoop = head_in_scoop || tool_back_in_scoop;
    
    if (in_scoop) {
        // We're in the scoop; sometimes this is okay.
        if (power.attached_grinder() && fabs(power.attached.grinder.tool)>small)
            return "can't spin inside scoop";
    }
    
    // Mining head on scoop
    float dist_to_scoop_bottom = point_to_line_dist(SCOOP_HAZ_MID, SCOOP_HAZ_LOWER, tip);
    bool head_near_bottom = dist_to_scoop_bottom<MINING_HEAD_R+SAFE_DIST;
    bool head_under_scoop = (tip.z-MINING_HEAD_R<SCOOP_HAZ_MID.z) || (tip.z-MINING_HEAD_R<SCOOP_HAZ_LOWER.z);
    
    float dist_to_scoop_back = point_to_line_dist(SCOOP_HAZ_MID, SCOOP_HAZ_UPPER, tip);
    bool head_near_back = dist_to_scoop_back<MINING_HEAD_R+SAFE_DIST;
    bool head_behind_scoop = (tip.y<SCOOP_HAZ_MID.y) && (tip.z-MINING_HEAD_R<SCOOP_HAZ_UPPER.z);
    
    if (head_near_bottom && !head_under_scoop) {
        // We're in and going below the scoop surface; not okay.
        if (power.boom>small) return "boom pushing tool into scoop"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (power.tilt>small) return "tilting tool into scoop";
        if (power.dump>small) return "dump pushing scoop into tool"; // moving scoop
        if (power.fork>small) return "fork pushing scoop into tool";
    }
    if (head_near_bottom && head_under_scoop) {
        // We're in and going below the scoop surface; not okay.
        if (power.boom<-small) return "boom pushing tool into scoop"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (power.tilt<-small) return "tilting tool into scoop";
        if (power.dump<-small) return "dump pushing scoop into tool"; // moving scoop
        if (power.fork<-small) return "fork pushing scoop into tool";
    }
    if (head_near_back && !head_behind_scoop) {
        // We're in and going behind the scoop; not okay.
        if (power.boom<-small) return "boom pushing tool into scoop"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (power.tilt<-small) return "tilting tool into scoop";
        if (power.dump<-small) return "dump pushing scoop into tool"; // moving scoop
        if (fabs(power.fork)>small) return "fork pushing scoop into tool";
    } 
    if (head_near_back && head_behind_scoop) {
        // We're not in the scoop, but we're trying to be, by going through the scoop.
        if (power.boom>small) return "boom pushing tool into scoop"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (fabs(power.tilt)>small) return "tilting tool into scoop (use stick/boom)";
        if (power.dump>small) return "dump pushing scoop into tool"; // moving scoop
        if (power.fork>small) return "fork pushing scoop into tool";
    }

    // Back of tool on scoop
    float tool_upper_scoop_upper_dist = point_to_line_dist(tool_back_upper, tool_back_lower, SCOOP_HAZ_UPPER);
    float tool_lower_scoop_upper_dist = point_to_line_dist(tool_back_lower, tip, SCOOP_HAZ_UPPER);
    float tool_upper_scoop_lower_dist = point_to_line_dist(tool_back_upper, tool_back_lower, SCOOP_HAZ_LOWER);
    float tool_lower_scoop_lower_dist = point_to_line_dist(tool_back_lower, tip, SCOOP_HAZ_LOWER);
    
    bool tool_back_near_scoop_upper = (tool_upper_scoop_upper_dist < SAFE_DIST) || (tool_lower_scoop_upper_dist < SAFE_DIST);
    bool tool_back_near_scoop_lower = (tool_upper_scoop_lower_dist < SAFE_DIST) || (tool_lower_scoop_lower_dist < SAFE_DIST);
    
    if (tool_back_near_scoop_upper) {
        if (fabs(power.boom)>small) return "boom pushing tool into scoop (use stick!)"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (power.tilt<-small) return "tilting tool into scoop";
        if (fabs(power.dump)>small) return "dump pushing scoop into tool"; // moving scoop
        if (power.fork>small) return "fork pushing scoop into tool";
    }
    
    if (tool_back_near_scoop_lower) {
        if (power.boom>small) return "boom pushing tool into scoop"; // moving arm
        if (power.stick<-small) return "stick pushing tool into scoop"; 
        if (power.tilt<-small) return "tilting tool into scoop";
        if (power.dump<-small) return "dump pushing scoop into tool"; // moving scoop
        if (power.fork>small) return "fork pushing scoop into tool";
    }
        
    // Mining head on on boom (add head on frame, too? May not be needed.)
    vec3 tip_to_boom = boom.local_from_world(tool.world_from_local(vec3(0,0,0)));
    vec3 tool_back_to_boom = boom.local_from_world(tool.world_from_local(TOOL_BACK_LOWER));
    
    float head_dist_to_boom = point_to_line_dist(BOOM_HAZ_LOWER, BOOM_HAZ_UPPER, tip_to_boom);
    float tool_dist_to_boom = point_to_line_dist(BOOM_HAZ_LOWER, BOOM_HAZ_UPPER, tool_back_to_boom);
    
    bool in_boom = (head_dist_to_boom < MINING_HEAD_R+SAFE_DIST) || (tool_dist_to_boom < SAFE_DIST);
    
    if (in_boom) {
        if (power.stick<-small) return "stick pushing tool into boom"; 
        if (power.tilt<-small) return "tilting tool into boom";
    }
    
    // Print debug messages
//#ifdef __AURORA_ROBOTICS__DISPLAY_H
    //robotPrintln(" Tool tip to boom: %.3f, %.3f, %.3f",point_finder.x,point_finder.y,point_finder.z); // debugging
//#endif
    
    // Otherwise we don't see any hazards
    return NULL;
}


/**
 Solves inverse kinematics (positions to joint angles)
 problems for the excahauler robot.  This robot is mostly 2D motion
 in the YZ plane, so it's much easier than general multi-link IK. 
*/
class excahauler_IK {
public:
	/** Given a 3D vector in frame coords, return the angle of this
	 direction vector in the YZ plane.
	 The Y axis has an angle of 0, the Z axis has an angle of +90 degrees.
	 Angle is returned in degrees around the X axis, rotating the Y axis upward.
	*/
	static float frame_degrees(const vec3 &v) {
		float rad=atan2(v.z,v.y);
		return RAD2DEG*rad;
	}
    
	/**
	  Given a 3D vector for the origin of the tilt link at the end of the stick,
	  update the boom and stick joint angles to reach that point,
	  and the tilt angle to reach that tool orientation (excahauler_IK::frame_degrees(tool.Y)).
	  
	  Returns 1 if the joint was reachable, -1 if too far
	  (Future: -2 if collision?)
	*/
	int solve_tilt(robot_joint_state &joint, const vec3 &tilt_loc, float tool_deg)
	{
		vec3 tilt_rel = tilt_loc - boomG.origin; // to target from boom start
		float tilt_len = length(tilt_rel);
		float tilt_deg = frame_degrees(tilt_rel); // angle of vector from boom start to tilt pivot
		
		// Use law of cosines to solve for the angle from boom to tilt (BT)
		//  Side a = boom
		//  Side b = tilt
		/// Side c = stick
		float a=boom_len, b=tilt_len, c=stick_len;
		float cos_tb = (a*a + b*b - c*c)/(2.0f*a*b);
		if (cos_tb>1.0f || cos_tb<-1.0f) return -1; // no good
		float tb_deg=RAD2DEG*acos(cos_tb);
		joint.angle.boom=tilt_deg + tb_deg - boom_start; // frame to boom = frame to tilt - boom to tilt
        
        // Use law of cosines again on angle from stick to boom (SB)
		float cos_sb = (a*a + c*c - b*b)/(2.0f*a*c);
		if (cos_sb>1.0f || cos_sb<-1.0f) return -1; // no good
		float sb_deg=RAD2DEG*acos(cos_sb);
		joint.angle.stick=sb_deg - stick_start + boom_start - 180.0f ;
		
		// Update the stick-to-tool tilt angle (ST)
		joint.angle.tilt = tool_deg - joint.angle.stick - joint.angle.boom; 
		if (joint.angle.tilt<-180.0f) joint.angle.tilt+=360.0f;
        
		return 1;
	}


	excahauler_IK() 
		:frameG(link_geometry(link_frame)),
		 boomG(link_geometry(link_boom)),
		 stickG(link_geometry(link_stick)),
		 tiltG(link_geometry(link_tilt))
	{
		boom_len=length(stickG.origin); // boom connects frame to stick
		stick_len=length(tiltG.origin); // stick connects boom to tilt
		boom_start=frame_degrees(stickG.origin);
		stick_start=frame_degrees(tiltG.origin);
	}

private:
	const robot_link_geometry &frameG, &boomG, &stickG, &tiltG; 
	// Length of arm links relative to frame
	float boom_len, stick_len;
	// Angle of arm link origins
	float boom_start, stick_start;

};




const robot_link_geometry &link_geometry(robot_link_index L) 
{
    static const robot_link_geometry geom[link_count] = {
        { 
            "pit",
            link_pit, linktype_fixed,
            link_pit, vec3(0,0,0),
            axisNONE, 0.0f, -1
        },
        { 
            "frame",
            link_frame, linktype_revolute,
            link_pit, vec3(0,0,0),
            axisZ, 0.0f, -1
        },
        
        { /* Lift the whole front scoop assembly */
            "fork",
            link_fork, linktype_revolute,
            link_frame, vec3(0,0.455,0.150),
            axisX, 0.0f, 0,
            -58.7, +10
        },
        { /* Dump the front scoop out */
            "dump",
            link_dump, linktype_revolute,
            link_fork, vec3(0,0.250,0.020),
            axisX, 0.0f, 1,
            -80, -10
        },
        
        { /* First arm link */
            "boom",
            link_boom, linktype_revolute,
            link_frame, vec3(0,0.570,0.215),
            axisX, 0.0f, 2,
            -58, +52
        },
        {
            "stick",
            link_stick, linktype_revolute,
            link_boom, vec3(0,-0.312,0.750),
            axisX, 0.0f, 3,
            -32, +60
        },
        {
            "tilt",
            link_tilt, linktype_revolute,
            link_stick, vec3(0,0.735,0.012),
            axisX, 0.0f, 4,
            -75, +52
        },
        {
            "spin",
            link_spin, linktype_revolute,
            link_tilt, vec3(0,0.000,-0.075),
            axisY, 0.0, 5,
            -30, +30
        },
        {
            "coupler",
            link_coupler, linktype_fixed,
            link_spin, vec3(0,0.0,0.035),
            axisNONE, 0.0f, -1
        },
        {
            "grinder",
            link_grinder, linktype_fixed,
            link_coupler, vec3(0,0.475,-0.311),
            axisNONE, 0.0f, -1
        },
        /*
        {
            "spin",
            link_spin, linktype_revolute,
            link_tilt, vec3(0,0.060,-0.075),
            axisY, 0.0, 5,
            -30, +30
        },
        {
            "coupler",
            link_coupler, linktype_fixed,
            link_spin, vec3(0,0.0,0.085),
            axisNONE, 0.0f, -1
        },
        {
            "grinder",
            link_grinder, linktype_fixed,
            link_coupler, vec3(0,0.513,-0.311),
            axisNONE, 0.0f, -1
        },
        */
        
        
        { /* Realsense depth camera on top of stick */
            "depthcam",
            link_depthcam, linktype_revolute,
            link_stick, vec3(0,0.490,0.500),
            axisX, -180+57+1, -1
        },
        {
            "drivecamflip",
            link_drivecamflip, linktype_revolute,
            link_frame, vec3(0,-0.575,0.270+0.215),
            axisZ, 180.0f, -1
        },
        { /* Genius 120 FOV camera on back electronics box */
            "drivecam",
            link_drivecam, linktype_revolute,
            link_drivecamflip, vec3(0,0,0),
            axisX, -90.0f, -1
        },
    };
    
    if (L<0 || L>=link_count) throw std::runtime_error("Invalid link index passed to link_geometry");
    return geom[L];
}


}; /* end namespace aurora */


