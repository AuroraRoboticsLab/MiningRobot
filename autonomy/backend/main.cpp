/**
  Aurora Robotics Backend Code

  Orion Sky Lawlor, lawlor@alaska.edu, 2014--2023 (Public Domain)
*/
#define AURORA_IS_BACKEND 1

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <mutex>

#include "gridnav/gridnav_RMC.h"

#include "aurora/robot_base.h"
#include "aurora/robot_states.cpp"
#include "aurora/display.h"

#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"

#include "aurora/network.h"
#include "aurora/ui.h"

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

#include "aurora/simulator.h"
#include <iostream>


#include "aurora/lunatic.h"
#include "nanoslot/nanoslot_sanity.h"

using namespace aurora;


// Inertial measurement unit sanity check
bool robot_IMUs_OK = true; 

void IMU_sanity_check(const nanoslot_IMU_state &s,const char *what)
{
    const vec3 global_should(0,0,9.8); // Earth gravity vector
    float g_err = length(s.global - global_should);
    if (g_err > 3.0*length(s.vibe) + 4.0) { // global gravity vector is totally wrong--either sudden-onset vibe or a bad IMU
        robot_IMUs_OK = false;
        printf("IMU %s error: g_err = %.3f m/s^2\n", what,g_err);
        static FILE *IMU_errs = fopen("imu.errs","w+");
        if (IMU_errs) {
            fprintf(IMU_errs, "IMU %s error: g_err = %.3f m/s^2   info:", what,g_err);
            s.print(what,IMU_errs);
            fprintf(IMU_errs,"\n");
            fflush(IMU_errs);
        }
    }
}


// Global variables for lunatic data exchange with Arduinos via nanoslot
MAKE_exchange_nanoslot();
void arduino_setup_exchange()
{
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.sanity_check_size();
    nano.backend_heartbeat=0;
    exchange_nanoslot.write_end();
}

// Before we shut down, we need to unplug the exchange
void arduino_exit_exchange()
{
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.autonomy.mode=(int)0;
    nano.backend_heartbeat=0xDE;
    exchange_nanoslot.write_end();
}

// Rolling filtered mining rates (to avoid odd zero/low dropouts)
float filter_minerate(float next_rate)
{
    enum {n=3}; // number of spins to average: smooth (higher) vs responsive (low)

    static float lastspins[n]={0};
    static int index=0;
    lastspins[index] = next_rate;
    index++;
    if (index>=n) index=0;
    
    float peak=0.0;
    for (float s : lastspins) if (s>peak) peak=s;
    //sum+=s; return sum*(1.0/n);
    return peak;
}


void arduino_sensor_read(robot_base &robot)
{
    // Read sensor data from the exchange
    const nanoslot_exchange &nano=exchange_nanoslot.read();
    
    robot.sensor.load_TL=nano.slot_A1.state.load_L;
    robot.sensor.load_TR=nano.slot_A1.state.load_R;
    robot.sensor.load_SL=nano.slot_F1.state.load_L;
    robot.sensor.load_SR=nano.slot_F1.state.load_R;
    
    robot.sensor.cell_M = nano.slot_C0.state.cell;
    robot.sensor.charge_M = nano.slot_C0.state.charge;
    robot.sensor.cell_D = nano.slot_F0.state.cell;
    robot.sensor.charge_D = nano.slot_F0.state.charge;
    
    robot.sensor.minerate = filter_minerate(nano.slot_C0.state.spin);
    
    robot.sensor.Mcount = nano.slot_C0.sensor.spincount;
    robot.sensor.Mstall = (0.0==robot.sensor.minerate);
    
    const static float pitch_cal = 4.0;
    robot.sensor.frame_yaw   = nano.slot_F1.state.frame.yaw;
    robot.sensor.frame_pitch = nano.slot_F1.state.frame.pitch - pitch_cal;
    robot.sensor.frame_roll  = nano.slot_F1.state.frame.roll;
    
    const auto &driveslot = nano.slot_D0;
    int left_wire = 0;
    int right_wire = 1;
    robot.sensor.DRcount =   driveslot.sensor.counts[right_wire];
    robot.sensor.DRstall =   driveslot.sensor.stall&(1<<right_wire);
    
    robot.sensor.DLcount =   driveslot.sensor.counts[left_wire];
    robot.sensor.DLstall =   driveslot.sensor.stall&(1<<left_wire);
    
    robot.sensor.heartbeat = driveslot.debug.packet_count;
    
    robot.sensor.encoder_raw=int(driveslot.sensor.raw);
    robot.sensor.stall_raw=int(driveslot.sensor.stall);
    
    int connected=0;
    connected |= ((1&nano.slot_D0.state.connected) << robot_sensors_arduino::connected_D0);
    connected |= ((1&nano.slot_F0.state.connected) << robot_sensors_arduino::connected_F0);
    connected |= ((1&nano.slot_F1.state.connected) << robot_sensors_arduino::connected_F1);
    connected |= ((1&nano.slot_A0.state.connected) << robot_sensors_arduino::connected_A0);
    connected |= ((1&nano.slot_A1.state.connected) << robot_sensors_arduino::connected_A1);
    connected |= ((1&nano.slot_C0.state.connected) << robot_sensors_arduino::connected_C0);
    robot.sensor.connected = 0xFF & connected;
    
    // Copy joint orientations from IMU data
    
    //  For safe autonomy, really need some additional sanity checking (here, or in slot program?)
    robot_IMUs_OK = true;
    IMU_sanity_check(nano.slot_F1.state.frame,"frame");
    IMU_sanity_check(nano.slot_F1.state.boom,"boom");
    IMU_sanity_check(nano.slot_A1.state.stick,"stick");
    IMU_sanity_check(nano.slot_A1.state.tool,"tool");
    IMU_sanity_check(nano.slot_F1.state.fork,"fork");
    IMU_sanity_check(nano.slot_F1.state.dump,"dump");
    
    robot.joint.angle.boom=nano.slot_F1.state.boom.pitch;
    robot.joint.angle.stick=nano.slot_A1.state.stick.pitch;
    float tool_pitch_cal = +7.0;
    robot.joint.angle.tilt=nano.slot_A1.state.tool.pitch + tool_pitch_cal;
    robot.joint.angle.spin=0.0f; // nano.slot_A1.state.tool.roll; // now hardware locked
    
    robot.joint.angle.fork=nano.slot_F1.state.fork.pitch;
    robot.joint.angle.dump=nano.slot_F1.state.dump.pitch;
}

/*
 Convert -1.0 to +1.0 float power to discrete -100 to +100 motor percent.  
*/
nanoslot_motorpercent_t motor_scale(float power,const char *what)
{
    const float sanity_limit=4.0;
    if (power<-sanity_limit || power>sanity_limit || power!=power) {
        printf("Power %s ERROR: value %f insane, using 0\n", what,power);
        return 0;
    }
    if (power>1.0) power=1.0;
    if (power<-1.0) power=-1.0;
    
    const float send_limit=100.0;
    return (nanoslot_motorpercent_t)(send_limit*power);
}

void arduino_command_write(robot_base &robot)
{
    // Write commands to the exchange
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.autonomy.mode=(int)robot.state;
    
    // arm power
    if (robot.power.attached_arm()) {
        nano.slot_70.command.torque[0] = motor_scale(robot.power.attached.arm.joint[0],"arm0");
        nano.slot_71.command.torque[0] = motor_scale(robot.power.attached.arm.joint[1],"arm1");
        nano.slot_72.command.torque[0] = motor_scale(robot.power.attached.arm.joint[2],"arm2");
        nano.slot_73.command.torque[0] = motor_scale(robot.power.attached.arm.joint[3],"arm3");
    } 
    
    // mining head power
    float minePower = 0.0;
    if (robot.power.attached_grinder()) minePower=robot.power.attached.grinder.tool;
    nano.slot_C0.command.mine = motor_scale(minePower,"mine");
    
    // load cell read side
    nano.slot_A1.command.read_L = robot.power.read_L;
    nano.slot_F1.command.read_L = robot.power.read_L;
    
    auto &armslot = nano.slot_A0;
    armslot.command.motor[0]=0; // was: -motor_scale(robot.power.spin,"spin");
    armslot.command.motor[1]=motor_scale(
        robot.power.attached_arm()?robot.power.attached.arm.joint[4]:0,
        "clamp"
    );
    armslot.command.motor[2]=motor_scale(robot.power.tilt,"tilt");
    armslot.command.motor[3]=motor_scale(robot.power.stick,"stick");
    
    
    auto &frontslot = nano.slot_F0;
    frontslot.command.motor[0]=-motor_scale(robot.power.dump,"dump");
    frontslot.command.motor[1]=-motor_scale(robot.power.fork,"fork");
    frontslot.command.motor[2]=0; // spare
    frontslot.command.motor[3]=motor_scale(robot.power.boom,"boom");
    
    auto &driveslot = nano.slot_D0;
    nanoslot_motorpercent_t L=motor_scale(robot.power.left,"left");
    nanoslot_motorpercent_t R=motor_scale(robot.power.right,"right");
    driveslot.command.motor[0]=-L;
    driveslot.command.motor[1]=-R;
    driveslot.command.motor[2]=-L;
    driveslot.command.motor[3]=-R;
    
    nano.slot_EE.command.LED=robot.power.right; // just for debugging
    
    nano.backend_heartbeat++;
    exchange_nanoslot.write_end();
}


MAKE_exchange_backend_state();
MAKE_exchange_mining_depth();
MAKE_exchange_drive_encoders();
MAKE_exchange_plan_target();
MAKE_exchange_drive_commands();
//Needed for localization
MAKE_exchange_plan_current();
aurora::robot_loc2D currentLocation;

bool show_GUI=true;
bool simulate_only=false; // --sim flag
bool should_plan_paths=true; // --noplan flag
bool driver_test=false; // --driver_test, path planning testing

bool nodrive=false; // --nodrive flag (for testing indoors)

/* Bogus path planning target when we don't want any path planning to happen. */
aurora::robot_navtarget no_idea_loc(0.0f,0.0f,0.0f);

/** X,Y field target location where we drive to, before finally backing up */
aurora::robot_navtarget dump_target_loc(field_x_trough_center,field_y_trough_stop+20.0,field_angle_trough,
    20.0,30.0,70.0); // get back to starting area

aurora::robot_navtarget dump_align_loc(field_x_trough_center,field_y_trough_stop,field_angle_trough,
    20.0,10.0,5.0); // final alignment

/** X,Y field target location that we target for mining */
aurora::robot_navtarget mine_target_loc(field_x_trough_center,field_y_size-45,90,
    aurora::robot_navtarget::DONTCARE, 30.0,80.0);

/* Convert this unsigned char difference into a float difference */
float fix_wrap256(unsigned char diff) {
  if (diff>128) return diff-256;
  else return diff;
}

int last_Mcount=0;
int speed_Mcount=0;
float smooth_Mcount=0.0;


float last_drive_L=0.0f;
float last_drive_R=0.0f;
void smooth_robot_drive(robot_base &robot,float amount)
{
    robot.power.left  = amount * last_drive_L + (1.0f-amount) * robot.power.left;
    robot.power.right = amount * last_drive_R + (1.0f-amount) * robot.power.right;
    last_drive_L = robot.power.left;
    last_drive_R = robot.power.right;
}



/*********** Robot Joint Planning **************/
// Configuration for weighing scoop: level, with pins aligned vertically
const robot_joint_state weigh_joint_scoop={0,-20, 0,0,0,0};
const robot_joint_state weigh_joint_finish={6,-15,0,0,0,0};

const robot_joint_state drive_joint_scoop={10,-40, 0,0,0,0};


const robot_joint_state dump1_joint_scoop={-10,-80, 0,0,0,0};
//const robot_joint_state dump2_joint_scoop={-20,-80, 0,0,0,0};
//const robot_joint_state dump3_joint_scoop={5,-75, 0,0,0,0};


// Balance a heavy front load by leaning arm way back (balances 2kg on front)
// angles	FD	 10.2	-12.7	BSTS	 34.9	 76.3	-23.0	  0.0
const robot_joint_state balance_drive_joint_state={10,-10, 35,75,-20,0};


/*********** Mining Path Planning ***************/
#include "aurora/mining.h"

/// Starting configuration during mining
//const robot_joint_state mine_joint_base={-17,-30, 20,0,-30,0}; // back
const robot_joint_state mine_joint_base={-17,-30, 10,0,-30,0}; // fairly forward


const robot_joint_state mine_joint_finish={-17,-30, 40,7,-45,0};

/// 0-1 progress of mine cut (0 at start, 1 at end)
float mine_progress=0.0f;


/// Distance away from scoop tip to start mining
const float mine_start_distance=0.25; // allows full depth cut
//const float mine_start_distance=0.0; // viable only up high

// Split single progress into out and up components. 
//   Progress is 0-1 distance into the full cut
//   length determines how far up we will cut
void split_progress(float progress,float &out,float &up,float length) 
{
#if 1 // use 'aggro' to determine how far up we go
    float upstart = 0.0;
    float uplen = length;
#elif 0 // top half, up cut
    float upstart=0.5; // where "up" begins
    float uplen=0.5; // length of "up" cut
#elif 0// top half, down cut
    float upstart=1.0; // where "up" begins
    float uplen=-0.6; // length of "up" cut
#elif 0 // bottom half cut
    float upstart=0.0; // where "up" begins
    float uplen=0.6; // length of "up" cut
#else // full length cut
    float upstart=0.0; // where "up" begins
    float uplen=1.0; // length of "up" cut
#endif

    float iend=0.15; // fraction of cut for lead in/out
    float oend=0.03; // fraction of cut for lead in/out
    float lead=0.05; // meters length of lead in/out
    if (progress<iend) { // start of cut: lead in
        up=upstart;
        out=(iend-progress)/iend*lead;
    }
    else if (progress>1.0-oend) { // end of cut: lead out
        up=upstart+uplen;
        out=(progress - (1.0-oend))/oend*lead;
    }
    else { // middle of cut
        out=0.0;
        up=upstart+uplen*(progress-iend)/(1.0-iend-oend);
    }
}


class mine_planner {
public:
    /// Location of the tip of the mining head scoop, in frame coordinates
    vec3 scoop_tip;

    /// Orientation of mining head while cutting, relative to robot frame coords
    const float mine_tilt_slope=1.2; // 1.0 -> 45 deg.  2.0 -> about 60 deg

    const robot_coord3D mine_cut_coord=robot_coord3D(
        vec3(0,0,0), 
        vec3(1,0,0), // X axis is straight and level
        vec3(0,1,-mine_tilt_slope).dir(), // Y axis is pointing diagonal down
        vec3(0,mine_tilt_slope,+1).dir(), // Z axis is pointing diagonal up
        99.0 // confidence value
    );
    
    vec3 vec_from_mineangle(float angle_deg) {
        vec3 r=vec3_from_angle(angle_deg);
        return vec3(r.z,r.x,r.y);
    }

    /// Compute the mining target (in frame coords) for this amount of mining progress.
    ///  Frame pitch is in degrees. 
    int lookup_mine_target(float frame_pitch,float progress,float depth,vec3 &mine_target) {
        vec3 up = vec_from_mineangle(mine_pit_angle-frame_pitch);
        vec3 in = vec3(0,1,0); // advance along Y (forward only)
        vec3 start = scoop_tip + vec3(0,mine_start_distance,mine_floor_height);
        mine_target = start + up*progress + in*depth;
        
        return 1;
    }


    /// Given a 3D frame-coordinates point for the tip of the rock grinder,
    /// set these joints to put the arm at that point. 
    int target_plan(const vec3 &mine_target,robot_joint_state &mine_joint)
    {
        // mine head angle pivots around this point
        //vec3 head_center = vec3(0,0,1.2); // with tool coupler
        vec3 head_center = vec3(0,-0.2,1.2); // without tool coupler
        //vec3 head_center = vec3(0,0.5,0.5); // angle head up during hard cuts (broke gear?)
        
        vec3 head_look = (mine_target - head_center).dir();
        robot_coord3D head_coords=robot_coord3D(
            vec3(0,0,0),
            vec3(1,0,0), // X axis is straight and level
            head_look, // Y axis is pointing diagonal down
            vec3(0,-head_look.z,head_look.y), // Z axis is pointing diagonal up
            99.0 // confidence value
        );
        
        // Figure out the tilt axis target
        robot_coord3D tool_coords = robot_link_coords::parent_from_child(
            link_tilt, link_grinder, head_coords);
       
        vec3 tilt_target = mine_target - head_coords.world_from_local(tool_coords.origin);
        
        float tilt_deg = excahauler_IK::frame_degrees(head_coords.Y);

        // Figure out the joint angles to reach that target
        int ret = ik.solve_tilt(mine_joint,tilt_target,tilt_deg);
        if (ret<=0) return ret; // couldn't do IK solve
        
        if (1) 
            robotPrintln("  Grinding head target %.3f, %.3f -> joint BS %.0f %.0f\n",
                mine_target.y, mine_target.z, 
                mine_joint.angle.boom, mine_joint.angle.stick);
        
        // Sanity & safety check
        if (!joint_state_sane(mine_joint)) {
            return -99;
        }
        
        return 1;
    }

    // Given a depth image, plan the joint states for a mining pass.
    //  Returns positive value if this joint state seems reachable and safe,
    //  negative on error.
    int mine_plan(float frame_pitch,float progress,float depth,robot_joint_state &mine_joint)
    {
        vec3 target;
        if (lookup_mine_target(frame_pitch,progress,depth,target)<=0) return -1;
        return target_plan(target,mine_joint);
    }

    mine_planner(const aurora::mining_depth &mining_view)
        :mining(mining_view)
    {
        robot_link_coords coord(mine_joint_base);
        
        vec3 scoop_tip_local(0,0.308,0.168);
        
        scoop_tip = coord.coord3D(link_dump).world_from_local(scoop_tip_local);
    }

private:
    const aurora::mining_depth &mining;
    excahauler_IK ik;
    

};


/**
  This class is used to localize the robot
*/
class robot_locator {
public:
  /** Merged location */
  robot_localization merged;
};


/**
 This class represents everything the back end knows about the robot.
*/
class robot_manager_t
{
public:
  robot_base robot; // overall integrated current state
  
  // Attached tool
  robot_power::attach_mode_t attach_mode = robot_power::attach_none; 

  int substep=0; // within an autonomous step, this is a sub-step (starts at 0)

  // Read (write?) copy of nano data
  nanoslot_exchange nano;
  
  robot_locator locator; // localization
  robot_telemetry telemetry; // next-sent telemetry value
  robot_command command; // last-received command
  robot_comms comms; // network link to front end
  robot_ui ui; // keyboard interface
  
  // Autonomous mining interface
  aurora::mining_depth mining; // view of mined area
  mine_planner mp;
  float stall_backoff = 0.0f; // mining head stall response

  robot_simulator sim;
  int robot_insanity_counter = 0;

  robot_manager_t() 
    :mp(mining)
  {
    // Zero out the joints until we hear otherwise
    for (int i=0;i<robot_joint_state::count;i++) robot.joint.array[i]=0.0f;
    
    // Restore previous accumulated data (so we don't lose daily totals)
    const robot_base &old_state = exchange_backend_state.read();
    robot.accum = old_state.accum;
    
    
    ui.joystickState = state_backend_driver; // we're the backend
    
    arduino_setup_exchange();
    atexit(arduino_exit_exchange);

    // Start simulation in random real start location
    sim.loc.y=80.0;
    sim.loc.x= (rand()%10)*20.0+100.0;
    sim.loc.angle=((rand()%8)*8)/360;
    sim.loc.percent=50.0;

    // robot.state = state_STOP;  // physical robot: safe mode
    robot.state = state_backend_driver; // sim or testing: drive from backend (e.g., joystick)
    ui.power.torque=0;
  }

  // Do robot work.
  void update(void);
  
  // Do robot work, and display it onscreen
  void update_GUI(void);
  
  // Switch active camera (heading 0 is facing forward)
  void point_camera(float heading) {
  }


private:

  /* Use OpenGL to draw this robot navigation grid object */
  template <class grid_t>
  void gl_draw_grid(grid_t grid)
  {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (int y=0;y<rmc_navigator::GRIDY;y++)
    for (int x=0;x<rmc_navigator::GRIDX;x++)
    {
      int height=grid.at(x,y);
      if (height>0) {
        if (height>50) glColor3f(0.0f,1.0f,1.0f); // cyan trough / walls
        else if (height<15) glColor3f(1.0f,0.5f,1.0f); // purple very short
        else if (height<20) glColor3f(1.0f,0.0f,0.0f); // red short-ish
        else  glColor3f(1.0f,1.0f,1.0f); // white tall
        glVertex2f(
          rmc_navigator::GRIDSIZE*x,
          rmc_navigator::GRIDSIZE*y);
      }
    }
    glEnd();
  }

  // Autonomy support:
  double cur_time; // seconds since start of backend program
  double state_start_time; // cur_time when we entered the current state
  double mine_start_time; // cur_time when we last started mining
  double autonomy_start_time; // cur_time when we started full autonomy
  
  // If true, the mining head has been extended
  bool mining_head_extended=false;
  // If true, the mining head is down in the dirt
  bool mining_head_lowered=true;

  robot_state_t last_state;

  // Enter a new state (semi)autonomously
  void enter_state(robot_state_t new_state)
  {
    // Flush old planned path on state change
    exchange_plan_target.write_begin()=no_idea_loc;
    exchange_plan_target.write_end();

    if (new_state==state_autonomy) { autonomy_start_time=cur_time; }

    // Log state timings to dedicate state timing file:
    static FILE *timelog=fopen("timing.log","w");
    if (timelog) {
        fprintf(timelog,"%4d spent %6.3f seconds in %s\n",
          (int)(cur_time-autonomy_start_time),
          cur_time-state_start_time, state_to_string(robot.state));
        fflush(timelog);
    }

    // Make state transition
    last_state=robot.state; // stash old state
    robot.state=new_state;
    robotPrintln("Entering new state %s",state_to_string(robot.state));
    state_start_time=cur_time;
    substep=0;
    last_drive_L=0.0f;
    last_drive_R=0.0f;
  }

  // Advance autonomous state machine
  void autonomous_state(void);

  // Dump bucket encoder target a/d values


  // Limit this value to lie in this +- range
  template <typename T>
  T limit(T v,T range) {
    if (v>range) return range;
    if (v<-range) return -range;
    else return v;
  }

  // Run autonomous mining, if possible
  bool tryMineMode(void) {
    //if (drive_posture()) {    
    robot.power.attached.grinder.tool=0.5; // TUNE THIS mining head rate
    robot.power.dump=0; // TUNE THIS lowering rate
    mining_head_lowered=true;
    
    
    return true;
  }
  
  /// Set power values to move this joint.  Returns true once we're there
  bool move_single_joint(float target, float cur, float &power,float scale=1.0, float cap=1.0)
  {
    float err=target-cur;
    const float P=0.2; 
    float command=P*scale*err; // + a derivative term from IMU rates?
    command = limit(command,cap);
    power=command;
    
    return fabs(err)<1.5;
  }
  
  /// Set power values to move the front scoop (fork & dump) to this joint state. 
  ///   Returns true when we're basically there.
  bool move_scoop(const robot_joint_state &j)
  {
    robot.joint_plan.angle.fork = j.angle.fork;
    robot.joint_plan.angle.dump = j.angle.dump;
    
    // SUBTLE: can't use short-circuit AND && here, it serializes joint motion.
    bool scoop = 
        move_single_joint(j.angle.fork,robot.joint.angle.fork,robot.power.fork) &
        move_single_joint(j.angle.dump,robot.joint.angle.dump,robot.power.dump);
    return scoop;
  }
  
  /// Set power values to move the robot arm (boom, stick, tilt) to this joint state.
  /// Returns true when we're basically there.
  bool move_arm(const robot_joint_state &j,float speed=1.0)
  {
    robot.joint_plan = j;
    robotPrintln(" move_arm target\tFD\t%5.1f\t%5.1f\tBSTS\t%5.1f\t%5.1f\t%5.1f\t%5.1f",
                j.angle.fork, j.angle.dump,   j.angle.boom, j.angle.stick, j.angle.tilt, j.angle.spin);
    
    bool arm = 
        move_single_joint(j.angle.boom,robot.joint.angle.boom,robot.power.boom,-speed) &
        move_single_joint(j.angle.stick,robot.joint.angle.stick,robot.power.stick,speed) &
        move_single_joint(j.angle.tilt,robot.joint.angle.tilt,robot.power.tilt,speed);
        // &
        //move_single_joint(j.angle.spin,robot.joint.angle.spin,robot.power.spin);
    
    return arm;        
  }
  
  

  // Set the mining head linear and dump linear to natural driving posture
  //  Return true if we're safe to drive
  bool drive_posture() {
/*
    if(mining_head_lowered && cur_time-state_start_time <10)
      robot.power.dump = 1.0;
    if (sim.bucket>0.9) { // we're back up in driving range
      mining_head_lowered=false;
    }
*/
    return true; // Kept for compatiiblity
  }

  // Autonomous driving rate:
  //  Returns 0-1.0 float power value.
  float drive_speed(float forward,float turn=0.0) {
    return robot.tuneable.drive;
  }

  // Autonomous drive power from float values:
  //   "drive": forward +1.0, backward -1.0
  //   "turn": left turn +1.0, right turn -1.0 (like angle)
  void set_drive_powers(double forward,double turn=0.0)
  {
    double max_autonomous_drive=1.0; //<- can set a cap for debugging autonomous

    double drive_power=drive_speed(+1.0);
    double t=limit(turn,drive_power);
    double d=limit(forward,drive_power);
    double L=d-t;
    double R=d+t;
    robot.power.left= limit(L,max_autonomous_drive);
    robot.power.right=limit(R,max_autonomous_drive);
  }

  // Autonomous feeler-based backing up: drive backward slowly until both switches engage.
  //  Return true when we're finally backed up properly.
  bool back_up()
  {
    if(!(drive_posture())) {return false;}
    else {
      set_drive_powers(-0.1);

      // FIXME: back-up sensors?
      return true; // (robot.sensor.backL && robot.sensor.backR);
    }
  }

  //  Returns true once we're basically at the target location.
  //  Uses the autonomous driving path planner, a separate process.
  bool autonomous_drive_planner(const aurora::robot_navtarget &target) {
    if (!drive_posture()) return false; // don't drive yet
     vec2 cur(locator.merged.x,locator.merged.y); // robot location
    // Send off request to the path planner
    exchange_plan_target.write_begin()=target;
    exchange_plan_target.write_end();
    
    // Check for a response from the path planner
    static aurora::drive_commands last_drive={0.0f,0.0f};
    static double last_drive_update=0.0;
    const double max_drive_seconds=1.0; // drive this many long on an old plan
    
    if (exchange_drive_commands.updated()) {
      last_drive=exchange_drive_commands.read();
      last_drive_update=cur_time;
    }
    if (cur_time - last_drive_update<max_drive_seconds && last_drive.is_sane()) 
    {
      robot_insanity_counter = 0;
      if(last_drive.left < 0 && last_drive.right < 0)
      {
        point_camera(180);
      }
      else 
      {
        point_camera(0);
      }
      float autonomous_drive_power = .5 ; // scale factor for drive in autonomous
      robot.power.left =last_drive.left * autonomous_drive_power;
      robot.power.right=last_drive.right * autonomous_drive_power;
    }
    else 
    { // Fall back to greedy local autonomous driving: set powers to drive toward this field X,Y location
      robotPrintln("Invalid drive command dectected increasing robot insanity counter");
      robot_insanity_counter ++;
      // Tune this value based on path planning time on pi.
      if (robot_insanity_counter >= 10) 
      {
        robotPrintln("Robot insanity counter has reached 10.. exiting autonomy");
        enter_state(state_drive);
      }
      
    }

    return target.matches(locator.merged); // we're basically there
  }

  // Force this angle (or angle difference) to be between -180 and +180,
  //   by adding or subtracting 360 degrees.
  void reduce_angle(double &angle) {
    while (angle>=180) angle-=360; // reduce
    while (angle<-180) angle+=360; // reduce
  }
  
  //  Drive in straight line toward this target. 
  //   Ideally put the target just past where you want to end up.
  bool autonomous_drive_dumb(const aurora::robot_navtarget &target, float speed=1.0) {
    if (!drive_posture()) return false; // don't drive yet
    vec2 cur2D(locator.merged.x,locator.merged.y); // robot location
    vec2 target2D(target.x,target.y);
    
    // vec2 target_orient = target.forward(); //<- FIXME: this is ignored for now
    
    double drive_power=speed * drive_speed(+1.0);
    
    vec2 orient = locator.merged.forward(); // orientation vector (forward vector of robot)
    vec2 should=normalize(cur2D-target2D); // we should be facing this way

    double turn=orient.x*should.y-orient.y*should.x; // cross product (like sin of angle)
    
    if (orient.dot(should)>0.0) turn = -turn; // if backing up, turn the opposite way
    
    double drive=dot(orient,should); // dot product (like distance)

    double t=limit(turn,drive_power);
    double d=limit(drive,drive_power);
    double L=-d+t;
    double R=-d-t;
    robot.power.left=limit(L,drive_power);
    robot.power.right=limit(R,drive_power);

    return length(cur2D-target2D)<20.0; // we're basically there
    
    
  }


  // Autonomous turning: rotate robot so it's facing this direction.
  //  Returns true once we're basically at the target angle.
  // ToDo: Point camera to an appropriate angle as you turn
  bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
  {
    //if (do_posture) { if (!drive_posture()) return false; } // don't drive yet 
    double angle_err_deg=locator.merged.angle-angle_target_deg;
    reduce_angle(angle_err_deg);
    robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
      angle_target_deg, locator.merged.angle);
   
    double turn=angle_err_deg*0.1; // proportional control
    double maxturn=0.8*drive_speed(0.0,1.0);
    turn=limit(turn,maxturn);
    set_drive_powers(0.0,-turn);
    return fabs(angle_err_deg)<5.0; // angle error tolerance
  }

  /// Call when something has gone wrong with autonomous operations
  void autonomous_fail(const char *what) {
    printf("\n\nAUTONOMOUS FAIL: %s\n\n",what);
    
    // Log state timings to dedicate state timing file:
    static FILE *autolog=fopen("autonomous.log","w");
    if (autolog) {
        fprintf(autolog,"%.3f: Fail at time %.3f into state %s: %s\n",
          cur_time,
          cur_time-state_start_time, 
          state_to_string(robot.state),
          what);
        fflush(autolog);
    }

    enter_state(state_drive);
  }

  // Make sure we're still facing this angle.  If not, pivot to face it.
  bool check_angle(double target_deg) {
    if (locator.merged.percent<10.0) autonomous_fail("check_angle needs location");
    double err=locator.merged.angle-target_deg;
    robotPrintln("check_angle: cur %.1f deg, target %.1f deg",locator.merged.angle,target_deg);
    
    reduce_angle(err);
    if (fabs(err)<10.0) return true; // keep driving--straight enough
    else return autonomous_turn(target_deg,false); // turn to face target
  }
  
  bool haul_out_phase = true; // outbound: increasing Y.  inbound: decreasing Y
  
  
  /// Check the drive battery, if bad fail now
  void drive_battery_check(float threshold_percent=0.0) {
    if (robot.sensor.charge_D<threshold_percent) autonomous_fail("Low drive battery");
  }
  
  
  /// Return true if we're done doing autonomous hauling trip
  bool haul_drive_done() {
    const float haul_distance = 500.0; // meters to drive
    
    const float haul_Y_start = 15.0;
    const float haul_Y_dist = 8.0;
    const float haul_X_target = 5.0;
    aurora::robot_navtarget target_out(haul_X_target, haul_Y_start + haul_Y_dist + 5.0,90.0);
    aurora::robot_navtarget target_back(haul_X_target, haul_Y_start - 5.0,90.0);
    
    
    drive_battery_check();
    
    // Stop driving when we reach the total required distance
    if (robot.accum.drive >= haul_distance) return true;
    
    /* Else we're on a drive cycle: */
    // if (check_angle(90.0f)) 
    {
        float progress = (locator.merged.y - haul_Y_start)/haul_Y_dist;
        if (progress<0.0) progress=0.0;
        if (progress>1.0) progress=1.0;
        if (!haul_out_phase) progress = 1.0-progress;
        
        if (progress >= 1.0) { // we're there!
            haul_out_phase = !haul_out_phase; // flip to next phase
        }
        
        if (haul_out_phase==true) {
            autonomous_drive_dumb(target_out);
        }
        else {
            autonomous_drive_dumb(target_back);
        }
        robotPrintln("Autohaul: progress %.2f   %s", 
            progress, haul_out_phase?"out":"back");
        
        //set_drive_powers(power * haul_power, 0.0);
        
    }
    // Avoid jerky driving by averaging drive commands
    smooth_robot_drive(robot,0.9); 

    return false; //<- still trying!
  }
};


// Return true if the mining head is stalled (according to our sensors
bool is_stalled(const robot_base &robot) {
  return robot.sensor.Mstall;
}


/* Utility function: slow down speed as cur approaches target
  Returns false if already past target.
*/
bool speed_limit(int &howfast,int cur,int target,int dir=+1)
{
  int dist_left=(target-cur)*dir;
  if (dist_left<=0) {
    return false;
  }
  int max_speed=10+dist_left/5;
  if (howfast>max_speed) howfast=max_speed;
  return true;
}

void robot_manager_t::autonomous_state()
{
  robot.power.stop(); // each state starts from scratch

  double time_in_state=cur_time-state_start_time;
  robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);

  // full autonomy start
  if (robot.state==state_autonomy) {
    enter_state(state_scan);
  }
  // Clear accumulated data to start a new day
  else if (robot.state==state_daily_start)
  {
    robot.accum.scoop=0;
    robot.accum.scoop_total=0;
    robot.accum.drive=0;
    robot.accum.drive_total=0;
    robot.accum.op_total=0;
  }
  
  // scan terrain before mining
  else if (robot.state==state_scan)
  {
    
    if(time_in_state<2.0) // stare at terrain
    {
      // FIXME: activate vision_mining (via backend state?)
    }
    else{
      mine_start_time=cur_time; // update mine start time
      enter_state(state_mine_start);
    }
  }
  //state_mine_lower: enter mining state
  else if (robot.state==state_mine_start) {
    robot_joint_state mine_joint=mine_joint_base;
    mine_progress=0.0f;
    stall_backoff=0.0f;
    
    /*if (!robot_IMUs_OK) {
        enter_state(state_drive); // bad IMUs, don't try to mine
    }
    else*/
    if ( // move_scoop(mine_joint) && 
        move_arm(mine_joint)) {
        enter_state(state_mine);
    }
  }
  else if (robot.state==state_mine)
  {
    // Tool is running
    robot.power.attached.grinder.tool=std::min(robot.tuneable.tool, mine_power_limit);
    
    float aggro = 0.5; // robot.tuneable.aggro; // aggression during mining
    bool advance = true; // cutting head should progress along the cut
    bool backoff = false; // cutting head should move back from the cut face
    
    // Stall check using mining rate:
    if (robot.sensor.minerate < 50.0) { // stall potential?
        advance = false;
        //stall_backoff += 0.001f; // ease back just a bit
        if (robot.sensor.minerate ==0.0) { // definitely stalled!
            backoff = true;
        }
    }
    
    // Side wedge detection using tool load cell:
    if (robot.sensor.load_TR<-10.0f) {
        advance=false;
        backoff=true;
    }
    
    // Apply bools to adjust cut parameters
    const float cap_backoff = 0.1f; // don't physically back off more than this

    if (backoff) 
    { // cut not going well, increase backoff
        stall_backoff += 0.02f;
        const float max_backoff = 0.3f;
        if (stall_backoff > max_backoff) {
            stall_backoff=max_backoff*0.4; //< allow a faster restart
            enter_state(state_STOP); 
        }
    }
    else if (advance) { // normal cut, reduce backoff
        stall_backoff = std::min(cap_backoff,stall_backoff); // limit backoff
        stall_backoff = stall_backoff*0.96 - 0.005*aggro;
        if (stall_backoff<0.0) stall_backoff=0.0;
    }
    
    if (stall_backoff>0.0f) advance=false;
    
    // Path planning into the cut face
    robot_joint_state mine_joint=mine_joint_base;
    float out=0.0f; // meters extra distance back
    float up=0.0f; // 0-1 progress up
    split_progress(mine_progress,out,up,robot.tuneable.aggro);
    
    /// Current depth to mine below the observed surface (meters)
    /// Negative = clearance above surface, to clear obstacles.
    float mine_cut_depth=0.0f + 0.01f*robot.tuneable.cut 
        - std::min(cap_backoff, stall_backoff) - out; // m
    
    if (mp.mine_plan(robot.sensor.frame_pitch,up,mine_cut_depth,mine_joint)<0) enter_state(state_STOP);
    robotPrintln("Mining: progress %.3f -> out %.3f up %.3f",
        mine_progress,out,up);
    
    // move_scoop(mine_joint); //<- keep the scoop firmly in place
    if (move_arm(mine_joint)) 
    {
        if (advance) {
            mine_progress+=0.004*aggro;
        }
        
        if (mine_progress>=1.0f) {
             mine_progress=0.0f;
            robot.power.attached.grinder.tool=0.0;
            enter_state(state_mine_finish);
        }
    }
    
    /*
    // Don't mine forever (timing leash)
    double mine_time=cur_time-mine_start_time;
    double mine_duration=30.0;
    if(mine_time>mine_duration)
    {
        enter_state(state_mine_finish);
    } // done mining
    */
    
    // if (robot.sensor.Mstall) enter_state(state_mine_stall);
  }

  // state_mine_stall: Detect mining head stall. Raise head until cleared
  else if (robot.state==state_mine_stall)
  {
    if (time_in_state<1) {
        
        enter_state(state_mine); 
    }
    else { // too long
        enter_state(state_STOP); 
    }
    
    if(robot.sensor.Mstall && time_in_state<1)
    {
      robot.power.boom=-1.0f; // retract the boom (pull out of cut)
    }
    else {enter_state(state_mine);} // not stalled? Then back to mining
  }

  //Done mining: Raise scoop
  else if (robot.state==state_mine_finish)
  {
    if (move_arm(mine_joint_finish)) { 
        enter_state(state_STOP); 
        // enter_state(state_weigh);
    }
  }
  
  //Weigh material before leaving pit
  else if (robot.state==state_weigh)
  {
    switch (substep) {
    case 0: // move to weigh configuration
        if (!robot_IMUs_OK) { // avoid dumping material all over ground
            enter_state(state_drive);
        }
        else {
            state_start_time=cur_time; // hack!  need a "time in sub-state" here?
            if (move_scoop(weigh_joint_scoop)) 
            {
                substep++;
            }
        }
        break;
    case 1: // weigh left        
        if(time_in_state<1.5) {
            // let dirt settle, read right channel
            robot.power.read_L=0;
        }
        else substep++;
        break;

    case 2: // weigh right
        if (time_in_state<3.0) { // read left channel
            robot.power.read_L=1;
        }
        else {
            substep++;
            
            // Record total weight here
            float total = -(robot.sensor.load_SL + robot.sensor.load_SR);
            robotPrintln("Total scoop weight: %.2f kgf\n",total);
            
            robot.power.read_L=0;
            robot.accum.scoop=total;
        }
        break;
     
     case 3: // move back to driving configuration
        if (move_scoop(weigh_joint_finish)) 
        {
                //enter_state(state_haul_start);
                enter_state(state_STOP); // manual control
        }
        break;
    }
  }
  
  // Begin haul cycle
  else if (robot.state==state_haul_start)
  {
    robot.accum.drive_total += robot.accum.drive;
    robot.accum.drive = 0.0f;
    enter_state(state_haul_out);
  }

  // Drive back to dump area
  else if (robot.state==state_haul_out)
  {
    if (haul_drive_done()) enter_state(state_drive); // position for dumping
    
  }
  // Dump material
  else if (robot.state==state_haul_dump)
  { 
    robot.accum.drive_total += robot.accum.drive;
    robot.accum.drive = 0.0f;
    
    if (move_scoop(dump1_joint_scoop)) 
    {
      robot.accum.scoop_total += robot.accum.scoop;
      robot.accum.scoop = 0.0;
      robot.accum.drive_total += robot.accum.drive;
      robot.accum.drive = 0.0;

      // enter_state(state_haul_back); // need to lower scoop before we can drive again
      enter_state(state_STOP);
    }
  }
  // Drive back into pit
  else if (robot.state==state_haul_back)
  {
    if (haul_drive_done()) enter_state(state_drive); // position for dumping
    
  }
  else if (robot.state==state_haul_finish)
  {
    robot.accum.drive_total += robot.accum.drive;
    robot.accum.drive = 0.0f;
    
    enter_state(state_drive); // manual control
  }
  
  
  
  
  // Stow the robot (like for moving it)
  else if (robot.state==state_stow)
  {
    if(mining_head_lowered)
      drive_posture();
    if(time_in_state<20)
      robot.power.dump=-1.0f;
    enter_state(state_stowed);

  }
  else if (robot.state==state_stowed)
  {
    /* wait here forever */
  }
  else
  { // what?  unrecognized state?!  manual mode...
    robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
    enter_state(state_drive);
  }

  if (nodrive)
  { // do not drive!  (except for state_drive)
    robotPrintln("NODRIVE");
    set_drive_powers(0.0,0.0);
  }
}


#include <chrono>

typedef std::chrono::high_resolution_clock roboclock;


void robot_manager_t::update(void) {
  static auto clock_start=roboclock::now();
    
  cur_time=0.001*(std::chrono::duration_cast<std::chrono::milliseconds>(
        roboclock::now() - clock_start
      ).count());

  static double last_time=cur_time;
  double dt=cur_time-last_time;
  if (dt>0.1) dt=0.1;
  last_time=cur_time;
  
  
#if 1 /* enable for backend UI: dangerous, but useful for autonomy testing w/o frontend */
  // Keyboard control
  //ui.power.attach_mode = attach_mode;
  ui.update(oglKeyMap,robot);

  // Click to set state:
  if (robotState_requested<state_last) {
    robot.state=robotState_requested;
    robotPrintln("Entering new state %s (%d) by backend UI request",
      state_to_string(robot.state),robot.state);
    robotState_requested=state_last; // clear UI request
  }
#endif

  
// Check for a command broadcast (briefly)
  int n;
  while (0!=(n=comms.available(10))) {
    if (n==sizeof(command)) {
      comms.receive(command);
      if (command.command==robot_command::command_STOP)
      { // ESTOP command
        enter_state(state_STOP);
        robot.power.stop();
        robotPrintln("Incoming STOP command");
      }
      else if (command.command==robot_command::command_state)
      {
        if (command.state>=state_STOP && command.state<state_last)
        {
          robot.state=(robot_state_t)command.state;
          telemetry.ack_state=robot.state;
          robotPrintln("Entering new state %s (%d) by frontend request",
            state_to_string(robot.state),robot.state);
        } else {
          robotPrintln("ERROR!  IGNORING INVALID STATE %d!!\n",command.state);
        }
      }
      else if (command.command==robot_command::command_power)
      { // manual driving power command
        robotPrintln("Incoming power command: %d bytes",n);
        robot.tuneable = command.tuneable;
        if (robot.state==state_drive || robot.state==state_driveraw)
        {
          // if (robot.state==state_drive) FIXME: sanity-check the frontend drive commands
          robot.power=command.power;
        }
        else
        {
          robotPrintln("IGNORING frontend power: not in drive state\n");
        }
      }
    } else {
      robotPrintln("ERROR: COMMAND VERSION MISMATCH!  Expected %d, got %d",
        sizeof(command),n);
    }

  }

// Perform action based on state recieved from FrontEnd
  //E-Stop command
  if(robot.state==state_STOP)
  {// All stop
    robot.power.stop();
    state_start_time=cur_time;
  }
  else if (robot.state==state_drive || robot.state==state_driveraw)
  { // do nothing-- already got power command
    state_start_time=cur_time;
  }
  else if (robot.state==state_backend_driver)
  { // set robot power from backend UI
    robot.power=ui.power;
  }
  else if (robot.state>=state_autonomy) { // autonomous mode!
    autonomous_state();
  }
  
  
  // Sanity check joint state and power commands
  bool driveraw = robot.state==state_driveraw; 
  const char *hazards = joint_move_hazards(robot.joint,robot.power);
  if (hazards !=NULL && !driveraw) 
  {
    robotPrintln(" -- HAZARD: %s --", hazards);
    // robot.power.stop();
  }


  // Send commands to Arduino
  robot_sensors_arduino old_sensor=robot.sensor;
    
  if (simulate_only) { // build fake arduino data
    robot.joint = sim.joint;
    robot.sensor.Mcount=0xff&(int)sim.Mcount;
    robot.sensor.DLcount=0xffff&(int)sim.DLcount;
    robot.sensor.DRcount=0xffff&(int)sim.DRcount;
    robot.sensor.connected=0x3F; // bits 0-5 all set
  }
  else { // Send data to/from real arduino
    arduino_sensor_read(robot);
    nano=exchange_nanoslot.read();
  }
  if (nano.slot_A0.sensor.stop && robot.state!=state_STOP) {
    enter_state(state_STOP);
    robot.power.stop();
    robotPrintln("Slot A0 STOP command");
  }
  
  if (nano.slot_C0.state.connected) {
    attach_mode = robot_power::attach_grinder;
    robotPrintln("Mining head: %5.3f  %5.3f V   %.2f mine\n",
        nano.slot_C0.state.load, nano.slot_C0.state.cell, robot.power.attached.grinder.tool); 
  }
  if (nano.slot_70.state.connected) {
    attach_mode = robot_power::attach_arm;
    robotPrintln("Arm: angles %5.1f    torque %.2f (%s)\n",
        nano.slot_70.state.angle[0], robot.power.attached.arm.joint[0],
        robot.power.attached_arm()?"attached":"??"); 
  }

   
  // Accumulate drivetrain encoder counts into actual distances
  float fudge=1.0; // fudge factor to make distance equal reality
  float rightCal=0.7; // (did we lose a magnet?)
  float drivecount2m=fudge*0.96/12; // meters of driving per wheel encoder tick == circumference of wheel divided by encoder ticks per revolution
  float driveL = fix_wrap256(robot.sensor.DLcount-old_sensor.DLcount)*drivecount2m;
  float driveR = fix_wrap256(robot.sensor.DRcount-old_sensor.DRcount)*drivecount2m*rightCal;
  
  // Flip encoder signs to match last nonzero drive power value
  static robot_power last_nonzero_power;
  if (robot.power.left!=0 || robot.power.right!=0) last_nonzero_power=robot.power;
  if (last_nonzero_power.left<0) driveL=-driveL;
  if (last_nonzero_power.right<0) driveR=-driveR;

  // robot.accum.drive += fabs(driveR + driveL)*0.5; // average wheel drive distance (meters)
  
  static double last_distance_time = cur_time;
  const double time_per_distance_check=0.3;
  const double max_speed = 2.0; // maximum plausible drive speed, m/s
  if ((driveL !=0 || driveR != 0) && (cur_time - last_distance_time)>time_per_distance_check) 
  { // Track accumulated driving distance
    static vec2 old_loc = locator.merged.center();
    vec2 new_loc = locator.merged.center();
    float dist = length(new_loc - old_loc);
    old_loc = new_loc;
    const double max_dist = max_speed * time_per_distance_check;
    if (dist>0.0 && dist<max_dist) { // seems like a valid motion
        robot.accum.drive += dist;
    }
    last_distance_time = cur_time;
  }
  
  if (robot.state > state_STOP) robot.accum.op_total += dt;
  
  // Update drive encoders data exchange
  static aurora::drive_encoders::real_t totalL = -driveL; //<- hacky!  Need to total up distance
  static aurora::drive_encoders::real_t totalR = -driveR;
  totalL += driveL;
  totalR += driveR;
  aurora::drive_encoders enc;
  enc.left =totalL;
  enc.right=totalR;
  exchange_drive_encoders.write_begin()=enc;
  exchange_drive_encoders.write_end();
  
  locator.merged=exchange_plan_current.read();


// Send out telemetry
  arduino_command_write(robot);

  static double last_send=0.0;
  if (cur_time>last_send+0.050)
  {
    last_send=cur_time;
    // robotPrintln("Sending telemetry, waiting for command");
    robot.loc=locator.merged;
    locator.merged.percent*=0.999; // slowly lose location fix

    // Wacky way to copy over all robot_base fields from robot to telemetry:
    *static_cast<robot_base *>(&telemetry) = robot; 
    
    telemetry.count++;
    telemetry.state=robot.state; // copy current values out for send
    
    comms.broadcast(telemetry);
  }

  if (locator.merged.percent>=10.0)  // make sim track reality
    sim.loc=locator.merged;

  if (simulate_only) // make reality track sim
  {
    locator.merged.percent=std::min(100.0,locator.merged.percent*(1.0-dt));
  }
  sim.simulate(robot.power,dt);
  
  // Drop the current state onto the exchange
  aurora::backend_state s{robot};
  s.cur_time = cur_time;
  s.state_start_time = state_start_time;
  exchange_backend_state.write_begin() = s;
  exchange_backend_state.write_end();
  
}


robot_manager_t *robot_manager;
unsigned int video_texture_ID=0;

void robot_manager_t::update_GUI(void) {
    update();

    // Show estimated robot location
    robot_2D_display(locator.merged);
    robot_display_autonomy(telemetry.autonomy);
    
    // Draw current robot joint configuration (side view)
    robot_3D_setup();
    if (0) { // visually depict physical robot tilts (neat, but mining is robot relative)
        glRotatef(nano.slot_F1.state.frame.pitch,1,0,0);
        glRotatef(nano.slot_F1.state.frame.roll,0,1,0);
    }
    tool_type tool=robot.sensor.connected_tool();
    robot_3D_draw(robot.joint,tool);
    
    // Draw mining depths
    mining=exchange_mining_depth.read();
    glColor3f(0,1,0);
    glBegin(GL_LINES);
    for (int d=0;d<aurora::mining_depth::ndepth;d++) //< vertical samples across image
    {
        vec3 v=mining.depth[d]; // 3D viewed spot, in frame coords
        if (v.z!=0.0)
            glVertex3fv(v); 
    }
    glEnd();
    
    robot_3D_draw(robot.joint_plan,tool,0.3f);
    
    robot_3D_cleanup();

}


void display(void) {
  robot_display_setup(robot_manager->robot);

  robot_manager->update_GUI();
  
  robot_display_finish(robot_manager->robot);

  if (video_texture_ID) {
    glTranslatef(field_x_GUI+350.0,100.0,0.0);
    glScalef(300.0,200.0,1.0);
    glBindTexture(GL_TEXTURE_2D,video_texture_ID);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUAD_STRIP);
    glTexCoord2f(0.0,0.0); glVertex2f(0.0,0.0);
    glTexCoord2f(1.0,0.0); glVertex2f(+1.0,0.0);
    glTexCoord2f(0.0,1.0); glVertex2f(0.0,+1.0);
    glTexCoord2f(1.0,1.0); glVertex2f(+1.0,+1.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D,0);
  }

  glutSwapBuffers();
  glutPostRedisplay();
}

int main(int argc,char *argv[])
{
  // Set screen size
  int w=1000, h=600;
  for (int argi=1;argi<argc;argi++) {
    if (0==strcmp(argv[argi],"--sim")) {
      simulate_only=true;
      if (argi+1<argc) srand(atoi(argv[++argi])); // optional seed argument
      else srand(1);
    }
    else if (0==strcmp(argv[argi],"--noplan")) {
      should_plan_paths=false;
    }
    else if (0==strcmp(argv[argi],"--driver_test")) {
      simulate_only=true;
      driver_test=true;
    }
    else if (0==strcmp(argv[argi],"--nogui")) { 
      show_GUI=false;
      robotPrintgl_enable=false;
    }
    else if (0==strcmp(argv[argi],"--nodrive")) {
      nodrive=true;
    }
    else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
    else {
      printf("Unrecognized argument '%s'!\n",argv[argi]);
      exit(1);
    }
  }
  
  if (show_GUI) {
      // setenv("DISPLAY", ":0",1); // never forward GUI over X
      glutInit(&argc,argv);
  }

  robot_manager=new robot_manager_t;
  robot_manager->locator.merged.y=100;
  if (simulate_only) robot_manager->locator.merged.x=150;

  if (show_GUI) 
  { // interactive GUI version (for debugging)
    glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
    glutInitWindowSize(w,h);
    glutCreateWindow("Robot Backend");
    robotMainSetup();

    glutDisplayFunc(display);
    glutMainLoop();
  }
  else
  { // fast stripped-down no-GUI version (for headless robot)
    while (true) {
      robot_manager->update();
      robot_display_telemetry(robot_manager->robot);
      
      aurora::data_exchange_sleep(30); // limits CPU usage
    }
  }
  return 0;
}

