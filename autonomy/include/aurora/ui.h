/**
  Aurora Robotics keyboard/joystick user interface code.
  
  This is used by both frontend (drive) and backend (backend_driver)
  to convert keyboard and joystick input into robot power commands.
  
  This is the place to add new user interface modes and features.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__UI_H
#define __AURORA_ROBOTICS__UI_H

#include "ogl/event.h" /* for joystick */

#ifdef MSL
	#include "msl/joystick.hpp"
	#include "msl/joystick.cpp"
#endif
#include <iostream>

/**
 Keyboard-based user interface for robot.
 Inherits from robot_power to control those variables.
*/
class robot_ui : private robot_power {
public:
	float driveLimit=0.6; /* robot driving power (0.0 - 1.0) */
	robot_tuneables tuneable;
	
	robot_state_t joystickState=state_drive;
	
	enum {
	    joyModeSTOP=0, // don't drive
	    joyModeLow=1, // bottom of robot: drive and scoop
	    joyModeHigh=3, // top of robot arm / grinder
	    joyModeArm=4, // move robot arm
	} joyMode=joyModeLow; // joystick mode selected
	
	robot_power power; // Last output power commands
	
	#ifdef MSL
		msl::joystick_t* joystick;
	#endif

	// Human-readable description of current state
	std::string description;

	bool js_button(int button,const std::string& label)
	{
		#ifdef MSL
			if(button==1)
				button=15;
			else if(button==2)
				button=12;
			else if(button==3)
				button=14;
			else if(button==4)
				button=13;
			else if(button==5)
				button=0;
			else if(button==6)
				button=3;
			else
				button=-1;
			if(button>-1&&joystick!=NULL&&joystick->good()&&button<(int)joystick->button_count())
				return joystick->button(button);
		#else
			return oglButton(button,label.c_str());
		#endif
		return false;
	}

	bool js_button_once(const int button,const std::string& label)
	{
		#ifdef MSL
			return js_button(button,label);
		#else
			return oglButtonOnce(button,label.c_str());
		#endif
		return false;
	}

	float js_axis(int axis,const std::string& label)
	{
		#ifdef MSL
			--axis;
			if(axis==2)
				axis=3;
			else if(axis==3)
				axis=2;
			std::cerr<<axis<<"="<<joystick->axis(axis)<<std::endl;
			if(joystick!=NULL&&joystick->good()&&axis<(int)joystick->axis_count())
				return joystick->axis(axis);
		#else
			return oglAxis(axis,label.c_str());
		#endif
		return false;
	}

	void stop(void) {
		robot_power::stop();
		power.stop();
		description="Sending STOP";
	}

	// Respond to these keystrokes.
	//  The "keys" array is indexed by the character, 0 for up, 1 for down.
	void update(int keys[],const robot_base &robot);

	robot_ui()
	{
		#ifdef MSL
		joystick=NULL;
		#endif
		stop();
		description="Starting up";
		
		tuneable.tool=0.46;
		tuneable.cut=5.0;
		tuneable.aggro=0.5;
		tuneable.drive=0.6;
	}

	// Clamp this float within this maximum range
	float limit(float v,float maxPower) const {
		if (v>maxPower) v=maxPower;
		if (v<-maxPower) v=-maxPower;
		return v;
	}

	// Convert a raw float to a motor command, with this maximum range
	byte toMotor(float v,float maxPower) const {
		v=limit(v,maxPower);
		int iv=(int)(v*100);
		if (iv<-100) iv=-100;
		if (iv>100) iv=100;
		return iv;
	}
	
	// Filter a raw joystick axis (remove jittery deadband)
	float filter_axis(float v) {
	    const float minV=0.03f;
	    if (v>minV) {
	        return v-minV;
	    }
	    else if (v<-minV) {
	        return v+minV;
	    }
	    else /* v is tiny, between min and -min */ {
	        return 0.0f;
	    }
	}
	
	
    /* Set this keyboard-controlled power limit--
      Use P + number keys to set drive power, in percent:
        P-1 = 10%, P-2=20%, etc. 
    */
    void setPowerLimit(int keys[],char lowercase,char uppercase,float &limit,
        float base=0.0, float scale=1.0)
    {
        const int nkeys=13; // 0 through 12 inclusive
        const static char powerkeys[nkeys]={
            '`','1','2','3','4','5','6','7','8','9','0','-','='
        };
        
	    if (keys[lowercase]||keys[uppercase]) 
	    {
	        for (int i=0;i<nkeys;i++)
	            if (keys[powerkeys[i]]) limit = base + scale*0.1f*i;
	    /*
	        for (int num=1;num<=9;num++)
	            if (keys['0'+num]) 
	                limit=base+scale*0.1f*num;
	        if (keys['0']) limit=base+scale*1.0f;
	        if (keys['`'] || keys['~']) limit=base+scale*0.0f;
        */
	    }
	}
	
	std::string showPowerPercent(const float &limit)
	{
	    char buf[32];
	    snprintf(buf,32,"%.0f%% ",(double)limit*100.0f);
	    return buf;
    }
	std::string showPowerFrac(const float &limit)
	{
	    char buf[32];
	    snprintf(buf,32,"%.2f  ",(double)limit);
	    return buf;
    }

};


void robot_ui::update(int keys[],const robot_base &robot) {
	#ifdef MSL
		if(joystick==NULL)
		{
			auto joysticks=msl::joystick_t::list();
			if(joysticks.size()>0)
			{
				joystick=new msl::joystick_t(joysticks[0]);
				joystick->open();
			}
		}
		if(joystick!=NULL&&!joystick->good())
		{
			delete joystick;
			joystick=NULL;
		}
	#endif

	static int keys_last[256];
	int keys_once[256]; // key down only *one* time per press
	for (int i=0;i<256;i++) {
		keys_once[i]= (keys[i] && !keys_last[i]);
		keys_last[i]=keys[i];
	}
	description="UI ";

// Power limits:
	float scoopLimit=1.0; // limit on fork & dump
	float armLimit=1.0; // limit on boom, stick, tilt

// Prepare a command:
	left=right=0.0;
	float forward=0.0, turn=0.0; //<- turned into left and right
	
	fork=dump=0.0;
	
	boom=stick=tilt=0.0; 

/*
Joysticks have different axis and button numbering:

 Logitech Gamepad F310:
    axis 1: left analog X
    axis 2: left analog Y
    axis 3: left trigger analog
    axis 4: right analog X
    axis 5: right analog Y    
    axis 6: right trigger analog
        (Releasing trigger analog drops it to -max, but we start it at 0)
    
    button 1: A (green)
    button 2: B (red)
    button 3: X (blue)
    button 4: Y (orange)
    button 5: left top trigger
    button 6: right top trigger
 
 Saitek PLC Cyborg Force Rumble Pad
    axis 1: left analog X
    axis 2: left analog Y
    axis 3: right analog Y
    axis 4: right analog X
    
    buttons 1-6 as labelled
    button 7: left trigger
    button 8: right trigger
*/
    // Defaults are for Logitech
    int axis_lx=1, axis_ly=2;
    int axis_rx=4, axis_ry=5;
    int button_stop=3, button_low=1, button_arm=4, button_high=2;
    int button_topleft=5, button_topright=6; // shoulder buttons
    
    const char *joystick_name=oglJoystickName();
    if (joystick_name[0]=='S') { // Saitek
        axis_ry=3; // for some reason this uses axis 3
        button_stop=1; button_low=3; button_arm=2; button_high=4;
        button_topleft=7; button_topright=8;
    }

	/* Read left analog stick X and Y axes*/
	float ly=filter_axis(-js_axis(axis_ly,"")); 
	float lx=filter_axis(js_axis(axis_lx,"")); 

    /* Read the right analog stick */
    float ry=filter_axis(-js_axis(axis_ry,""));
    float rx=filter_axis(js_axis(axis_rx,""));
    
	// Left shoulder acts as positive confirmation joystick switch
	//   so joystick axes are ignored if it's not held down.
	if (js_button(button_topleft,"live shoulder")) {
	    description += "joystick ";
	} else {
	    // killswitch released, disable joystick axes
	    lx = ly = rx = ry = 0.0;
	}
	
	// Treat the WASD keys like left analog stick (for keyboard-only driving)
	if (keys['a'] || keys['A']) lx=-1.0;
	if (keys['d'] || keys['D']) lx=+1.0;
	if (keys['w'] || keys['W']) ly=+1.0;
	if (keys['s'] || keys['S']) ly=-1.0;
	
	// Treat the arrow keys like right analog stick
	if (keys[oglSpecialLeft])  rx=-1.0;
	if (keys[oglSpecialRight]) rx=+1.0;
	if (keys[oglSpecialUp])    ry=+1.0;
	if (keys[oglSpecialDown])  ry=-1.0;
	
	// Pressing a button changes the mode persistently
	if (js_button(button_low,"low") || keys['b'])  
    { 
        joyMode=joyModeLow; 
        robotState_requested=joystickState; 
    }
	if (js_button(button_high,"high") || keys['h'])  
	{ 
	    joyMode=joyModeHigh; 
        robotState_requested=joystickState; 
	}
	
	if (js_button(button_arm,"arm") || keys['j'])  
    { 
        joyMode=joyModeArm; 
        attach_mode=attach_arm;
        robotState_requested=joystickState; 
    }
	
	// Pop previous state (for hierarchical autonomy stub)
	if (keys_once['P'])
	{ 
	    if (joyMode==joyModeSTOP) joyMode=joyModeHigh; 
	    robotState_requested = state_POP; 
	}

	if (js_button(button_stop,"stop button") || 
	    keys[' '])  // spacebar killswitch
    { 
        joyMode=joyModeSTOP;
        robotState_requested=state_STOP;
	}
	
	// Apply joystick (or keyboard) inputs:
    switch (joyMode) {
    case joyModeSTOP: 
        stop(); 
	    break;
    case joyModeLow: 
        description += " Low: drive fork-dump ";
        forward=ly;
        turn=lx;
        
        fork=-ry;
        dump=-rx;
        break;
    case joyModeHigh:
        description += " High: stick-boom tilt-mine ";
        
        stick=ly;
        boom=lx;
        
        tilt=ry;
        if (attached_grinder()) attached.grinder.tool=rx;
        else if (attached_arm()) attached.arm.joint[0]=rx;
        break;
    case joyModeArm:
        if (attached_arm()) {
    	    description += " Arm: swing-nod slant-spin ";
            attached.arm.joint[0]=lx;
            attached.arm.joint[1]=rx;
            attached.arm.joint[2]=ly;
            attached.arm.joint[3]=ry;
        }
        break;
    default:
    	break;
    }
    
	if (joyMode == joyModeArm) {
        float grab=0.0;
        if (js_button(button_topright,"do grab") || keys['g']) 
            grab=0.2; // positive grab
        if (keys['r']) 
            grab=-0.2; // release
        attached.arm.joint[4]=grab;
	}

// Adjust power limits
    
	setPowerLimit(keys,'p','p',driveLimit);
	setPowerLimit(keys,'t','T',tuneable.tool,0.4f,0.2f);
	setPowerLimit(keys,'c','C',tuneable.cut,0.0f,10.0f);
	setPowerLimit(keys,'v','V',tuneable.cut,0.5f,10.0f);
	setPowerLimit(keys,'g','G',tuneable.aggro);
	setPowerLimit(keys,'f','F',tuneable.drive);
	
    std::cout<<"UI desc pretune: "<<description<<std::endl;
    description+="\n  Tuneables:";
	description+="  Drive "+showPowerPercent(driveLimit);
	description+="  Tool "+showPowerPercent(tuneable.tool);
	description+="  Cut "+showPowerFrac(tuneable.cut);
	description+="  Aggro "+showPowerFrac(tuneable.aggro);
	description+="  Auto "+showPowerPercent(tuneable.drive);
	description+="\n";
	

// Drive keys:
	left=driveLimit*(forward+turn);
	right=driveLimit*(forward-turn);
    
    power.read_L = keys['l'] || keys['L'];
    /*
    if (keys_once['l']||keys_once['L']) {
        power.read_L = !power.read_L;
    }
    */

// Limit powers, and write them to the output struct
	left=limit(left,driveLimit);
	right=limit(right,driveLimit);
	
	fork=limit(fork,scoopLimit);
	dump=limit(dump,scoopLimit);
	
	boom=limit(boom,armLimit);
	stick=limit(stick,armLimit);
	tilt=limit(tilt,armLimit);
	
	if (attached_grinder()) {
	    attached.grinder.tool=limit(attached.grinder.tool,tuneable.tool);
    }
    if (attached_arm()) {
        for (int j=0;j<robot_power::njoints;j++) {
            attached.arm.joint[j]=limit(attached.arm.joint[j],0.4*armLimit);
        }
    }
    
	// Blend in power to smooth our motion commands, for less jerky operation
	if (power.attach_mode != attach_mode) {
	    // Just changed attachment mode--don't blend, just copy
	    power = *this;
	}
	else { // same mode, smooth blend
        power.blend_from(*this,0.2);
    }
    
    std::cout<<"UI desc end: "<<description<<std::endl;
	robotPrintLines(description);
	power.print("UI power");
}



#endif

