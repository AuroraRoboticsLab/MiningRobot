/*
 Robot joint designed around a planetary gearset that hangs on a single pivot bearing.

 Screws hold the printed parts to the bearing and a steel frame. 
 
 Target is to get from like 10K rpm (for a 12V x 1000 RPM/V brushless motor),
 down to like 10 rpm (60 deg/sec) for a robot arm joint.
 
 As designed reduction ratio: 1300:1 (!)
 
 The planet carrier also needs to be tightly constrained: possibly by cone-shaped 
 protrusions in the middle of the stepped planets?  
 Extra M3 bearings to constrain it vertically?  
 Side-mounted into the carrier to push up on the main case, and down on the drive sun
    These need to be 623ZZ, which have a big enough diameter for a 1.5mm lip to hold on.
 
 FIXME:
    - Add cones to planets to self-align carrier.
    - Center the boundary between gearplanes, leaving more wire room on top above the carrier?
    - Design 3D printed wire guide / encoder magnet holder parts (standardize on 7/8" wire hole in the steel?)
 

 Assembly order:
    - Add tiny 683ZZ bearings to planet gears
    - Thread planet gears to carrier with M3x30 screws
        - There are two planet timings, planets across from each other should be identical.
    - Attach sun gear to planet carrier:
        - Add 623ZZ guide bearings
 
 
 
 
 Dr. Orion Lawlor, lawlor@alaska.edu 2024 fall (Public Domain)
*/
include <AuroraSCAD/bearing.scad>;
include <AuroraSCAD/gear.scad>;
include <AuroraSCAD/screw.scad>;
include <AuroraSCAD/motor.scad>;
include <AuroraSCAD/bevel.scad>;

$fs=0.1; $fa=3;

inch = 25.4; // file units: mm

// Central space for wiring
wireholeID = 22.0; // 0.75*inch; // diameter for wiring: 18mm is enough for a USB A plug
#cylinder(d=wireholeID,h=50,center=true);
wireholeOD = 26.0; // hole through sun gear

/*
 Main pivot bearing: secures primary metal part loads together.
 
 6815 bearing: rated static / dynamic load: 12.7 kN radial.
 "Thin-section deep groove ball bearings can support axial loads of between 10 and 30 percent of the bearing's static radial load rating"
 That'd be 1.3 - 3.8 kN axial. 
*/
mainbearing = bearing_6815;
bearingBC = [0,0,-bearingZ(mainbearing)]; // center of bearing bottom face
bearingTC = [0,0,0]; // center of bearing top face

// Halfway out is the handoff between the bearing halves
mainbearingR = (bearingIR(mainbearing)+bearingOR(mainbearing))/2;



/* 
Screws that mount plastic parts to the bearing.
*/
Bscrew = US10_24_pan_screw;

TscrewZ = 3; // vertical space between gearplanes

// Z clearance for head of Bscrew
BscrewZ = 5;  // space between bearing and bottom frame
BscrewR = screw_diameter(Bscrew)/2+0.1; // centerline to edge of plastic
BscrewIR = bearingIR(mainbearing) - BscrewR; // centerline of screws inside bearing
BscrewOR = bearingOR(mainbearing) + BscrewR; // centerline of screws outside bearing

module screw_array(R,N=12,sinlimit=1.1)
{
    dA = 360/N;
    for (angle=[dA/2:dA:360-1]) 
        if (abs(sin(angle))<sinlimit)
            rotate([0,0,angle]) translate([R,0,0]) children();
}

module BscrewIR_array() {
    translate(bearingTC) screw_array(BscrewIR,12,0.8) screw_3D(Bscrew);
}
module BscrewOR_array() {
    translate(bearingBC) screw_array(BscrewOR,16,0.7) rotate([180,0,0]) screw_3D(Bscrew);
}


// Bottom (B) input gearplane: sun driven by motor
gearZ = 10; // thickness of planetary gears
geartypeB = geartype_create(1.0,gearZ);
gearplaneB = [geartypeB, 32, 12, 4];
gearBC = bearingBC + [0,0,0]; // bottom gearplance start

// Top (T) output gearplane
gearplaneT = gearplane_stepped(gearplaneB, +1/2);
gearTC = gearBC + [0,0,gearZ + TscrewZ]; // top gearplane start


// Planet carrier axle screw: M3 x 30mm
Pscrew = M3_cap_screw;
PscrewZ = 30.5;

// Total height of planet gear stack
Pbase=6; // base of planet carrier, below gear
Ptop=5; // top of planet carrier, above gear
Ptopcap=2; // top of planet carrier below bolt caps
Pheight = PscrewZ - Pbase - Ptopcap; // height of full planetary gear stack


ring_gear_B_thick=BscrewZ+2.5; // height of bottom ring gear plate, under bearing
ring_gear_TZ = 22; // height of top ring gear and cover


sundrive_bearing = bearing_6704;
carrier_bearing = bearing_6704;
carrier_bearingC=gearTC+[0,0,gearZ+Ptopcap];

carrier_baseC = gearBC + [0,0,-Pbase];

carrier_boltR = 4.0; // material around planet carrier thru bolts
carrier_clearR = gearplane_Oradius(gearplaneB)+carrier_boltR+0.5; // carrier spinning keep-out zone
  

geartypeMR = geartype_550; // motor output gear
motor_gearZ=8;
geartypeM = geartype_create(geartypeMR[0],motor_gearZ,geartypeMR[2]);

sundrive_gearZ = 11;
geartypeS = geartype_create(geartypeM[0],sundrive_gearZ,geartypeM[2]);

// Drives sun gear above
sundrive_gear = gear_create(geartypeS,70);
sundrive_gearR = gear_R(sundrive_gear);
sundrive_gearC = bearingBC - [0,0,ring_gear_B_thick+0.5+sundrive_gearZ];

frameB_clearR = gear_OR(sundrive_gear)+2.0; // frame cut to clear sundrive gear

// Reduces motor to sundrive
reduce_gearZ = sundrive_gearZ;
reduce_ratio = 3.5; // motor to sungear reduction ratio

reducesun_gear = gear_create(geartypeS,12); // mates with sun on top
reducesun_gearR = gear_R(reducesun_gear);

reducemotor_gear = gear_create(geartypeM,gear_nteeth(reducesun_gear)*reduce_ratio); // motor drives the bottom
reducemotor_gearR = gear_R(reducemotor_gear);

reduce_gearR = gear_R(reducesun_gear) + gear_R(reducemotor_gear);

reduce_gearC = sundrive_gearC + [0,sundrive_gearR+reducesun_gearR,-1-motor_gearZ];

// Directly attached to motor shaft
motor_gear = gear_create(geartypeM,15); // metal gears, "32 pitch"
motor_gearR = gear_R(motor_gear);
motor_gearC = reduce_gearC + [-(reducemotor_gearR+motor_gearR),0,0];



retain_bearing = bearing_623; // larger M3 bearings retain sun gear and planet carrier
planet_bearing = bearing_683; // micro M3 bearing holds planets to M3 shaft

retainsunB_bearing = bearing_683; // holds sun gear to bottom frame
retainsunB_R = 22; // radius where those bearings spin
retainsunB_Z = sundrive_gearC[2]+2; // bottom of mounting screw cap



// Spaces for M3 bolt and planet-size bearings inside gear
module bearing_spaces(loZ,hiZ)
{
    cylinder(d=4,h=200,center=true); // clear center axle hole
    
    for (z=[loZ-0.1, hiZ+0.1-bearingZ(planet_bearing)])
        translate([0,0,z])
            bearing3D(planet_bearing,hole=0);
}


// Reduction gear
module reduce_gear_whole()
{
    fullz = motor_gearZ+1+reduce_gearZ;
    L = reducemotor_gear;
    H = reducesun_gear;
    translate(reduce_gearC) 
    difference() {
        union() {
            gear_3D(L);
            gear_3D(H, height=fullz);
            // Bevel up to avoid stress concentration
            translate([0,0,motor_gearZ-0.01])
                cylinder(d1=gear_OD(H),d2=gear_ID(H),h=1);
        }
        bearing_spaces(0,fullz);
    }
}



// Draw 2D version of all gears
module illustrate_gears_2D() 
{
    color(0.7*[0,1,1]) translate(gearBC) gearplane_2D(gearplaneB);
    translate(gearTC) gearplane_2D(gearplaneT);
    translate(sundrive_gearC) gear_2D(sundrive_gear);
    translate(motor_gearC) gear_2D(motor_gear);
}


// Angle that planet carrier retaining bolts are rotated to
carrier_retainA = 360/gearplane_Pcount(gearplaneB)/2;
carrier_retainZ = -Pbase-1.5; // Z base of sun gear retaining bearings (relative to gearBC)
// Radius of surface where retain bearings run on sun gear
carrier_retainIR = gearplane_Oradius(gearplaneB) - bearingOR(retain_bearing);
carrier_retainIH = bearingZ(retain_bearing)+0.3; // height of inside channel, plus wiggle room

carrier_retainRT = 1.5; // lip R thickness
carrier_retainZT = 2.0; // lip Z thickness
carrier_retainB = 0.8; // bevel on lips
carrier_retainOR = carrier_retainIR+carrier_retainRT; // outside space
carrier_retainOH = carrier_retainIH+2*carrier_retainZT; // outside height

// Makes children at center of each planet carrier thru bolt
module planet_carrier_bolts() {

    // Top-down bolts through planet gears
    gearplane_planets(gearplaneB) 
        translate([0,0,Pheight+Ptop-3])
            children();

    // Bottom-up bolts through sun retain bearings
    rotate([0,0,carrier_retainA])
    gearplane_planets(gearplaneB) 
        translate([0,0,carrier_retainZ]) 
            scale([1,1,-1]) // facing up
                children();
}

// Cap of bolts used to retain planet carrier
module planet_carrier_boltcap(capH=3)
{
    cylinder(d=screw_head_diameter(Pscrew)+0.2,h=capH);
}

// Space for M3 x 30mm bolt, with cap at Z==0, shaft facing down
module planet_carrier_bolt(capH=3.5,len=30,taplen=7,extratap=7) {
    // Socket cap space on top
    planet_carrier_boltcap(capH);
    
    // Tap down into plastic on bottom
    translate([0,0,-len-extratap]) 
        cylinder(d=screw_tap_diameter(Pscrew),h=taplen+extratap);
    
    // Shaft space
    translate([0,0,-len+taplen-0.01]) 
        cylinder(d=screw_diameter(Pscrew),h=len-taplen+0.02);
}

// 2D outline of outside of planet carrier
module planet_carrier2D() {
    hull() 
    planet_carrier_bolts() circle(r=carrier_boltR);
}

// Tiny upright tapered pad that holds M3 bearing away from printed parts
module M3_bearing_pad(z=0,padZ=0.5,flip=+1) {
    translate([0,0,z])
        scale([1,1,flip])
            cylinder(d1=6, d2=4.5,h=padZ);
}

// Space around each planet gear
module planet_space(tipclear,holeZ)
{
    difference()
    {
        translate([0,0,-tipclear])
        bevelcylinder(d=gear_OD(gearplane_Pgear(gearplaneB))+2*tipclear,h=holeZ,bevel=1.2*tipclear);
        
        padZ=0.5; // little pads that bearings run on
        for (side=[0,1]) {
            z1 = -tipclear-0.01;
            z2 = holeZ-tipclear+0.01;
            z = side?z2:z1;
            M3_bearing_pad(z,padZ,side?-1:+1);
        }
    }
}

// Space for planet carrier-to-sundrive retaining bearings
module carrier_retain_bearing()
{
    //#bearing3D(retain_bearing);
    type=retain_bearing;
    space=0.5; // space around the bearing
    z=bearingZ(type)+space;
    difference() {
        bevelcylinder(d=bearingOD(type)+2*space,h=z,bevel=space);
        M3_bearing_pad(z,space,-1);
    }
}

// Frame that holds the M3 screws that secure the planet gears, 
//   and keeps them from tilting.
// This part takes the final output gear torque, so it needs to be strong.
module planet_carrier() {
    tipclear=1.0; // clearance around gear tips (for debris / print fuzz)
    round=8;
    difference() {
        translate(carrier_baseC)
        linear_extrude(height=Pbase+Pheight+Ptop,convexity=6)
        offset(r=-round) offset(r=+round)
        union() {
            // Walls of carrier
            difference() {
                hull() {
                    planet_carrier2D();
                    circle(r=6+gear_OR(gearplane_Sgear(gearplaneB))+tipclear);
                }
                offset(r=-8) planet_carrier2D(); // inside walls
            }
            //planet_carrier_bolts() circle(r=carrier_boltR);
        }
        
        // Clearance for tips of the larger sun gear
        translate(carrier_baseC+[0,0,-0.01])
            cylinder(r=gear_OR(gearplane_Sgear(gearplaneB))+tipclear,h=Pbase+Pheight+tipclear);
        
        translate(carrier_bearingC) hull() bearing3D(carrier_bearing);
        
        // space for planet gears in middle
        holeZ = Pheight+2*tipclear;
        translate(gearBC) 
        gearplane_planets(gearplaneB) 
        planet_space(tipclear,holeZ);
        
        // M3 thru bolts
        translate(gearBC)
        planet_carrier_bolts() 
            planet_carrier_bolt();
        
        // Space for bearings on bottom, to retain sundrive gear
        translate(gearBC)
        rotate([0,0,carrier_retainA])
        gearplane_planets(gearplaneB) 
        {
            // base to retain sundrive
            translate([0,0,carrier_retainZ])
                carrier_retain_bearing();
        }
        
        // Space for lips that cling to retain bearings
        lipclear=0.5;
        translate(gearBC + [0,0,carrier_retainZ-carrier_retainZT])
        {
            bevelcylinder(d=2*(carrier_retainOR+lipclear),h=carrier_retainOH+lipclear,bevel=carrier_retainB);
        }
        
    }
}

// Clearance around planet carrier
module planet_carrier_clear() {
    translate(carrier_baseC) cylinder(r=carrier_clearR,h=Pbase+Pheight+Ptop+0.5);
}

// Planet gear interior spaces
module planet_gear_minus() {
    // Top and bottom bearings on planet axles
    loZ = gearBC[2];
    hiZ = gearTC[2]+gearZ;
    
    bearing_spaces(loZ,hiZ);
    
    // Central clearance hole, to push solid profiles closer to shear zone
    dZ = 0.65*(hiZ-loZ); // Z diameter of clearance sphere
    gD = gear_ID(gearplane_Pgear(gearplaneT))*0.60; // XY diameter of hole
    translate([0,0,(loZ+hiZ)/2]) scale([gD/dZ,gD/dZ,1.0]) sphere(d=dZ);
}

// Planet gear set
module planet_gears()
{
    extratooth = 3; // extend gear teeth in Z this far, for strength
    h = gearZ+extratooth;
    difference() {
        union() {
            gearplane_planets(gearplaneB) {
                translate(gearBC) 
                    gear_3D(gearplane_Pgear(gearplaneB),height=h); 
            }
            gearplane_planets(gearplaneT) {
                translate(gearTC-[0,0,extratooth]) 
                    gear_3D(gearplane_Pgear(gearplaneT),height=h);
            }
        }
        gearplane_planets(gearplaneB)
            planet_gear_minus();
    }
}


// Channel that carrier retain bearings run in, used by sun gear and bottom ring
//    (R,Z) is the center of the bottom of the retaining bolt cap.
// Leaves space for M3 retaining bolts
module retain_bearing_channel(enlarge=0, accessholes=0, 
    bearing=retain_bearing,
    R=gearplane_Oradius(gearplaneB), Z=gearBC[2]+carrier_retainZ, flip=0)
{        
    capZ=3.2; // height of M3 retaining bolt cap
    capD=6.5;
    capR=1.0; // extra rounding on cap slot (stress riser)
    retain_round=0.5;
    rotate_extrude()
    offset(r=+retain_round+enlarge) offset(r=-retain_round)
    translate([R,Z,0])
    rotate([0,0,flip*180])
    {
        // Space for the bearings proper
        BR = bearingOR(bearing);
        BH = bearingZ(bearing)+0.3;
        translate([0,+BH/2])
            square([2*BR,BH],center=true);
        
        // Space for retaining bolt caps to cycle around
        translate([0,-(capZ/2-capR)+0.01])
            offset(r=+capR) offset(r=-capR)
                square([capD,capZ+2*capR],center=true);
    }
    
    // Access holes through sun to tighten M3 screws that retain bearings
    if (accessholes) translate(gearBC + [0,0,carrier_retainZ])
        rotate([0,0,carrier_retainA])
            gearplane_planets(gearplaneB) 
                cylinder(d=capD,h=50,center=true);
}        


// Bottom ring gear: held by Bscrews
module ring_gear_B()
{
    difference() {
        union() {
            // Material inside bearing
            translate(gearBC) cylinder(d=bearingID(mainbearing),h=bearingZ(mainbearing));
            // Material outside
            taper=5;
            translate(gearBC) scale([1,1,-1])
                cylinder(r=mainbearingR,h=ring_gear_B_thick); // plate under ring
        }
        translate(gearBC) ring_gear_cut(gearplane_Rgear(gearplaneB)); // teeth
        BscrewIR_array();
        planet_carrier_clear();
        frame_steel(0);
        hull() retain_bearing_channel(enlarge=0.2); // a little clearance
        
        translate(reduce_gearC) scale([1,1,-1]) planet_carrier_bolt();
    }
}

// Top ring gear and frame: held by Bscrews
module ring_gear_T()
{
    start=-4; // Z coordinate where this object begins
    difference() {
        union() {
            //translate(gearTC) cylinder(d=bearingOD(mainbearing)+3,h=bearingZ(mainbearing));
            
            translate(bearingTC+[0,0,start])
            {
                // Tapered dust cover over the bearing outside surface
                hull() {
                    cylinder(d=2+bearingOD(mainbearing),h=2-start);
                    translate([0,0,2+TscrewZ-start])
                        cylinder(d=-5+bearingID(mainbearing),h=2);
                }
                
                linear_extrude(height=ring_gear_TZ-start)
                {
                    hull() offset(r=3)
                        projection(cut=true)
                            BscrewOR_array();
                            
                }
            }
        }
        
        // Inside ring gear teeth
        start=-5;
        translate(gearTC+[0,0,start]) gear_3D(gearplane_Rgear(gearplaneT), height=gearZ-start+0.5);

        // Screw holes (tapped into plastic)
        BscrewOR_array();
        
        // Space for the planet carrier and bearing
        planet_carrier_clear();
        translate(carrier_bearingC) hull() bearing3D(carrier_bearing);

        frame_steel();
        translate(bearingBC) cylinder(d=bearingOD(mainbearing),h=bearingZ(mainbearing));
        
        translate(bearingTC+[0,0,-0.01+start]) {
            // Space above moving bearing surface
            cylinder(r=bearingOD(mainbearing)/2-2,h=1-start);
            
            // Space above inside bolt heads
            cylinder(r=BscrewIR+screw_head_diameter(Bscrew)/2+1,h=TscrewZ-start);
        }
    }
}


// Sundrive and bottom sun gear:
module sundrive_gear_whole() 
{
    D = sundrive_gear;
    DT = sundrive_gearC+[0,0,gear_height(D)-0.01]; // top of drive gear
    SB = gearBC + [0,0,0.01]; // bottom of sun gear
    S = gearplane_Sgear(gearplaneB);
    
    difference() {
        union() {
            translate(sundrive_gearC) gear_3D(D);
            translate(DT) {
                ID = gear_ID(S);
                // Straight wall all the way up to top gears
                cylinder(d=ID,h=gearTC[2]-DT[2]-0.5);
                // Tapered transition
                //for (t=[0:3]) cylinder(d1=6-t+ID,d2=ID,h=2+t);
            }
            for (zcopy=[0,0.5,1.1]) // extend gear teeth down to DT
                translate(gearBC*(1.0-zcopy)+DT*zcopy) gear_3D(S,height=gearZ+TscrewZ-1.0);
            
            // Lips that cling to retain bearings
            translate(gearBC + [0,0,carrier_retainZ-carrier_retainZT])
            {
                bevelcylinder(d=2*(carrier_retainOR),h=carrier_retainOH,bevel=carrier_retainB);
            }
        }
        
        // Retain bearings holding sun to carrier
        retain_bearing_channel(accessholes=1);
        
        // Retain bearings holding sun to bottom frame
        for (inset=[0,1])
        retain_bearing_channel(accessholes=0, bearing=retainsunB_bearing, R=retainsunB_R-inset, Z=retainsunB_Z, flip=1);
        
        
        // Wiring thru hole
        cylinder(d=wireholeOD,h=100,center=true);

        // Slope out end of wiring channel
        slope=4;
        translate(sundrive_gearC) 
        for (ds=[0:0.5:slope])
            cylinder(d2=wireholeOD, d1=wireholeOD+2*slope-2*ds, h=slope+ds);
    }
}

module motor_gear_whole() 
{
    translate(motor_gearC) gear_3D(motor_gear) {
        translate([0,0,-5]) motor_3D_shaft(motortype);
    }
}


module sunidler_gear_whole() 
{
    translate(gearTC) gear_3D(gearplane_Sgear(gearplaneT)) {
        cylinder(d=wireholeOD,h=50,center=true);
    }
}

module illustrate_gears() 
{
    reduce_gear_whole();
    sundrive_gear_whole();
    motor_gear_whole();


    planet_gears();
    planet_carrier();

    //sunidler_gear_whole();

    //ring_gear_T();
    //ring_gear_B();

    
    //#translate(carrier_bearingC) bearing3D(carrier_bearing);
}





echo("Reduction ratio: ",reduce_ratio * sundrive_gearR / motor_gearR * gearplane_stepped_ratio(gearplaneB,gearplaneT));
echo("Gear pitches: ",geartype_Dpitch(geartypeM),geartype_Dpitch(geartypeB),geartype_Dpitch(gearplane_geartype(gearplaneT)));
echo("Wiring channel: ",wireholeOD," in sun gear: ",gear_ID(gearplane_Sgear(gearplaneT)));
echo("Motor gear center: ",motor_gearC);


frameZ = 1.0*inch; // thickness of steel frame bar
frameBevel = 0.7;
frameBC = bearingBC + [0,0,-BscrewZ-0.5*frameZ]; // Center of bottom frame bar
frameTC = bearingTC + [0,0,0.5*frameZ]; // Center of top frame bar

/*
 Drive motor
*/
motortype = motortype_NEMA17;
motorC = [motor_gearC[0],motor_gearC[1],frameBC[2]-frameZ/2];// center of motor mount

// Outline of 1x1 inch steel tubing, minus clearance cuts
module frame_steel(clearance=1) {
    difference() {
        for (Z=[frameBC,frameTC]) translate(Z) 
            bevelcube(bevel=frameBevel,[5,1,1]*inch,center=true); // steel frame
        
        if (clearance) {
            // Top tube clearance cut
            translate(bearingTC+[0,0,-0.01]) cylinder(r=mainbearingR,h=carrier_bearingC[2]-bearingTC[2] + bearingZ(carrier_bearing));
            
            // Bottom tube clearance cut (FIXME, need space for sundrive!)
            translate(bearingBC+[0,0,+0.01]) scale([1,1,-1]) cylinder(r=frameB_clearR,h=bearingBC[2]-sundrive_gearC[2]+1);
        }
    }
}


module illustrate_covers() {
    ring_gear_T();
    ring_gear_B();
}

module illustrate_frame() {
    #translate(bearingBC) bearing3D(mainbearing);

    #frame_steel();

    #translate(motorC) motor_3D(motortype);


    #BscrewIR_array();
    #BscrewOR_array();
}

module printable_gears() 
{
    translate(-carrier_baseC) rotate([0,0,45]) planet_carrier();
    rotate([180,0,0]) translate(-(gearTC+[1.1*mainbearingR,0,gearZ])) rotate([0,0,45]) planet_gears();
    //translate(-gearTC) sunidler_gear_whole();
    translate(-sundrive_gearC+[1.0*mainbearingR,0,0]) sundrive_gear_whole();
    translate(-motor_gearC) motor_gear_whole();
}



illustrate_frame();
//illustrate_covers();

//#BscrewIR_array();
//illustrate_gears_2D();
if (1) difference() { union() {
        illustrate_gears();
        ring_gear_B();
    }
    translate([0,0,-100]) cube([200,200,200]);
}

//translate(-carrier_baseC) planet_carrier();
//translate([0,-65,0]) printable_gears();
//translate(-(gearBC-[0,0,ring_gear_B_thick])) ring_gear_B();
//rotate([180,0,0]) translate([0,0,-ring_gear_TZ]) ring_gear_T();
