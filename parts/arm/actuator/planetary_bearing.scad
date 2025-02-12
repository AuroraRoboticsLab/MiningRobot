/*
 Robot joint designed around a planetary gearset that hangs on a single 
 large pivot bearing.

 #10-24 steel screws hold the printed parts to the bearing and a steel tubes. 
 
 As designed gear reduction ratio: 2500:1 (!)
 Target is to get from like 12K rpm (for a 12V x 1000 RPM/V brushless motor),
 down to like 5 rpm (30 deg/sec) for a robot arm joint. 
 
 The planet carrier is constrained by cone-shaped 'ride' 
 protrusions in the middle of the stepped planets.
 
 Extra M3 bearings constrain the sun and planet carrier vertically. 

 As-built: on a 0.5 meter steel level arm, backdrive force is greater than 10 kgf (100N, 200 N-m).
 Motor driven torque is about 6-7kgf at 0.5m (120-140 N-m) before the top cover ring gear slips.
 (Tested with PLA+, printed at just 25% infill and 3 perimeters.)
 

 Assembly order:
    - Add tiny 683ZZ bearings to the ends of the planet gears
    - Thread planet gears to carrier with M3x30 screws
        - There are two planet timings, and two orientations. Planets across from each other should match.  Match numbers on top!
         (Could fairly easily double the reduction ratio using 4 different planet timings.)
    - Slide carrier into bottom frameB and make sure it spins easily.
    - Attach sun gear to planet carrier:
        - Rotate to each side and add four 623ZZ guide bearings between sun and carrier.
        - Line up the sun access holes with the guide bearings and secure with M3x30mm bolts.
        - The sun gear, frameB, carrier, and planets are now assembled, and stay together as a unit.

 
FIXME: near term
    - Design cordless drill manual drive slot, so you can time it for disassembly without using the motor.
    - Design 3D printed wire guide / encoder magnet holder parts (standardize on 7/8" wire hole in the steel?)

Possible future expansion:
    - Center the boundary between gearplanes, leaving more wire room on top above the carrier?
    - Consider a print-in-place design, for a lower labor assembly? 
 
 Dr. Orion Lawlor, lawlor@alaska.edu 2024 fall (Public Domain)
*/
include <AuroraSCAD/bearing.scad>;
include <AuroraSCAD/gear.scad>;
include <AuroraSCAD/screw.scad>;
include <AuroraSCAD/motor.scad>;
include <AuroraSCAD/bevel.scad>;

$fs=0.1; $fa=3;

inch = 25.4; // file units: mm

// Steel structural tube frame
framethick = 1.0*inch + 0.2; // some print clearance, for assembly
frameSteel = [4.5*inch,framethick,framethick];

// Central space for wiring
wireholeID = 22.0; // 0.75*inch; // diameter for wiring: 18mm is enough for a USB A plug
wireholeOD = 26.0; // hole through sun gear
wiretubeOD = wireholeOD-1.0; // non-spinning tube for wire protection. Also holds encoder magnet?
wireslope = 3.0; // taper outward smoothly



/*
 Main pivot bearing: secures primary metal part loads together.
 
 6815 bearing: rated static / dynamic load: 12.7 kN radial.
 "Thin-section deep groove ball bearings can support axial loads of between 10 and 30 percent of the bearing's static radial load rating"
 That'd be 1.3 - 3.8 kN axial. 
*/
mainbearing = bearing_6815;
mainbearing_clearance = 0.2; // printed-in clearance around bearing surface
bearingBC = [0,0,-bearingZ(mainbearing)]; // center of bearing bottom face
bearingTC = [0,0,0]; // center of bearing top face

// Halfway out is the handoff between the bearing halves
mainbearingR = (bearingIR(mainbearing)+bearingOR(mainbearing))/2;


/* 
Screws that mount plastic parts to the bearing.
*/
Bscrew = US10_24_pan_screw;

BTspaceZ = 3; // vertical space between gearplanes

// Z clearance for head of Bscrew
BscrewZ = 5;  // space between bearing and bottom frame
BscrewR = screw_diameter(Bscrew)/2+0.1; // screw centerline to edge of plastic
BscrewIR = bearingIR(mainbearing) - BscrewR; // centerline of screws inside bearing
BscrewOR = bearingOR(mainbearing) + BscrewR; // centerline of screws outside bearing

module screw_array(R,N=12,sinlimit=1.1)
{
    dA = 360/N;
    for (angle=[dA/2:dA:360-1]) 
        if (abs(sin(angle))<sinlimit)
            rotate([0,0,angle]) translate([R,0,0]) children();
}

// Inside screws hold the bearing inner ring to frameB
module BscrewIR_array() {
    translate(bearingTC) screw_array(BscrewIR,12,0.8) 
        screw_3D(Bscrew, thru=2);
}

// Outside screws hold the bearing outer ring to frameT
//   These get tapped into the plastic
module BscrewOR_array() {
    translate(bearingBC) screw_array(BscrewOR,16,0.7) rotate([180,0,0]) 
        screw_3D(Bscrew,thru=3,length=50);
}


// Bottom (B) input gearplane: sun driven by motor
gearZ = 10; // thickness of planetary gears
geartypeB = geartype_create(1.0,gearZ);
nplanets = 4;
gearplaneB = [geartypeB, 32, 12, nplanets];
gearBC = bearingBC + [0,0,0]; // bottom gearplance start

// Top (T) output gearplane
gearplaneT = gearplane_stepped(gearplaneB, +1/4);

gearplane_print(gearplaneB,"B");
gearplane_print(gearplaneT,"T");

gearTC = gearBC + [0,0,gearZ + BTspaceZ]; // top gearplane start

// Ride cylinder aligns each planet
rideOD = 2*gearplane_Oradius(gearplaneB); // planet orbit diameter
rideTD = gear_OD(gearplane_Pgear(gearplaneB)); // planet gear tip diameter
rideB = 3.0; // bevel thickness (maximum)
rideZ = BTspaceZ*0.5+2*rideB;
rideC = gearBC + [0,0,gearZ + BTspaceZ/2]; // centered between gearplanes

// Cuts into ring gears
module ride_space() {
    translate(rideC)
        bevelcylinder(d=rideOD+rideTD+0.5, h=rideZ, bevel=rideB, center=true);
}
// Added to planets
module ride_planet() {
    translate(rideC)
        bevelcylinder(d=rideTD, h=rideZ, bevel=rideB, center=true);
}

// Planet carrier axle screw: M3 x 30mm
Pscrew = M3_cap_screw;
PscrewZ = 30.5;

// Total height of planet gear stack
Pbase=6; // base of planet carrier, below gear
Ptop=5; // top of planet carrier, above gear
Ptopcap=2; // top of planet carrier below bolt caps
Pheight = PscrewZ - Pbase - Ptopcap; // height of full planetary gear stack


frame_B_thick=BscrewZ+2.5; // height of bottom ring gear plate, under bearing
frame_TZ = 22; // height of top ring gear and cover


carrier_bearing = bearing_6704;
carrier_bearingC=gearTC+[0,0,gearZ+Ptopcap];

carrier_baseC = gearBC + [0,0,-Pbase];

carrier_boltR = 4.0; // material around planet carrier thru bolts
carrier_clearR = gearplane_Oradius(gearplaneB)+carrier_boltR+0.7; // carrier spinning keep-out zone
  

geartypeMR = geartype_550; // motor output gear
motor_gearZ=9;
geartypeM = geartype_create(geartypeMR[0],motor_gearZ,geartypeMR[2]);

sundrive_gearZ = 12; // thickness of sun drive gear
sundrive_lips = 1; // 1: retain sundrive to carrier using lip bearings. 0: no bearings (easier to assemble, but more wear)
geartypeS = geartype_create(geartypeM[0],sundrive_gearZ,geartypeM[2]);

// Drives sun gear above
sundrive_gear = gear_create(geartypeS,71);
sundrive_gearR = gear_R(sundrive_gear);
sundrive_gearC = bearingBC - [0,0,frame_B_thick+0.5+sundrive_gearZ];


frameT_clearR = mainbearingR;
frameT_clearZ = 20;
frameT_bevel = frameT_clearZ/2-0.01; // heavy radius, for strength
frameB_clearR = gear_OR(sundrive_gear)+2.0; // frame cut to clear sundrive gear
frameB_outsideR = bearingIR(mainbearing)+3.0; // material outside main bearing

// Reduces motor to sundrive
reduce_gearZ = sundrive_gearZ;
reduce_ratio = 3.5; // motor to sungear reduction ratio

reducesun_gear = gear_create(geartypeS,12); // mates with sun on top
reducesun_gearR = gear_R(reducesun_gear);

reducemotor_gear = gear_create(geartypeM,gear_nteeth(reducesun_gear)*reduce_ratio); // motor drives the bottom
reducemotor_gearR = gear_R(reducemotor_gear);

reduce_gearR = gear_R(reducesun_gear) + gear_R(reducemotor_gear);

reduce_gearC = sundrive_gearC + [0,sundrive_gearR+reducesun_gearR,-1-motor_gearZ];

// Extended Z height for bearing holder
reduce_gear_fullZ = motor_gearZ + 1 + sundrive_gearZ + 4;

// Directly attached to motor shaft
motor_gear = gear_create(geartypeM,15); // metal gears, "32 pitch"
motor_gearR = gear_R(motor_gear);
motor_gearC = reduce_gearC + [-(reducemotor_gearR+motor_gearR),0,0];



retain_bearing = bearing_623; // larger M3 bearings retain sun gear and planet carrier
planet_bearing = bearing_683; // micro M3 bearing holds planets to M3 shaft

retainsunB_bearing = bearing_683; // holds sun gear to bottom frame
retainsunB_R = 22; // radius where those bearings spin
retainsunB_Z = sundrive_gearC[2]+2; // bottom of mounting screw cap



// Draw 2D version of all gears
module illustrate_gears_2D(animate=0) 
{
    color(0.7*[0,1,1]) translate(gearBC) gearplane_2D(gearplaneB);
    translate(gearTC) gearplane_2D(gearplaneT);
    translate(sundrive_gearC) gear_2D(sundrive_gear);
    translate(reduce_gearC) gear_2D(reducemotor_gear);
    translate(motor_gearC) gear_2D(motor_gear);
}

// Animated version of planetary gearset
module animate_gears_2D() {
    gearplane_2D(gearplaneB,aligned=1,animate=$t);

    translate([0,0,10]) color([0.2,0.3,0.7])
    gearplane_2D(gearplaneT,aligned=0,animate=$t,drawS=0);
}


// 3D illustration of geartrain
module illustrate_gears() 
{
    reduce_gear_whole();
    sundrive_gear_whole();
    motor_gear_whole();

    planet_gears();
    planet_carrier();

    //sunidler_gear_whole();
    
    //#translate(carrier_bearingC) bearing3D(carrier_bearing);
}




// Spaces for M3 bolt and planet-size bearings inside gear
module bearing_spaces(loZ,hiZ)
{
    extraD=0.1; // let bearings fit in without hammer
    extraZ=0.2; // upward-facing overhangs need this much extra space
    start=-2; // Z coordinate of start point (avoid roundoff and bevel problems)
    cylinder(d=4,h=200,center=true); // clear center axle hole
    
    for (side=[0,1]) {
        z=side? (hiZ) : (loZ+extraZ);
        translate([0,0,z])
            scale([1,1,-1])
                translate([0,0,start])
                bevelcylinder(d=bearingOD(planet_bearing)+extraD,h=bearingZ(planet_bearing)-start,bevel=0.2);
    }
}


// Reduction gear
module reduce_gear_whole()
{
    L = reducemotor_gear;
    H = reducesun_gear;
    translate(reduce_gearC) 
    difference() {
        union() {
            gear_3D(L);
            gear_3D(H, height=reduce_gear_fullZ);
            // Bevel up to avoid stress concentration
            translate([0,0,motor_gearZ-0.01])
                cylinder(d1=gear_OD(H),d2=gear_ID(H),h=1);
            // Bevel down around bearing
            translate([0,0,reduce_gear_fullZ]) scale([1,1,-1])
                bevelcylinder(d=gear_OD(H),h=4,bevel=1);
        }
        bearing_spaces(0,reduce_gear_fullZ);
    }
}

// Cordless drill manual operation (e.g., for repairing motor)
module reduce_cordless_stick()
{
    gOD = gear_OD(reducesun_gear);
    OD=0.375*inch; // narrower to fit in smaller drill chuck 
    stickZ=50; // length of stick needs to extend past frame and circuits
    bevel=2; // long stick part, goes in drill chuck
    
    difference() {
        union() {
            tooth=sundrive_gearZ+6; // length of toothed portion
            gear_3D(reducesun_gear, height=tooth+bevel);
            
            tipZ=1.0; // front cylinder Z thickness
            bevelcylinder(d=gOD-1.5,h=tipZ*2+0.3,bevel=tipZ); // round off tip
            
            // long stick portion
            translate([0,0,tooth-bevel])
                bevelcylinder(d=OD,h=stickZ,bevel=bevel);
        }
        
        // make it a tube, lighter and stronger
        cylinder(d=OD*0.5,h=200,center=true);
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
    top = Pheight+Ptop-3;
    
    // Top-down bolts through planet gears
    gearplane_planets(gearplaneB) 
        translate([0,0,top])
            children();

    rotate([0,0,carrier_retainA])
    gearplane_planets(gearplaneB) 
        if (sundrive_lips) {
            // Bottom-up bolts through sun retain bearings
                translate([0,0,carrier_retainZ]) 
                    scale([1,1,-1]) // facing up
                        children();
        }
        else { // Top-down bolts for steel stiffened carrier
            translate([0,0,top])
                children();
        }
}

// Cap of bolts used to retain planet carrier
module planet_carrier_boltcap(capH=3)
{
    bevelcylinder(d=screw_head_diameter(Pscrew)+0.1,h=capH,bevel=0.3);
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
                carrierID = 1.0+wireS[0]*(wireholeID+2*wirewall);
                circle(d=carrierID);
                // inside wall space, tuned to not hit the wiring funnel
                //offset(r=-9.5) planet_carrier2D();
            }
            //planet_carrier_bolts() circle(r=carrier_boltR);
        }
        
        // Clearance for teeth of the larger sun gear
        translate(carrier_baseC+[0,0,-0.01])
        {
            r = gear_OR(gearplane_Sgear(gearplaneB))+0.5*tipclear;
            h = gearTC[2] - carrier_baseC[2]+tipclear;
            b = 3; 
            bevelcylinder(d=2*r,h=h,bevel=b);
            // wiring clearance on top
            translate([0,0,Pbase+Pheight+Ptop+1])
            {
                bevelcylinder(d=2*r,h=2*b+0.1,bevel=b,center=true);
                
            }
        }
        
        translate(carrier_bearingC) hull() bearing3D(carrier_bearing);
        
        // space for planet gears in middle
        holeZ = Pheight+2*tipclear;
        translate(gearBC) 
        gearplane_planets(gearplaneB) 
        planet_space(tipclear,holeZ);
        
        // Text labels for planets
        translate(carrier_baseC + [0,0,Pbase+Pheight+Ptop-0.5]) rotate([0,0,15]) planet_numbers();
        
        // M3 thru bolts
        translate(gearBC)
        planet_carrier_bolts() 
            planet_carrier_bolt();
        
        // Space for bearings on bottom, to retain sundrive gear
        if (sundrive_lips) 
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
        if (sundrive_lips) 
        translate(gearBC + [0,0,carrier_retainZ-carrier_retainZT])
        {
            bevelcylinder(d=2*(carrier_retainOR+lipclear),h=carrier_retainOH+lipclear,bevel=carrier_retainB);
        }
        
        // Space around sun gear
        if (!sundrive_lips)
        {
            ID = gear_ID(gearplane_Sgear(gearplaneB));
            translate(gearBC+[0,0,1]) scale([1,1,-1])
                cylinder(d1=ID, d2=ID+12,h=gearBC[2]-sundrive_gearC[2]-sundrive_gearZ);
        }
    }
}

// Clearance around planet carrier
module planet_carrier_clear() {
    translate(carrier_baseC) cylinder(r=carrier_clearR,h=Pbase+Pheight+Ptop+1.0);
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
module planet_gears(clearance=0.05)
{
    extratooth = 3; // extend gear teeth in Z this far, for strength
    h = gearZ+extratooth;
    difference() {
        union() {
            gearplane_planets(gearplaneB) {
                translate(gearBC) 
                    gear_3D(gearplane_Pgear(gearplaneB),height=h,clearance=clearance); 
                ride_planet();
            }
            gearplane_planets(gearplaneT,0) {
                translate(gearTC-[0,0,extratooth]) 
                    gear_3D(gearplane_Pgear(gearplaneT),height=h,clearance=clearance);
            }
        }
        gearplane_planets(gearplaneB)
            planet_gear_minus();
        
        /*
        // Text labels on top bearing face (almost illegible)
        translate(gearBC+[0,0,Pheight - bearingZ(planet_bearing)]) 
            planet_numbers();
        */
        
        translate(rideC) 
            planet_number_grooves();
        
    }
}

// Carve grooves into the planets, so you can get them in the right slots.
//   Carves up from Z=0 plane, around each planet's central ride cylinder
module planet_number_grooves() {
    grooveZ=1.0;
    for (i=[1:nplanets]) rotate([0,0,i*360/nplanets])
        translate([0,gearplane_Oradius(gearplaneB),+0.01])
        for (copy=[0,180]) rotate([0,0,copy])
        {
            for (pip=[1:i]) rotate([0,0,pip*20])
                translate([rideTD/2-1.1,0,-grooveZ/2])
                    rotate([0,0,-45])
                        bevelcube([3,3,grooveZ],bevel=grooveZ/3);
        }
}

// Carve numbers into the planets, so you can get them in the right slots.
//   Carves up from Z=0 plane, around each planet
module planet_numbers() {
    for (i=[1:nplanets]) rotate([0,0,i*360/nplanets])
        translate([0,gearplane_Oradius(gearplaneB),+0.01])
            linear_extrude(height=1,convexity=4)
            {
                text(str(i), size=6, halign="center", valign="center");
            }
}

// Channel that carrier retain bearings run in, used by sun gear and bottom ring
//    (R,Z) is the center of the bottom of the retaining bolt cap.
// Leaves space for M3 retaining bolts
module retain_bearing_channel(enlargeR=0, enlargeH=0.75, accessholes=0, 
    bearing=retain_bearing,
    R=gearplane_Oradius(gearplaneB), Z=gearBC[2]+carrier_retainZ, flip=0)
{        
    capZ=3.2; // height of M3 retaining bolt cap (with moving part clearance)
    capD=6.3;
    capR=1.0; // extra rounding on cap slot (stress riser)
    retain_round=0.5;
    rotate_extrude()
    offset(r=+retain_round) offset(r=-retain_round)
    translate([R,Z,0])
    rotate([0,0,flip*180])
    {
        // Space for the bearings proper
        BR = bearingOR(bearing)+enlargeR;
        BH = bearingZ(bearing)+enlargeH;
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
                cylinder(d=capD+0.8,h=50,center=true);
}        

// Ring gears are printed at zero clearance, any needed clearance happens in the planets (easier to re-print if they wear)
ring_gear_clearance=0.0;

// 2D outline of bottom frame segment
module frame_B_2D()
{
        intersection() {
            circle(r=mainbearingR);
            hull() {
                circle(r=frameB_outsideR); // plate under ring
                square([2*mainbearingR,framethick],center=true); // support frame
            }
        }
}

// Makes children at possible motor orientations
module motor_symmetry() {
    for (flipY=[-1,+1]) for (flipX=[-1,+1])
        scale([flipX,flipY,1])
            children();
}

// Bottom ring gear: held by Bscrews
module frame_B()
{
    difference() {
        union() {
            // Material inside bearing
            translate(gearBC) cylinder(d=bearingID(mainbearing),h=bearingZ(mainbearing));
            // Material outside
            taper=5;
            translate(gearBC) scale([1,1,-1])
            linear_extrude(height=frame_B_thick) frame_B_2D();
        }
        
        // Cut in the gear teeth
        Zspace=0.5;
        translate(gearBC+[0,0,-Zspace]) 
            gear_3D(gearplane_Rgear(gearplaneB),height=gearZ+Zspace+BTspaceZ,clearance=ring_gear_clearance); 
        
        // Bevel top of gear teeth with ride feature
        translate([0,0,-0.5*Zspace]) ride_space();
        
        BscrewIR_array();
        planet_carrier_clear();
        frame_steel(0);
        hull() retain_bearing_channel(enlargeR=0.2); // a little clearance
        
        // Places motor could be mounted 
        //   (motor plate is welded to steel frame, so can weld in several orientations)
        motor_symmetry()
        {
            // space around motor shaft
            translate(motor_gearC) cylinder(d=8,h=25);
            
            // Space for and shaft of reduce gear and/or manual override
            translate(reduce_gearC) {
                translate([0,0,8]) scale([1,1,-1]) planet_carrier_bolt(extratap=15);
                bevelcylinder(d=gear_OD(reducesun_gear)+1.5,h=reduce_gear_fullZ+1,bevel=1);
            }
        }
    }
}



// Wire thru hole in frame_T: may have a second sleeve inserted
wireTC = bearingTC + [0,0,frameT_clearZ];
wireS = [1.2,1,-1]; // <- scaled wider for extra room, and key for rotation
wirewall=1.2;

// Sloped entrance to wiring channel of this diameter.
module wire_funnel(OD)
{
    // Slope out end of wiring channel
    slope=2;
    for (ds=[0:0.5:slope])
        cylinder(d2=OD, d1=OD+2*slope-2*ds, h=slope+ds);
}


// Top ring gear and frame: held by Bscrews
module frame_T(limit_frameOD=500)
{
    start=-bearingZ(mainbearing)+0.01; // Z coordinate where this object begins (amount of bearing to cover)
    difference() {
        union() {
            translate(bearingTC+[0,0,start])
            {
                // Tapered dust cover over the bearing outside surface
                hull() {
                    cylinder(d=3+bearingOD(mainbearing),h=2-start);
                    translate([0,0,2+BTspaceZ-start])
                        cylinder(d=-5+bearingID(mainbearing),h=5);
                }
                
                linear_extrude(height=frame_TZ-start)
                {
                    hull() offset(r=3)
                        projection(cut=true)
                            BscrewOR_array();
                            
                }
            }
            
            children();
        }
        
        // Inside ring gear teeth
        gstart=-3;
        Zspace=0.5;
        translate(gearTC+[0,0,gstart]) gear_3D(gearplane_Rgear(gearplaneT), height=gearZ-gstart+Zspace, clearance=ring_gear_clearance);
        translate([0,0,0.5*Zspace]) ride_space();

        // Screw holes (tapped into plastic)
        BscrewOR_array();        
        
        // Space for the planet carrier and bearing
        difference() {
            planet_carrier_clear();
            
            // Put back wire thru hole wall
            translate(wireTC+[0,0,-1]) scale(wireS)
                wire_funnel(wireholeID+2*wirewall);
        }
        //translate(carrier_bearingC) hull() bearing3D(carrier_bearing);

        intersection() {
            frame_steel();
            cylinder(d=limit_frameOD,h=100,center=true);
        }
        
        translate(bearingBC) 
            cylinder(d=bearingOD(mainbearing)+mainbearing_clearance,h=bearingZ(mainbearing)+mainbearing_clearance);
        
        translate(bearingTC+[0,0,-0.01+start]) {
            // Space above moving bearing surface
            cylinder(r=bearingOD(mainbearing)/2-2,h=1-start);
            
            // Space above inside bolt heads
            cylinder(r=BscrewIR+screw_head_diameter(Bscrew)/2+1,h=BTspaceZ-start);
        }
        
        // Wiring thru hole
        scale(wireS) cylinder(d=wireholeID,h=100,center=true);
        translate(wireTC) scale(wireS)
            wire_funnel(wireholeID);
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
            // Bottom gear teeth
            translate(sundrive_gearC) gear_3D(D);
            
            translate(DT) {
                ID = gear_ID(S);
                // Straight wall all the way up to top gears
                cylinder(d=ID,h=gearTC[2]-DT[2]-0.5);
                
                // Tapered transition up to sun gear teeth
                cylinder(d1=12+ID,d2=ID,h=gearBC[2]-DT[2]);
                //for (t=[0:3]) cylinder(d1=6-t+ID,d2=ID,h=2+t);
            }
            
            // Top sun gear teeth
            for (zcopy=[0,0.5,1.1]) // extend gear teeth down to DT
                translate(gearBC*(1.0-zcopy)+DT*zcopy) 
                    rotate([0,0,360/gear_nteeth(S)*0.5]) // timed to planets
                    gear_3D(S,height=gearZ+BTspaceZ-1.0, clearance=0.0);
            
            // Lips that cling to retain bearings
            if (sundrive_lips) 
            translate(gearBC + [0,0,carrier_retainZ-carrier_retainZT])
            {
                bevelcylinder(d=2*(carrier_retainOR),h=carrier_retainOH,bevel=carrier_retainB);
            }
        }
        
        // Retain bearings, holding sun to carrier
        if (sundrive_lips) retain_bearing_channel(accessholes=1);
        
        // Space for ride protrusions on planets
        translate(rideC) rotate_extrude() {
            Zspace=0.5; // clearance around moving parts
            translate([rideOD/2,0,0])
                offset(r=+Zspace)
                    bevelsquare([rideTD,rideZ],bevel=rideB,center=true);
        }
        
        /*
        // Retain bearings holding sun to bottom frame (not needed)
        if (sundrive_lips) for (inset=[0,1.0])
        retain_bearing_channel(accessholes=0, bearing=retainsunB_bearing, R=retainsunB_R-inset, Z=retainsunB_Z, flip=1);
        */
        
        
        // Wiring thru hole
        cylinder(d=wireholeOD,h=100,center=true);
        translate(sundrive_gearC) wire_funnel(wireholeOD);
    }
}




stepped_ratio = gearplane_stepped_ratio(gearplaneB,gearplaneT);
echo("Sun to rings reduction:",stepped_ratio);
echo("Total reduction ratio: ",reduce_ratio * sundrive_gearR / motor_gearR * stepped_ratio);
echo("Gear pitches: ",geartype_Dpitch(geartypeM),geartype_Dpitch(geartypeB),geartype_Dpitch(gearplane_geartype(gearplaneT)));
echo("Wiring channel: ",wireholeOD," in sun gear: ",gear_ID(gearplane_Sgear(gearplaneT)));
echo("Motor gear center: ",motor_gearC);
echo("FrameT clear Z: ",carrier_bearingC[2]-bearingTC[2] + bearingZ(carrier_bearing));

frameZ = 1.0*inch; // thickness of steel frame bar
frameBevel = 0.7;
frameBC = bearingBC + [0,0,-BscrewZ-0.5*frameZ]; // Center of bottom frame bar
frameTC = bearingTC + [0,0,0.5*frameZ]; // Center of top frame bar


/*
 Drive motor
*/
motortype = motortype_3674; // motortype_NEMA17;
motorplateZ = 1.6; // steel plate, welded to frame
motorC = [motor_gearC[0],motor_gearC[1],frameBC[2]-frameZ/2-motorplateZ];// center of motor mount
motorA = [0,0,360/6/2]; // angle of rotation (mostly to align the mount screws)

// Clearance cut on top frame steel
module frameT_clearance(extraZ=0) {
    // Top tube clearance cut
    translate(bearingTC+[0,0,-0.01-extraZ]) {
        bevelcylinder(d=2*frameT_clearR,h=frameT_clearZ+extraZ,bevel=frameT_bevel);
        cylinder(r=frameT_clearR,h=frameT_clearZ/2+extraZ); // bottom, no bevel
    }
}
// Clearance cut on bottom frame steel
module frameB_clearance() {
    // Bottom tube clearance cut 
    translate(bearingBC+[0,0,+0.01]) scale([1,1,-1]) 
        bevelcylinder(d=2*frameB_clearR,h=bearingBC[2]-sundrive_gearC[2]+3,bevel=3);
}

// Outline of 1x1 inch steel tubing, minus clearance cuts
module frame_steel(clearance=1) {
    difference() {
        for (Z=[frameBC,frameTC]) translate(Z) 
            bevelcube(bevel=frameBevel,frameSteel,center=true); // steel frame
        
        if (clearance) {
            frameT_clearance();
            frameB_clearance();
        }
    }
}

// Motor and reduction gear mounting plate, 2D version.
//   Real version is cut from metal and welded to the frame.
//   (Printed could work, but can soften and sag from motor heat.)
module motorplate2D()
{
    OD = motor_diameter(motortype);
    dx=50; // full width of starting plate
    dy=45; // sets overlap with frame
    
    difference() {            
        translate([motorC[0]-OD/2, motorC[1]+OD/2-dy, motorC[2]])
            square([dx,dy]);
        
        translate(motorC) rotate(motorA) motor_faceholes_2D(motortype);
        translate(reduce_gearC) circle(d=3.2); // space for M3 on reduce gear
    }
}

module motorplate3D() 
{
    translate([0,0,motorC[2]]) linear_extrude(height=motorplateZ,convexity=4) motorplate2D();
}

// Drill jig for manufacturing motor plate
module motorplate_jig()
{
    linear_extrude(height=motorplateZ)
        motorplate2D();
    
    wall=3;
    linear_extrude(height=2*motorplateZ)
        difference() {
            offset(r=+wall) hull() motorplate2D();
            hull() motorplate2D();
        }
}

module whole_motor3D()
{
    translate(motorC) rotate(motorA) motor_3D(motortype);
}

// Tapped M3 holes in steel parts, to mount cover plates / endplugs (in the future)
module cover_mount_holes() 
{
    translate(frameBC) {
        for (x=[-1,+1]) translate([x*40,0])
            rotate([90,0,0])
                cylinder(d=2.5,h=50,center=true);
    }
}


// Jig for manufacturing metal frame bottom stick
module frame_B_jig()
{
    rotate([0,90,0])
    {
        clearance=0.3; 
        c3 = [0,2,2]*clearance; // space on inside of frame
        
        wall = 2.4;
        w3 = [-2,2,2]*wall; // enlarge steel base frame
        difference() {
            // Outside
            translate(frameBC) bevelcube(frameSteel+w3,bevel=frameBevel,center=true);
            
            // Tube hole
            translate(frameBC) bevelcube(frameSteel+c3,bevel=frameBevel,center=true);
            
            frameB_clearance();
            BscrewIR_array();
            motorplate3D();
            cover_mount_holes();
            
            // wiring space (drill bit alignment hole)
            cylinder(d=2*3.2,h=100,center=true);

            // chop end flat
            translate([95,0,0]) cube([100,100,100],center=true);
        }
    }
}

// Jig for manufacturing metal frame top stick
module frame_T_jig()
{
    rotate([0,90,0])
    {
        clearance=0.3; 
        c3 = [0,2,2]*clearance; // space on inside of frame
        
        wall = 2.4;
        w3 = [-2,2,2]*wall; // enlarge steel base frame
        difference() {
            // Outside
            translate(frameTC) bevelcube(frameSteel+w3,bevel=frameBevel,center=true);
            
            // Tube hole
            translate(frameTC) bevelcube(frameSteel+c3,bevel=frameBevel,center=true);
            
            frameT_clearance(5);
            BscrewOR_array();
            motorplate3D();
            cover_mount_holes();
            
            // wiring space (drill bit alignment hole)
            cylinder(d=2*3.2,h=100,center=true);

            // chop end flat
            translate([104,0,0]) cube([100,100,100],center=true);
        }
    }
}



module illustrate_covers() {
    frame_T();
    frame_B();
}

module illustrate_frame() {
    #translate(bearingBC) bearing3D(mainbearing);

    #frame_steel();

    #motorplate3D();
    #whole_motor3D();

    #BscrewIR_array();
    #BscrewOR_array();
}

// All large gears
module printable_gears(withmotor=0) 
{
    translate(-carrier_baseC) rotate([0,0,45]) planet_carrier();
    translate(-sundrive_gearC+[1.4*mainbearingR,0,0]) sundrive_gear_whole();
    translate(-reduce_gearC+[0,-1.1*mainbearingR,0]) reduce_gear_whole();
    if (withmotor) translate(-motor_gearC) motor_gear_whole();
    
    //translate(-gearTC) sunidler_gear_whole(); // no longer used
}

// Small planet gears (often needs a brim to stay stuck down)
module printable_planets() 
{
    translate(-(gearBC+[1.2*mainbearingR,0,0])) rotate([0,0,45]) planet_gears();
}

// Section in XY plane
module cutaway_gearbox()
{
    difference() { 
        union() {
            illustrate_gears();
            frame_B();
            frame_T();
        }
        for (cutangle=[0,45]) color([1,0,0]) rotate([0,0,cutangle])
        translate([0,0,-100]) cube([200,200,200]);
    }
    reduce_gear_whole();
}

// Section along Z
module cutaway_gearZ()
{
    intersection() {
        union() {
            frame_T();
            frame_B();
            //#planet_carrier();
            sundrive_gear_whole();
            planet_gears();
        }
        translate(gearTC) color([1,0,0]) 
            cube([200,200,10],center=true);
    }
}

// Illustrations:
//cutaway_gearbox();
//cutaway_gearZ();

//illustrate_frame();
//illustrate_covers();
//illustrate_gears_2D();
//animate_gears_2D();

//#BscrewIR_array();

//frame_T();
//frame_B();
//planet_carrier();
//sundrive_gear_whole();
//reduce_gear_whole();

// Printable batches: "Bottomset":
//translate([0,-70,0]) printable_gears();
//translate(-(gearBC-[0,0,frame_B_thick])) frame_B();

//printable_planets();

//rotate([180,0,0]) translate([0,0,-frame_TZ]) frame_T();
reduce_cordless_stick();


// Spares:
//translate(-carrier_baseC) planet_carrier();


// Computer Aided Manufacturing (CAM) jigs:
//motorplate2D(); // DXF for plasma cutting
//motorplate_jig(); // 3D printed jig
//frame_B_jig(); // bottom steel cut/mark jig
//frame_T_jig(); // top steel cut/mark jig


