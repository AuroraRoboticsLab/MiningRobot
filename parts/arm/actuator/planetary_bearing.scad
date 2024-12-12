/*
 Robot joint designed around a planetary gearset that hangs on a single pivot bearing.

 Screws hold the printed parts to the bearing and a steel frame. 
 
 Current gear reduction is about 135:1, which actually seems to not nearly be slow enough!
 To get from like 10K rpm (for a 12V 1kV brushless motor) down to like 10 rpm (60 deg/sec)
 is 1000:1 reduction. 
 
 The planet carrier also needs to be tightly constrained by more bearings. 
 
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
wirehole = 18.0; // 0.75*inch; // diameter for wiring


/*
 Main pivot bearing
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

// Z clearance for head of Bscrew
BscrewZ = 3;  // space between gearplanes
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
gearplaneB = [geartypeB, 24, 16, 4];
gearBC = bearingBC + [0,0,0]; // bottom gearplance start

// Top (T) output gearplane
gearplaneT = gearplane_stepped(gearplaneB, -1);
gearTC = gearBC + [0,0,gearZ + BscrewZ]; // top gearplane end


// Planet carrier axle screw: M3 x 30mm
Pscrew = M3_cap_screw;
PscrewZ = 30.5;

// Total height of planet gear stack
Pbase=6; // base of planet carrier, below gear
Ptop=5; // top of planet carrier, above gear
Ptopcap=2; // top of planet carrier below bolt caps
Pheight = PscrewZ - Pbase - Ptopcap; // height of full planetary gear stack


sundrive_bearing = bearing_6704;
carrier_bearing = bearing_6704;
carrier_bearingC=gearTC+[0,0,gearZ+Ptopcap];

carrier_baseC = gearBC + [0,0,-Pbase];

carrier_boltR = 4; // material around planet carrier thru bolts
carrier_clearR = gearplane_Oradius(gearplaneB)+carrier_boltR+0.5; // carrier spinning keep-out zone

planet_bearing = bearing_683; // micro M3 bearing 

geartypeM = geartype_550; // motor output gear
// Drives sun gear above
sundrive_gear = gear_create(geartypeM,47);
sundrive_gearR = gear_R(sundrive_gear);
sundrive_gearC = bearingBC - [0,0,18];

// Directly attached to motor shaft
motor_gear = gear_create(geartypeM,15); // metal gears, "32 pitch"
motor_gearR = gear_R(motor_gear);
motor_gearC = sundrive_gearC + [0,sundrive_gearR+motor_gearR,0];


ring_gear_B_thick=5; // height of bottom ring gear plate, under bearing
ring_gear_TZ = 22; // height of top ring gear and cover

// Makes children at center of planet carrier thru bolts
module planet_carrier_bolts() {
    for (angle=[0,360/gearplane_Pcount(gearplaneB)/2]) rotate([0,0,angle])
        gearplane_planets(gearplaneB) 
            children();
}

// 2D outline of outside of planet carrier
module planet_carrier2D() {
    hull() 
    planet_carrier_bolts() circle(r=carrier_boltR);
}

// Frame that holds the M3 screws that secure the planet gears, and keeps them from tilting
module planet_carrier() {
    tipclear=1.0; // clearance around gear tips
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
                offset(r=-11) planet_carrier2D(); // inside clearance
            }
            //planet_carrier_bolts() circle(r=carrier_boltR);
        }
        
        // Clearance for tips of the larger sun gear
        #translate(carrier_baseC+[0,0,-0.01])
            cylinder(r=gear_OR(gearplane_Sgear(gearplaneB))+tipclear,h=Pbase+Pheight+tipclear);
        
        translate(carrier_bearingC) hull() bearing3D(carrier_bearing);
        
        // space for planet gears in middle
        holeZ = Pheight+2*tipclear;
        translate(gearBC) 
        gearplane_planets(gearplaneB) 
        difference()
        {
            translate([0,0,-tipclear])
            bevelcylinder(d=gear_OD(gearplane_Pgear(gearplaneT))+2*tipclear,h=holeZ,bevel=1.2*tipclear);
            
            padZ=tipclear*0.5; // little pads that bearings run on
            for (z=[-tipclear-0.01,holeZ-tipclear-padZ]) translate([0,0,z])
                cylinder(d=5,h=padZ+0.01);
        }
        
        // M3 thru bolts
        translate(gearBC)
        planet_carrier_bolts() 
        { 
            // Tap down into plastic on bottom
            translate([0,0,-Pbase-0.01]) cylinder(d=screw_tap_diameter(Pscrew),h=Pbase+1);
            // Axle hole on top
            translate([0,0,0.01]) cylinder(d=screw_diameter(Pscrew),h=Ptop+Pheight+1);
            // Socket cap space on top
            translate([0,0,Pheight+Ptopcap]) cylinder(d=screw_head_diameter(Pscrew)+0.3,h=Ptop+1);
        }
    }
}

// Clearance around planet carrier
module planet_carrier_clear() {
    translate(carrier_baseC) cylinder(r=carrier_clearR,h=Pbase+Pheight+Ptop+0.5);
}

// One planet gear
module planet_gear() 
{
    B = gearplane_Pgear(gearplaneB);
    T = gearplane_Pgear(gearplaneT);
    
    difference() {
        union() {
            translate(gearBC) {
                gear_3D(B,height=gearZ+5); // extend teeth up to next gear
                Z = BscrewZ/2; // transition height
                translate([0,0,gearZ+BscrewZ-Z]) { // transition up to next gear
                    cylinder(d1=gear_ID(B),d2=gear_ID(T),h=Z+0.02);
                }
            }
            translate(gearTC) gear_3D(T);
        }
        
        cylinder(d=4,h=200,center=true); // clear center axle hole
        
        // Top and bottom bearings on planet axles
        loZ = gearBC[2];
        hiZ = gearTC[2]+gearZ;
        
        // Central clearance hole, to push solid profiles closer to shear zone
        dZ = 0.65*(hiZ-loZ); // Z diameter of clearance sphere
        gD = gear_ID(B)*0.65; // XY diameter of hole
        translate([0,0,(loZ+hiZ)/2]) scale([gD/dZ,gD/dZ,1.0]) sphere(d=dZ);
        
        for (z=[loZ-0.1, hiZ+0.1-bearingZ(planet_bearing)])
            translate([0,0,z])
                bearing3D(planet_bearing,hole=0);
    }
}

// Planet gear set
module planet_gears()
{
    gearplane_planets(gearplaneB) {
        planet_gear();
    }
    
}


/*
// Draw all gears
translate(gearBC) gearplane_2D(gearplaneB);
translate(gearTC) gearplane_2D(gearplaneT);
*/

// Bottom ring gear: held by Bscrews
module ring_gear_B()
{
    difference() {
        union() {
            translate(gearBC) cylinder(d=bearingID(mainbearing),h=bearingZ(mainbearing));
            translate(gearBC) scale([1,1,-1])
                cylinder(r=mainbearingR-1,h=ring_gear_B_thick); // plate under ring
        }
        translate(gearBC) ring_gear_cut(gearplane_Rgear(gearplaneB));
        BscrewIR_array();
        planet_carrier_clear();
        frame_steel();
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
                    translate([0,0,2+BscrewZ-start])
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
            cylinder(r=BscrewIR+screw_head_diameter(Bscrew)/2+1,h=BscrewZ-start);
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
            translate(sundrive_gearC) gear_3D(D) {
                translate([0,0,-0.1]) bearing3D(sundrive_bearing,hole=0);
            }
            translate(DT) {
                ID = gear_ID(S);
                // Straight wall all the way up to top gears
                cylinder(d=ID,h=gearTC[2]-DT[2]-0.5);
                // Tapered transition
                for (t=[0:3]) cylinder(d1=6-t+ID,d2=ID,h=2+t);
            }
            for (zcopy=[0,0.5,1.1]) // extend gear teeth down to DT
                translate(gearBC*(1.0-zcopy)+DT*zcopy) gear_3D(S,height=gearZ+BscrewZ-1.0);
        }
        
        // Wiring thru hole
        cylinder(d=wirehole,h=100,center=true);
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
        cylinder(d=wirehole,h=50,center=true);
    }
}

module illustrate_gears() 
{
    sundrive_gear_whole();
    motor_gear_whole();


    planet_gears();
    planet_carrier();

    ring_gear_B();

    sunidler_gear_whole();

    ring_gear_T();

    
    #translate(carrier_bearingC) bearing3D(carrier_bearing);
}





echo("Gear pitches: ",geartype_Dpitch(geartypeM),geartype_Dpitch(geartypeB),geartype_Dpitch(gearplane_geartype(gearplaneT)));
echo("Wiring space: ",wirehole);
echo("Reduction ratio: ",sundrive_gearR / motor_gearR * gearplane_stepped_ratio(gearplaneB,gearplaneT));


frameZ = 1.0*inch; // thickness of steel frame bar
frameBC = bearingBC + [0,0,-BscrewZ-0.5*frameZ]; // Center of bottom frame bar
frameTC = bearingTC + [0,0,0.5*frameZ]; // Center of top frame bar

/*
 Drive motor
*/
motortype = motortype_NEMA17;
motorC = frameBC + [0,0,-frameZ/2] + [0,motor_gearC[1],0]; // center of motor mount

// Outline of 1x1 inch steel tubing, minus clearance cuts
module frame_steel() {
    difference() {
        for (Z=[frameBC,frameTC]) translate(Z) cube([5,1,1]*inch,center=true); // steel frame
        
        // Top tube clearance cut
        translate(bearingTC+[0,0,-0.01]) cylinder(r=mainbearingR,h=carrier_bearingC[2]-bearingTC[2] + bearingZ(carrier_bearing));
        
        // Bottom tube clearance cut
        translate(bearingBC+[0,0,+0.01]) scale([1,1,-1]) cylinder(r=carrier_clearR,h=bearingBC[2]-sundrive_gearC[2]);
        
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
    translate(-gearTC) sunidler_gear_whole();
    translate(-sundrive_gearC+[1.0*mainbearingR,0,0]) sundrive_gear_whole();
    translate(-motor_gearC) motor_gear_whole();
}



//illustrate_frame();
//illustrate_covers();

if (0) difference() {
    illustrate_gears();
    translate([0,0,-100]) cube([200,200,200]);
}

//translate(-carrier_baseC) planet_carrier();
rotate([180,0,0]) planet_gear();
//translate([0,-65,0]) printable_gears();
//translate(-(gearBC-[0,0,ring_gear_B_thick])) ring_gear_B();
//rotate([180,0,0]) translate([0,0,-ring_gear_TZ]) ring_gear_T();
