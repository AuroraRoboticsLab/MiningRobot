/*
 Robot joint designed around a planetary gearset that hangs on a bearing.

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
 6813 bearing: rated static / dynamic load: 12.7 kN radial.
 "Thin-section deep groove ball bearings can support axial loads of between 10 and 30 percent of the bearing's static radial load rating"
 That'd be 1.3 - 3.8 kN axial. 
*/
mainbearing = bearing_6813;
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

module screw_array(R,N=12)
{
    dA = 360/N;
    for (angle=[dA/2:dA:360-1]) rotate([0,0,angle]) translate([R,0,0]) children();
}

module BscrewIR_array() {
    translate(bearingTC) screw_array(BscrewIR,10) screw_3D(Bscrew);
}
module BscrewOR_array() {
    translate(bearingBC) screw_array(BscrewOR,16) rotate([180,0,0]) screw_3D(Bscrew);
}


// Bottom (B) input gearplane: sun driven by motor
gearZ = 10; // thickness of planetary gears
geartypeB = geartype_create(1.0,gearZ);
gearplaneB = [geartypeB, 24, 12, 4];
gearBC = bearingBC + [0,0,0]; // bottom gearplance start

// Top (T) output gearplane
gearplaneT = gearplane_stepped(gearplaneB, -1);
gearTC = gearBC + [0,0,gearZ + BscrewZ]; // top gearplane end


// Planet carrier axle screw: M3 x 30mm
Pscrew = M3_cap_screw;
PscrewZ = 30.5;

// Total height of planet gear stack
Pbase=5; // base of planet carrier, below gear
Ptop=5; // top of planet carrier, above gear
Ptopcap=2; // top of planet carrier below bolt caps
Pheight = PscrewZ - Pbase - Ptopcap; // height of full planetary gear stack

sundrive_bearing = bearing_6704;
carrier_bearing = bearing_6704;
carrier_bearingC=gearTC+[0,0,gearZ+Ptopcap];

carrier_baseC = gearBC + [0,0,-Pbase];

carrier_boltR = 4; // material around planet carrier thru bolts
carrier_clearR = gearplane_Oradius(gearplaneB)+carrier_boltR+0.5; // carrier spinning keep-out zone


// 2D outline of outside of planet carrier
module planet_carrier2D() {
    hull() 
    gearplane_planets(gearplaneB) circle(r=carrier_boltR);
}

// Frame that holds the M3 screws that secure the planet gears, and keeps them from tilting
module planet_carrier() {
    tipclear=0.5; // clearance around gear tips
    round=2;
    difference() {
        translate(carrier_baseC)
        linear_extrude(height=Pbase+Pheight+Ptop,convexity=6)
        offset(r=-round) offset(r=+round)
        union() {
            // Walls of carrier
            difference() {
                planet_carrier2D();
                offset(r=-4) planet_carrier2D();
                // Clearance for tips of the larger sun gear
                circle(r=gear_OR(gearplane_Sgear(gearplaneB))+tipclear);
            }
            gearplane_planets(gearplaneB)  circle(r=carrier_boltR);
        }
        
        translate(carrier_bearingC) bearing3D(carrier_bearing);
        
        translate(gearBC) 
        gearplane_planets(gearplaneB) 
        { 
            // Tap down into plastic on bottom
            translate([0,0,-Pbase-0.01]) cylinder(d=screw_tap_diameter(Pscrew),h=Pbase+1);
            // Axle hole on top
            translate([0,0,Pheight-0.01]) cylinder(d=screw_diameter(Pscrew),h=Ptop+1);
            // Socket cap space on top
            translate([0,0,Pheight+Ptopcap]) cylinder(d=screw_head_diameter(Pscrew)+0.3,h=Ptop+1);
            // space for planet gears in middle
            bevelcylinder(d=gear_OD(gearplane_Pgear(gearplaneT))+2*tipclear,h=Pheight+tipclear,bevel=1);
        }
    }
}

// Clearance around planet carrier
module planet_carrier_clear() {
    translate(carrier_baseC) cylinder(r=carrier_clearR,h=Pbase+Pheight+Ptop+0.5);
}

planet_bearing = bearing_683; // micro M3 bearing 

// Planet gears
module planet_gears()
{
    B = gearplane_Pgear(gearplaneB);
    T = gearplane_Pgear(gearplaneT);
    gearplane_planets(gearplaneB) {
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
            
            cylinder(d=3.2,h=200,center=true); // clear center axle hole
            
            for (z=[gearBC[2]-0.1, 0.1+gearTC[2]+gearZ-bearingZ(planet_bearing)])
                translate([0,0,z])
                    bearing3D(planet_bearing,hole=0);
        }
    }
    
}



geartypeM = geartype_550; // motor output gear
// Drives sun gear above
sundrive_gear = gear_create(geartypeM,47);
sundrive_gearR = gear_R(sundrive_gear);
sundrive_gearC = bearingBC - [0,0,18];

// Directly attached to motor shaft
motor_gear = gear_create(geartypeM,15); // metal gears, "32 pitch"
motor_gearR = gear_R(motor_gear);
motor_gearC = sundrive_gearC + [0,sundrive_gearR+motor_gearR,0];

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
                cylinder(r=mainbearingR-1,h=5); // plate under ring
        }
        translate(gearBC) ring_gear_cut(gearplane_Rgear(gearplaneB));
        BscrewIR_array();
        planet_carrier_clear();
        frame_steel();
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
sundrive_gear_whole();
ring_gear_B();



planet_gears();
//planet_carrier();

#translate(carrier_bearingC) bearing3D(carrier_bearing);


translate(gearTC) gear_3D(gearplane_Sgear(gearplaneT)) {
    cylinder(d=wirehole,h=50,center=true);
}

translate(motor_gearC) gear_3D(motor_gear) {
    translate([0,0,-5]) motor_3D_shaft(motortype);
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
        for (Z=[frameBC,frameTC]) translate(Z) cube([4,1,1]*inch,center=true); // steel frame
        
        // Top tube clearance cut
        translate(bearingTC+[0,0,-0.01]) cylinder(r=mainbearingR,h=carrier_bearingC[2]-bearingTC[2] + bearingZ(carrier_bearing));
        
        // Bottom tube clearance cut
        translate(bearingBC+[0,0,+0.01]) scale([1,1,-1]) cylinder(r=carrier_clearR,h=bearingBC[2]-sundrive_gearC[2]);
        
    }
}

module illustrate_frame() {
    #translate(bearingBC) bearing3D(mainbearing);

    #frame_steel();

    #translate(motorC) motor_3D(motortype);


    #BscrewIR_array();
    #BscrewOR_array();
}

//illustrate_frame();
