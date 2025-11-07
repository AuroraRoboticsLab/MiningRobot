/*
 A rack-and-pinion gear system that attaches to a linear actuator.
 Provides a lot of rotation distance in a small space.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2025-11-06 (Public Domain)
*/
include <AuroraSCAD/gear.scad>;
include <AuroraSCAD/bevel.scad>;

$fs=0.1; $fa=2;

inch=25.4; // file units are mm

actuator_tubeOD=20.0+0.4; // tube body of actuator, plus printed clearance
actuator_pinOD=1/4*inch; // crosspin
actuator_pinX=-8.5; // centerline of actuator crosspin from front of tube

actuator_mount=18; // length of actuator mounting portion
actuator_endY=-4; // sets plastic thickness behind gear teeth
actuator_startX=-actuator_mount; // start of teeth on X
actuator_startY=-14; // start of teeth on Y
actuator_travel=4.2*inch; // travel of linear actuator

tooth_clearance=0.0;

drivebox = 1.0*inch+0.5; // box section that we drive

gearZ = 25.4;
/* gear module, Z thickness, pressure angle, addendum, dedendum */
geartype = [2.0, gearZ, 14.5, 0.32, 0.4];
gearP = gear_create(geartype,28);
gearR = gear_create(geartype,5000); // hacky way to make a rack
gearRT = actuator_travel / geartype_Cpitch(geartype)+5;

pinion_circle = 3.141592*gear_D(gearP);
echo("Pinion circumfrence = ",pinion_circle/inch," inches");
echo("Linear actuator travel = ",actuator_travel/inch," inches");
echo("Rotation travel = ",360*actuator_travel/pinion_circle," degrees");

module drivegear2D() {
    difference() {
        rotate([0,0,360/gear_nteeth(gearP)/2])
            gear_2D(gearP);
        bevelsquare([1,1]*drivebox,bevel=0.7,center=true);
        // off-center axle!
        sep=0.0; // plastic between steel drive parts
        axleOD=3/8*inch;
        translate([0,-drivebox/2-axleOD/2]) circle(d=axleOD);
    }
}

module drivegear3D() {
    difference() {
        linear_extrude(height=gearZ,convexity=6)
            drivegear2D();

        // M3 socket cap mounting bolt holes (threaded into steel drivebox)
        for (angle=[0,90,180]) rotate([0,0,angle])
            translate([0.8*inch,0,gearZ/2]) rotate([0,90,0])
            {
                cylinder(d=3.1,h=20,center=true); // shaft
                cylinder(d=6.0,h=20); // cap
            }
    }
}

module rack2D() {
    round=6;
    intersection() {
        difference() {
            offset(r=-round) offset(r=+round)
            union() {
                // Outside profile of rack driver
                translate([0,-gearZ]) square([actuator_travel,gearZ+actuator_endY]);
                // Meat around actuator tube
                translate([actuator_pinX,-gearZ])
                    square([actuator_mount+2*2.4,actuator_tubeOD+2*(2.4+gearZ)],center=true);
            }
            
            // Actuator mounting pin hole
            translate([actuator_pinX,0])
                circle(d=actuator_pinOD); 
        }
        // Gear teeth (heaven help me!)
        translate([actuator_startX,actuator_startY])
        rotate([0,0,-1]) //<- superhack to make bottom basically flat
        //hull() //<- check for flatness 
        translate([0,+gear_R(gearR),0])
        rotate([0,0,-90+360/gear_nteeth(gearR)/2])
        gear_2D(gearR,numteeth=gearRT);
    }
}

module rack3D() {
    difference() {
        linear_extrude(height=gearZ,center=true,convexity=8) 
            rack2D();
        
        rotate([0,-90,0]) bevelcylinder(d=actuator_tubeOD,h=30,bevel=0.5);
    }
}

translate([-actuator_travel/2-actuator_startX,+gear_D(gearP)/2-actuator_startY+5,gearZ/2]) 
    rack3D();
drivegear3D();
