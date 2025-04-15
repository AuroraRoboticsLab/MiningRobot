/*
 "Yin-Yang" charger shape: fully reversible coupler design, with each side having one rod and one clamp.
 
 Typical rod material: high conductivity metal like aluminum (ISRU), copper (conductivity), or nickel (wear resistance).
 
 Typical clamp: spring-loaded electrical contact, like an Anderson PowerPole 
 
*/
$fs=0.1; $fa=2;
include <charger_interface.scad>;
include <AuroraSCAD/powerpole.scad>; 

charger_thick=15; // thickness of charger along Z
charger_wall = 2.4;
charger_vert = charger_dockOD/2*charger_dockA[1];
charger_back = charger_vert + charger_wall;
charger_wide = 2*(charger_rodRX + charger_dockOD/2)+charger_wall;

charger_rod_support = 6;

clamp_clearance=0.3;
pp = powerpole_45A; // powerpole used in clamp
ppC = [-charger_rodRX+charger_rodOD/2,0];

//#color([0.7,0.7,1.0]) translate([charger_rodRX,0,0]) circle(d=charger_rodOD);

// 2D cross section of charger
module charger_section2D()
{
    round=3;
    offset(r=-round) offset(r=+round)
    difference() {
        union() {
            translate([charger_wall/2+charger_dock_space/4,-charger_back/2])
                square([charger_wide,charger_back],center=true);
            
            translate([-charger_rodRX,0]) scale(charger_dockA)
                circle(d=charger_dockOD-0.5*charger_dock_space);
        }
        
        // Empty space behind rod
        translate([+charger_rodRX,0]) 
        difference() {
            scale(charger_dockA)
                circle(d=charger_dockOD+0.5*charger_dock_space);
            difference() { 
                translate([-charger_rod_support/2,-charger_back])
                    square([charger_rod_support,charger_back]);
                circle(d=charger_rodOD);
            }
        }
        
        // Space around clamp
        translate([-charger_rodRX,0])
        {
            taper=3;
            d=charger_rodOD+clamp_clearance;
            // fine alignment taper
            hull() {
                circle(d=d);
                translate([0,charger_vert])
                    circle(d=d+taper);
            }
            // coarse alignment tapered dock
            rotate([0,0,45])
                square([100,100]);                
        }
    }
}

// Orient children like the left powerpole
module powerpole_orientation() {
    translate(ppC) rotate([0,90,0]) rotate([-90,0,0]) translate([0,0,-powerpole_length(pp)+3]) 
        children();
}

// Make 3D charger shape, including all holes for bolts and power.
//   If this includes a 2D child, it is unioned to the charger rod.
module charger_3D() {
    difference() {
        linear_extrude(height=charger_thick,convexity=6,center=true)
        {
            charger_section2D();
            translate([charger_rodRX,0,0]) children();
        }
        
        powerpole_orientation() {
            powerpole_3D(pp,wiggle=0.2,pins=0,matinghole=0);
            translate([0,+10]) powerpole_pins3D(pp,2,40);
        }
        
        // screws to hold rod, and feed it power
        for (z=[-1,+1])
            translate([+charger_rodRX,0,z*5]) rotate([90,0,0]) 
                cylinder(d=3.2,h=50);
        
        // Mounting screw holes in back
        for (x=[-1,0,+1]) translate([x*45,-charger_back-1]) rotate([-90,0,0])
            cylinder(d=2.6,h=8);
    }
}


charger_3D(); // metal rod bolts on
//charger_3D() circle(d=charger_rodOD);


