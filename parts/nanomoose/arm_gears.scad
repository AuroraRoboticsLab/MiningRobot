/*
 Idea: arm motion motor has a small pinion gear,
 that drives a larger gear on the robot arm.
 
 Goal is more arm gear reduction (and slower speed).

 Dr. Orion Lawlor, lawlor@alaska.edu, 2026-03-10 (Public Domain)
*/

include <AuroraSCAD/bevel.scad>;
include <AuroraSCAD/gear.scad>;
include <TT_holder.scad>;

gearZ=8.0; // Z height of gear teeth

// Arm gear tooth style: module, height, pressure angle, addendum/dedendum
geartype_arm = [ 1.5, gearZ, 20, 0.32, 0.4 ]; 

gearM = gear_create(geartype_arm,8);
gearA = gear_create(geartype_arm,31);
translateMA = gear_R(gearM)+gear_R(gearA); // distance between centers
rotateMA = 20; // rotation between motor and gear (relative to +X)
echo("motor-arm pivot distance: ",translateMA);
echo("arm gear OD: ",gear_OD(gearA));

armAxleOD=3.0; // diameter of arm pivot bolts
armAxleTap=2.6; // tap diameter

armTiltDY=18; // distance up on gear to arm tilt linkage

armpivotOD = 6; // size of arm pivot points (same as M3 head)
rotateAL=[0,0,-30]; // gear-relative rotation between arm gear and scoop low pivot
translateAL=[60,0,0]; // translation distance between arm and scoop low pivot

armAL=-55; // angle of arm in low position
armAH=+125; // angle of arm in high position

trimangle=-armAL; // gear cut angle (assuming 180 deg of travel)
limittab=8; // height of tabs to stop overrotation

// 3D version of motor-side little gear pinion
module armgearM() {
    difference() {
        gear_3D(gearM,bevel=0);
        TTshaft(enlarge=-0.0);
    }
}

// 3D version of arm-side big gear
module armgearA() {
    difference() {
    linear_extrude(height=gearZ) 
    {
        difference() {
            union() {
                difference() {
                    gear_2D(gearA);
                    // Trim teeth that are beyond travel range
                    difference() {
                        rotate([0,0,trimangle]) translate([0,-limittab-100]) square([200,200],center=true);
                    }
                }
                
                round=5;
                offset(r=-round) offset(r=+round) 
                {
                    // Solid limit tab
                    rotate([0,0,trimangle]) translate([0,-limittab/2]) square([gear_OD(gearA)+1,limittab],center=true);
                    // Any fixed child geometry
                    children();
                }
            }
            
            // Cut hole for gear pivot bolt
            circle(d=armAxleOD);
            
            translate([0,armTiltDY,0]) circle(d=armAxleTap);
            
            rotate(rotateAL) translate(translateAL) 
circle(d=armAxleOD);
            
            // Limit travels
            squish=[1,0.9,1]; // push gear back into teeth slightly
            limitcut=gear_OR(gearM); // remove teeth at travel limits
            for (angle=[-armAL,-armAH]) 
                rotate([0,0,angle]) translate([-translateMA,0,0]) scale(squish) circle(r=limitcut);
        }
    }
        // Lighten holes
        for (a=[0:360/6:360-1]) rotate([0,0,a])
            translate([gear_IR(gearA)*0.6,0,2])
                bevelcylinder(d=11,h=gearZ,bevel=2);
    }
}

// Frame parts that reach down to arm pivot point
module armframe() {
    wid=2.4;
    for (angle=[-armAL,-armAH]) 
        hull() {
            rotate([0,0,angle]) translate([-gear_R(gearA),0,0]) 
                circle(r=wid);
            rotate(rotateAL) translate(translateAL) 
circle(r=wid);
        }
    // Extra material around arm pivot
    rotate(rotateAL) translate(translateAL) 
 circle(d=armpivotOD);
}

wall=1.6; // case wall

// Geometry added to motor case, to hold arm pivot bolt
module armpivotcase()
{
    // Dimensions of added box
    X=8; // 31;
    Y=10;
    Z=10;
    difference() {
        translate([0,0,-Z+wall])
        hull() {
            translate([0,-TTboxY/2-wall,0])
                cube([X,Y,Z]);
            rotate([0,0,rotateMA]) translate([translateMA,0,0])
                cylinder(d=5.8,h=Z);
        }
        
        // Tap thru at pivot
        rotate([0,0,rotateMA]) translate([translateMA,0,0])
            cylinder(d=armAxleTap,h=40,center=true);
    }
}

// Demo of all arm gears and case
module demo_arm() {
    rotate([0,0,90]) rotate([90,0,0]) // in frame orientation
    {
        rotate([0,0,rotateMA]) {
            armgearM();
            translate([translateMA,0,0]) armgearA() armframe();
        }
        translate([0,0,-wall]) {
            TTmotor_case(wall=wall) armpivotcase();
            #TTmotorclear();
        }
    }
}

// Printable arm raise components
module printable_arm_raise(with_case=0) {
    if (with_case)
        TTmotor_printable(wall=wall) armpivotcase();
    
    translate([-30,-15,0]) armgearM();
    translate([0,-40,0]) armgearA() armframe();
}

// Printable arm tilt components
module printable_arm_tilt(with_case=0) {    
    translate([-30,15,0]) armgearM();
    translate([0,40,0]) difference() {
        armgearA();
        translate([0,0,gearZ-3.0]) cylinder(d=6,h=10);
    }
}


//demo_arm();
//printable_arm_raise();

