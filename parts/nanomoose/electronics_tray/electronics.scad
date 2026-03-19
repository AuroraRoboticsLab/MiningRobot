/*

Arduino Uno (R3 or R4)

3S lipo battery: up to 26mm X, 145mm Y, 60mm Z

Dual 298D drive motor controller: 37x37mm

*/

include <../nanomoose_frame.scad>;

eplateZ = 72; // height of electronics box mounting plate above frame centerline (high enough to go on top of big batteries
eboltTap = 2.7; // tap diameter for electronics box mounting screws
eboltThru = 3.2; // thru diameter


waffleSz=[2*subframeDX, 30+mainframeDY]; // dimensions of waffle top plate
waffleCenter=[0,-mainframeDY+waffleSz[1]/2,eplateZ];


// Arduino Uno hole locations, from https://cdn-shop.adafruit.com/datasheets/arduino_hole_dimensions.pdf
arduHole0 = [0,0]; // hole closest to power inlet jack
arduHole1 = [50.8+1.3,5.1]; // by analog pins
arduHole2 = arduHole1 + [0,27.9]; // by ICSP
arduHole3 = [1.3,arduHole2[1]+15.2]; // too close to aref (not useable)
arduHoles = [arduHole0, arduHole1, arduHole2];

arduinoCenter=[5,-15,eplateZ]; // shifted off-center to avoid punching through battery

module arduCenters() {
    translate(arduinoCenter)
    rotate([0,0,-90])
    translate([-25,-25,0])
    for (h=arduHoles) translate(h) 
        children();
}

// Raspberry Pi 4 or 5
piCenter=[0,-10,eplateZ];

module piCenters() {
    translate(piCenter)
    leftright() frontback()
        translate([49/2,58/2,0])
            children();
}

// L298 dual channel drive motor controller
mcCenter = [0,-90,eplateZ];
module mcCenters() {
    translate(mcCenter)
    leftright() frontback()
        translate([37/2,37/2,0])
            children();
}

bigBat=[26,150,60];

module bigBat2D(enlarge=0) {
    translate([0,-mainframeDY+bigBat[1]/2+2,0])
        offset(r=enlarge) bevelsquare(bigBat,bevel=3,center=true);
}

module M3screw(add) {
    if (add) 
        scale([1,1,-1]) cylinder(d1=8,d2=6,h=5); // plastic to tap down into
    else
        cylinder(d=eboltTap,h=12,center=true); // hole for screw
}

// add=1: geometry added
// add=0: geometry removed
module ebox_parts(add=1) {
    clearance=0.5; // space around other parts
    
    if (add) {
        translate(waffleCenter) scale([1,1,-1])
            waffleFloor(wid=waffleSz[0],ht=waffleSz[1]);
    
        wall=1.6;
        translate([0,0,eplateZ])
            scale([1,1,-1]) linear_extrude(height=20,convexity=4)
            difference() {
                union() {
                    translate(waffleCenter) 
                    difference() {
                        square(waffleSz,center=true);
                        offset(r=-wall) square(waffleSz,center=true);
                    }
                    difference() {
                        bigBat2D(wall);
                        bigBat2D(0);
                    }
                }
                translate([0,30,0]) circle(d=25); // wire access
            }
        
        // Motor hold down plates
        motorcenters(with_front=0) {
            difference() {
                union() {
                    TTmainbody(wall+clearance,0);
                        
                    scale([1,-1,1]) for (x=[-23,+7]) translate([x,0,0]) 
                        motorGusset();
                }
                translate([0,TTboxY/2-3-100,0]) cube(200*[1,1,1],center=true);
            }
        }
    }
    else { // holes
        wheelwell3D(0.7); // don't occupy same space as wheel wells
        
        // Avoid all motors
        motorcenters() {
            TTmotorclear(enlarge=clearance, accurate=0, with_bolts=1);
        }
        
        // Electronics wiring through holes
        leftright() translate([28,-60,eplateZ]) scale([1,2,1]) cylinder(d=15,h=20,center=true);
    }
    
    arduCenters() M3screw(add);
    piCenters() M3screw(add);
    mcCenters() M3screw(add);

    ebox_post_centers() {
        dz = eplateZ-ebox_mountF[2];
        if (add) translate([0,0,eplateZ-ebox_mountF[2]-dz/2])
            bevelcube([ebox_sz[0],ebox_sz[1],dz],center=true,bevel=2);
        else
            cylinder(d=eboltTap,h=150,center=true); // hole for long screw
    }
}

module ebox() {
    difference() {
        union() {
            ebox_parts(add=1);
            intersection() {
                wheelwell_trimbox();
                cube([(subframeDX+6)*2,400,2*eplateZ],center=true);
                difference() {          
                    wheelwell3D_solid(enlarge=wheelwellwall);
                    wheelwell3D_solid();
                }
            }
        }
        ebox_parts(add=0);
    }
}

#mainframeSteel();
//#wheelwell3D();

ebox();