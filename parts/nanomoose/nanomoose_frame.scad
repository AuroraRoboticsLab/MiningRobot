/*
Nanomoose: 1/4 scale multipurpose robot

Coordinate system is centered between the robot's 4 wheels
  X: across the robot
  Y: forwards
  Z: up
  
 Tires:
   Inside fits directly to TT motor axle
   Outside held by M3 tapped into frame, bearing in tire
  
 Dr. Orion Lawlor, lawlor@alaska.edu, 2026-03-10 (Public Domain)
*/

include <AuroraSCAD/bearing.scad>;
include <TT_holder.scad>;
include <arm_gears.scad>;
$fs=0.2; $fa=4; // coarse

overscale=1/4; // scale factor to real version
sinch=25.4*overscale; // scaled inches

frameW = 1.0*sinch; // width
frameH = 2.0*sinch; // Z height (thicker for better stiffness)
frameN = 0.5*sinch; // narrow width for selected frame areas

mainframeDY=19*sinch; // half the front-back edge-to-edge length
mainframeDX=14*sinch; // half the left-right edge-to-edge width
subframeDX=6.5*sinch; // half the left-right subframe width

// Make a frame-type box, centered, with these dimensions
module mainframeStick(dim,enlarge=0,bevel=0,threeD=0) {    
    if (threeD) {
        // Direct to 3D version:
        bevelcube(dim-2*enlarge*[1,1,1],center=true,bevel=bevel);
    }
    else {
        // 2D version, allowing bevels
        square([dim[0],dim[1]]-2*enlarge*[1,1],center=true);
    }
}

// Battery box with charger and 18650
batterySZ=[25.4,98,23];
batteryC=[0,-mainframeDY+batterySZ[1]/2,batterySZ[2]/2];
module battery_box() {
    translate(batteryC) cube(batterySZ,center=true);
}

// Make all the frame steel (starting from 3D)
module mainframeSteel(enlarge=0,bevel=0) {
    round=frameW;
    linear_extrude(height=frameH,center=true,convexity=4) 
    offset(r=-round) offset(r=+round)
    {
        // Front and back crossbars
        for (frontback=[-1,+1]) translate([0,frontback*(mainframeDY-0.5*frameW),0]) 
            mainframeStick([2*mainframeDX,frameW,frameH],enlarge=enlarge,bevel=bevel);
        
        // Left and right sides
        for (leftright=[-1,+1]) {
            extra=5; // extra forward length
            translate([leftright*(subframeDX-0.5*frameW),+extra/2,0])
                mainframeStick([frameW,2*mainframeDY+extra,frameH],enlarge=enlarge,bevel=bevel);
            
            translate([leftright*(mainframeDX-0.5*frameN),0,0])
                mainframeStick([frameN,2*mainframeDY,frameH],enlarge=enlarge,bevel=bevel);
            
            // Slim support between wheels
            translate([leftright*(mainframeDX+subframeDX)/2,0,0])
                mainframeStick([mainframeDX-subframeDX,frameN,frameH],enlarge=enlarge,bevel=bevel);
        }
        
        // Crossbar under battery
        translate([0,batteryC[1]+batterySZ[1]/2,0])
            mainframeStick([2*subframeDX,frameW,frameH],enlarge=enlarge,bevel=bevel);
    }
}



tireOD=15*sinch; // diameter of tire (before grousers)
tireZ=6*sinch; // thickness of tire
tireB=0.75*sinch; // bevels
tireclear=2.0; // clearance around wheel moving parts

// Beveled outline cylinder bounding tire
module tire_cylinder(extraR=0,extraB=0,extraZ=0)
{
    bevelcylinder(d=tireOD+2*extraR,h=tireZ+extraZ,bevel=tireB+extraB,center=true);
}

ngrousers=16;
grouserangle=25;
grouserht=0.1*tireOD/2; // 10% of tire radius

// Sloped grouser shapes
module tire_grousers() {
    for (angle=[0:360/ngrousers:360-1]) rotate([0,0,angle])
        translate([tireOD/2-grouserht*0.5,0,0])
            rotate([-grouserangle,0,0])
                cube([3*grouserht, 1.6, tireZ*1.1],center=true);
}

// Tire lighten spoke shapes
module tire_lighten() {
    spoke=1.6; // thickness of spokes (4 perimeters)
    nspoke=6;
    rim=1.6; // thickness of tire rim
    round=6;
    intersection() {
        tire_cylinder(extraR=-rim,extraB=+0.3*rim,extraZ=0.2);
        
        linear_extrude(height=tireZ+0.2,center=true,convexity=4) 
        offset(r=+round) offset(r=-round)
        difference() {
            circle(r=tireOD/2-rim); // outside
            circle(d=14); // center
            for (angle=[0:360/nspoke:360-1]) rotate([0,0,angle])
                square([tireOD,spoke],center=true);
        }
    }
}

tireshaftDZ=0.4*sinch; // support block around motor drive shaft
tireshaftZ=-tireZ/2-tireshaftDZ; // start Z location of motor drive shaft relative to tire center

// Tire axle spaces
module tire_axlespace() {
    cylinder(d=3.2,h=tireZ+0.2,center=true); // cut M3 throughout
    
    translate([0,0,tireshaftZ-0.1]) {
        TTshaft(enlarge=0.1); // motor shaft
        cylinder(d1=TTshaftOD+0.4,d2=TTshaftOD-0.6,h=1.5); // taper entrance
    }
    
    bearing=bearing_683; // micro M3
    translate([0,0,tireZ/2-bearingZ(bearing)+0.1])
    {
        bearing3D(bearing);
        cylinder(d=5,h=10,center=true); // space for inner spin
    }
}

// Full 3D tire
module tire_full() {
    difference() {
        union() {
            tire_cylinder();
            translate([0,0,tireshaftZ]) cylinder(d1=10,d2=12,h=tireshaftDZ);
        }
        tire_lighten();
        tire_axlespace();
        //rotate([0,0,45]) cube([100,100,100]); // cutaway
    }
    
    // Add trimmed grouser shapes
    intersection() {
        tire_grousers();
        difference() {
            tire_cylinder(extraR=grouserht,extraB=grouserht*0.5);
            tire_cylinder(extraR=-1);
        }
    }
}

mainaxleDY=9*sinch; // half the front-back distance between axles
mainaxleDX=10*sinch; // half the left-right distance between tire centers
mainaxleDZ=3*sinch; // height of wheel axles above frame centerline

// Put children at each tire axle center (preserves orientation)
module tireaxles(flipY=1) {
    for (frontback=[-1,+1]) for (leftright=[-1,+1]) 
        translate([leftright*mainaxleDX,frontback*mainaxleDY,mainaxleDZ])
            scale([leftright,flipY?frontback:1,1])
                children();
}

motorinset=2;
motorX=mainaxleDX+tireshaftZ-motorinset; // X center of all motor axles
frontmotorY=19*sinch; // Y center of front motor axle
frontmotorZ=3*sinch; // Z center of front motor axle
frontmotorA=[-30,0,0];

// Arm structural pivot point
frontpivotY=frontmotorY+translateMA;
frontpivotZ=frontmotorZ;

// Make these parts left-right symmetric around X
module leftright() {
    for (leftright=[-1,+1]) scale([leftright,1,1])
        children();
}

// Put children at each motor center
module motorcenters(with_tire=1,with_front=1) {
    if (with_tire)
        tireaxles(flipY=0) rotate([90,0,0]) rotate([0,90,0]) 
            translate([0,0,tireshaftZ-motorinset]) children();
    
    if (with_front)
        leftright()
            translate([motorX,frontmotorY,frontmotorZ])
                rotate(frontmotorA) rotate([90,0,0]) rotate([0,90,0]) 
                    children();
}

// Front frame steel prongs
module frontSteel() {
    start=[subframeDX-0.5*frameW,frontmotorY,0];
    back=15;
    pts=[
        [0,-back,15], // up in motor housing
        [0,-back,0], // below motor housing
        [0,6,0], // front lower elbow (arbitrary)
        [0,translateMA,frontmotorZ] // front pivot point
    ];
    // Chain hull through points
    for (i=[0:2])
        hull() {
            for (p=[start+pts[i], start+pts[i+1]])
                translate(p) rotate([0,90,0])
                    cylinder(d=frameH,h=frameW,center=true);
        }
}

// Wheelwells are mostly to keep dust away from electronics inside
wheelwellwall=1.2;
wheelwellCZ=3.2; // spacing along wheel centerline
wheelwellCR=grouserht+2.0; // spacing outside wheel radius
module wheelwell2D() {
    // XY -> ZY axis flip here
    hull() {
        for (side=[-1,+1]) translate([mainaxleDZ,side*mainaxleDY]) 
            circle(d=tireOD+2*wheelwellCR);
    }
} 

module wheelwell3D_solid(enlarge=0) {
    leftright() translate([mainaxleDX,0,0]) rotate([0,-90,0])
        bevel_extrude_convex(height=tireZ+2*wheelwellCZ+2*enlarge, center=true, bevel=tireB)      
            offset(r=+enlarge) wheelwell2D();
}

// Wheel well gets trimmed back by this box (separate piece added with electronics housing?)
module wheelwell_trimbox() {
    // to insert motors with wheels already attached, we need clearance down to the motors
    bevel=12;
    translate([0,TTindexdotDX/2,100+mainaxleDZ]) bevelcube([200,mainaxleDY*2+2*bevel-TTindexdotDX,200],center=true,bevel=bevel);
}

module wheelwell3D() {
    difference() {
        wheelwell3D_solid(enlarge=wheelwellwall);
        wheelwell3D_solid();
        
        leftright() { 
            // trim outside (only care about dust headed inwards
            translate([subframeDX+6+200,0,0]) cube([400,400,400],center=true);
            // trim below center
            translate([0,0,-frameH/2-200]) cube([400,400,400],center=true);
            
        }
        wheelwell_trimbox();
    }
}

// Electronics tray mounts on top of these bolt points
ebox_mountF=[-36,-16,32];
ebox_mountB=ebox_mountF+[0,-100,0];
ebox_sz=[10,8,ebox_mountF[2]+frameH/2];

module ebox_posts() {
    leftright() {
        for (m=[ebox_mountF,ebox_mountB]) translate(m) 
        difference() {
            translate([0,0,-ebox_sz[2]/2])
                bevelcube(ebox_sz,center=true,bevel=2);
            cylinder(d=armAxleTap,h=30,center=true);
        }
    }
}



// Gusset triangle underneath the motors (anti-rotation, and print support)
module motorGusset() {
    rotate([0,90,0])
        linear_extrude(height=2.4) hull() {
            translate([0,-13]) square([20,6]);
            translate([0,-25]) square([5,1]);
        }
}

// Ridge underneath motor case, to support print
module TTmotor_case_ridge(wall=1.6) {
    for (z=[0,-TTboxZ-wall]) 
    translate([0,-wall,z])
    linear_extrude(height=wall) 
        difference() {
            TTmainbody2D(wall);
            translate([0,100]) square([200,200],center=true);
        }
}

// Block to tap in a bolt that holds down the front of the motor case
module TTmotor_boltfront(wall=1.6) {
    block = TTnubXY+3*wall;
    translate([TTnubDX,0,TTnubDZ+TTnubZ/2+block/2]) 
    difference() {
        bevelcube(block*[1,1.2,1],center=true,bevel=wall,bz=0);
        cylinder(d=armAxleTap,h=20,center=true);
    }
}

// Frame with motor mount points
module mainframeMotors() {
    difference() {
        union() {
            mainframeSteel();
            
            // Motor cases
            motorcenters() {
                TTmotor_case() TTmotor_case_ridge();
                TTmotor_boltfront();
                for (dx=[0,-18]) translate([dx,0,0]) 
                    motorGusset();
            }
            motorcenters(with_front=0) {
                translate([-36,0,0]) 
                    motorGusset();
            }
            
            leftright() frontSteel();
            
            wheelwell3D();
            ebox_posts();
        }
        
        battery_box();
        
        // Re-clear space around motors
        motorcenters() { 
            // TTmainbody(); TTmotorbody(); 
            TTmotorclear(with_bolts=1, accurate=0); 
        }
        
        // Front arm gear axles (thru, tapped into the arm frame)
        translate([0,frontmotorY+translateMA,frontmotorZ])
            rotate([0,90,0]) cylinder(d=armAxleOD,h=mainframeDX,center=true);
        
        // Space around moving wheel parts
        tireaxles() rotate([0,90,0]) tire_cylinder(extraR=grouserht+tireclear,extraZ=2*tireclear);
    }
    
    // Tapped upright on outside of each wheel
    tireaxles() translate([tireZ/2,0,0]) 
    difference() {
        tirelong=4.5*sinch;
        wid=1.5*sinch;
        for (angle=[-45,+45]) rotate([angle,0,0])
            translate([0.5+frameW/2,0,-tirelong/2+wid/2])
                mainframeStick([frameW,wid,tirelong],threeD=1);
        translate([frameW/2,0,0]) rotate([0,90,0]) {
            cylinder(d=armAxleTap,h=20,center=true); // M3 thru
            cylinder(d=6.0,h=10); // M3 socket cap
        }
    }
}

armclear=0.3; // clearance between moving parts of arm

// The arm frame provides twist resistance, and tap points for the M3 bolts holding the whole arm assembly together
module armframeSteel() {
    round=6;
    X = -subframeDX+frameW+armclear; // half-width of full frame
    Y = translateAL[0]; // length of frame along Y
    intersection() {
        difference() {
            // Rounded beveled exterior
            hull() {
                for (y=[0,Y]) translate([0,y]) rotate([0,90,0])
                    bevelcylinder(d=frameH,h=2*(-X),center=true,bevel=1.5);
            }
            
            // Tap holes for M3
            for (y=[0,Y]) translate([0,y]) rotate([0,90,0])
                cylinder(d=armAxleTap, h=2*mainframeDX,center=true);
        }
        linear_extrude(height=frameH,center=true,convexity=4) 
        offset(r=-round) offset(r=+round) 
        leftright() {
            // straight push
            translate([X,-frameH/2])
                square([frameW,Y+frameH]);
            
            // crossbar
            hull() {
                for (p=[[X+frameW/2,0.1*Y], [-X-frameW/2,0.9*Y]])
                    translate(p) circle(d=frameW);
            }
        }
    }
}

// Demo of fully assembled rover
module demo_rover() {
    tireaxles() rotate([0,90,0]) tire_full();
    
    motorcenters() #TTmotorclear(dual_shaft=1);

    mainframeMotors();

    #battery_box();
    
    translate([0,frontpivotY,frontpivotZ]) armframeSteel();
}

module printable_tire() 
{
    translate([0,0,tireZ/2]) rotate([180,0,0]) tire_full();
}

//demo_rover();

//printable_tire();
intersection() { mainframeMotors(); translate([0,-5,-10]) cube([100,200,100]); } // trimmed test
//armframeSteel();
if (0) {
    printable_arm_raise();
    printable_arm_tilt();
}



