/*
Cycloplanetary is a sort of hybrid between:
  - Planetary, with one big planet matching ring gears
  - Cycloidal, driving the planet with an offset axle
  - Strain wave, advancing one tooth per revolution
(Similar to a trochoidal pump, or eccentric reducer.)

The basic idea is to rotate two stepped ring gears using a slightly smaller drive gear.
The drive gear gets pushed by an eccentric axle shaft.
The eccentric axle is where motor power comes in.

This is similar to pericyclic nutating gears, but without the weird rotational axis swap.
   https://www.youtube.com/watch?v=Z-zUTS5FPPc

Idea: slot the eccentric shaft, so you can bend it and slide the bearings on.  
    (As a rigid part, the bearings need to be added mid-print, which is tricky and keeps the layer lines from going the right way for bending stress.)
    In practice, it's not easy (possible?) to add the bearings even with the slot.

Testing slim version (6710/6704 bearings) in PETG, v1B with 0.05 clearance on the sun (too high), it skips teeth at 30-40N force at 275mm lever arm (8-11 N m).  Oddly this doesn't seem to damage anything, it just pops--possibly the gears deforming enough to skip teeth.

Testing heavy version (6813/6807 bearings) in Rapid PETG, v4 with 0.0 clearance, it skips teeth at about 70N force on a 275mm lever arm (20 N m).  Adding v5 preload of +0.1mm on the axle reduces the backlash and still assembles easily, but skips at the same torque.

A larger gear module would likely be stronger, but needs more space between the gears.

Attaching a brushless motor via another 4.2:1 drive gear (55:13 teeth), a 500:1 gear reduction is actually too high for reasonable wheel speed.  Target gear reduction is more like 100:1.

At motor speed, vibration is significant due to the off-center motion.  The 6807 bearings each mass 30 grams, and there's about another 3 grams of off-center plastic, radius 1.875mm.  A counterweight could be made from 6awg solid copper wire, diameter 4.1mm, which masses 2.4 grams per 20mm length. 

v7: Switched to slimmer 6706 bearings, which allow much larger gear module (and hopefully higher torque).  Gear reduction is theoretically 104:1, but in practice it seems to be more like 128:1 indicating I'm off by one somewhere?

Mass of two bearings is 16g, and mass of two bearings and larger eccentric gears is 32g, with radius 2.625mm. Counterweight at 12mm radius thus needs 32g * 2.625mm / 12mm = 7 grams of weight, about three sticks of copper. 

v8: Upgraded mount bolts to M5


References:
(Hsieh, 2014) The effect on dynamics of using a new transmission design for eccentric speed reducers
https://www.sciencedirect.com/science/article/abs/pii/S0094114X1400127X

Dr. Orion Lawlor, lawlor@alaska.edu, 2025-06-30 (Public Domain)
*/

include <AuroraSCAD/gear.scad>;
include <AuroraSCAD/bearing.scad>;
include <AuroraSCAD/bevel.scad>;
include <AuroraSCAD/motor.scad>;


// Face-mounted drive motor
//motor = motortype_NEMA17; // stepper
motor=motortype_3674; // brushless

drive_bolts=0; // 1: three M3 bolts on drive axle.  0: hex drive
drive_face=1; // 1: motor mounts directly to face of reducer.  0: motor mounts somewhere else
axle_slot=0; // 1: axledrive shaft has a slot down the middle. 0: axledrive is solid, print bearings in place.

// main bearing is the single bearing that holds top and bottom together
main_bearing = bearing_6813; 
//axle_bearing = bearing_6807; 
axle_bearing = bearing_6706; 

nring=25; // teeth in planet gear
gearZ = 10.0; // vertical height of gear teeth
axledrive_preload=0.0; // + gives additional kick-out in axle to take up gear+bearing slack
planet_gearclear=0.05; // + gives planet gear clearance: printer adds enough for assembly

// much slimmer hand demo version:
// main_bearing = bearing_6710;  
// axle_bearing = bearing_6704;
//nring=35; // teeth in planet gear
//gearZ = 6.0; // vertical height of gear teeth

floorW=1.5; // wall thickness of floors
floorZ=bearingZ(axle_bearing)+floorW; // thickness of base plates (with mounted axle bearings)
thruOD = bearingOD(axle_bearing)-3; // thru hole diameter

botZ = 0; // bottom surface of main bearing
topZ = bearingZ(main_bearing); // top surface of main bearing
gapZ = 3.2; // space for inside bolt heads

// bearings for axle: four small ring bearings
axleZE = 0.4; // extra Z on axle (clearance between moving parts, avoid rubbing)
axleZ=bearingZ(axle_bearing)+axleZE; // Z height, plus a little clearance to fit


// main bolts cling onto the main bearing: I for inside, O for outside
mainboltNI = 6; // number of main bolts (inside)
mainboltODI = 5.0; // shaft diameter + clearance, M5 bolt
mainboltTapI = 4.5; // tap diameter
mainboltShaftZI = 3; // length of clear shaft portion
mainheadODI=9.0;
mainheadZI=5.0;

mainboltNO = 6; // number of main bolts (outside)
mainboltODO = 5.0; // shaft diameter + clearance, M5 bolt
mainboltTapO = 4.5; // tap diameter
mainboltShaftZO = 3; // length of clear shaft portion
mainheadODO=9.0;
mainheadZO=5.0;

// Counterweight held on with M3 bolts
cwboltOD = 3.0;
cwboltTap = 2.7;
cwheadOD = 5.8;

module cwboltAngles() {
    delta=60; 
    for (angle=[-1,0,+1]) rotate([0,0,delta*angle])
        translate([+axle_retainboltR,0,0])
            children();
}


flatBZ=botZ - floorZ; // flat bottom start
flatTZ=topZ + gapZ + gearZ + floorZ; // flat top end

module motor_facemount() {
    if (drive_face==1) {
        translate([0,0,flatBZ]) motor_3D(motor);
        translate([0,0,flatBZ-1]) linear_extrude(height=25) motor_screwholes_2D(motor);
    }
}


mainboltRI = bearingIR(main_bearing) - mainboltODI/2; // inside bolt circle
mainboltRO = bearingOR(main_bearing) + mainboltODO/2; // outside bolt circle
mainboltRA = 7.5;
mainboltZI = topZ;
mainboltZO = botZ;
mainboltZA = flatBZ;


module mainbolt_centers(r, z, n, down, skip180=0) {
    for (angle=[0:360/n:360-1]) rotate([0,0,angle])
        if (skip180==0 || angle!=180)
        translate([r,0,z]) scale([1,1,down?-1:+1])
            children();
}

// Inside main bearing bolt centers, facing down
module mainbolt_centersI(n=mainboltNI) {
    mainbolt_centers(r=mainboltRI, z=mainboltZI, n=n, down=1) children();
}

// Outside main bearing bolt centers, facing up
module mainbolt_centersO(n=mainboltNO) {
    mainbolt_centers(r=mainboltRO, z=mainboltZO, n=n, down=0) children();
}

// Axle main bearing bolt centers, facing up
module mainbolt_centersA() {
    mainbolt_centers(r=mainboltRA, z=mainboltZA, down=0, skip180=1) children();
}

module mainboltI(shaft=50) { // smooth, for tap: mainboltShaftZI) {
    translate([0,0,-0.01]) cylinder(d=mainboltODI,h=shaft); // thru
    cylinder(d=mainboltTapI,h=shaft+50); // tap
    scale([1,1,-1]) cylinder(d=mainheadODI,h=mainheadZI); // head
}
module mainboltO(shaft=mainboltShaftZO) {
    translate([0,0,-0.01]) cylinder(d=mainboltODO,h=shaft); // thru
    cylinder(d=mainboltTapO,h=shaft+50); // tap
    scale([1,1,-1]) cylinder(d=mainheadODO,h=mainheadZO); // head
}


geartype = geartype_create(1.75,gearZ-0.5,20,0.2,0.25); // Stubby module 1.0 gears
tooth_clearance=0.0; // clearance between gear teeth

// bottom (B) and top (T) gearplanes
space=3; // fewer teeth in sun gear (gives space for gear tips to clear)
skip=2; // more teeth in output plane (sets ratio)
gB = [geartype,-nring+space,nring,1]; // bottom (B) gearplane
gBZ = botZ; // Z coordinate of start of bottom gearplane
gRB = gearplane_Rgear(gB); // ring gear
gPB = gearplane_Pgear(gB); // planet gear

gT = [geartype,-nring-skip+space,nring+skip,1]; // top (T) gearplane
gTZ = topZ + gapZ; // Z coordinate of start of top gearplane
gRT = gearplane_Rgear(gT); // ring gear
gPT = gearplane_Pgear(gT); // planet gear

gearbottomZ = 0.5; // Z clearance around bottom of gears
extendGZ=1; // extend gear teeth this far in Z (lets bevels taper off)

// Print the orbit radius values for the bottom and top.  
//   The orbit radii need to match.
module echo_orbits() {
    gearplane_print(gB, "Bottom (input) gears");
    gearplane_print(gT, "Top (output) gears");
    
    rB = gearplane_ratio_Rfixed(gB);
    rT = gearplane_ratio_Rfixed(gT);
    echo("OrbitB = ",gearplane_Oradius(gB),"  ratioB ",rB);
    echo("OrbitT = ",gearplane_Oradius(gT),"  ratioT ",rT);
    
    // Actual gear reduction ratio as measured: about 1:448
    //echo("Overall axle reduction ratio: ",1/(rB-rT));
    eRatio = nring / space; // reduction ratio axle to eccentric revs
    rRatio = nring / skip; // reduction ratio eccentric to ring shift
    echo("Approx ratio: ",eRatio," * ",rRatio," = ",eRatio*rRatio);
}


axle_offset=gearplane_Oradius(gB);

// List of axle bearing Z start coordinates:
axleZs=[
    gBZ-axleZ, // frameB bottom [0]
    gBZ-0.01, // eccentric bottom [1]
    gTZ+gearZ-axleZ+0.01, // eccentric top [2]
    gTZ+gearZ, // frameT top [3]
];

/* Demo gearplanes in 2D */
module demo2D()
{
    // Bottom gearplane (yellow)
    difference() { 
        gearplane_2D(gB,clearance=tooth_clearance); 
        gearplane_planets(gB) circle(d=bearingOD(axle_bearing)); 
    }

    // Top gearplane (purple)
    translate([0,0,gearZ]) color([1,0,1]) 
        difference() { 
            gearplane_2D(gT,clearance=tooth_clearance); 
            gearplane_planets(gT) circle(d=bearingOD(axle_bearing)); 
        }
    
    #mainbolt_centersI() circle(d=mainheadODI);
    #translate([0,0,botZ]) bearing3D(main_bearing);
    //#cube([75,25.4,1],center=true); // 1x1 steel frame
}

/* Use roof and extrude a bevel of this height */
module bevel_extrude(height,bevel,convexity=6)
{
    translate([0,0,bevel]) 
        linear_extrude(height=height-2*bevel,convexity=convexity)
            children();

    translate([0,0,bevel]) scale([1,1,-1])
        intersection() 
        {
            roof() children();
            translate([-1000,-1000,0]) cube([2000,2000,bevel]);
        }
}

/* Top frame portion, covers outside of main bearing */
module frameT(versionText="") {
    difference() {
        union() {
            hull() {
                z=flatTZ-botZ;
                // Reach out to main bolts (or not quite as long?)
                translate([0,0,botZ]) mainbolt_centersO() cylinder(d=mainheadODO,h=z);
                
                // Cover the main bearing
                translate([0,0,botZ]) cylinder(d=bearingOD(main_bearing)+3,h=bearingZ(main_bearing)+1);
                // Cover the top bearing
                translate([0,0,flatTZ-floorZ]) cylinder(d=bearingOD(axle_bearing)+3,h=floorZ);
                // Cover the top gear area
                translate([0,0,flatTZ-floorZ]) cylinder(d=gear_ID(gRT)+3,h=floorZ);
                
            }
            children(); // any other parts mount on here
        }
        
        // Cut in gear teeth
        translate([0,0,gTZ-extendGZ]) gear_3D(gRT,height=gearZ+extendGZ+gearbottomZ);
        
        // Clearance for inner bolt heads to spin
        translate([0,0,botZ-0.01]) bevelcylinder(d=bearingOD(main_bearing)-2,h=gTZ-botZ,bevel=2);
        
        mainbolt_centersO() mainboltO();
        
        translate([0,0,botZ]) bearing3D(main_bearing);
        translate([0,0,axleZs[3]]) bearing3D(axle_bearing,extraZ=axleZE,hole=0);
        
        // Clearance around bottom bearing
        translate([0,0,flatTZ-1]) scale([1,1,-1]) bevelcylinder(d=bearingOD(axle_bearing)-2,h=20,bevel=1);
        
        translate([0,0,axleZs[3]]) cylinder(d=thruOD,h=30);
        
        /*
        // NEMA-style drive bolt mounting points, for M3 screws
        mountR=30;
        for (angle=[0:360/8:360-1]) rotate([0,0,angle])
            translate([mountR,0,flatTZ-10]) cylinder(d=mainboltTap,h=20);
        */
                    
        // Lighten gaps
        difference() {
            boltOD=8;
            round=4;
            rib=1.6;
            translate([0,0,gTZ+floorW]) 
                bevel_extrude(height=100,bevel=round,convexity=6) 
                offset(r=+round) offset(r=-round)
                difference() {
                    // outside walls
                    offset(r=-rib) 
                        hull() mainbolt_centersO() circle(d=mainheadODO);

                    // material around bolt heads
                    mainbolt_centersO() circle(d=mainheadODO);
                    //hull() 
                    //mainbolt_centers(r=mountR,z=0) circle(d=boltOD);

                    // central ribs
                    for (angle=[0:360/mainboltNO:360-1]) rotate([0,0,angle])
                        square([2*mainboltRO,rib],center=true);
                    
                    // Don't encroach on ring gear
                    circle(d=gear_OD(gRT)+3*rib);
                }
        }
        
        // Version text
        translate([0,mainboltRI-7,flatTZ]) linear_extrude(height=1,center=true) version2D(versionText);

        
    }
}

/* Bottom frame portion, goes inside main bearing */
module frameB() {
    difference() {
        union() {
            z=topZ-flatBZ;
            // Reach out to main bolts
            translate([0,0,flatBZ]) 
            linear_extrude(height=z) hull() {
                mainbolt_centersI() circle(d=8);
            }
            // Cover lip of main bearing
            OD=bearingID(main_bearing);
            taper=2;
            translate([0,0,botZ-taper/2]) cylinder(d1=OD-0.3,d2=OD+taper,h=taper/2+0.01);
            translate([0,0,botZ]) cylinder(d=OD+taper,h=topZ-botZ);
        }
        
        translate([0,0,gBZ-gearbottomZ]) gear_3D(gRB,height=gearZ+extendGZ+gearbottomZ);
        
        //translate([0,0,gBZ-0.2]) cylinder(d=gear_ID(gRB),h=gearZ+2);
        
        mainbolt_centersI() mainboltI();
        
        translate([0,0,botZ]) bearing3D(main_bearing);
        translate([0,0,axleZs[0]]) bearing3D(axle_bearing,extraZ=axleZE);
        
        // Thru hole out bottom axle bearing
        translate([0,0,flatBZ-0.1]) cylinder(d=thruOD,h=20);
        
        // Thru hole out top
        translate([0,0,gBZ]) cylinder(d=gear_ID(gRB),h=20);
        
        // optional motor face mount holes
        motor_facemount();
        
        // Version text
        translate([0,mainboltRI-5,flatBZ]) scale([-1,1,1]) linear_extrude(height=1,center=true) version2D();
    }
}

/* Print text version number centered at origin */
module version2D(versionText="") {
    text(str(versionText,"v8-M5"),size=3,halign="center",valign="center");
}

/* Eccentric gears mate to both ring gears, and transfer power */
module eccentric_gears() {
    difference() {
        union() {
            solidOD = min(gear_ID(gPB),gear_ID(gPT));
            translate([0,0,gBZ]) {
                gearplane_planets(gB) {
                    gear_3D(gPB,height=gearZ+extendGZ,clearance=planet_gearclear);
                    cylinder(d=solidOD,h=gTZ+gearZ-gBZ); // solid connection between gears
                    
                    donutZ = (gBZ + gTZ+gearZ)/2; // Z height of merge ring donut: centered halfway between gears
                    
                    rotate_extrude() translate([solidOD/2-0.1,donutZ]) 
                        rotate([0,0,-30]) scale([1,2]) circle(r=1.3);
                }
            }
            translate([0,0,gTZ-extendGZ])
                gearplane_planets(gT) gear_3D(gPT,height=gearZ+extendGZ,clearance=planet_gearclear);
            
        }
        // Clearance for drive bearings
        gearplane_planets(gB) {
            for (z=[axleZs[1],axleZs[2]]) translate([0,0,z])
                bearing3D(axle_bearing,extraZ=axleZE);
            
            cylinder(d=bearingOD(axle_bearing)-2,h=100,center=true); // thru
        }
    }
}

/* 2D shape that drives the axle */
hexdrive_flats=20; // distance across flats of hex
module hexdrive2D(enlarge=0) {
    round=0.3;
    rotate([0,0,30])
    offset(r=+round) offset(r=-round)
    circle(d=hexdrive_flats/cos(30),$fn=6);
}

axleslotW=2;

/* Slot cut into the axle, acts as flexure and drive spline.
   extraW makes the slot wider
   extraL makes the slot longer
*/
module axleslot(extraW=0,extraL=0) {
    topZ = (axleZs[2]+axleZs[3])/2;
    hull() 
        for (z=[flatBZ,topZ]) translate([0,0,z])
            rotate([90,0,0])
                cylinder(d=axleslotW+extraW*2,
                        h=bearingID(axle_bearing)-1+extraL, center=true);
}

/* Fits in axledrive, holds stepper shaft */
module axlehex3D(h=flatTZ-flatBZ-3)
{
    difference() {
        union() {
            translate([0,0,flatBZ]) linear_extrude(height=h) hexdrive2D();
            if (axle_slot) axleslot();
        }
        
        motor_facemount();
    }
}

// Retaining bolt radius in axle
axle_retainboltR = hexdrive_flats/2+3.5;

/* Central block that drives the eccentric */
module axledrive() {
    OD=bearingID(axle_bearing)+0.5;
    ID=11; // inside hole, for encoder or some such
    
    Z = flatTZ-flatBZ-1.5;
    centerZ=axleZs[2]+axleZ-axleZs[1];
    difference() {
        union() {
            // Ends cylinder
            translate([0,0,flatBZ]) cylinder(d=OD,h=Z);
            
            // Extra material in middle
            gearplane_planets(gB) { //<- picks up XY offset to match planets
            
                translate([axledrive_preload,0,axleZs[1]+0.01]) 
                    cylinder(d=bearingID(axle_bearing)+2,h=centerZ-0.02);
            }
            
            children(); // any extra drive gizmos go at the top
        }
        insertZ=3; // extra Z to allow parts to be slid on (not feasible for interior, hits exterior though)
        e=axleZE+insertZ;
        
        // Bottom
        translate([0,0,axleZs[0]-insertZ]) bearing3D(axle_bearing,extraZ=e);
        // Top
        translate([0,0,axleZs[3]]) bearing3D(axle_bearing,extraZ=axleZE);
        
        
        // Eccentrics
        gearplane_planets(gB) { //<- picks up XY offset to match planets
            
        
            // Trim whole interior to allow slide-over (spaced by gear)
            translate([axledrive_preload,0,axleZs[1]]) bearing3D(axle_bearing,extraZ=axleZs[2]-axleZs[1]+axleZE);
            
            /*
            // Separate parts
            translate([0,0,axleZs[1]]) 
            {
                bearing3D(axle_bearing,extraZ=axleZE);
                // Trim central shaft
                bearing3D(axle_bearing,clearance=-2.0,extraZ=centerZ-axleZ);
            }
            translate([0,0,axleZs[2]]) bearing3D(axle_bearing,extraZ=axleZE);
            */
        }
        
        drive_clearance=0.25; // space around drive spline
        if (axle_slot) axleslot(extraL=5,extraW=drive_clearance);
        
        if (drive_bolts==1) {
            // Drive bolt holes
            mainbolt_centersA() mainboltI(shaft=drive_clearance);
            
            // Thru hole inside
            translate([0,0,flatBZ-0.1]) cylinder(d=ID,h=100);
        }
        else { // hex drive
            translate([0,0,flatBZ-0.1]) linear_extrude(height=100) 
                hexdrive2D(enlarge=drive_clearance);
            
            // Steel reinforcing screw or shaft can be added here
            cwboltAngles()
                translate([0,0,flatBZ-0.1]) cylinder(d=cwboltTap,h=100);
        }
        
        // optional motor face mount decreases bottom thickness
        //motor_facemount();
        
    }
}

// Stick on counterweight: compensates for off-center rotation torque
module counterweight() {
    thick=11; // needs to be thick enough to hold weights
    counterC = [-(63*1.875/(4*2.4)),0,thick/2]; // counterweight center
    weightOD=4.2;
    weightY=20;
    weightDX=3;
    weightDZ=thick*0.22;
    weightCross=0.5; // slanted hole spacing
    
    wall=2.4;
    mate=8; // fits down into hex drive
    difference() {
        union() {
            linear_extrude(height=thick+mate) hexdrive2D(); // male side, plugs into axle
            hull() {
                cwboltAngles() cylinder(d=cwheadOD,h=thick);
                linear_extrude(height=thick) offset(r=+wall) hexdrive2D(); 
                translate(counterC) bevelcube([2*weightDX+weightOD+2*wall,weightY,thick],center=true,bevel=2);
            }
        }
        
        // Counterweight slots
        for (dx=[-1,+1]) for (dz=[-1,+1])
            translate(counterC+[(dx-weightCross*dz)*weightDX,-weightY/2,dz*weightDZ])
                rotate([-90,0,0])
                    cylinder(d=weightOD,h=weightY+10);
        
        // Bolt
        cwboltAngles()
        translate([0,0,thick/2]) scale([1,1,-1]) {
            cylinder(d=cwboltOD+0.2,h=100,center=true);
            cylinder(d=cwheadOD+0.5,h=100);
        }
        
        // Thru hole in center
        linear_extrude(height=100) offset(r=-wall) hexdrive2D(); 
    }
}


/* Full 3D demo with cutaway */
module demo3D(cutaway=1) {
    difference() {
        union() {
            frameT();
                eccentric_gears();
                axledrive();
            frameB();
            if (1) {
                #translate([0,0,botZ]) bearing3D(main_bearing);
                #for (z=[axleZs[0],axleZs[3]]) translate([0,0,z])
                    bearing3D(axle_bearing,extraZ=axleZE);
            }
        }
        if (cutaway)
            for (angle=[0,45]) rotate([0,0,angle])
            color([1,0,0]) translate([0,0,-50]) cube([100,100,100]);
    }
    
}



/* Small parts to demonstrate principle of operation */    
module printable_demo() {
    gearclear=0.1; // planet gear clearance
    clearance=0.15; // space for moving parts
    rB = gearplane_Rgear(gB);
    rT = gearplane_Rgear(gT);
    
    // Eccentric planet gears
    difference() {
        union() {
            gearplane_planets(gB) gear_3D(gearplane_Pgear(gB),height=gearZ,bevel=0,clearance=gearclear);
            translate([0,0,gearZ])
                gearplane_planets(gT) gear_3D(gearplane_Pgear(gT),height=gearZ,bevel=0,clearance=gearclear);
        }
        // Thru clearance for bearing (etc)
        gearplane_planets(gB) 
            cylinder(d=bearingOD(axle_bearing),h=100,center=true);
    }
    
    // Bottom ring
    translate([0,-50,0]) {
        linear_extrude(height=gearZ) {
            gearplane_ring_2D(gB);
            translate([gear_OR(rB),0,0]) square([5,2]);
        }
        linear_extrude(height=2*gearZ) 
        difference() { // shell to hold bottom and top together
            rim=2; // baked into gearplane_ring_2D
            circle(r=gear_OR(rB)+rim+0.5);
            circle(r=gear_OR(rT)+rim+clearance);
        }
    }
    
    // Top ring (fits in top of bottom ring)
    translate([0,+50,0])
    linear_extrude(height=gearZ) {
        gearplane_ring_2D(gT);
        //translate([gear_OR(rT),0,0]) square([4,2]);
    }
    
    
}

// Scale factor to compensate for post-printing part shrinkage
scale=1.0025; // typical for PLA

module printable_frameB() {
    scale([1,1,1]*scale) translate([0,0,-flatBZ]) frameB();
}
module printable_frameT() {
    scale([1,1,1]*scale) rotate([180,0,0]) translate([90,0,-flatTZ]) frameT();
}
module printable_eccentric_gears() {
    scale([1,1,1]*scale) translate([-70,0,-gBZ]) eccentric_gears();
}
module printable_axledrive() {
    scale([1,1,1]*scale) translate([0,100,0])
    {
        if (axle_slot) { // print sideways, to get layers the right way
            rotate([-90,0,0]) translate([0,-bearingIR(axle_bearing),-flatBZ]) axledrive();
        } else { // print upright, so bearings can be added during print
            translate([0,0,-flatBZ]) axledrive();
        }
    }
}

module printable_axlehex() {
    scale([1,1,1]*scale) translate([0,50,0])
        translate([0,0,-flatBZ]) axlehex3D();
}

// Handheld spinner
module printable_axlehandspin() {
    translate([0,0,-flatBZ]) {
        axlehex3D(h=10);
        d=10;
        cylinder(d=d,h=20);
        bevel=4;
        rotate_extrude() 
        translate([d/2,1]) 
        difference() {
            square(bevel*[1,1]);
            translate(bevel*[1,1]) circle(r=bevel);
        }
    }
}

/*
#translate([0,0,-bearZ]) bearing3D(axle_bearing);
#translate([0,0,2*gearZ]) bearing3D(axle_bearing);
#translate([axle_offset,0,gearZ-bearZ/2]) bearing3D(axle_bearing);
*/

//printable_demo();
echo_orbits();
echo("Bolt circle inside radius: ",mainboltRI);
echo("Bolt circle outside radius: ",mainboltRO);
echo("Drive hex diameter across flats: ",hexdrive_flats);


if (0) { /* don't print anything (include) */ }
else if (0) {
    demo2D();
}
else if (1) {
    demo3D();
}
else if (0) { // printable pieces
    printable_eccentric_gears();

    printable_frameB();
    printable_frameT();

    //printable_axledrive();
} 
else if (0) { // separate batch for axle, so bearings can be 3D printed into it
    //printable_axlehex();
    printable_axledrive();
}
else if (0) {
    counterweight();
}
else if (1) { // CAM cross sections
    projection(cut=true) translate([0,0,-gTZ-0.1*gearZ]) frameT();
    //projection(cut=true) translate([0,0,-gBZ-0.5*gearZ]) frameB();
    //projection(cut=true) translate([0,0,-gTZ-0.5*gearZ]) eccentric_gears();
    //projection(cut=true) translate([0,0,-gBZ-0.5*gearZ]) eccentric_gears();
}
else if (1) { // balance check
    difference() {
        printable_axledrive();
        side=-1;
        translate([side*100,100,0]) cube([200,200,200],center=true);
    }
}
else if (0) {
    // Drive hex for hand testing
    printable_axlehandspin();
}






