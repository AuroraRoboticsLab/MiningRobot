/*
 Snowblower attachment for Nanomoose.
 Single stage, mostly to test robotic vehicle dynamics, driving and path planning.
 
 An auger's spin throws snow at a given tangential velocity, 
 which you can derive from the radius and spin rate:
    https://www.artificial-gravity.com/sw/SpinCalc/
 
 Single stage design is tricky: to spin fast enough to throw, it's hard to limit the centrifugal force.
 1/10 scale:
    0.017780 meter radius (14 inch diameter/ 2 / 10th scale)
    270 rpm spin rate
    0.5 m/s tangential throw velocity (5 m/s / 10th scale)
    1.43 g centripetal acceleration
 
 1/4 scale:
    0.044 meter radius 
    270 rpm spin rate
    1.25 m/s tangential throw velocity
    3.6 g centripetal acceleration
 
 full scale:
    0.178 meter radius (14 inch diameter/2)
    270 rpm spin rate
    5 m/s tangential throw velocity
    14.3 g centripetal acceleration
 
 Reference:
 Ariens single-stage just has two U shaped rubber paddles:
    https://www.youtube.com/watch?v=XswHMXY_Gq0
 
 Lessons learned from prototyping:
    - Reductions in the snow flow path are a bad idea, snow will pack in and stick.
    - Large open thrower shapes are ideal
    - Faster spinning is generally better, something like 1K RPM for the little 1/10 scale worked best, arcing about a half meter (predicted for 270 rpm).
 
 Input brushless motor RPM: about 10K
 Output auger RPM: about 1K or less, so at least a 10:1 total reduction ratio
 
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2026-03-12 (Public Domain)
*/
//$fs=0.2; $fa=3;

include <AuroraSCAD/bearing.scad>;
include <AuroraSCAD/bevel.scad>;
include <AuroraSCAD/gear.scad>;
include <AuroraSCAD/motor.scad>;

include <../interface.scad>; // tool interface to nanomoose

//inch=25.4; // actual inch
//sinch=inch*1/4; // scaled inch

augerD = 14*sinch; // auger diameter
augerL = 28*sinch; // auger length (gets bearings and drive added on)
augerB = 1.5*sinch; // bevels at edges (less pointy, stronger housing)

// Overall outside limit of auger
module auger_cylinder(enlarge=0) {
    bevelcylinder(d=augerD+2*enlarge,h=augerL+2*enlarge,center=true,bevel=augerB+0.7*enlarge);
}



// Run children for +Z and -Z ends
module both_ends() {
    for (side=[-1,+1]) scale([1,1,side]) 
        children();
}

hexdrive_clear=0.2; // printed clearance around hex drive (fairly generous, so it can be slid around)
hexdrive_flats=12.0+2*hexdrive_clear; // flat-to-flat diameter of hex drive shaft
hexdriveOD = hexdrive_flats/cos(30); // tip-to-tip diameter of hex drive shaft
hexdrive_out=0; // 1: hex plastic part facing out.  0: hex space through part (for metal hex)

hexbearing_clearance=0.2;
hexbearing=bearing_6704; // was bearing_608;
hexbearingID = bearingID(hexbearing)-hexbearing_clearance;
hexbearingOD = bearingOD(hexbearing)+hexbearing_clearance;
hexbearingL = bearingZ(hexbearing)+hexbearing_clearance; // length of bearing itself
bearing_seat = 1; // lip for securing bearing

auger_ground_clearance=1.5; // clearance between ground and auger
auger_housing_clearance=2.0; // between auger and housing (in radius)
auger_housing_clearanceZ=2.0; // on ends

augerBSL = 2*bearing_seat+2*auger_housing_clearanceZ + augerL; // length to start of bearings
augerBEL = augerBSL+2*hexbearingL; // bearing end length

augerW=2.4; // wall thickness of auger parts
augerHR = 10; // hexdriveOD/2+augerW; // radius of plastic around hexdrive

augerIL = 6*sinch; // central spherical throw area: makes throw sphere bigger
augerOL = (augerL-augerIL)/2; // Auger outer helix length
augerOT = 145; // auger outside total twist
augervanes = 3; // number of vanes in auger

augerPR = 0.22*augerD; // snow-pitching radius
augerPX = 0.33*augerD; // snow-pitching X shift of radius circle
augerPY = 0; // snow-pitching Y shift of radius circle (rotate to make this always 0?)
augerPB = 0.1*augerD; // bevel rounding


// Outer housing parameters:
housing_wall=1.6;
housingID = augerD + 2*auger_housing_clearance;
housingL=augerL+2*auger_housing_clearanceZ;
housing_topY=8*sinch; // height of top front lip (snow cutting surface)

chuteH = 12*sinch; // height of outlet chute above centerline
chuteX=-0.3*augerD;
chuteD=0.45*augerD; // X diameter of snow chute
chuteL=1.5*chuteD; // Z length of snow chute
chuteRY = 0.25*augerD; // Y coordinate of straight part of snow chute
chuteRA = 50; // degrees of rotation
chuteRC = [0,chuteRY,-0.5*augerL]; // center point of rotation

// Mounting bolts on perimeter of housing, holding drive system (and allowing assembly)
houseboltThru = 3.1; // M3 thru
houseboltTap = 2.6; // M3 tap diameter
houseboltHead = 6.0; // head diameter (socket cap)
houseboltCutZ = housingL/2; // split Z plane, after edge of bevel
houseboltZ = houseboltCutZ + 3.0; // under center of bolt head
houseboltR = augerD/2+3.0; // centerline to centerline distance
houseboltL = 12; // max length of bolt
houseboltAngles = [-135,70,150];



// Gear parameters:
gearZ = 8; // Z height of gears
gearC = 1.0; // clearance around gears (all sides)
gearS = auger_housing_clearanceZ; // Z spacing between motor and auger side gear teeth
geartypeM = [ 0.8, gearZ, 20, 0.32, 0.4 ]; ; // 32 pitch / M0.8 motor gear teeth
geartypeA = [ 1.5, gearZ, 20, 0.32, 0.4 ]; // M1.5 auger drive gear teeth

motor = motortype_2845; // purple Turnigy drive motor, XK2845

gearM = gear_create(geartypeM, 10); // motor pinion (metal, existing on shaft)
gearI = gear_create(geartypeM, 27); // idler (spaces away from motor)
gearRM = gear_create(geartypeM, 27); // reducer input on motor side

gearRA = gear_create(geartypeA, 8); // reducer pinion on auger side
gearA = gear_create(geartypeA, 44, ring=1); // auger input gear (inside)

echo("Total gear reduction: ",gear_nteeth(gearRM)/gear_nteeth(gearM)*gear_nteeth(gearA)/gear_nteeth(gearRA));

bearingG = bearing_683; // tiny M3 bearings

spacingAR = gear_R(gearA)-gear_R(gearRA); // gear-to-gear spacing, auger to reducer
spacingRI = gear_R(gearRM)+gear_R(gearI); // reducer to idler
spacingIM = gear_R(gearI)+gear_R(gearM); // idler to motor
angleAR = 15; // rotation angle for auger-reducer centerline (around Z, starting from Y)
angleRI = 15; 
angleIM = 15;
startA = [0,0,augerL/2+auger_housing_clearanceZ/2 - gearZ - gearC]; // auger gear
startRA = startA + [-sin(angleAR)*spacingAR,cos(angleAR)*spacingAR, gearC]; // start point of reducer auger side
startRM = startRA + [0,0,gearZ+gearS]; // start point of reducer motor side gear
startI = startRM + [-sin(angleRI)*spacingRI,cos(angleRI)*spacingRI, 0]; // idler
startM = [startI[0],startI[1],0]+[-sin(angleIM)*spacingIM,cos(angleIM)*spacingIM, houseboltCutZ]; // start motor at dividing plane
motorboltZ=2.4; // Z height of plate with motor bolts

gearboltTap=2.7; // space for tapping in M3 bolt
gearboltHead=6; // M3 socket head
gearboltHeadZ=3;

// Put children at motor center
module motor_center() {
    translate(startM) rotate([0,0,angleIM]) children();
}


// Cylinder bounding the auger gear 
module gearA_bound(enlarge=0) {
    intersection() {
        // Don't exceed the housing length
        cube([200,200,housingL-auger_housing_clearanceZ],center=true);
        
        translate(startA+[0,0,-enlarge])
        {
            bevelcylinder(d=gear_OD(gearA)+2*enlarge,h=gearZ+2*enlarge,bevel=0.7*enlarge);
            scale([1,1,-1]) // taper cone to distribute torque
                cylinder(d1=40,d2=10,h=20);
        }
    }
}

// Negative tooth space around the auger gear.  
//   Must be centered at startA first
module gearA_toothspace() {
    difference() {
        gear_3D(gearA,height=gearZ+3);
        
        // Don't mess with material close to the bearings
        //cylinder(d=bearingOD(hexbearing),h=40);
        round=4;
        rotate_extrude() translate([0,-0.01])
        offset(r=-round) offset(r=+round)
        {
            square([bearingOR(hexbearing),40]);
            scale([1,-1]) square([gear_OR(gearA),10]);
        }
    }
}

// Reducer gear, starting at small reducer gearRA
//   Must be centered at startRA first
module gearR_full() {
    difference() {
        union() {
            // Extend bottom gear to meet top
            gear_3D(gearRA,height=gearZ+gearS+gearZ/2);
            translate([0,0,gearZ+gearS]) gear_3D(gearRM);
            
            // Distribute torque between gears smoothly
            translate([0,0,gearZ+0.5])
                cylinder(d1=gear_ID(gearRA), d2=gear_OD(gearRA)+2, h=gearS);
        }
        
        // Thru hole to leave space for M3 to spin freely
        cylinder(d=(bearingOD(bearingG)+bearingID(bearingG))/2,h=50,center=true);
        
        // Topside bearing
        translate([0,0,gearZ+gearS+gearZ-bearingZ(bearingG)])
            bearing3D(bearingG);
        
        // Space for bottom bearing and M3 bolt head
        translate([0,0,-0.01]) cylinder(d=bearingOD(bearingG)+hexbearing_clearance, h=bearingZ(bearingG)+gearboltHeadZ);
    }
}

// Generic gear bounding
module gear_bound(gear,enlarge=0,extraZ=0) {
    translate([0,0,-enlarge])
        bevelcylinder(
            d=gear_OD(gear)+2*enlarge,
            h=gearZ+2*enlarge+extraZ,
            bevel=0.5+0.7*enlarge);
}

// Top geartrain bounding volume 
module geartop_bound(enlarge=0) {
    difference() {
        hull() {
            translate(startRM) gear_bound(gearRM,enlarge=enlarge);
            translate(startI) gear_bound(gearI,enlarge=enlarge);
        }
        
        if (enlarge<=gearC) // inside cut: leave spacers above bearings
        for (center=[startRM,startI]) translate(center+[0,0,gearZ])
            cylinder(d1=5,d2=8,h=2);
    }
    translate(startM) gear_bound(gearM,enlarge=enlarge,extraZ=5);
}

// Material to add to top endcap
module gear_housing_plus() {
    wall=2.4;
    difference() {
        union() {
            // Material surrounding gears
            geartop_bound(gearC+wall);
            
            // Motor faceplate
            motor_center() {
                linear_extrude(height=motorboltZ,convexity=2)
                    motor_face_2D(motor);
            }
            
            // Reinforcing around gear bolt tap locations
            for (center=[startRM,startI]) translate(center)
            rotate([0,0,angleRI])
            {
                h=gearZ+wall+10;
                d=gearboltTap+2*wall;
                cylinder(d=d,h=h);
                for (angle=[0:360/4:360-1]) rotate([0,0,45+angle])
                    translate([0,-wall/2,0])
                    hull() {
                        cube([d/2,wall,h]);
                        cube([12,wall,gearZ+wall]);
                    }
            }
        }
        
        // Space for main motor shaft
        motor_center()
            cylinder(d=5,h=20); // gear_bound(gearM,enlarge=gearC,extraZ=10);
        
        // space to insert gears
        for (insert=[0,1]) translate([0,0,-5*insert])
            geartop_bound(gearC);
        
        // Don't add stuff below the cut line
        housingcut();
    }
}

// Material to remove from housing (after adding)
module gear_housing_minus() {
    // Tap screws
    for (center=[startRM,startI]) translate(center)
        cylinder(d=gearboltTap,h=30);
    
    // Carve space to insert gears
    difference() {
        for (insert=[0,1]) translate([0,0,-5*insert])
            geartop_bound(gearC);

        housingcut();
    }
    
    // Motor and bolts
    motor_center() {
        motor_3D(motor);
        translate([0,0,motorboltZ]) motor_bolts(motor);
    }
    
    // Leave space to insert bearing down in (and drain debris down out)
    translate([0,0,augerBSL/2]) bearing3D(hexbearing,extraZ=20);
}

// Idler gear: stuff hole full of bearings, leave space for M3 socket head.
module gear3D_I() {
    difference() {
        gear_3D(gearI);
        translate([0,0,2.5]) cylinder(d=bearingOD(bearingG)+hexbearing_clearance,h=20);
        cylinder(d=gearboltHead,h=20,center=true); // M3 socket head space here
    }
}

// Printable version of small gears
module printable_smallgears() {
    gear3D_I();
    rotate([180,0,0]) translate([30,0,-gearZ-gearS-gearZ]) gearR_full();
}


// 3D illustration of geartrain
module gear_demo() {
    translate(startA) #gearA_toothspace();
    translate(startRA) gearR_full();
    
    translate(startI) gear3D_I();
    
    translate(startM) gear_3D(gearM);
    
    difference() {
        gear_housing_plus();
        gear_housing_minus();
        cube([200,200,200]); // cutaway
    }
    
    motor_center() {
        #motor_3D(motor);
        translate([0,0,motorboltZ]) #motor_bolts(motor);
    }
    
    translate([0,0,augerBSL/2]) #bearing3D(hexbearing);
}





// Put children at each auger thrower vane (typically 2-3)
module auger_vanes() {
    // for (side=[-1,+1]) scale([side,side,1])
    for (angle=[0:360/augervanes:360-1]) rotate([0,0,angle])
        children();
}

// Cross section of primary part of snow auger helix
module auger_helix2D()
{
    offset(r=-augerPB) offset(r=+augerPB)
    {
        // Central support around hex
        circle(r=augerHR);
        
        //square([augerD, augerW],center=true);
        auger_vanes() {
            intersection() {
                circle(d=augerD); // outside trim
                difference() {
                    // outside shifted by wall
                    translate([augerPX,augerPY])
                        circle(r=augerPR+augerW);

                    // inside
                    translate([augerPX,augerPY])
                    {
                        circle(r=augerPR);
                        // Trim off bottom of circle (arc continues there)
                        translate([0,-100,0]) square([200,200],center=true);
                    }
                }

            }
        }
    }
}

// 3D curved part of snowblower
module auger_helix3D() {
    intersection() {
        auger_cylinder();
        
        for (zside=[-1,+1]) scale([1,1,zside]) 
            translate([0,0,augerIL/2]) 
                linear_extrude(height=augerOL,twist=augerOT,convexity=4)
                    auger_helix2D();
    }
}

// Pythagorean length of hypotenuse with arms A and B
function hypot(A,B) = sqrt(A*A+B*B);

// Radius of snow-facing surface of snow-throwing pocket
frontR = hypot(augerPR,augerIL/2);
// Radius of back surface of snow-throwing pocket
backR = hypot(augerPR+augerW,augerIL/2);

// Ejection cylinder for snow to leave pocket.
//   Needs to be called per vane, and translated by augerPX PY
module auger_ejectpath3D(Xscale=1, enlarge=0) {
    ejectbackA=25; // <- back angle, needed because augerPX is forward
    rotate([0,0,ejectbackA]) {    
        // Pick radius to match up with helix cuts
        frontR = hypot(augerPR+enlarge,augerIL/2);
        
        scale([Xscale,1,1]) sphere(r=frontR); // smooth entrance taper
        
        // Tapered snow ejection path
        hull()
        for (out=[0,1]) translate([0,-out*10,0])
            rotate([0,90,0]) cylinder(r=frontR+out*2,h=augerD);
    }
}

// Auger central snow-throwing pocket
module auger_pocket3D() {
    intersection() {
        cylinder(d=augerD,h=augerIL,center=true); // center section
        
        union() {
            cylinder(r=augerHR,h=augerIL,center=true); // around hex center
            
            auger_vanes() {
                translate([augerPX,augerPY,0])
                {
                    difference() {
                        auger_ejectpath3D(enlarge=augerW);
                        auger_ejectpath3D(enlarge=0);                        
                        
                        // trim off duplicate leading surfaces
                        translate([0,-100,0]) cube([200,200,200],center=true);
                    }
                }
            }
        }
    }
}

// Add material to the helix near the pocket
module auger_helixthicken3D(wall=0) {
    intersection() {
        auger_cylinder(); // limits outside length
        
        auger_vanes()
            translate([augerPX,augerPY,0]) 
            difference() {
                auger_ejectpath3D(enlarge=augerW);
                
                // Trim to avoid messing up the inside surface (brittle hack!)
                hull() {
                    sphere(r=frontR+0.1*augerW); // front of pocket
                    translate([-0.08*augerD,-0.2*augerD,0]) 
                        sphere(r=0.46*augerD);
                    
                }
            }
   }
}

// Full ejection path for snow (blends between auger helix and thrower areas)
module auger_eject3D(wall=0) {
    auger_vanes()
        translate([augerPX,augerPY,0]) 
            auger_ejectpath3D(Xscale=0.6,enlarge=0);
}

// Overall snow pushing auger
module auger3D() {
    difference() {
        union() {
            // Helix sides pull snow in to center
            auger_helix3D();
            
            // Central pocket ejects snow
            auger_pocket3D();
            
            // Thicken helix walls near pocket
            auger_helixthicken3D();
            
            if (hexdrive_out) {
                // Drive hex
                cylinder(d=hexdriveOD,h=augerL/2+15,$fn=6);
            }
            
            both_ends() {
                // Plastic that goes inside the bearings
                translate([0,0,-augerBEL/2])
                    cylinder(d=hexbearingID,h=20);
                
                // Plastic that goes outside, to index to bearings
                translate([0,0,-augerBSL/2-0.5])
                    cylinder(d1=hexbearingID+3,d2=10,h=25);
            }
            
            // Material around the ring gear
            gearA_bound(enlarge=augerW);
        }
        // Cut for the ring gear
        translate(startA) gearA_toothspace();
        
        // M3 tap down centerline (for strength, optional)
        cylinder(d=houseboltTap,h=2*augerL,center=true);
        
        // Clear ejection path for leaving snow
        auger_eject3D();
        
        // Thru hole for hex (in full size version)
        if (hexdrive_out==0) cylinder(d=hexdriveOD,center=true, h=augerBEL+15,$fn=6);
    }
}

// Put children at housebolt center points
module houseboltcenters() {
    for (a=houseboltAngles) rotate([0,0,a]) translate([houseboltR,0,houseboltZ])
        children();
}

// Exterior cylinders surrounding housebolts
module houseboltexteriors() {
    houseboltcenters() translate([0,0,2]) rotate([180,0,0])
         bevelcylinder(d=houseboltTap+2*housing_wall,h=houseboltL+housing_wall,bevel=housing_wall);
}

// The housing wraps around the auger, channeling snow's path
module housing2D(enlarge=0) {
    hull() {
        circle(d=housingID+2*enlarge);
        offset(r=+enlarge) translate([0.4*augerD,0])
            square([housing_wall,housing_topY]);
    }
}

// Cylinder for auger area
module housing3Dcylinder(enlarge=0) {
    bevel_extrude_convex(height=housingL+2*enlarge,center=true,bevel=augerB+0.5*enlarge)
        circle(d=housingID+2*enlarge);
}

// Snow outflow chute cross section (XY here -> XZ in housing coords)
module housing2Dchute(enlarge=0) {
    bevelsquare([chuteD,chuteL]+2*enlarge*[1,1],center=true,bevel=augerB+0.7*enlarge);
}


// Full solid housing, with bearing mounts and eject path
module housing3Dsolid(enlarge=0, with_bearing=1, with_chute=1) {
    bevel_extrude_convex(height=housingL+2*enlarge,center=true,bevel=augerB+0.5*enlarge)
        housing2D(enlarge=enlarge);
    
    if (with_bearing) {
        both_ends() {
            for (seat=[0,1]) translate([0,0,housingL/2-0.01+(seat?0:bearing_seat)])
                cylinder(d=hexbearingOD+2*enlarge-2*seat,h=hexbearingL -0.01*enlarge);
        }
    }
    
    // Snow outflow path
    if (with_chute)  {
        rotate([-90,0,0]) // XY -> XZ
            linear_extrude(height=chuteRY) 
                translate([chuteX,0,0])
                    housing2Dchute(enlarge=enlarge);
        translate(chuteRC) 
            rotate([0,90,0]) // make rotation around X
                rotate_extrude(angle=-chuteRA+0.2*enlarge) // around Z
                    translate([chuteRC[2],chuteX]) rotate([0,0,90]) 
                        housing2Dchute(enlarge=enlarge);
    }
    
    /*
    // Previous chute: simple hull shape
    hull() {
        OD= chuteD + 2*enlarge;
        exitH=1.5*OD;
        exitY=chuteH-4*enlarge;
        exitA=0;
        for (o=[0,1]) 
            translate([chuteX,o*exitY,0]) rotate([o*exitA,0,0]) bevelcylinder(d=OD,h=exitH,center=true,bevel=augerB+0.7*enlarge);
    }*/
}

// Radius to arm mounting bolts
mountboltR = augerD/2+auger_housing_clearance+housing_wall+8;
mountlowZ = subframeDX-frameW+armclear; // start of low mount point
mountlowA = 30; // angle of low mount bolt points (lift)
mounthighA = -15; // angle of high mount bolt point (tilt)
mounthighZ = mountlowZ+frameW+(gearZ-frameW); // mounts to tilt arm
 

// Beveled mount point, origin at bolt hole inside center
module mountpoint3D(hole=armAxleOD) {
    inset=4;
    reach=25;
    thick=frameW;
    bevel=0.3*frameW;
    difference() {
        union() {
            scale([1.5,1,1]) // stretch the mount point
                rotate([0,0,-45]) translate([-inset,-inset,0]) 
                    bevelcube([reach,reach,thick],bevel=bevel);
            
            dA = 15; // angle to add fillet
            round=3; // size of weld fillets to add
            translate([+mountboltR,0,0]) // get back to auger centerline
            rotate([0,0,180-dA/2])
            rotate_extrude(angle=dA) {
                outsideR = augerD/2+auger_housing_clearance+housing_wall;
                translate([outsideR-1,thick/2]) 
                offset(r=-round) offset(r=+round) {
                    square([2,2*thick],center=true);
                    square([2*round+2,thick],center=true);
                }
            }
        }
                
        cylinder(d=hole,h=3*thick,center=true);
    }
}

// Housing with snow ejection paths
module housing3Deject() {
    difference() {
        union() {
            housing3Dsolid(housing_wall); // exterior

            // Hull to smooth transition to bearings
            hull() {
                housing3Dsolid(housing_wall,with_chute=0);
                houseboltexteriors();
            }
            
            // Arm mount points
            both_ends() rotate([0,0,mountlowA]) 
                translate([-mountboltR,0,mountlowZ]) 
                    mountpoint3D();
            // tilt
            rotate([0,0,mounthighA])
                translate([-mountboltR,0,-mounthighZ]) 
                    scale([1,1,-1]) // tab faces up
                        mountpoint3D(hole=armAxleTap);
            
            gear_housing_plus();
        }
        gear_housing_minus();
        
        housing3Dsolid(0); // interior
        
        // Snow intake path
        hull() { 
            for (inlet=[0,100]) translate([inlet,0,0])
                housing3Dsolid(0,with_bearing=0,with_chute=0);
        }
        
        // Snow chute path
        // for (out=[0,+housing_wall*2]) translate([out,0,0])  //<- 3 sided, for access
        if (1) {
            housing3Dsolid(0,with_bearing=0);
        } 
        
        // Assembly boltholes        
        houseboltcenters() {
             cylinder(d=houseboltTap,h=2*houseboltL,center=true);
             cylinder(d=houseboltThru,h=2*(houseboltZ-houseboltCutZ),center=true);
             cylinder(d=houseboltHead,h=20);
        }
        
        // Trim bottom flat to ground
        translate([0,-100-augerD/2-auger_ground_clearance,0]) cube(200*[1,1,1],center=true);
    }
}

// Cutting plane for housing endcap separation
module housingcut(top=0) {
    translate([0,0,houseboltCutZ]) rotate([top*180,0,0])
        translate([0,0,-200]) cube([400,400,400],center=true);
}

// Endcap bolts on
module housing3Dendcap() {
    difference() {
        housing3Deject();
        housingcut();
    }
}

// Main portion
module housing3D() {
    intersection() {
        housing3Deject();
        housingcut();
    }
}

// Link to main arm
module auger_arm_link() {
    round=
}

// Demo of full assembly
module demo_snowblower() {
    auger3D();
    housing3D();
    //housing3Dendcap();
}


//rotate([90,0,0]) 
if (0) { // demonstration of assembly
    demo_snowblower();
    //gear_demo();
}
else 
{ // printable parts:
    printscale=1.003; // compensate for 0.3% post-print shrinkage
    scale(printscale*[1,1,1]) {
        //#auger_cylinder(); // Auger working volume
        
        //auger3D(); 
        
        auger_arm_link();
        //printable_smallgears();
        
        //translate([0,0,+houseboltCutZ]) rotate([180,0,0]) housing3D(); // main housing
        //translate([50,0,-houseboltCutZ]) housing3Dendcap(); // endcap

    }
}

