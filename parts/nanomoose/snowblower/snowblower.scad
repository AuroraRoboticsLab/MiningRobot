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
 
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2026-03-12 (Public Domain)
*/
$fs=0.2; $fa=3;

include <AuroraSCAD/bearing.scad>;
include <AuroraSCAD/bevel.scad>;

inch=25.4; // actual inch
sinch=inch*1/10; // scaled inch

augerD = 14*sinch; // auger diameter
augerL = 28*sinch; // auger length (gets bearings and drive added on)
augerB = 1.5*sinch; // bevels at edges (less pointy, stronger housing)

// Overall outside limit of auger
module auger_cylinder(enlarge=0) {
    bevelcylinder(d=augerD+2*enlarge,h=augerL+2*enlarge,center=true,bevel=augerB+0.7*enlarge);
}

hexdrive_clear=0.25; // printed clearance around hex drive (fairly generous, so it can be slid around)
hexdrive_flats=1/4*inch; // 12.0+hexdrive_clear; // flat-to-flat diameter of hex drive shaft
hexdriveOD = hexdrive_flats/cos(30); // tip-to-tip diameter of hex drive shaft
hexdrive_out=1; // 1: hex plastic part facing out.  0: hex space through part

hexbearing_clearance=0.2;
hexbearing=bearing_608;
hexbearingID = bearingID(hexbearing)-hexbearing_clearance;
hexbearingOD = bearingOD(hexbearing)+hexbearing_clearance;
hexbearingL = bearingZ(hexbearing)+hexbearing_clearance;
augerBL = 2*hexbearingL+augerL; // auger length including end bearings

augerW=2.4; // wall thickness of auger parts
augerHR = hexdriveOD/2+augerW; // radius of plastic around hexdrive

augerIL = 6*sinch; // central spherical throw area: makes throw sphere bigger
augerOL = (augerL-augerIL)/2; // Auger outer helix length
augerOT = 180; // auger outside total twist

augerPR = 0.20*augerD; // snow-pitching radius
augerPX = 0.33*augerD; // snow-pitching X shift of radius circle
augerPY = 0; // snow-pitching Y shift of radius circle (rotate to make this always 0?)
augerPB = 0.1*augerD; // bevel rounding

// Put children at each auger thrower vane (typically 2-3)
module auger_vanes() {
    // for (side=[-1,+1]) scale([side,side,1])
    for (angle=[0:360/3:360-1]) rotate([0,0,angle])
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
module auger_ejectpath3D(Xscale=0.85, enlarge=0) {
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
                
                // Trim to avoid messing up the inside surface (hack!)
                hull() {
                    sphere(r=frontR+0.1*augerW); // front of pocket
                    translate([-0.1*augerD,-0.2*augerD,0]) 
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
                cylinder(d=hexdriveOD,h=augerBL/2+15,$fn=6);
            
                // Bearing axle areas on ends
                cylinder(d=hexbearingID,h=augerBL,center=true);
                // Bearing indexing
                cylinder(d=hexbearingID+3,h=housingL,center=true);
            }
        }
        
        // M3 tap down centerline (for strength, optional)
        cylinder(d=houseboltTap,h=2*augerL,center=true);
        
        // Clear ejection path for leaving snow
        auger_eject3D();
        
        // Thru hole for hex (in full size version)
        if (hexdrive_out==0) cylinder(d=hexdriveOD,center=true, h=augerBL+15,$fn=6);
    }
}

auger_ground_clearance=1.0; // clearance between ground and auger
auger_housing_clearance=1.0; // between auger and housing (in radius)
auger_housing_clearanceZ=1.0; // on ends

housing_wall=1.6;
housingID = augerD + 2*auger_housing_clearance;
housingL=augerL+2*auger_housing_clearanceZ;
housing_topY=8*sinch; // height of top front lip (snow cutting surface)

outletH = 18*sinch; // height of outlet above centerline
outletX=-0.3*augerD;
outletD=0.45*augerD; // diameter of snow outlet

// Mounting bolts on perimeter of housing, holding drive system (and allowing assembly)
houseboltThru = 3.1; // M3 thru
houseboltTap = 2.6; // M3 tap diameter
houseboltHead = 6.0; // head diameter (socket cap)
houseboltCutZ = housingL/2; // split Z plane, after edge of bevel
houseboltZ = houseboltCutZ + 3.0; // under center of bolt head
houseboltR = augerD/2+3.0; // centerline to centerline distance
houseboltL = 12; // max length of bolt
houseboltAngles = [-135,70,150];

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

// Full solid housing, with bearing mounts and eject path
module housing3Dsolid(enlarge=0, with_bearing=1, with_outlet=1) {
    bevel_extrude_convex(height=housingL+2*enlarge,center=true,bevel=augerB+0.5*enlarge)
        housing2D(enlarge=enlarge);
    
    if (with_bearing)
        cylinder(d=hexbearingOD+2*enlarge,h=housingL+2*hexbearingL -enlarge,center=true);
    
    // Snow outflow path
    if (with_outlet) 
    hull() {
        OD= outletD + 2*enlarge;
        exitH=1.5*OD;
        exitY=outletH-4*enlarge;
        exitA=0;
        for (o=[0,1]) 
            translate([outletX,o*exitY,0]) rotate([o*exitA,0,0]) bevelcylinder(d=OD,h=exitH,center=true,bevel=augerB+0.7*enlarge);
    }
}

// Housing with snow ejection paths
module housing3Deject() {
    difference() {
        union() {
            housing3Dsolid(housing_wall); // exterior

            // Hull to smooth transition to bearings
            hull() {
                housing3Dsolid(housing_wall,with_outlet=0);
                houseboltexteriors();
            }
        }
        housing3Dsolid(0); // interior
        
        // Snow intake path
        hull() { 
            for (inlet=[0,100]) translate([inlet,0,0])
                housing3Dsolid(0,with_bearing=0,with_outlet=0);
        }
        
        // Snow outlet path
        for (out=[0,+housing_wall*2]) translate([out,0,0]) {
            housing3Dsolid(0,with_bearing=0);
            translate([outletX,outletH,0]) 
                rotate([-90-45,0,0]) cylinder(d=outletD,h=outletD);
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

// Demo of full assembly
module demo_snowblower() {
    auger3D();
    housing3Deject();
}


//demo_snowblower();

//#auger_cylinder(); // Auger working volume

auger3D();

//housing3Deject(); // full housing
//translate([0,0,+houseboltCutZ]) rotate([180,0,0]) housing3D(); // main housing
//translate([50,0,-houseboltCutZ]) housing3Dendcap(); // endcap


