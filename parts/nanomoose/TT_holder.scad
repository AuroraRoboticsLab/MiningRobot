/*
Holds "TT" gearbox (small and cheap plastic gearbox)

90:1 ratio metal gear example:  https://www.amazon.com/dp/B099MX2J5B

Dr. Orion Lawlor, lawlor@alaska.edu, 2026-03-09 (Public Domain)
*/
$fs=0.1; $fa=2;
clearance=0.2; // as-printed clearance

// Drive shaft
TTshaftOD=5.43+clearance;
TTshaftflat=3.72+clearance; // across the flats
TTshaftZ=8.7+clearance;
TTshaftbackZ=9.3+clearance; // rear shaft is longer
TTshaftcollarOD=7.3+clearance;
TTshaftcollarZ=1.0+clearance;

// Indexing dot on same side as shaft
TTindexdotOD=4.0+clearance; // indexing dot
TTindexdotZ=2.0+clearance;
TTindexdotDX=-11.25;

// Thru holes for mounting M3 bolts
TTthruID=3.0+clearance; // thru holes (for M3)
TTthruDY=17.60/2; // centerline to thru distance
TTthruDX=TTindexdotDX-8.70;

// Front nub with M3 hole
TTnubID=3.0+clearance;
TTnubDX=14.0;
TTnubXY=5.0+clearance;
TTnubZ=3.0+clearance;
TTnubDZ=-10.2; // centerline below shaft face (not centered!)

// Overall outside box
TTboxDX=11.2;
TTboxY=22.6+clearance;
TTboxX=37.3+clearance;
TTboxZ=19.0+clearance;
TTboxR=5.0; // rounding radius on nub side

// Rear motor (bounding volume)
TTmotorOD=22.6+clearance;
TTmotorDX=-25.7; // start point
TTmotorSX=29.5+clearance; // size

// TT motor shaft
module TTshaft(enlarge=0)
{
    intersection() {
        cylinder(d=TTshaftOD+2*enlarge,h=TTshaftZ+enlarge);
        cube([TTshaftOD,TTshaftflat,2*TTshaftZ]+2*enlarge*[1,1,1],center=true);
    }
    
}

// Main square block body of gearbox
module TTmainbody2D(enlarge=0, with_bolts=0)
{
    offset(r=+enlarge)
    difference() {
        hull() {
            for (y=[-1,+1])
                translate([TTboxDX-TTboxR,y*(TTboxY/2-TTboxR)])
                    circle(r=TTboxR);
            translate([TTboxDX-TTboxX,-TTboxY/2])
                square([1,TTboxY]);
        }
        // bolt holes
        if (!with_bolts)
        for (y=[-1,+1]) 
            translate([TTthruDX,y*TTthruDY])
                circle(d=TTthruID);
    }
}

// Main square block body of gearbox
module TTmainbody(enlarge=0, with_bolts=0)
{
    translate([0,0,-TTboxZ/2])
    linear_extrude(height=TTboxZ+2*enlarge,convexity=6,center=true) 
        TTmainbody2D(enlarge=enlarge, with_bolts=with_bolts);
}

// Motor body itself
module TTmotorbody(enlarge=0, extra_motorY=0)
{
    translate([TTmotorDX+enlarge,0,-TTboxZ/2])
        rotate([0,-90,0])
        cylinder(d=TTmotorOD+2*enlarge,h=TTmotorSX+2*enlarge+extra_motorY);
}


// TT motor clearance volume.
//  Origin is center of top shaft
//  Shaft facing up along +Z
//  Motor off to -X
module TTmotorclear(enlarge=0, 
    with_bolts=0,bolt_extend=0,bolt_extraD=0,
    with_shaft=1,dual_shaft=0,with_motor=1,with_nub=1,
    extra_collarZ=0,extra_motorY=0,accurate=1)
{
    if (with_shaft) {
        for (sz=dual_shaft?[[-1,-TTboxZ],[+1,0]]:[[+1,0]]) 
        translate([0,0,sz[1]]) scale([1,1,sz[0]])
        if (accurate==0) {
            cylinder(d=TTshaftcollarOD+2*enlarge,h=TTshaftZ+enlarge);
        } else {
            TTshaft(enlarge=enlarge);
            
            // Small collar around base of shaft
            cylinder(d=TTshaftcollarOD+2*enlarge,h=TTshaftcollarZ+enlarge+extra_collarZ);
        }
    }
    
    if (with_motor) 
        TTmotorbody(enlarge=enlarge, extra_motorY=extra_motorY);
    
    // Index dot
    if (enlarge<=0) //<- omit on casing, for clearance
    translate([TTindexdotDX,0,0]) {
        cylinder(d=TTindexdotOD+2*enlarge,h=TTindexdotZ+enlarge);
    }
    
    // Main body box
    TTmainbody(enlarge=enlarge,with_bolts=with_bolts);
    
    // Front nub
    translate([TTnubDX,0,TTnubDZ])
    linear_extrude(height=TTnubZ+2*enlarge,convexity=4,center=true) 
    offset(r=+enlarge)
    difference() {
        round=1.6*accurate;
        offset(r=-round) offset(r=+round) {
            square([TTnubXY,TTnubXY],center=true);
            translate([TTboxDX-TTnubDX-1,0])
                square([2,2*TTnubXY],center=true);
        }
        
        // bolt holes
        if (!with_bolts)
                circle(d=TTnubID);
    }
    
    // Bolts extending from holes
    if (with_bolts)
    {
        for (t=[
            [TTthruDX,-TTthruDY,-TTboxZ/2],
            [TTthruDX,+TTthruDY,-TTboxZ/2],
            [TTnubDX,0,TTnubDZ],
            ]) 
            translate(t)
                cylinder(d=TTthruID+2*enlarge+bolt_extraD,h=TTboxZ+2*bolt_extend,center=true);
    }
}

// Example motor case: bottom half
//   Children are unioned with solid exterior before differencing
module TTmotor_case(wall=1.6) {
    difference() {
        // Exterior of case
        union() {
            TTmotorclear(enlarge=+wall,with_shaft=0,extra_motorY=-18);
            children();
        }
        
        // Carve motor cavity
        TTmotorclear(extra_collarZ=+wall,with_bolts=1,dual_shaft=1,bolt_extend=10,bolt_extraD=-0.5);

        // remove top half, so motor can drop down from the top
        translate([0,+100,0]) cube([200,200,200],center=true);
    }
}

// Rotate case to printable orientation
module TTmotor_printable(wall=1.6) {
    translate([0,0,TTboxY/2+wall]) rotate([90,0,0]) TTmotor_case(wall=wall) children();
}

//TTmotor_case();
//TTmotor_printable();


