/*
 Geometric exploration for a 3D truss arch assembled from flat segments.
 This version joins trusses with pins and prongs.
 
 Constraints:
    - Segments should stack together for shipping
    - Segments should primarily self-align for assembly
    - Robot-friendly joints

 Vertexes:
    0 and 1: top endplates
    2 and 3: bottom endplates
    4: middle top diagonal link
    5 and 6: bottom diagonal links (identical)
  Only for odd edgestyle, with top diagonals
    7 and 8: top farside clip-on

 1          4           0
   3   5          6   2
    
    
Full size 10ft / 3m version:
ECHO: "Top long rods: ", 2994
ECHO: "Bot long rods: ", 2803.94
ECHO: "Uprights: ", 567.479
ECHO: "Bay diag: ", 911.616
ECHO: "Top diag: ", 911.616

Half size 5ft / 1.5m version:
ECHO: "Top long rods: ", 1494
ECHO: "Bot long rods: ", 1398.97
ECHO: "Uprights: ", 263.74
ECHO: "Bay diag: ", 431.574
ECHO: "Top diag: ", 431.574
    
    
 Dr. Orion Lawlor, lawlor@alaska.edu, 2024-09-09 (Public Domain) 
*/
$fs=0.1; $fa=5;

inch=25.4; // file units are mm

scale=1.0; // scale for overall length of part


// Diameters of frame rods
// Full-scale steel versions
rodOD_big = 34; // thin-wall steel greenhouse tubing (plus some clearance)
rodOD_diag = 18.3; // 1/2" EMT tubing (plus some clearance)


rodOD_top = rodOD_big; // top long rods
rodOD_bot = rodOD_big; // bottom long rods
rodOD_up = rodOD_diag; // upright rods (doubled)
rodOD_ldia = rodOD_diag; // long diagonals
rodOD_sdia = rodOD_diag; // short diagonals


// Vertex locations are where tube centerlines cross

// Full size 10ft / 3m truss:
trussXtop = 1500; // origin-centerline distance along top, between mating segments
trussZ = 600; // top to bottom height, centerline to centerline

// Half size 5ft / 1.5m truss:
//trussXtop = 1500/2; // origin-centerline distance along top, between mating segments
//trussZ = 600/2; // top to bottom height, centerline to centerline


trussY = trussZ; // side-to-side truss spacing, centerline to centerline

truss_angle = 18; // degrees rotation to next truss segment = 90 deg / 5 segments
truss_hangle = truss_angle/2; // half-angle is the angle of one sloped end of a truss

truss_wall = 1.6;

truss_clearance=0.1; // distance between two adjacent trusses

trussXbot=trussXtop-trussZ*tan(truss_hangle);

trimE=[0,-90+truss_hangle,0]; // euler rotation for trim


// Advance to next truss center
module truss_advance(dir=+1)
{
    translate([trussXtop*dir,0,0]) 
        rotate([0,truss_angle*dir,0])
            translate([trussXtop*dir,0,0])
                children();
}

// Advance to this n'th truss center
module truss_advanceN(n)
{
    if (n==0) children();
    if (n>0) truss_advance(+1) truss_advanceN(n-1) children();
    if (n<0) truss_advance(-1) truss_advanceN(n+1) children();
}

// Truss vertex locations
setbackX = 11; // each long tube ends this much before endplate (to leave space for joint)
// Mirror this 3D point around X axis, with setback
function flip(p) = [-p[0],p[1],p[2]];


TL = [ trussXtop-setbackX, 0,0 ];
BL = [ trussXbot-setbackX, 0,-trussZ];

// List of corner vertexes
vertexlist_corners = [TL,flip(TL),BL,flip(BL)];


bays = 2; // on each side, this many diagonal bays (currently must be even)




// Compute the center of this top bay vertex
bayX = 22; //<- push each diagonal rod end closer to each other
bayXend = 0.6*bayX; // <- inward push on end diagonal rods
bayY = 0; // -rodOD_big/2 + rodOD_diag/2; // <- can put diagonals flat on build plate
bayXtotalspan = trussXtop-setbackX-bayXend; // range for bays
function topbay(b) = [b*bayXtotalspan/bays,bayY,0];
function botbay(b) = [b*bayXtotalspan/bays,bayY,-trussZ];

// Compute centerpoints where bays meet
function bay_vertex_center(b) = (b%2)?
    ( botbay(b) ):
    ( topbay(b) );

// Compute end e (0 start or 1 end) of bay b (0..bays)
function bay_vertex(b,e) = ((b%2)?
    ( e?topbay(b+1):botbay(b) ):
    ( e?botbay(b+1):topbay(b) )) + [bayX,0,0]*(e?+1:-1);

// For top edge, this is the out-of-plane bay
bay_outreach = trussZ;//-rodOD_big/2; // center-to-center distance
function upright_bay(p) = [ p[0], -p[2]*bay_outreach/trussZ, p[1] ];

// For connecting edges, these diagonals connect to next truss
function shift_bay(p,shiftdir=+1) = [ p[0], shiftdir*trussY + p[1], p[2] ];

// For short edge, this is the center diagonal vertex
short_edge_bay = [0,-bay_outreach,-trussZ];


// List of possible diagonal vertexes
vertexlist_diag=[
    bay_vertex_center(0),
    bay_vertex_center(1),
    flip(bay_vertex_center(1)),
    
    // edgestyle%2==1, out of plane
    upright_bay(bay_vertex_center(1)),
    upright_bay(flip(bay_vertex_center(1))),
    
    
];




/* --------- endplates: where truss segments meet each other ------------ */
// This is the symmetry center of the truss end interface: 180 degree symmetry about this point
endplate_center = [(trussXtop+trussXbot)/2, 0, -trussZ/2];

// Distance from center to outside corner where truss segments meet

endplateY = 90; // Y thickness of endplates (matches nominal 2x4 inch lumber)
endplateZ = 90; // Z height of endplates
endplate_wall=truss_wall; // thickness of endplate
endplate_raiseZ=30; // height above centerline for corner

truss_top_plateX=30; // depth of plate that joins truss ends

truss_top_screwX=20; // distance from joint to screw hole
truss_top_screwOD=0.2*inch; // space for 10-24 shaft
truss_top_screwT=6; // taper on this screw entrance
truss_top_screwsR=32; // spacing between screws on top

// alignment cones on endplate faces, for assembly
truss_alignZ=2*endplate_raiseZ; // distance down from corner
truss_alignY=endplateY*0.25; // distance out from centerline
truss_alignOD=10; // diameter of tip
truss_alignID=5; // diameter of hole for tapping in retaining screw
truss_alignT=12; // taper
truss_alignC=0.5; // clearance when mated

// Screw holes in endplates, mostly for fixing to wood for testing
endplate_screwY=endplateY/2-10;
endplate_screwZ=[19,41,endplateZ-10];
endplate_screwOD=5; // space for a wood screw

module endplate_screwcenters() {
    for (y=[-1,+1]) for (z=endplate_screwZ)
    translate([0,y*endplate_screwY,z]) rotate([0,90,0]) children();
}



// Move to this end of this truss in the XZ plane:
//   flipX flips left-right.  FlipZ flips up-down.
// End coordinate system is centered on the outside corner, with +X facing in, +Y to the side, +Z facing down.
module endplate_shift(flipX=0,flipY=0,flipZ=0)
{
    scale([flipX?-1:1,1,1])
        translate(endplate_center) 
            rotate([0,truss_hangle,0]) // line up with truss mate angle
                scale([1,flipY?-1:1,flipZ?-1:1]) 
                    translate([0,0,(trussZ/2/cos(truss_hangle) + endplate_raiseZ)]) // shift up to outside corner
                        scale([-1,1,-1]) // +X and +Z face into truss
                            children();    
}

// This is the 2D XZ interface of the front top end of the truss.
//   Flipping and rotating makes the other dimensions from this.
module endplate_XZ() {
    round=4;
    offset(r=-round) offset(r=+round)
    {
        intersection() {
            // Base shape
            union() {
                circle(r=10); // big corner fillet
                
                square([endplate_wall,endplateZ]); // straight Z side
                rotate([0,0,truss_hangle])
                    square([truss_top_plateX,endplate_wall]); // top
                
                // bump out reinforcing lines
                rebar=endplate_wall*2;
                for (z=[endplate_raiseZ,endplateY-rebar/2])
                    translate([endplate_wall,z])
                        circle(d=rebar);
                 
                // bump out end of top plate
                rotate([0,0,truss_hangle])
                    translate([truss_top_plateX-rebar/2,endplate_wall])
                        circle(d=rebar);
            }
            
            // Trim front surface
            big=2*endplateZ;
            square([big,big]);
            // Trim top surface
            rotate([0,0,+truss_hangle]) 
                square([big,big]);
        }
    }
}

// This is the 2D YZ interface 
module endplate_YZ() {
    round=5;
    rebar=endplate_wall*2;
    offset(r=-round) offset(r=+round)
    {
        translate([-endplateY/2,0])
            square([endplateY,endplate_wall]);
        
        // Side rod reinforcing
        translate([0,setbackX]) circle(d=rodOD_diag+2*truss_wall);
        
        // Circular ends
        for (end=[-1,+1]) translate([(endplateY/2-rebar/2)*end,endplate_wall])
            circle(d=rebar);
    }
}

// Extrude XZ 3D truss end
module endplate_extrudeXZ() {
        rotate([90,0,0]) // rotate from XY to XZ plane
        linear_extrude(height=endplateY,center=true,convexity=4)
            children();
}
// Extrude YZ 3D truss end
module endplate_extrudeYZ() {
        rotate([0,90,0]) rotate([0,0,-90]) rotate([90,0,0]) 
        linear_extrude(height=endplateZ,convexity=4)
            children();
}

// Transform to match an alignment cone
module truss_align_cone_at() 
{
    translate([0,truss_alignY,truss_alignZ])
        rotate([0,90,0])
            children();
}

// Make a truss assembly alignment cone, in the endplate_shift coordinates
module truss_align_cone(enlarge=0)
{
    truss_align_cone_at() {
        cylinder(d1=truss_alignOD+2*truss_alignT+2*enlarge,
                 d2=truss_alignOD+2*enlarge, 
                 h=truss_alignT+enlarge);
        children();
    }
}

// Create positive truss ends: mounting tabs
module endplate_positive() {
    for (flipX=[0,1]) for (flipZ=[0,1])
        endplate_shift(flipX,flipX!=flipZ,flipZ) {
            // Truss endplate
            endplate_extrudeXZ() endplate_XZ();
            endplate_extrudeYZ() endplate_YZ();
            
            // Cone male (switched to a separate part that nominally bolts on)
            //scale([-1,-1,1]) truss_align_cone(0);
            // Cone female material behind hole
            truss_align_cone(truss_alignC + endplate_wall);
            
            // Bosses on endplate screws
            endplate_screwcenters() cylinder(d1=12,d2=8,h=3);
            
            // Material around joining screw
            rotate([0,-truss_hangle,0]) 
            translate([truss_top_screwX,0,0])
            {
                round=8;
                linear_extrude(height=endplate_raiseZ,convexity=4) 
                offset(r=-round) offset(r=+round)
                {
                    // under screw
                    circle(d=truss_top_screwOD+truss_top_screwT+2*truss_wall);
                    // along spine
                    square([2*truss_top_screwX,4],center=true);
                    // end of spine
                    translate([truss_top_screwX,0]) circle(d=round);
                }
                
                // Cone tapers around screws, for self-aligning
                tOD=16; // OD of taper cylinder
                tZ=8; // height of taper cylinder
                for (dY=[-1,0,+1]) translate([0,truss_top_screwsR*dY,0]) 
                difference() {
                    cylinder(d=tOD,h=tZ);
                    translate([0,0,truss_wall])
                    cylinder(d1=truss_top_screwOD,d2=tOD,h=tZ+0.1);
                }
            }
        }
}

// Create negative truss ends, with clearance for nut
module endplate_negative() {
    for (flipX=[0,1]) for (flipZ=[0,1])
        endplate_shift(flipX,flipX!=flipZ,flipZ) 
        {
            // space for next truss end in front of us
            scale([-1,1,1]) 
            difference() {
                extra=50; // extra space to flatten
                translate([-truss_clearance,-endplateY/2-extra,-extra]) 
                    cube([extra,endplateY+2*extra,endplateZ+2*extra]);
                
                // leave male cone in place
                //scale([1,-1,1]) truss_align_cone(0);
            }
            
            translate([-0.01,0,0]) 
            {
                // Cone male screw hole
                scale([-1,-1,1]) truss_align_cone_at() 
                    cylinder(d=truss_alignID,h=2*(truss_alignT+5),center=true);
            
                // Cone female space, and hole
                truss_align_cone(truss_alignC)
                    cylinder(d=truss_top_screwOD,h=truss_alignT+5);
            }
            
            endplate_screwcenters() cylinder(d=endplate_screwOD,h=10,center=true);
            
            // space for top joining plate
            rotate([0,-truss_hangle,0]) 
            {
                // Clearance on top, facing down
                scale([1,1,-1]) translate([0,-endplateY/2,0]) 
                    cube([truss_top_plateX,endplateY,endplateZ]);
                
                // Space for screw
                translate([truss_top_screwX,0,-0.01])
                {
                    OD = truss_top_screwOD;
                    taper = truss_top_screwT;
                    cylinder(d1=OD+2*taper,d2=OD,h=taper);
                    
                    for (dY=[-1,0,+1]) translate([0,truss_top_screwsR*dY,0]) {
                        cylinder(d=OD,h=(dY==0?50:10));
                    }
                }
            }
        }
}

/* ------- tubes: long non-printed tubes provide structural strength ----------- */

// Make this 3D vector have unit length
function normalize(p) = p * (1.0/norm(p));

/// Create a transform matrix so the Z axis is facing this way,
///   and S is the new origin.
module point_Z_axis_toward(Znew,S)
{
    Z=normalize(Znew); // direction (new Z)
    Y=normalize(cross(Z,[0.001,-0.001,1])); // up vector (new Y)
    X=normalize(cross(Y,Z)); // out vector (new X)
    m=[
        [X.x,Y.x,Z.x,S.x],
        [X.y,Y.y,Z.y,S.y],
        [X.z,Y.z,Z.z,S.z]
    ];
    multmatrix(m) children();
}

// Cut retaining pin holes at this centerline location
module tube_retaining_pins(OD,enlarge=0)
{
    rotate([0,0,45])
    for (angle=[[0,90,0],[90,0,0]]) rotate(angle)
        cylinder(d=truss_top_screwOD+2*enlarge, h=OD+5-0.1*enlarge,center=true);
}

// Retaining bolt holes on diagonal trusses
module truss_bolthole_diag(p,enlarge=0) {
    translate(p) rotate([0,90,0]) 
        for (a=[0,1]) translate([0,0,a?-12.5:+12.5])
            rotate([0,0,-45+(a?0:0)]) {
                tube_retaining_pins(rodOD_big,enlarge);
            }
}


// Create a tube between these two points
//   OD is diameter of tube
//   tube extends beyond the points by lengthen on both sides
module tube_between(p1,p2, ODraw, enlarge, retain, lengthen)
{
    OD = ODraw + 2*enlarge;
    point_Z_axis_toward(p2-p1,p1)
    translate([0,0,-lengthen-enlarge]) {
        len = norm(p2-p1)+2*lengthen+2*enlarge;
        cylinder(d=OD,h=len);
        if (ODraw<rodOD_big) {
            // Add closing tapered cylinders
            taper=rodOD_diag*0.4;
            scale([1,1,-1]) translate([0,0,-0.1]) cylinder(d1=OD,d2=OD-taper,h=taper/2);
            translate([0,0,len-0.1]) cylinder(d1=OD,d2=OD-taper,h=taper/2);
            
        }
        if (enlarge==0) 
        {
            // Cut retaining pin / rivet holes
            for (end=[+retain,len-retain]) translate([0,0,end])
                tube_retaining_pins(ODraw,enlarge);
        }
    }
}


// Draw a tube passing through these points
module tube_list(list,OD,enlarge,retain,lengthen)
{
    for (i=[0:len(list)-1]) 
        tube_between(list[i], list[(i+1)%len(list)], OD, enlarge,retain, lengthen);
}

// Tube list, then flipped
module tube_listF(pair,OD,enlarge,retain,lengthen)
{
    tube_list(pair,OD,enlarge,retain,lengthen);
    tube_list([flip(pair[0]),flip(pair[1])],OD,enlarge,retain,lengthen);
}

// Lengthen or (shorten) each end of these tubes
lengthen_long=8;
lengthen_side=-rodOD_big/2-3;
lengthen_diag=-(rodOD_big/2 + rodOD_diag/2)/cos(45);

// Distance from tube end for retaining pin holes
retain_long=61;
retain_side=35;
retain_diag=19; 
    
// Full set of truss tubes
module truss_tubes(enlarge=-0.01,outofplane=0,edgestyle=1,longonly=0,longextra=0)
{
    // Long top and bottom edges
    tube_list([flip(TL),TL],rodOD_top,enlarge,retain_long,lengthen_long+longextra);
    tube_list([flip(BL),BL],rodOD_bot,enlarge,retain_long,lengthen_long+longextra);
    
    if (outofplane!=0 && edgestyle%2==1) { // place where out-of-plane diagonals can grab tube
        tube_list([upright_bay(flip(BL)),upright_bay(BL)],rodOD_top,enlarge,retain_long,lengthen_long+longextra);
        
    }

    if (!longonly) {
        // Side diagonals
        tube_list([TL,BL],rodOD_up,enlarge,retain_side,lengthen_side);
        tube_list([flip(TL),flip(BL)],rodOD_up,enlarge,retain_side,lengthen_side);
        
        // Diagonal bays, starting out from center
        d = rodOD_sdia;
        for (b=[0:bays-1])
        {
            p=bay_vertex(b,0);
            n=bay_vertex(b,1);
            
            tube_listF([p,n],d,enlarge,retain_diag,lengthen_diag); // in-plane diagonals
            
            if (edgestyle%2==1) { // out-of-plane diagonals
                tube_listF([upright_bay(p),upright_bay(n)],d,enlarge,retain_diag,lengthen_diag);
            }
        }
        
        if (edgestyle==4) { // diagonal from short edge
            L = botbay(1);
            C = short_edge_bay;
            R = botbay(-1);
            tube_listF([L,C,R],d,enlarge,retain_diag,lengthen_diag);
        }
        
        if (edgestyle==3) { // end diagonals (on either end)
            tube_list([BL,shift_bay(TL,+1)],d,enlarge,retain_diag,lengthen_diag); 
            tube_list([TL,shift_bay(BL,-1)],d,enlarge,retain_diag,lengthen_diag); 
        }
    }
}


/* -------- vertexes: join the tubes ----------- */
// Create a plate to connect this pointlist
plate_thick=3;
module make_plate(pointlist)
{
    hull() intersection() {
        hull() union() {
            for (p=pointlist) translate(p) cube(plate_thick*[1,1,1],center=true);
        }
        children();
    }
}

// Diagonals plate 
module truss_diagonal_plate(edgestyle=1)
{
    make_plate(vertexlist_corners) children();
    
    if (edgestyle%2==1) { // out-of-plane diagonals
        expand=[50,0,0]; // need to reach beyond start point
        make_plate([TL,flip(TL),
             upright_bay(bay_vertex(bays-1,0))+expand,
        flip(upright_bay(bay_vertex(bays-1,0)))-expand]) children();
    }
}

// One generic truss vertex, centered at this point
module truss_vertex(p,wall=truss_wall,range=55,aspect=[1,1,1],edgestyle=1,longonly=0) 
{
    intersection() {
        translate(p) scale(aspect) sphere(r=range);
        
        truss_tubes(wall,1,edgestyle,longonly,12);
    }
}

// One truss vertex, with webbed walls connecting the convex hull of adjoining tubes.
//    end==1 on the endplates, 0 on the diagonal plates
//    range is the radius of the limiting sphere
//    aspect is the scale on the limiting sphere
module truss_vertex_webbed(p,end=0,range=65,aspect=[1,1,1],edgestyle=1)
{
    hollow_wall=truss_wall;
    difference() {
        union() {
            // Core walls around main tubes
            truss_vertex(p,range=range,aspect=aspect,edgestyle=edgestyle);
    
            // central blob hull connecting all parts
            difference() {
                wall=-rodOD_diag*0.05;
                r = 0.75*range;
                hull() truss_vertex(p,range=r,aspect=aspect,wall=wall,edgestyle=edgestyle);
                
                // Hollow, to reduce material when printed
                hull() truss_vertex(p,range=r-hollow_wall,aspect=aspect,wall=wall-hollow_wall,edgestyle=edgestyle);
            }        
            
            // Bosses around retaining bolt holes
            if (end==0) truss_bolthole_diag(p,truss_wall*1.5);
        }
        
        // Retaining bolts for tubes
        if (end==0) truss_bolthole_diag(p,0);
    }
    
    // Thicker ring around tube exits
    difference() {
        ring_OD=1.5*truss_wall;
        ring_deep=14/aspect[0];
        truss_vertex(p,range=range,aspect=aspect,wall=ring_OD,edgestyle=edgestyle);
        translate(p) scale(aspect) sphere(r=range-ring_deep);
    }
    
    // diagonal plate(s)
    truss_diagonal_plate(edgestyle=edgestyle)
        truss_vertex(p,wall=0,range=0.98*range,aspect=aspect,edgestyle=edgestyle);
    
    // Any other reinforcing goes in children
    children();
}
    
// Front or back vertex, that has an endplate
module truss_vertex_frontback(p,back=0,edgestyle=1)
{
    fp = back?flip(p):p;
    truss_vertex_webbed(fp,1,edgestyle=edgestyle) children();
}

// Reinforcing around front/back truss vertex
module truss_frontback_reinforcing_solid(enlarge)
{
    translate([0,0,endplate_raiseZ]) // <- move down to long edge intersection
    {
        // Long rod area 
        rotate([0,90,0]) {
            cylinder(d=rodOD_big*0.5+2*enlarge,h=50+enlarge); // narrow top
            cylinder(d=rodOD_big*1.1+2*enlarge,h=1); // big base
        }
        
        // These side wings, for side stability
        joinplate=6;
        translate([-joinplate/2,0,0])
            cube([joinplate+2*enlarge,endplateZ+0.7*enlarge,joinplate+2*enlarge],center=true);
        //rotate([90,0,0]) cylinder($fn=8,d=joinplate+2*enlarge,h=endplateZ-joinplate+enlarge,center=true);
    }
}

// Hollow reinforcing around front/back truss vertex
module truss_frontback_reinforcing_walls()
{
    // hull from long edges down, for endplate reinforcing
    difference() {
        hull() { // outside
            truss_frontback_reinforcing_solid(0);
        }
        hull() { // hollow inside
            w=-truss_wall;
            truss_frontback_reinforcing_solid(w);
        }
    }
}

// The full set of front & back vertexes
module truss_vertexes_frontback(edgestyle=1)
{
    for (flipX=[0,1]) {
        truss_vertex_frontback(TL,flipX,edgestyle=edgestyle) endplate_shift(flipX,0,0) truss_frontback_reinforcing_walls();
        truss_vertex_frontback(BL,flipX,edgestyle=edgestyle) endplate_shift(flipX,0,1) truss_frontback_reinforcing_walls();
    }
}

// The upright truss vertex that clamps the next row
module truss_vertex_uprightclamp(p,bayrange,aspect,edgestyle)
{
    difference() {
        union() {
            truss_vertex_webbed(p,range=bayrange,aspect=aspect,edgestyle=edgestyle);
            // Thicker walls for screws to hold from one side
            wall = 1.5*truss_wall;
            translate(p) scale(aspect) rotate([0,90,0]) cylinder(d=rodOD_big+2*wall,h=1.8*bayrange,center=true);
        }
        
        // Space for tube
        translate(p) rotate([0,90,0]) cylinder(d=rodOD_big,h=2*bayrange,center=true);
        
        // Leave holes in place
        truss_bolthole_diag(p,0);
        
        // Trim out a width, so we can fit over the long tube:
        //   spring_clamp 0.8, wall 1.5*1.6 => 1kgf snap-over force
        spring_clamp=0.8; // mm of spring needed to fit over tube
        trimR=rodOD_big*0.5-spring_clamp;
        translate(p+[0,trimR,0]) cube([150,2*trimR,2*trimR],center=true);
        
        // Bevel entrance on far side, so we easily slide over the tube
        bevelR = 0.65*rodOD_big; // trim cylinder
        translate(p+[0,0+bevelR,0]) rotate([0,90,0]) 
            cylinder(r=bevelR,h=150,center=true);
    }
}

// The full set of vertexes in the truss
module truss_vertexes(edgestyle=1)
{
    truss_vertexes_frontback(edgestyle=edgestyle);
    
    bayrange=55; // needs different coverage than endplates
    aspect=[0.75,1,1]; // squish bounding sphere for less long bar
    for (b=[0:bays-1]) {
        p=bay_vertex_center(b);
        //#translate(p) sphere(r=50);
        truss_vertex_webbed(p,range=bayrange,aspect=aspect,edgestyle=edgestyle);
        if (b>0) truss_vertex_webbed(flip(p),range=bayrange,aspect=aspect,edgestyle=edgestyle);
        if ((edgestyle%2==1) && (b%2)!=0) 
        { // uprights:
            up=upright_bay(p);
            for (p=[up,flip(up)])
                truss_vertex_uprightclamp(p,bayrange,aspect,edgestyle);
        }
    }
    
    if (edgestyle%8==4)
    { // short edge
        L = botbay(1);
        C = short_edge_bay;
        R = botbay(-1);
        for (p=[L,C,R])
            truss_vertex_webbed(p,range=bayrange,aspect=aspect,edgestyle=edgestyle);
    }
}

// Full truss vertexes, including all endplates
module truss_full_vertexes(edgestyle=1)
{
    difference() {
        union() {
            truss_vertexes(edgestyle=edgestyle);
            endplate_positive();
        }
        endplate_negative();
    }
}

// Full truss: vertexes, tubes, endplates
module truss_full(Yscale=1,tubeYshift=0,edgestyle=1) {
    difference() {
        union() {
            truss_vertexes(edgestyle=edgestyle);
            translate([0,tubeYshift,0]) scale([1,Yscale,1]) 
                truss_tubes(edgestyle=edgestyle);
            endplate_positive();
        }
        endplate_negative();
    }
    
}

// Printable full truss model, with printable tubes (for modeling only)
module truss_printable(edgestyle=1) {
    rotate([90,0,0]) 
        scale([1,1,1]*scale) 
        difference() {
            Yscale=1.0; // stretch Y to reach flat base while printing
            Yshift=2;
            truss_full(Yscale,-Yshift,edgestyle);
            // trim base flat for printing
            base_startY=endplateY/2;
            translate([0,-1000-base_startY,0]) cube([2000,2000,2000],center=true);
        }
    //translate([0,0,trussXtop]) printable_front_box(1);
    //translate([0,0,trussXtop]) printable_back_box(1);
}


// Print length between these two points
module truss_length_print(desc, p1, p2, lengthen=0)
{
    l = norm(p1-p2) + 2*lengthen;
    echo(desc,l);
}

// Print lengths of steel / composite rods needed (for manufacturing)
module truss_lengths() {
    truss_length_print("Top long rods: ",flip(TL),TL,lengthen_long);
    truss_length_print("Bot long rods: ",flip(BL),BL,lengthen_long);
    
    truss_length_print("Front upright: ",TL,BL,lengthen_side);
    truss_length_print("Back upright: ",flip(TL),flip(BL),lengthen_side);
    
    for (b=[0:bays-1]) {
        truss_length_print("Bay diag: ",bay_vertex(b,0),bay_vertex(b,1), lengthen_diag);
        truss_length_print("Top diag: ",upright_bay(bay_vertex(b,0)),upright_bay(bay_vertex(b,1)), lengthen_diag);
        
    }
    //truss_length_print("Front bot cross: ",BL,BR);
    //truss_length_print("Back diagonal cross: ",flip(TL),flip(BR));
}

// Show a cross section of the truss
module truss_demo_section(edgestyle=1) {
    difference() {
        truss_full_vertexes(edgestyle=edgestyle);
        truss_tubes(-0.01);
        translate([0,4000,0]) cube([8000,8000,8000],center=true);
    }
}

// Demo of mating surfaces
module truss_demo_mate(arch=1) {
    truss_full();

    if (arch) {
        #if (1) for (dir=[-1,+1]) truss_advance(dir) 
            truss_full();
    }
    else {
        #for (dir=[-1,+1]) translate([dir*(trussXtop+trussXbot),0,-trussZ]) rotate([180,0,0]) 
            truss_full();
    }
}

// Full demo of truss arch
module truss_demo_fullarch(len=3)
{
    for (n=[-len:+len]) truss_advanceN(n)
        truss_full();
    // Extra crossbar at start
    truss_advanceN(-len-1) {
        truss_vertexes_front();
        truss_tubes_front();
    }
}

// Demonstrates trusses stacked up for shipping
module truss_demo_stack(n=3)
{
    deltaX=0;
    deltaY=endplateY+1;
    deltaZ=25;
    rotate([90-4,0,0])
    for (y=[0:n-1]) {
        odd=(y%2);
        translate([odd*deltaX,y*deltaY,y*deltaZ])
        //rotate([0,0,odd?180:0]) // flip every other truss
            truss_full();
    }
}

// Demonstrates trusses arranged in a tower or box beam.
module truss_demo_tower(L=1) 
{
    for (stack=[0:2])
    translate([stack*(trussXtop+trussXbot),0,0])
    {
        flip=stack%2;
        rotate(flip*[0,180,0]) translate(flip*[0,0,trussZ])
        {
            translate ([0,L==0?trussZ:0,0]) truss_full(edgestyle=L); 
            translate([0,L==0?0:trussZ,-trussZ]) rotate([180,0,0]) truss_full(edgestyle=L);
        }
    }
}
// Demonstrates trusses arranged in a box arch
module truss_demo_boxarch(len=3) 
{
    for (n=[-len:+len]) truss_advanceN(n)
    {
        truss_full(edgestyle=1); 
        translate([0,trussZ,0]) //rotate([180,0,0]) 
            truss_full(edgestyle=0);
    }
}

// 6 trusses in an arc, plus one straight on each end
module truss_demo_8arch()
{
    rotate([0,-truss_hangle,0]) 
    {
        len=3;
        for (n=[-len+1:+len]) truss_advanceN(n)
            truss_full(edgestyle=1); 
        
        // Back straight segment
        truss_advanceN(-len+1) {
            translate([-(trussXtop+trussXbot),0,-trussZ]) 
            rotate([180,0,0]) rotate([0,0,180])
                truss_full(edgestyle=0);
        }
        
        // Front straight segment
        truss_advanceN(+len) {
            translate([+(trussXtop+trussXbot),0,-trussZ]) 
            rotate([180,0,0]) rotate([0,0,180])
                truss_full(edgestyle=0);
        }        
    }
}


cornerv=4; // count of corner vertexes

// Vertex sitting on a printable cut plane
module truss_vertex_printable(v=0,edgestyle=1)
{
    rotlist=[
        90-truss_hangle,
        -90+truss_hangle,
        90-truss_hangle,
        -90+truss_hangle,
    ];
    p=(v<cornerv)?vertexlist_corners[v]:vertexlist_diag[v-4]; // center point
    r=(v<cornerv)?rotlist[v]:90; // Y axis rotation
    trimZ=(v<cornerv)?30:34; // Z height of bottom trim plane
    
    translate([0,0,trimZ])
    intersection() {
        rotate([0,r,0])
        //rotate([90,0,0])
        translate(-p)
        difference() {
            truss_full_vertexes(edgestyle=edgestyle);
        
            truss_tubes(edgestyle=edgestyle);
        }
        translate([0,0,200-trimZ]) cube([400,400,400],center=true);
    }
}

// Set of printable vertexes
module truss_vertex_printable_set(vlist, edgestyle=1)
{
    locs=[
        [-40,0],
        [+40,0],
        [-70,100],
        [+70,100],
        [20,100]
    ];
    for (v=vlist) translate((v<cornerv)?locs[v]:locs[v-cornerv])
        truss_vertex_printable(v,edgestyle=edgestyle);
}

// Computer aided manufacturing (CAM) outputs:
edgestyle=1;
truss_tubes(-0.1,edgestyle=edgestyle);
truss_full_vertexes(edgestyle=edgestyle);
//truss_vertex_printable(5,edgestyle=edgestyle);
//truss_vertex_printable_set([0,1,2,3], edgestyle=edgestyle);
//truss_vertex_printable_set([4,5,6,7,8], edgestyle=edgestyle);

//truss_printable();
truss_lengths();

// Demo versions:
//truss_full(edgestyle=edgestyle);
//truss_demo_section(edgestyle=1);
//truss_demo_tower(0);
//truss_demo_mate(1);
//truss_demo_mate(0);
//truss_demo_fullarch(); translate([0,trussZ,0]) truss_demo_fullarch();
//truss_demo_boxarch();
//truss_demo_8arch();
//truss_demo_stack();

