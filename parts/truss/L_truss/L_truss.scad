/*
 Geometric exploration for a 3D truss arch assembled from flat segments.
 This version joins trusses with pins and prongs.
 
 Constraints:
    - Segments should stack together for shipping
    - Segments should primarily self-align for assembly
    - Robot-friendly joints

 Dr. Orion Lawlor, lawlor@alaska.edu, 2024-09-09 (Public Domain) 
*/
$fs=0.1; $fa=5;

inch=25.4; // file units are mm

scale=0.5; // scale for overall length of part


// Diameters of frame rods
//rodOD_big = 0.300*inch + 0.4;  // driveway marker rods, plus some clearance
//rodOD_diag = 3.3; // <- carbon fiber rods, plus minimal clearance

// scale-up versions
rodOD_big = 10;
rodOD_diag = 6; 


rodOD_top = rodOD_big; // top long rods
rodOD_bot = rodOD_big; // bottom long rods
rodOD_up = rodOD_diag; // upright rods (doubled)
rodOD_ldia = rodOD_diag; // long diagonals
rodOD_sdia = rodOD_diag; // short diagonals


// Coupling happens via M3 screws tapped in
couplerID = 2.4/scale;
couplerOD = 3.1/scale;
couplerwall=2/scale;
couplertaper=2/scale;
couplerZ = 10/scale; // length embedded into vertex


// Vertex locations are where tube centerlines cross
trussXtop = 230; // origin-centerline distance along top, between mating segments
trussZ = 100; // top to bottom height

truss_angle = 18; // degrees rotation to next truss segment = 90 deg / 5 segments
truss_hangle = truss_angle/2; // half-angle is the angle of one sloped end of a truss

truss_wall = 1.6;

truss_clearance=0.1; // distance between two adjacent trusses

trussXbot=trussXtop-trussZ*tan(truss_hangle);

baseplate=2; // thickness of plate on front face

trimE=[0,-90+truss_hangle,0]; // euler rotation for trim

couplerF=[trussXtop,0,0]; // center of front coupler
couplerB=[-trussXbot,0,-trussZ]; // center of back coupler

couplerE=[0,-90+truss_hangle,0]; // euler rotation for couplers (positive on front, negative on back)

// This is the symmetry center of the truss end interface: 180 degree symmetry about this point
truss_end_center = [(trussXtop+trussXbot)/2, 0, -trussZ/2];
// Same thing in XZ plane
truss_end_centerXZ = [truss_end_center[0],-truss_end_center[2],0];

truss_endZ = 16; // Z thickness of end prongs
truss_end_prong = truss_endZ; // end prong sticks out this far
truss_endY = 5; // end prong Y thickness
truss_end_support=20; // rounded area behind truss end
truss_end_raiseY=rodOD_big/2; // height above centerline for prong start

// Move to this end of this truss in the XZ plane
module truss_end_shiftXZ(flipX=0,flipZ=0)
{
    rotate([-90,0,0]) 
        translate(flipX?flip(truss_end_centerXZ):truss_end_centerXZ) 
            rotate(flipX*[0,0,180-truss_angle])
                rotate(flipZ*[0,0,180])
                        children();    
}

// This is the 2D XZ interface of the front top end of the truss.
//   Flipping and rotating makes the other dimensions from this.
module truss_end_XZ(Ythick=truss_endY,Xshift=truss_clearance) {
    rotate([0,0,truss_hangle])
    translate((trussZ/2 + truss_end_raiseY)*[0,1])
    {
        translate([Xshift,0])
        scale([1,-1]) intersection() {
            circle(r=truss_end_support);
            square([truss_end_support,truss_end_support]);
            rotate([0,0,+truss_hangle]) // trim back corner
                square([truss_end_support,truss_end_support]);
        }
        
        rotate([0,0,-truss_hangle])
            translate([0,Ythick/2])
                square([2*truss_end_prong,Ythick],center=true);
    }
}

// Extrude 3D truss end, and add mounting bolt support
module truss_end_extrude(add=1) {
    // Mounting bolt centerline
    boltcenter=(trussZ/2 + truss_end_raiseY)*[-sin(truss_hangle),cos(truss_hangle)]-[truss_end_prong/2,0];
    boltrot=[90,0,0]; // tilt angle for mounting bolt

    difference() {
        linear_extrude(height=truss_endZ,center=true,convexity=4)
            children();
        
        // Add hole through tab
        if (add) translate(boltcenter) rotate(boltrot) 
            cylinder(d=couplerOD,h=3*truss_endY,center=true);
    }
    
    // Subtract thread tap space
    if (add==0) translate(boltcenter) rotate(boltrot) {
        cylinder(d=couplerID,h=couplerZ);
        // tapered cylinder to guide screw or tap in
        taper=couplertaper;
        cylinder(d1=taper+couplerID,d2=couplerID,h=taper);
    }
}

// Create positive truss ends: mounting tabs
module truss_end_positive() {
    for (flipX=[0,1])
        truss_end_shiftXZ(flipX,1) truss_end_extrude(1) truss_end_XZ();
}

// Create negative truss ends, with clearance for nut
module truss_end_negative() {
    clearance=truss_clearance;
    for (flipX=[0,1])
        truss_end_shiftXZ(flipX,0) truss_end_extrude(0) offset(r=clearance) truss_end_XZ(2*truss_endY,0);
}



// Tap plastic for M3 assembly screw at this diameter
M3_tapID = 2.3;
M3_shaftOD = 3.1;
M3_thru = 3.3;
M3_headOD=7; // head of M3, plus clearance



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
setbackX = rodOD_big*0.6; // each side ends this early, to leave space for joint
TL = [ trussXtop-setbackX, 0,0 ];
TR = TL + [0,+8,0];
BL = [ trussXbot-setbackX, 0,-trussZ];
BR = BL + [0,-8,0];


bays = 2; // on each side, this many diagonal bays


// Mirror this 3D point around X axis, with setback
function flip(p) = [-p[0],p[1],p[2]];
function minusX(p) = [-p[0],p[1],p[2]];



// Compute the center of this top bay vertex
bayY = -2; // -rodOD_big/2 + rodOD_diag/2; // <- put diagonals flat on build plate
function topbay(b) = [b*(trussXtop-setbackX)/bays,bayY,0];
function botbay(b) = [b*(trussXbot-setbackX)/bays,bayY,-trussZ];
// Compute vertex v (0 or 1) of bay b (0..bays)
//function bay_vertex(b,v) = (b%2)?
//    ( v?topbay(b):botbay(b+1) ):
//    ( v?botbay(b):topbay(b+1) );
function bay_vertex(b,v) = (b%2)?
    ( v?botbay(b):topbay(b+1) ):
    ( v?topbay(b):botbay(b+1) );
    
// For long edge, this is the out-of-plane bay
bay_outreach = trussZ-rodOD_big/2;
function upright_bay(p) = [ p[0], -p[2]*bay_outreach/trussZ, p[1] ];

// For short edge, this is the center diagonal vertex
short_edge_bay = [0,-bay_outreach,-trussZ];





// Draw a tube passing through these points
module tube_list(list,OD,end=0)
{
    fn=8;
    for (i=[0:end?end:len(list)-2]) 
    hull() {
        translate(list[i]) sphere($fn=fn,d=OD);
        translate(list[(i+1)%len(list)]) sphere($fn=fn,d=OD);
    }
}

// Tube list, then flipped
module tube_listF(pair,OD)
{
    tube_list(pair,OD);
    tube_list([flip(pair[0]),flip(pair[1])],OD);
}

module tube_list_closed(list,OD) {
    tube_list(list,OD,len(list)-1);
}

// Front set of truss tubes only
module truss_tubes_front(enlarge=0)
{
    tube_list([TL,BL],rodOD_up+2*enlarge);
}
// Back set of truss tubes only
module truss_tubes_back(enlarge=0)
{
    tube_list([flip(TL),flip(BL)],rodOD_up+2*enlarge);
}
    
// Full set of truss tubes
module truss_tubes(enlarge=0,longedge=1)
{
    
    tube_list([flip(TL),TL],rodOD_top+2*enlarge);
    tube_list([flip(BL),BL],rodOD_bot+2*enlarge);

    truss_tubes_front(enlarge);
    truss_tubes_back(enlarge);
    
    // Diagonals
    d = rodOD_sdia+2*enlarge;
    
    // Bays, starting from center
    for (b=[0:bays-1])
    {
        p=bay_vertex(b,0);
        n=bay_vertex(b,1);
        
        tube_listF([p,n],d); // in-plane diagonals
        
        if (longedge) { // out-of-plane diagonals
            tube_listF([upright_bay(p),upright_bay(n)],d);
        }
    }
    
    if (!longedge) {
        L = botbay(1);
        C = short_edge_bay;
        R = botbay(-1);
        tube_listF([L,C,R],d);
    }
    
}

// One generic truss vertex, centered at this point
module truss_vertex(p,wall=truss_wall,range=25,longedge=1) 
{
    difference() {
        union() {
            
            intersection() {
                translate(p) sphere(r=range);
                
                truss_tubes(wall,longedge);
            }
        }
        
        //if (wall>0)
        //    truss_tubes();
    }
}

// One truss vertex, with webbed walls connecting the convex hull
module truss_vertex_webbed(p,end=0,extraweb=0,range=20,longedge=1)
{
    truss_vertex(p,range=1.1*range,longedge=longedge);
    hull() {
        truss_vertex(p,wall=0,range=range,longedge=longedge);
        if (end) {// reinforcing around mount bolt
            translate(p) rotate([90,0,0]) cylinder(d=16+2*extraweb,h=truss_endZ,center=true);
        }
        children();
    }
}

    
// Trim off the front face here
module truss_front_trim(trimdir=+1)
{
    translate([trussXtop,0,0]) rotate(trimE)
        translate([0,0,(-500+truss_clearance)*trimdir]) cube([1000,1000,1000],center=true);
}

// Project 3D point onto the YZ plane.
function projectYZ(p,henlarge) = [ p[1]+henlarge, p[2] ];

// Y-Z cross section of truss, enlarged in horizontal axis
module truss_sectionYZ(henlarge=0)
{
    polygon([
        projectYZ(TL,henlarge),projectYZ(TR,-henlarge),
        projectYZ(BR,-henlarge), projectYZ(BL,henlarge),
    ]);
}
    
// Front or back vertex, that has an assembly nut
module truss_vertex_frontback(p,back=0,extraweb=0,longedge=1)
{
    fp = back?flip(p):p;
    difference() {
        union() {
            truss_vertex_webbed(fp,1,extraweb,longedge=longedge) {
                children(); //<- add any reinforcing here
            }
        }
        
        // Don't clog up tubes
        //truss_tubes();
        
        // Trim flat to avoid interfering with next truss
        if (back) {
            truss_advance(-1)
                truss_front_trim(-1);
        } else {
            truss_front_trim();
        }
    }
}

// The full set of front vertexes
module truss_vertexes_front(longedge=1)
{
    truss_vertex_frontback(TL,0,0,longedge=longedge) children();
    truss_vertex_frontback(BL,0,2,longedge=longedge) children();
}

// The full set of back vertexes
module truss_vertexes_back(longedge=1)
{
    truss_vertex_frontback(TL,1,4,longedge=longedge) children();
    truss_vertex_frontback(BL,1,0,longedge=longedge) children();
}

// The full set of vertexes in the truss
module truss_vertexes(longedge=1)
{
    truss_vertexes_front(longedge=longedge);
    truss_vertexes_back(longedge=longedge);
    
    bayrange=15;
    for (b=[0:bays-1]) {
        p=bay_vertex(b,1);
        truss_vertex_webbed(p,range=bayrange,longedge=longedge);
        if (b>0) truss_vertex_webbed(flip(p),range=bayrange,longedge=longedge);
        if (longedge && (b%2)!=0) {
            up=upright_bay(p);
            truss_vertex_webbed(up,range=bayrange,longedge=longedge);
            truss_vertex_webbed(flip(up),range=bayrange,longedge=longedge);
        }
    }
    
    if (longedge==0)
    { // short edge
        L = botbay(1);
        C = short_edge_bay;
        R = botbay(-1);
        for (p=[L,C,R])
            truss_vertex_webbed(p,range=bayrange,longedge=longedge);
    }
}

// Full truss
module truss_full(Yscale=1,tubeYshift=0,longedge=1) {
    difference() {
        union() {
            truss_vertexes(longedge=longedge);
            translate([0,tubeYshift,0]) scale([1,Yscale,1]) 
                truss_tubes(longedge=longedge);
            truss_end_positive();
        }
        truss_end_negative();
    }
    
}

// Printable end tubes
module truss_printable(longedge=1) {
    rotate([90,0,0]) 
        scale([1,1,1]*scale) 
        difference() {
            Yscale=1.0; // stretch Y to reach flat base while printing
            Yshift=2;
            truss_full(Yscale,-Yshift,longedge);
            // trim base flat for printing
            base_startY=truss_endZ/2;
            translate([0,-1000-base_startY,0]) cube([2000,2000,2000],center=true);
        }
    //translate([0,0,trussXtop]) printable_front_box(1);
    //translate([0,0,trussXtop]) printable_back_box(1);
}


// Print length between these two points
module truss_length_print(desc, p1, p2)
{
    l = norm(p1-p2);
    echo(desc,l);
}

// Print lengths of composite rods needed
module truss_lengths() {
    truss_length_print("Top long rods: ",flip(TL),TL);
    truss_length_print("Bot long rods: ",flip(BL),BL);
    truss_length_print("Bot diagonal: ",flip(BL),BR);
    truss_length_print("Up diagonal: ",flip(BL),TL);
    truss_length_print("Front uprights: ",TL,BL);
    truss_length_print("Front top cross: ",TL,TR);
    truss_length_print("Front bot cross: ",BL,BR);
    truss_length_print("Back diagonal cross: ",flip(TL),flip(BR));
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
module truss_demo_stack(n=5)
{
    deltaY=20;
    deltaZ=10;
    rotate([90-4,0,0])
    for (y=[0:n-1]) {
        odd=0; // (z%2);
        translate([odd*3,y*deltaY,(y%2)*deltaZ])
        rotate([0,0,odd?180:0]) // flip every other truss
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
            translate ([0,L==0?trussZ:0,0]) truss_full(longedge=L); 
            translate([0,L==0?0:trussZ,-trussZ]) rotate([180,0,0]) truss_full(longedge=L);
        }
    }
}
// Demonstrates trusses arranged in a box arch
module truss_demo_boxarch(len=3) 
{
    for (n=[-len:+len]) truss_advanceN(n)
    {
        truss_full(longedge=1); 
        translate([0,trussZ,0]) //rotate([180,0,0]) 
            truss_full(longedge=0);
    }
}

// 6 trusses in an arc, plus one straight on each end
module truss_demo_8arch()
{
    rotate([0,-truss_hangle,0]) 
    {
        len=3;
        for (n=[-len+1:+len]) truss_advanceN(n)
            truss_full(longedge=1); 
        
        // Back straight segment
        truss_advanceN(-len+1) {
            translate([-(trussXtop+trussXbot),0,-trussZ]) 
            rotate([180,0,0]) rotate([0,0,180])
                truss_full(longedge=0);
        }
        
        // Front straight segment
        truss_advanceN(+len) {
            translate([+(trussXtop+trussXbot),0,-trussZ]) 
            rotate([180,0,0]) rotate([0,0,180])
                truss_full(longedge=0);
        }        
    }
}

//truss_vertexes();
//truss_printable();
//truss_lengths();

// Demo versions:
//truss_full(longedge=0);
//truss_demo_tower(0);
//truss_demo_mate(1);
//truss_demo_mate(0);
//truss_demo_fullarch(); translate([0,trussZ,0]) truss_demo_fullarch();
//truss_demo_boxarch();
truss_demo_8arch();
//truss_demo_stack();

