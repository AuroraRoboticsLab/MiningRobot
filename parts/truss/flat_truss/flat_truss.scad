/*
 Geometric exploration for a 3D truss arch assembled from flat segments.
 
 Constraints:
    - Segments should stack together for shipping
    - Segments should primarily self-align for assembly
    - Robot-friendly joints

 Dr. Orion Lawlor, lawlor@alaska.edu, 2024-09-09 (Public Domain) 
*/
$fs=0.1; $fa=5;

inch=25.4; // file units are mm

scale=0.5; // scale for overall length of part

// Coupler via short steel pin
couplerOD = (2.9+0.3)/scale;
couplerwall=2/scale;
couplerZ = 10/scale; // length on both sides


// Vertex locations are where tube centerlines cross
trussXtop = 240; // origin-centerline distance along top
trussZ = 100; // top to bottom height

truss_angle = 18; // degrees rotation to next truss segment = 90 deg / 5 segments

truss_wall = 1.6;

trussXbot=trussXtop-trussZ*tan(truss_angle/2);

baseplate=2; // thickness of plate on front face

trimE=[0,-90+truss_angle/2,0]; // euler rotation for trim

couplerF=[trussXtop,0,0]; // center of front coupler
couplerB=[-trussXbot,0,-trussZ]; // center of back coupler

couplerE=[0,-90+truss_angle/2,0]; // euler rotation for couplers (positive on front, negative on back)




// Diameters of frame rods
//rodOD_big = 0.300*inch + 0.4;  // driveway marker rods, plus some clearance
//rodOD_diag = 3.3; // <- carbon fiber rods, plus minimal clearance

// scale-up versions
rodOD_big = 10;
rodOD_diag = 6; 


rodOD_top = rodOD_big; // top long rods
rodOD_bot = rodOD_big; // bottom long rods
rodOD_up = rodOD_big; // upright rods
rodOD_ldia = rodOD_diag; // long diagonals
rodOD_sdia = rodOD_diag; // short diagonals



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
setbackX = rodOD_big*0.5; // each side ends this early, to leave space for joint
TL = [ trussXtop-setbackX, 0,0 ];
TR = TL + [0,+8,0];
BL = [ trussXbot-setbackX, 0,-trussZ];
BR = BL + [0,-8,0];


bays = 2; // on each side, this many diagonal bays


// Mirror this 3D point around X axis, with setback
function flip(p) = [-p[0],p[1],p[2]];
function minusX(p) = [-p[0],p[1],p[2]];




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

// Compute the center of this top bay vertex
bayY = -rodOD_big/2 + rodOD_diag/2; // <- put diagonals flat on build plate
function topbay(b) = [b*(trussXtop-setbackX)/bays,bayY,0];
function botbay(b) = [b*(trussXbot-setbackX)/bays,bayY,-trussZ];
// Compute vertex v (0 or 1) of bay b (0..bays)
//function bay_vertex(b,v) = (b%2)?
//    ( v?topbay(b):botbay(b+1) ):
//    ( v?botbay(b):topbay(b+1) );
function bay_vertex(b,v) = (b%2)?
    ( v?botbay(b):topbay(b+1) ):
    ( v?topbay(b):botbay(b+1) );

// Go to each coupler location that has a pin: +Z faces into the pin
module coupler_pin_locations() {
    translate(couplerF) rotate(couplerE) children();
    
    translate(couplerB) rotate(-couplerE) children();
}

// Go to each coupler location that has a slot: +Z faces into slot
module coupler_slot_locations() {
    truss_advance(+1) 
        translate(couplerB) rotate(-couplerE) scale([1,1,-1]) children();
    
    truss_advance(-1) 
        translate(couplerF) rotate(couplerE) scale([1,1,-1]) children();
}

// Go to every coupler location
module coupler_all_locations() {
    coupler_pin_locations() children();
    coupler_slot_locations() children();
}
    
// Coupler holes
module coupler_holes(enlarge=0)
{
    e=enlarge*couplerwall/truss_wall;
    
    coupler_all_locations() {
        d=couplerOD+2*e;
        z=couplerZ+e;
        
        // main shaft
        cylinder(d=d, h=z);
        if (enlarge==0) scale([1,1,-1]) cylinder(d=d,h=z); // clearance
        
        // tapered end
        translate([0,0,z-0.01])
            cylinder(d1=d,d2=d*0.5,h=d*0.25);
        
        // tapered entrance
        if (enlarge==0) translate([0,0,-0.01]) {
            taper=1/scale;
            cylinder(d1=d+taper,d2=d,h=taper);
        }
    }
}
    
// Full set of truss tubes
module truss_tubes(enlarge=0,coupler=1)
{
    if (coupler) coupler_holes(enlarge);
    
    tube_list([flip(TL),TL],rodOD_top+2*enlarge);
    tube_list([flip(BL),BL],rodOD_bot+2*enlarge);

    truss_tubes_front(enlarge);
    truss_tubes_back(enlarge);
    
    // Even bays, starting from center
    for (b=[0:bays-1])
    {
        p=bay_vertex(b,0);
        n=bay_vertex(b,1);
        
        tube_list([p,n],rodOD_sdia+2*enlarge);
        tube_list([flip(p),flip(n)],rodOD_sdia+2*enlarge);
    }
    
}

// One generic truss vertex, centered at this point
module truss_vertex(p,wall=truss_wall,range=25) 
{
    difference() {
        union() {
            
            intersection() {
                translate(p) sphere(r=range);
                
                truss_tubes(wall);
            }
        }
        
        if (wall>0)
            truss_tubes();
    }
}

// One truss vertex, with webbed walls connecting the convex hull
module truss_vertex_webbed(p,range=25)
{
    truss_vertex(p,range=range);
    hull() {
        truss_vertex(p,wall=0,range=range);
        children();
    }
}


// Given three vertex locations, compute the center of the assembly bolt-nut line
function truss_coupler(p,pacross,pdiag,weight=0.9) 
    = p*weight+(1.0-weight)*pacross+(pdiag-pacross);

weightT=0.92;
weightB=0.92;


    
// Trim off the front face here
module truss_front_trim(trimdir=+1)
{
    translate([trussXtop,0,0]) rotate(trimE)
        translate([0,0,-500*trimdir]) cube([1000,1000,1000],center=true);
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
    
// Front vertex that has an assembly nut
module truss_vertex_front(p,pacross,pdiag,weight)
{
    difference() {
        union() {
            truss_vertex_webbed(p) {
                children(); //<- add any reinforcing here
            }
        }
        
        // Don't clog up tubes
        truss_tubes();
        
        // next truss mates in here
        
        // Trim flat to avoid interfering with next truss
        truss_front_trim();
    }
}

// The full set of front vertexes
module truss_vertexes_front()
{
    truss_vertex_front(TL,BL,BR,weightT) children();
    truss_vertex_front(BL,TL,TR,weightB) children();
}

// Back (flip) vertex that includes a bolt (captive?)
module truss_vertex_back(p,pacross,pdiag,weight)
{
    bolt_shaft=8; // length of bolt in this part

    fp=flip(p);
    difference() {
        union() {
            truss_vertex_webbed(fp) {
                children(); //<- add any reinforcing here
            }
        }
        
        // Don't clog up tubes
        truss_tubes();
        
        // Trim at the next truss's front face
        truss_advance(-1)
            truss_front_trim(-1);
    }
}

// The full set of back vertexes
module truss_vertexes_back()
{
    truss_vertex_back(TL,BL,BR,weightT) children();
    truss_vertex_back(BL,TL,TR,weightB) children();
}

// The full set of vertexes in the truss
module truss_vertexes()
{
    truss_vertexes_front();
    truss_vertexes_back();
    for (b=[0:bays-1]) {
        bayrange=15;
        p=bay_vertex(b,1);
        truss_vertex_webbed(p,range=bayrange);
        if (b>0) truss_vertex_webbed(flip(p),range=bayrange);
    }
}

// 3D printable front box
module printable_front_box(rot=0)
{
    rotate(-rot*couplerE) {
        truss_vertexes_front();
        truss_tubes_front(truss_wall);
    }
}

// 3D printable back box
module printable_back_box(rot=0)
{
    rotate(rot*couplerE) {
        difference() {
            union() {
                truss_vertexes_back();
                truss_tubes_back(truss_wall);
            }
            
            // Trim flat so it's printable
            truss_advance(-1) truss_front_trim(-1);
        }
    }
}


// Printable end tubes
module truss_printable() {
    rotate([90,0,0]) 
        scale([1,1,1]*scale) 
        difference() {
            truss_full();
            // trim bottom flat for printing
            startY=rodOD_big/2-0.5;
            translate([0,-1000-startY,0]) cube([2000,2000,2000],center=true);
        }
    //translate([0,0,trussXtop]) printable_front_box(1);
    //translate([0,0,trussXtop]) printable_back_box(1);
}

// Full truss
module truss_full() {
    difference() {
        truss_tubes(coupler=0);
        coupler_holes();
    }
    
    truss_vertexes();
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
    deltaY=15;
    rotate([90-4,0,0])
    for (y=[0:n-1]) {
        odd=0; // (z%2);
        translate([odd*3,y*deltaY])
        rotate([0,0,odd?180:0]) // flip every other truss
            truss_full();
    }
}

//truss_vertexes();
truss_printable();
//truss_lengths();

// Demo versions:
//truss_demo_mate(1);
//truss_demo_mate(0);
//truss_demo_fullarch();
//truss_demo_stack();

