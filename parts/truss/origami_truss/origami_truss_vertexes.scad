/*
 Geometric exploration for a 3D truss that unfolds like an origami pattern,
 the Yoshimura diamond pattern.
 
 Constraints:
    Entire structure can fold flat
    Long tubes are valley folds, open to 60 degree angle
    
 
    
 Dr. Orion Lawlor, lawlor@alaska.edu, 2024-10-01 (Public Domain) 
*/
//$fs=0.1; $fa=5; // fine detail
$fs=0.5; $fa=10; // coarse lower poly version

inch=25.4; // file units are mm

scale=1.0; // scale for overall length of part

// Graphics mode:
//  0 is for 3D printing or CAM
//  1 is for rendering--suppresses internal features
graphics_mode=0;



truss_wall=1.6; // thickness of walls
truss_clearance=0.3; // extra diameter on moving parts


// Diameters of frame rods
// Full-scale steel versions
//rodOD_long = 34; // thin-wall steel greenhouse tubing (plus some clearance)
//rodOD_diag = 18.3; // 1/2" EMT tubing (plus some clearance)

// Scale model from driveway marker rods
rodOD_driveway = 0.300*inch+0.3;
rodOD_long = rodOD_driveway;
rodOD_diag = rodOD_driveway;

rodOD_top = rodOD_diag; // top rods connect valley ends
rodOD_support = rodOD_diag; // connect to valley middle


// Vertex locations are where tube centerlines cross

// Full size 10ft / 3m truss:
trussXlong = 150; // half-length of long rods

trussZ = 50; // top to bottom folded height, centerline to centerline

trussTX = 15; // X shift back where top meets long
trussTY = 0; // Y shift where top meets long
trussTZ = rodOD_top/2+rodOD_long/2+1; // Z shift where top meets long

trussSZ = 15; // Z shift where support meets long

trussDY = 20; // folded-up stack distance along Y

truss_unfold = 60; // valley folds open this far

truss_angle = 75; // angle between adjacent folded trusses (DIAL THIS IN)


// Advance to next truss center along long edge
module truss_advance(dir=+1)
{
    translate([trussXlong*dir,0,0]) 
        rotate([0,truss_angle*dir,0])
            translate([trussXlong*dir,0,0]) 
                    children();
}

// Shift to the next (+1) or last (-1) row of trusses
module truss_shift(dir=+1)
{
    translate([0,trussDY*dir,trussZ])
        rotate([0,truss_angle/2*dir,0])
            translate([trussXlong*dir,0,0])
                children();
}

// Advance to this n'th truss center
module truss_advanceN(n,dY=0)
{
    if (dY==0) {
        if (n==0) children();
        if (n>0) truss_advance(+1) truss_advanceN(n-1) children();
        if (n<0) truss_advance(-1) truss_advanceN(n+1) children();
    }
    if (dY>0) truss_shift(+1) truss_advanceN(n,dY-1) children();
    if (dY<0) truss_shift(-1) truss_advanceN(n,dY+1) children();
}

// Truss vertex locations
setbackX = 6; // each long tube ends this much before joint bolt

// Mirror this 3D point around X and Y axes
function flip(p) = [-p[0],p[1],p[2]];


// Long endpoints
FL = [ trussXlong-setbackX, 0,0 ]; // front long
BL = flip(FL); // back long

// Top middle
MC = [0,0,trussZ];

// Top endpoints
FC = MC+ [ 0, trussTY, 0 ]; // front top center
FT = FL + [-trussTX,trussTY,trussTZ]; // front top

BC = flip(FC); // back top center
BT = flip(FT); // back top

// Supports
SC = MC + [0,0,-6]; // support center
SL = [ SC[0], SC[1], 10 ];


truss_top_screwOD = 0.195*inch; // #10-24 machine screws


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
/*
    rotate([0,0,45])
    for (angle=[[0,90,0],[90,0,0]]) rotate(angle)
        cylinder(d=truss_top_screwOD+2*enlarge, h=OD+5-0.1*enlarge,center=true);
*/
}

// Retaining bolt holes on diagonal trusses
module truss_bolthole_diag(p,enlarge=0) {
    translate(p) rotate([0,90,0]) 
        for (a=[0,1]) translate([0,0,a?-12.5:+12.5])
            rotate([0,0,-45+(a?0:0)]) {
                tube_retaining_pins(rodOD_long,enlarge);
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
        if (graphics_mode==0 && ODraw<rodOD_long) {
            // Add closing tapered cylinders
            taper=rodOD_diag*0.4;
            scale([1,1,-1]) translate([0,0,-0.1]) cylinder(d1=OD,d2=OD-taper,h=taper/2);
            translate([0,0,len-0.1]) cylinder(d1=OD,d2=OD-taper,h=taper/2);
            
        }
        if (graphics_mode==0 && enlarge==0) 
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
lengthen_long=0;
lengthen_top=-3;
lengthen_support=0;

// Distance from tube end for retaining pin holes
retain_long=61;
retain_top=35;
retain_support=15; 
    
// Full set of truss tubes
module truss_tubes(enlarge=-0.01,outofplane=0,edgestyle=1,longextra=0)
{
    valleyOD = rodOD_long+((edgestyle>0)?truss_clearance:0);

    // Long valley edge
    tube_list([FL,BL],valleyOD,enlarge,retain_long,lengthen_long+longextra);

    if (edgestyle>0)
    {
        // Top diagonal edges
        tube_list([FC,FT],rodOD_top, enlarge,retain_top,lengthen_top);
        tube_list([BC,BT],rodOD_top, enlarge,retain_top,lengthen_top);
        
        // Short vertical support
        tube_list([SC,SL],rodOD_support, enlarge, retain_support,lengthen_support);
    }
}

endplate_center = [trussXlong,0,0];

module endplate_shift(flipX=0,flipY=0,flipZ=0)
{
    scale([flipX?-1:1,1,1])
        translate(endplate_center)
            rotate([90,0,0])
                scale([1,1,flipZ?-1:1])
                    children();
}

endplate_ringTY = rodOD_long/2+truss_wall/2; // thickness of end rings
endplate_ringR = 5; // radius of end rings
module endplate_positive() {
    for (flipX=[0,1]) 
        endplate_shift(flipX,0,flipX)
        {
            T = 4; // thickness of end rings
            translate([0,0,0.2+endplate_ringTY/2])
            hull() {
                cylinder(d=2*endplate_ringR,h=endplate_ringTY,center=true);
                translate([-15,0,0]) cube([endplate_ringTY,endplate_ringR*2,endplate_ringTY],center=true);
            }
        }
}
module endplate_negative() {
    for (flipX=[0,1]) 
        endplate_shift(flipX,0,flipX)
        {
            cylinder(d=3.2,h=12,center=true);
            scale([1,1,-1])
                cylinder(d=2*endplate_ringR+0.5,h=endplate_ringTY);
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
    make_plate([MC,FL,BL]) children();
}

range_sides=36; // vertex range on ends
range_tops=20; // needs different coverage than endplates


// One generic truss vertex, centered at this point
module truss_vertex(p,wall=truss_wall,range,aspect=[1,1,1],edgestyle=1) 
{
    intersection() {
        translate(p) scale(aspect) rotate([0,90,0]) sphere(r=range);
        
        truss_tubes(wall,1,edgestyle);
    }
}

// One truss vertex, with webbed walls connecting the convex hull of adjoining tubes.
//    end==1 on the endplates, 0 on the diagonal plates
//    range is the radius of the limiting sphere
//    aspect is the scale on the limiting sphere
module truss_vertex_webbed(p,end=0,range=range_sides,aspect=[1,1,1],edgestyle=1)
{
    hollow_wall=truss_wall;
    difference() {
        union() {
            // Core walls around main tubes
            truss_vertex(p,range=range,aspect=aspect,edgestyle=edgestyle);
    
            // central blob hull connecting all parts
            difference() {
                wall=hollow_wall; // -rodOD_long*0.05;
                r = 0.75*range;
                hull() truss_vertex(p,range=r,aspect=aspect,wall=wall,edgestyle=edgestyle);
                
                // Hollow, to reduce material when printed
                if (graphics_mode==0) 
                    hull() truss_vertex(p,range=r-hollow_wall,aspect=aspect,wall=wall-hollow_wall,edgestyle=edgestyle);
            }        
            
            // Bosses around retaining bolt holes
            //if (end==0) truss_bolthole_diag(p,truss_wall*1.5);
        }
        
        // Retaining bolts for tubes
        if (end==0) truss_bolthole_diag(p,0);
    }
    
    // Thicker ring around tube exits
    /*difference() {
        ring_OD=1.5*truss_wall;
        ring_deep=14/aspect[0];
        truss_vertex(p,range=range,aspect=aspect,wall=ring_OD,edgestyle=edgestyle);
        translate(p) scale(aspect) sphere(r=range-ring_deep);
    }*/
    
    // diagonal plate(s)
    if (edgestyle<2)
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
            cylinder(d=rodOD_long*0.5+2*enlarge,h=50+enlarge); // narrow top
            cylinder(d=rodOD_long*1.1+2*enlarge,h=1); // big base
        }
        
        // These side wings, for side stability
        joinplate=6;
        translate([-joinplate/2,0,-15])
            cube([joinplate+2*enlarge,endplateY+0.7*enlarge,joinplate+2*enlarge],center=true);
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
        if (graphics_mode==0) hull() { // hollow inside
            w=-truss_wall;
            truss_frontback_reinforcing_solid(w);
        }
    }
}

// The full set of front & back vertexes
module truss_vertexes_frontback(edgestyle=1)
{
    for (flipX=[0,1]) {
        truss_vertex_frontback(FL,flipX,edgestyle=edgestyle); // endplate_shift(flipX,0,0) truss_frontback_reinforcing_walls();
        truss_vertex_frontback(BL,flipX,edgestyle=edgestyle); // endplate_shift(flipX,0,1) truss_frontback_reinforcing_walls();
    }
}

// The upright truss vertex that clamps the next row
module truss_vertex_uprightclamp(p,bayrange,aspect,edgestyle)
{
    difference() {
        union() {
            truss_vertex_webbed(p,range=bayrange,aspect=aspect,edgestyle=edgestyle);
            // Thicker walls for screws to hold from one side
            //wall = 1.5*truss_wall;
            //translate(p) scale(aspect) rotate([0,90,0]) cylinder(d=rodOD_long+2*wall,h=1.8*bayrange,center=true);
        }
        
        // Space for tube
        translate(p) rotate([0,90,0]) cylinder(d=rodOD_long,h=2*bayrange,center=true);
        
        // Leave holes in place
        truss_bolthole_diag(p,0);
        
        translate(p) rotate([-10,0,0]) // tilt the approach angle to snap over
        {
            // Trim out a width, so we can fit over the long tube:
            //   spring_clamp 0.8, wall 1.5*1.6 => 1kgf snap-over force
            spring_clamp=0.8; // mm of spring needed to fit over tube
            trimR=rodOD_long*0.5-spring_clamp;
            translate([0,trimR,0]) cube([150,2*trimR,2*trimR],center=true);
            
            // Bevel entrance on far side, so we easily slide over the tube
            bevelR = 0.65*rodOD_long; // trim cylinder
            translate([0,0+bevelR,0]) rotate([0,90,0]) 
                cylinder(r=bevelR,h=150,center=true);
        }
    }
}

// The full set of vertexes in the truss
module truss_vertexes(edgestyle=1)
{
    truss_vertexes_frontback(edgestyle=edgestyle);
    
    aspect=[1,1,1.2]; // squish bounding sphere?
    for (p=[FC,BC,SL]) {
        truss_vertex_webbed(p,range=range_tops,aspect=aspect,edgestyle=edgestyle);
    }
    
}

// Full truss vertexes, including all endplates
module truss_full_vertexes(edgestyle=1,vertex=0)
{
    difference() {
        union() {
            truss_vertexes(edgestyle=edgestyle);
            endplate_positive();
        }
        endplate_negative();
        
        // Space for retaining bolt to tilt
        translate(FC) {
            // Space for bolt shaft
            for (Zflip=[+1,-1])
            hull() {
                for (tilt=[0,truss_unfold/2]) rotate([90+tilt,0,0])
                    scale([1,1,Zflip])
                    cylinder(d=3.2,h=50);
            }
            
            // Space for socket cap
            hull() 
            for (tilt=[0,truss_unfold/2]) rotate([90+tilt,0,0])
                translate([0,0,rodOD_top/2+truss_wall])
                    cylinder(d=7,h=10);
        }
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
    truss_length_print("Long rod: ",FL,BL,lengthen_long);
    truss_length_print("Top rod front: ",FC,FT,lengthen_top);
    truss_length_print("Top rod back:  ",BC,BT,lengthen_top);
    
    truss_length_print("Support: ",SC,SL,lengthen_support);
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
            translate([0,L==0?0:trussZ,-trussZ]) rotate([180,0,0]) 
                truss_full(edgestyle=L);
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
    vertexlist=[
        BL, FL,
        FL, BL,
        MC
    ];
    rotlist=[
        [0,+90,0], [0,-90,0],
        //[-90,0,0], [+90,0,0],
        [0,+90,0], [0,-90,0],
        [-90,0,0]
    ];
    
    cut=16; // boundary between rotating and side join
    trimlist=[
        cut,cut,
        -cut,-cut,
        5
    ];
    p=vertexlist[v]; // center point
    r=rotlist[v]; // Y axis rotation
    trimZ=trimlist[v]; // Z height of bottom trim plane
    edgestyle=floor(v/2); // (v>=2)?1:0; 
    
    translate([0,0,trimZ])
    intersection() {
        rotate(r) translate(-p)
        difference() {
            truss_full_vertexes(edgestyle=edgestyle,vertex=v);
        
            truss_tubes(0,edgestyle=edgestyle);
        }
        translate([0,0,50-trimZ]) cube([200,200,100],center=true);
    }
}

// Set of printable vertexes
module truss_vertex_printable_set(vlist, edgestyle=1)
{
    locs=[
        [-30,0],
        [+30,0],
        [-30,30],
        [+30,30],
        [0,50]
    ];
    for (v=vlist) translate(locs[v])
        truss_vertex_printable(v,edgestyle=edgestyle);
}

// Computer aided manufacturing (CAM) outputs:
edgestyle=1;
//translate([0,50]) truss_vertex_printable(4,edgestyle=edgestyle);
//truss_vertex_printable_set([0,1], edgestyle=edgestyle);
truss_vertex_printable_set([0,1,2,3,4], edgestyle=edgestyle);
//endplate_template();

//truss_printable();
truss_lengths();

// Demo versions:
if (0) { // full part demo
    truss_tubes(-0.1,edgestyle=edgestyle);
    truss_full_vertexes(edgestyle=edgestyle);
}
//for (shift=[-1:0]) for (advance=[-1:+1])
//    truss_advanceN(advance,shift) truss_full(edgestyle=edgestyle);
    
//truss_demo_section(edgestyle=1);
//truss_demo_tower(0);
//truss_demo_mate(3);
//truss_demo_mate(0);
//truss_demo_fullarch(); translate([0,trussZ,0]) truss_demo_fullarch();
//truss_demo_boxarch();
//truss_demo_8arch();
//truss_demo_stack();

