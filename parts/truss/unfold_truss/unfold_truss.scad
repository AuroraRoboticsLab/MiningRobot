/*
 Vague idea of an unfolding truss.
*/

$fs=0.1; $fa=5;

inch=25.4;



// Z heights to extrude models
extrude_floor = 2;
extrude_coverwall = 15;
wall_round=5;
wall_thick=2;

// Arch parameters
arch_height=200; // height of center of arch
arch_width = 300; // half-width of arch from centerline
arch_curve=arch_height/(arch_width*arch_width); // sets curvature of arch
function arch_y(x) = arch_height - arch_curve*x*x;

// 2D points making outline of arch interior
archpoints = [
    [0,0],
    for (x=[0:20:arch_width])
        [x, arch_y(x)]
];

// Scale factor that makes arch exterior
arch_thick_base=35; // X thickness at base
arch_thick_top=15; // Y thickness at top
arch_thick_cover=0.1; // XY thickness of cover

// Make aspect ratio for these additions to base arch
function arch_scale(add_base,add_top) = 
    [1+add_base/arch_width,1+add_top/arch_height,1];
arch_ext_aspect=arch_scale(arch_thick_base,arch_thick_top);

// Draw 2D outline of arch
module arch_2D(enlarge_inside=0) {
    difference() {
        scale(arch_ext_aspect) polygon(archpoints);
        scale(arch_scale(1.5*enlarge_inside,enlarge_inside)) polygon(archpoints);
    }
}
// Draw 2D outline of arch cover
module arch_2D_cover() {
    difference() {
        scale(arch_scale(arch_thick_base+arch_thick_cover,arch_thick_top+arch_thick_cover)) polygon(archpoints);
        scale(arch_ext_aspect) polygon(archpoints);
    }
}

// Controls arch slice process:
//  slice = [ x coordinate of base,  rotation angle of cut above X axis ]
arch_slices=[
    [0,90], // start
    [80,65],
    [160,45],
    [235,30],
    [300,0]
];
arch_nslices=4;

// Get XY base location from slice object
function arch_xyS(slice) = [slice[0], arch_y(slice[0]) ];

// Transform so our X lies along this slice start
module arch_transformS(Islice,enlarge=0) 
{
    slice=arch_slices[Islice];
    translate(arch_xyS(slice))
        rotate([0,0,slice[1]])
            translate([0,enlarge,0])
                children();
}

// Create the 2D interior of this slice
module arch_slice2D(Islice,enlarge=0)
{
    difference() {
        arch_transformS(Islice+1,enlarge)
            translate([-200,0])
                square([600,300]);
        arch_transformS(Islice,-enlarge)
            translate([-200,0])
                square([800,400]);
    }
}

// Translate to the center of the cover of this arch
module arch_cover_center(Islice)
{
    x=0.6*arch_slices[Islice][0]+0.4*arch_slices[Islice+1][0];
    r=0.5*(arch_slices[Islice][1]+arch_slices[Islice+1][1]);
    y=arch_y(x);
    translate([x*arch_ext_aspect[0],y*arch_ext_aspect[1]])
        rotate([0,0,r])
            children();
}

// Create 2D diagonals for this slice
module arch_diagonals(Islice) 
{
    for (side=[0,1]) 
    hull() {
        arch_transformS(Islice+side) circle(d=wall_thick);
        
        arch_cover_center(Islice) circle(d=wall_thick);
    }
}

// Intersect our children with this slice
module arch_sliceCut(Islice,enlarge=0)
{
    intersection() {
        arch_slice2D(Islice,enlarge);
        children();
    }
}


// Extrude 3D version of this slice
module arch_slice3D(Islice,enlarge=0,enlarge_inside=0)
{
    // base
    linear_extrude(height=extrude_floor)
        hull() //<- straightens the inside edge
        arch_sliceCut(Islice,enlarge) arch_2D(enlarge_inside);

    // simple cover representation
    linear_extrude(height=extrude_coverwall)
        arch_sliceCut(Islice,enlarge) arch_2D_cover();
    
}


// Make 3D full arch
module arch_full3D(Islice)
{
    space=0.1; // half the gap between slices
    difference() {
        union() {
            arch_slice3D(Islice,space);
            
            // Bevels up to cover
            hull() {
                arch_slice3D(Islice,space,arch_thick_top-3);
                if (Islice+1==arch_nslices) { // foot on last arch
                    arch_transformS(Islice+1) translate([0,0.1+space]) cylinder(r=0.1,h=0.1);
                }
            }
            
            // boss around pivot screw
            arch_transformS(Islice) cylinder(d=7,h=2);
            if (Islice+1<arch_nslices) // skip pivot on last one
                arch_transformS(Islice+1) cylinder(d=7,h=2);
            
        }
        
        // clear out interior
        difference() {
            translate([0,0,-0.1])
            linear_extrude(height=extrude_coverwall+1)
                offset(r=+wall_round) offset(r=-wall_round) 
                difference() {
                    offset(r=-wall_thick) // interior only
                        hull() arch_sliceCut(Islice,1) arch_2D();
                }
                    
            // But don't clear out diagonals
            linear_extrude(height=extrude_floor)
                arch_diagonals(Islice);
        }
        
        // Clear pivot bolt holes for M3 screw
        arch_transformS(Islice) cylinder(d=3.2,h=10,center=true);
        if (Islice+1<arch_nslices) // skip pivot on last one
            arch_transformS(Islice+1) cylinder(d=2.7,h=10,center=true);
    }
    
}

module arch_demo() 
{
    for (Islice=[0:arch_nslices-1])
    {
        arch_full3D(Islice);
    }

}

// 3D printable version of this slice
module arch_printable3D(Islice) {
    slice=arch_slices[Islice];
    sliceN=arch_slices[Islice+1];
    translate([Islice*25,0,0])
    rotate([0,0,-slice[1]]) // -0.5*(slice[1]+sliceN[1])])
        translate(-arch_xyS(slice))
            arch_full3D(Islice);
}

// Full set of printable slices
module arch_printable_set()
{
    for (Islice=[0:arch_nslices-1])
    {
        arch_printable3D(Islice);
    }
}

//arch_demo();
arch_printable_set();




