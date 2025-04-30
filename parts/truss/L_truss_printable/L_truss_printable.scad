/*
3D printable model of L-truss, for demonstrating tool free assembly.

*/
$fs=0.1; $fa=2; // smooth output



// Height of truss linear extrusion
trussZ=3.5;

// Height of truss end extrusions
trussE=6;

truss_length=140; // center to center length
truss_majorOD=trussZ; // major tube diameter
truss_diagOD=1.5; // diagonal tube diameter
truss_round=1.5; // corner rounding on structural tubes

truss_dY = 15; // centerline to Y axis center of main tubes
truss_angle = 22.5; // rotation angle for trimming tubes

// Z height of top surface of topper upright
topperZ=50;
truss_top = truss_length + 2*sin(truss_angle)*truss_dY; // length of top along centerline

vheight=20; // Centerline to top of grab block, along mating surface
gX = trussZ;  // size of grab block
gY = trussZ;

wall=1.2; // thickness of printed thumb (3 perimeters)
clearance=0.1; // as-printed space between modules

lY = 1.2; // thickness of lock prong on top
locktopY = vheight+wall/2+clearance; // centerline of lock top edge
thumboutX = gX/2+wall/2; // centerline of thumb start

lND = 2.0; // diameter negative space behind lock mechanism
lNX = -4; // X center of negative space
lNY = locktopY - wall/2 - lND/2; // Y center of negative space

lG = 1.5; // distance down that thumb grabs
lL = 4; // size of thumb latch

dockX = gX/2; // width of block used for docking area

// Shape of grab block, passive side.  Sits at -Y, Y axis as mating surface
module grab2D() {
    difference() {
        // Actual block
        translate([-gX/2,-vheight])
            square([gX,gY]);
        
        // Trim off front edge to smoothly enter thumb
        translate([0,-vheight]) rotate([0,0,45])
            translate([0,-50]) square([100,100],center=true);
    }
}

// Shape of connector between grab and lock
module connector2D() {    
    // Flat space for docking
    translate([-dockX/2-clearance/2,-clearance]) 
        square([dockX,2*vheight-2*gY],center=true);
}

// Rounded 2D stroke from start to end=start+dir
module stroke2D(start,dir,dia=wall)
{
    hull() {
        translate(start) circle(d=dia,$fn=16);
        translate(start+dir) circle(d=dia,$fn=16);
    }
}

// Negative space behind locking mechanism
module lockspace2D() {
    stroke2D([lNX,lNY],[-lNX,0],dia=lND);
}

// Shape of locking top, active side.  Sits at +Y, Y axis as mating surface
module lock2D() {
    difference() {
        round=1.5;
        offset(r=-round) offset(r=+round)
        union() {
            stroke2D([0,locktopY],[0+gX/2+wall/2,0]); // top
            difference() {
                offset(r=+wall) lockspace2D();
                translate([100-clearance,0]) square([200,200],center=true); // trim front
            }
            
            stroke2D([thumboutX,vheight-lG],[0,lG]); // down to latch
            stroke2D([thumboutX,vheight-lG],[lL,lL*0.75]); // out to latch
        }
        // space for the other grab
        rotate([0,0,180]) offset(r=+clearance) grab2D();
        lockspace2D();
    }
}

// One symmetric mating pair
module pair2D() {
    lock2D();
    connector2D();
    grab2D();
}

// Make children symmetric at truss ends
module truss_end_orients() {
    for (side=[0,+1]) 
    rotate([0,0,side*180])
    translate([truss_length/2,0])
    rotate([0,0,truss_angle*(side?+1:-1)])
    {
        children();
    }
}


// Z blocks lock trusses together along Z axis
Zblock=3;
ZblockZ=1.5;

// Center locations of Z blocks
module Zblock_locs() {
    truss_end_orients() 
        for (side=[1]) // 0,1])
            translate([0,side?truss_dY-Zblock-1:(-truss_dY+2*Zblock+1),trussE+ZblockZ/2])
                children();
          
}


// Truss tubes, 2D version
module tubes2D() {
    difference() {
        offset(r=-truss_round) offset(r=+truss_round)
        union() {
            difference() {
                // Main tubes
                for (y=[-1,+1]) translate([0,y*truss_dY])
                    square([truss_length+20,truss_majorOD],center=true);
                // Trim off ends of tubes
                truss_end_orients() translate([200-clearance,0]) square([400,400],center=true);
            }
            
            Zblock_locs() translate([-Zblock/2,0]) square([Zblock,Zblock],center=true);
            
            // Sidewalls (for rounding)
            truss_end_orients() connector2D();
            
            // Diagonals
            for (Xside=[-1,+1]) scale([Xside,1,1]) {
                ht=truss_dY*2;
                stroke2D([-truss_top/4,+ht/2],[-truss_top*0.175,-ht],dia=truss_diagOD);
                stroke2D([-truss_top/4,+ht/2],[truss_top*0.23,-ht],dia=truss_diagOD);
            }
            /*
            // One big diagonal
            hull() truss_end_orients() 
                translate([-truss_diagOD*0.7,+truss_dY*0.89]) circle(d=truss_diagOD);
            */
            
            // Add positive latching ends
            truss_end_orients() pair2D(); 
        }
        
        // Clear interior of latches
        truss_end_orients() lockspace2D();
        
        // Clear complement space from mating ends
        truss_end_orients() offset(r=+clearance) rotate([0,0,180]) pair2D(); 
    }
}

// Full truss, including 3D segments
module tubes3D() {
    linear_extrude(height=trussZ,convexity=6) tubes2D();
    
    intersection() {
        linear_extrude(height=trussE,convexity=6) tubes2D();
        
        truss_end_orients() rotate([90,0,0])
            linear_extrude(height=100,center=true)
            offset(r=-trussE) offset(r=+trussE) // rounded approach
            {
                square([10,2*trussE],center=true); // raised portion
                square([40,2*trussZ-0.1],center=true); // main portion
            }
    }

    // Locking blocks, to keep truss from shifting along Z
    Zblock_locs()
    difference() {
        cube([2*Zblock,Zblock,ZblockZ],center=true);
        translate([0,0,-ZblockZ/2]) // bevel entrance
            rotate([0,-20,0])
                translate([0,0,-100]) cube([200,200,200],center=true);
    }
}

topperOD=truss_diagOD+0.5;

// Truss including top surface mesh
module tubes3Dtopper() {
    // list of X coordinate centers for topper top clips
    //clipX_list=[0]; // 
    clipX_list = [+0.31*truss_top,-0.31*truss_top]; 
    
    // Center of clip, relative to tool clamp coords
    clip_center = [0,truss_dY+truss_majorOD/2-vheight,topperZ+truss_majorOD/2+clearance];
        
    round=truss_round; // rounding on 2D extrusions
    OD = topperOD; // diagonal width

    // Start with the flat part of the truss
    tubes3D();
      
    // Add up-to-out diagonal
    intersection() {
        linear_extrude(height=topperZ,convexity=4) truss_end_orients() connector2D();
        rotate([0,-90,0]) // upright: X,Y -> Z,Y
            linear_extrude(height=truss_top/2+1,convexity=4) 
            offset(r=-round) offset(r=+round)
            {
                z=trussE/2;
                diagZ=topperZ*0.8;
                diagY=(2*truss_dY)*0.8;
                stroke2D([z,truss_dY-diagY],[diagZ,diagY],dia=trussZ); // diag
                stroke2D([z,truss_dY],[0,-2*truss_dY],dia=trussE); // bottom
                stroke2D([z,truss_dY],[topperZ-z,0],dia=OD); // up
            }
    }
    
    // Main upright
    intersection() {
        // vertical cross section is primarily same as on flat:
        linear_extrude(height=topperZ,convexity=6) tubes2D();
        
        // trim back to only top surface and mating clips
        union() {
            // Boxes to grab the mating clips themselves
            intersection() {
                truss_end_orients() for (Yside=[-1,+1]) scale([1,Yside,1])
                    translate([0,vheight,topperZ])
                    hull() {
                        //rotate([0,0,truss_angle]) 
                            cube([14,10,trussE],center=true);
                        translate([-4,-4,-12])
                            cube([1,1,1],center=true); // taper base
                    }
                // trim bottom side
                translate([0,100,0]) cube([400,200,400],center=true);
            }
            
            // Center extrusion for reinforcing
            difference() {
                // Main extrusion
                translate([0,truss_dY,topperZ/2+trussZ/2-OD/2]) rotate([90,0,0]) 
                linear_extrude(height=truss_majorOD,convexity=4,center=true)
                    offset(r=-round) offset(r=+round)
                    difference() {
                        wid = truss_top+1;
                        ht = topperZ-trussZ+OD;
                        // Exterior of topper 
                        square([wid,ht],center=true);
                        difference() {
                            // Interior hollow of topper
                            square([wid-2*truss_majorOD,ht-2*OD],center=true);
                            // diagonals (for appearance and printability)
                            for (Xside=[-1,+1]) scale([Xside,1,1]) {
                                corner = [-truss_top/2,+ht/2];
                                stroke2D(corner,[truss_top/4,-ht],dia=OD);
                                middle = [-truss_top/4,-ht/2];
                                stroke2D(middle,[truss_top/4,+ht],dia=OD);
                                stroke2D([corner[0],middle[1]],[-clipX_list[0]-corner[0],+0.9*ht],dia=OD);

                            }
                            /*
                            for (side=[-1,+1]) rotate([0,0,side*45])
                                square([OD,300],center=true);
                            */


                            // clip backing
                            translate([0,ht/2-OD/2]) {
                                for (x=clipX_list) translate([x,0,0])
                                    circle(d=8);
                                    //square([8,8],center=true);
                            }
                        }
                    }
                // Voids behind each clip (so it can bend)
                for (x=clipX_list) 
                    translate([x,0,0]+clip_center) rotate([0,-90,0])
                        linear_extrude(height=trussE,convexity=4,center=true)
                            lockspace2D();

            }
        }
    }
    // Add in top clip(s) to hold onto next truss
    for (x=clipX_list) {
        translate([x,0,0]+clip_center) rotate([0,-90,0])
        {
            // support material under lock latch
            space=0.2;
            linear_extrude(height=trussE+3,convexity=4,center=true) 
                translate([-space,vheight-gY*0.4]) square([gX-space,0.8],center=true); 
            
            // Lock latch
            linear_extrude(height=trussE,convexity=4,center=true) {
                lock2D();
                
                //connector2D();
                x = -1;
                y = vheight-gY;
                polygon([
                    [-trussZ*1.5,y],
                    [-trussZ/2,y],
                    [x-clearance,y-2*clearance],
                    [x-clearance,y-dockX]
                ]);
            }
        }
        // Add small blocks on corresponding tube, to keep tube from shifting
        for (Yside=[-1,+1]) scale([1,Yside,1])
        translate([x,truss_dY+truss_majorOD*0.25,truss_majorOD/2])
        for (Xside=[-1,+1]) translate([Xside*(truss_majorOD/2+trussE*0.6),0,0])
            cube([truss_majorOD,truss_majorOD,truss_majorOD],center=true);
    }
}


// Mated pair
module demo_mate()
{
    pair2D();
    #rotate([0,0,180]) pair2D();
}

// Demo of ways 2D trusses mate
module demo_mate_truss()
{
    tubes2D();
    #truss_end_orients() rotate([0,0,180]) pair2D();
}

// Demo of ways two trusses can mate
module demo_mate_truss3D()
{
    tubes3D();
    #truss_end_orients() rotate([0,0,180+truss_angle]) translate([-truss_length/2,0,0]) tubes3D();
}

//demo_mate_truss3D();
//tubes3D();
tubes3Dtopper();

