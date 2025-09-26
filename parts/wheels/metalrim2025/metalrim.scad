/*
Metal-rimmed robot wheels, 2025-09 test version
    - Grousers bolted on outside: 1/4" ID aluminum trim channel
    - Sheet metal strip bolted down around the wheel rim
    - 3D printed wheel interior and spokes
(Perhaps add stronger basalt fiber spokes instead of 3D printed?)


Grouser material is tiny extruded aluminum C-channel.  
Example: 1/4" inside diameter trim channel
    - Height 12.5mm
    - Width 10mm
    - Wall 1.5mm
Height 12.5mm over radius 125mm is 10% normalize grouser height.
The (Intosume et al, 2019) chart recommends 20-30 grousers. 
    

Dr. Orion Lawlor, lawlor@alaska.edu, 2025-09-18 (Public Domain)
*/
$fs=0.1; $fa=2;
include <AuroraSCAD/bevel.scad>;


draw_solid=0; // if 1, skip interior and draw fast solid outside
draw_cutaway=0; // if 1, show cutaway interior


inch = 25.4; // file units are mm

scale=1; //<- thickens all walls, for printing a scaled-down model

wheelOD = 250; // diameter of wheels (flat to flat, not including grouser height)
wheelZ = 125; // thickness of wheel surface
wheelB = 15; // bevel insets around wheel perimeter (to easily slide over bumps)
wheelW=1.2*scale; // minimum wall thickness

grouserN = 16; // number of grousers around outside
grouserW = 10; // thickness of grouser metal (fully supported and flat)
grouserA = 25; // angle (degrees) of grousers around vertical
grouserZ = wheelZ/cos(grouserA)-2*wheelB-grouserW; // length of grouser segments
grouserboltZ = 40;
echo("Grouser segments: ",grouserZ,"  bolted at ",grouserboltZ);

grouserP = grouserZ+grouserW; // pointy grouser tip length
grouserPA = 3; // extra rotation on pointy tips
grouser_skew=0.17; // determines angle of bevel insets on outside
grouser_hole_ID = 2.6; // fits M3 machine screws

ribW=1.2*scale; // wall thickness of reinforcing ribs
ribH=8; // height of reinforcing ribs
ribA=0; // angle ribs are tilted back to meet grouser bolts
rib_minorH=3; // minor rib heights


insideW=draw_cutaway?0:0.7*scale; // wall thickness of inside plate (or 0 for open ribs)

// Make children centered at each grouser, with +X facing outward
module grouser_centers() {
    for (a=[0:360/grouserN:360-1]) rotate([0,0,a])
        translate([wheelOD/2,0,wheelZ/2]) rotate([grouserA,0,0])
            children();
}

// Grouser support blocks
module grouser_blocks(enlarge=0,expandZ=0,thick=3) {
    grouser_centers() 
        translate([-thick/2+enlarge,0,0])
        {
            roundR = 10; // outside rounding radius
            
            // Rounding sphere coordinate shifts to line up seams
            shiftX = -0.4; // back-forward
            shiftY = 2; // line up seams on vertical
            shiftZ = 5; // up-down
            
            bevelcube([thick,grouserW,grouserZ],bevel=0.5,center=true);
            
            // Rounded tip, to make smooth corners
            for (end=[-1,+1]) translate([+thick/2-roundR+shiftX,-end*shiftY,end*(grouserZ/2+shiftZ+expandZ)])
                sphere(r=roundR,$fn=32);
         }
}

// Tire top and bottom dots are on farthest outside corners (max Z)
module tire_ring_dots(enlarge=0,expandZ=0,expandR=0) {
    // Little dots at top and bottom diagonals
    Zradius = wheelZ/2/cos(grouserA);
    for (topbot=[-1,+1]) 
        rotate([0,0,360/grouserN*(topbot*(grouser_skew-0.015*enlarge))])
        grouser_centers() 
            translate([-wheelB+expandR+0.7*enlarge,0,(Zradius+expandZ-0.7*enlarge)*topbot])
                rotate([-grouserA,0,0])
                    cube([0.1,0.1,0.1],center=true);
        
}

// Solid version of outer tire ring (tread area)
module tire_ring_solid(enlarge=0,expandZ=0) {
    difference() {
        hull() {
            grouser_blocks(enlarge=enlarge,expandZ=expandZ*0.5);
            tire_ring_dots(enlarge=enlarge,expandZ=expandZ);
        }
        // Flat spots for each grouser
        thick=5;
        grouser_centers() translate([thick/2+enlarge,0,0])
                cube([thick,3*bayY,3*bayX],center=true);
    }
}

// Core out the center path
module tire_core(enlarge=0) {
    hull() tire_ring_dots(enlarge=enlarge,expandZ=-enlarge+0.01);
}

// Reinforcing bay dimensions
bayY=(wheelZ/2-wheelB); // Y height of top plate from center
bayX = grouserZ*0.49; // X width of grouser bay (trims the rib profiles)

// Rounds and trims children
module bay_ribs_trim_round2D() 
{
    rib_round=1.5*ribW;
    intersection() {
        square([bayX,1000],center=true); // don't go beyond our bay
        offset(r=-rib_round) offset(r=+rib_round) // rounds the rib intersections
            children();
    }
}

// Major ribs inside bays
module bay_ribs_major2D() 
{
    // Diagonal spokes
    for (angle=[-1,+1]) rotate([0,0,angle*grouserA])
                square([ribW,grouserZ*1.15],center=true);
    
    // Grouser bolt spokes
    rotate([0,0,-grouserA])
        for (shift=[-1,+1]) 
            translate([0,grouserboltZ*shift])
                rotate([0,0,+grouserA+90+grouserA])
                    square([30,ribW],center=true);
    
    // Flats
    for (y=[-bayY,0,+bayY]) translate([0,y])
        square([wheelZ,ribW],center=true);
    
    // outside edges
    for (side=[-1,+1]) translate([0.255*grouserZ*side,0,0])
            square([ribW,wheelZ-2*wheelB],center=true);

    // Vertical ribs on lips
    for (vrib=[-1,0,+1]) for (topbot=[-1,+1]) scale([1,topbot,1])
        translate([vrib*bayX*0.3-ribW/2,bayY,0])
            square([ribW,wheelZ]);
}

// Minor reinforcing ribs 
module bay_ribs_minor2D() {
    // Horizontal plates
    for (y=[-1:0.25:+1]) translate([0,bayY*y]) square([wheelZ,ribW],center=true);

    // Diagonals
    for (a=[-1,+1]) rotate([0,0,a*grouserA])
        for (d=[-1,+1]) translate([d*20,0,0]) 
            square([ribW,wheelZ],center=true);
    
    bay_ribs_major2D(); // <- include major ribs, to round to them properly
}

// Extrude bay ribs
module bay_ribs_extrude(height)
{
    rotate([0,-90,0]) rotate([0,0,90]) 
        linear_extrude(height=height,convexity=6)
            children();
}
    

// Ribs to reinforce outside of tire.  Trimmed to fit outside.
module tire_ribs() 
{
    full = 60; // extrusion thickness
    
    // Reinforcing around grousers
    grouser_centers() {
        // wide flat under grouser
        translate([0,0,0])
            cube([1.5*ribW,grouserW+2*ribW,2*bayY],center=true);
        
        rotate([-grouserA,0,0]) // undo grouser_centers rotate
        {
            // Support isogrid: X up, Y across here
            bay_ribs_extrude(full)
                bay_ribs_trim_round2D() bay_ribs_major2D();
            
            bevel=0.5*ribW;
            bay_ribs_extrude(wheelW+bevel) offset(r=+bevel)
                bay_ribs_trim_round2D() bay_ribs_major2D();
            
            // Minor isogrid ribs
            bay_ribs_extrude(rib_minorH)
                bay_ribs_trim_round2D() bay_ribs_minor2D();
            
        }
    }
    
    // Upright rib between grousers
    for (copy=[1]) rotate([0,0,copy*360/grouserN/2])
    grouser_centers() {
        rotate([-grouserA+ribA,0,0]) 
        {
            cube([ribH*3,ribW,wheelZ],center=true);
            translate([+1,0,0]) cylinder(d=3*ribW,h=wheelZ,center=true);
        }
    }
}

// Thin-wall outer rim of tire
module tire_rim(wall=wheelW, solid=0)
{
    difference() {
        // outside shape
        tire_ring_solid();
        
        // trim the inside (but include ribs)
        if (solid==0)
        difference() {
            tire_ring_solid(enlarge=-wall,expandZ=-1); // negative space
            
            // Carve ribs from the negative space
            tire_ribs();
            tire_core(enlarge=+2.5*wheelW);                    
                    
            // Solid ring on top and bottom lips
            for (topbot=[-1,+1]) translate([0,0,wheelZ/2+topbot*wheelZ/2])
                cube([wheelOD,wheelOD,6],center=true);
            
            if (insideW>0) tire_ring_solid(enlarge=-ribH+insideW,expandZ=+2); // flat inner wall
            
            // Any other internal reinforcing gets attached here
            children();
        }
        
        tire_ring_solid(enlarge=-ribH); // don't cut beyond rib distance
        //tire_core();
    }
}

// Mounting bolt hole centers
module grouser_bolthole_centers() {
    grouser_centers() {
        for (shift=[+1,0,-1]) translate([0,0,shift*grouserboltZ])
            rotate([0,-90,0]) // put +Z facing in
                children();
    }
}




motor_outsideN=8; // drive motor frame screw count
motor_outsideR=88.22/2; // drive motor outside hole centerline
motor_outsideW=7.0/2; // drive motor outside wall thickness (+clearance)
motor_outsideZ=28.7; // total Z height of motor stack

motor_angle=-6; // relative angle of motor and tire (to line up spokes on grousers)
motor_startZ=25; // Z height of motor output base relative to tire zero

motor_insideN=8; // drive motor attach screw count
motor_insideR=30; // drive motor attach screw hole centerline
motor_insideOD=3.2; // space for M3 screws

// Shift to motor coordinates
module motor_coords() {
    translate([0,0,motor_startZ])
    rotate([0,0,motor_angle])
        children();
}

// Space occupied by hub motor
module motor_outside(enlargeR=0,enlargeZ=0) {
    motor_coords()
    hull() {
        da=360/motor_outsideN; // delta angle
        for (angle=[da/2:da:360-1]) rotate([0,0,angle])
            translate([motor_outsideR,0,0])
                cylinder(r=motor_outsideW+enlargeR,h=motor_outsideZ+enlargeZ);
    }
}

spokeN=4; // number of spokes
spokeID=4.5; // printed hole diameter, for basalt rods (plus clearance for easy *long* insertion)
spokeW=2.4; // plastic walls around spokes
echo("Target centerR = ",motor_outsideR*cos(360/motor_outsideN/2)+motor_outsideW+spokeID/2+spokeW);

spoke_centerR=47.6;
spoke_centerZ=20; // Z height of center of spokes, in motor coords
spokeDA=16; // dip angle of spokes, relative to horizontal
spokeL=wheelOD*0.96; // length of spokes
echo("Spoke length: ",spokeL);

// Make children at each spoke
module spoke_centers() {
    motor_coords() {
        for (angle=[0:360/spokeN:360-1]) rotate([0,0,angle])
            translate([spoke_centerR,0,spoke_centerZ])
                rotate([90+spokeDA,0,0])
                    children();
    }
}

// Make spoke models
module spoke_models(extraD=0,extraLen=0,Zshift=0) {
    spoke_centers() translate([0,0,Zshift]) 
        cylinder(d=spokeID+extraD,h=spokeL+2*extraLen,center=true);
}

// Make reinforcing down the spoke attachments to the rim
module spoke_reinforcing() {
    round=2*ribW; // outside rounding
    span=40; // plates to distribute load around spokes
    spoke_centers() linear_extrude(height=spokeL+50,center=true,convexity=4)
        offset(r=-round) offset(r=+round)
        difference() {
            union() {
                circle(d=spokeID+2*spokeW);
                start_angle=+4; // line up rotated top spoke with other ribs
                for (angle=[0:60:180-1]) rotate([0,0,start_angle + angle])
                    square([span,ribW],center=true);
            }
            //circle(d=spokeID); // inside hole (cut later)
        }
}

// Make beyond-surface reinforcing of the spokes
module spoke_reinforcing_surface() {
    h=8;
    intersection() {
        spoke_centers() {
            for (end=[-1,+1]) scale([1,1,end])
                translate([0,0,-spokeL/2+ribH*0.5])
                    cylinder(d1=1.3*h+spokeID,d2=spokeID+2,h=h);
        }
        tire_ring_solid(enlarge=-ribH+ribW/2);
    }
}

// Central hub holds spokes to motor
module hub() {
    wall=1.2; // plastic around motor itself
    floor=1.5;
    trimZ = 20; // height of hub piece
    
    spokeL = 40; // length holding on to spoke
    
    difference() {
        union() {
            motor_outside(enlargeR=wall,enlargeZ=floor);
            spoke_centers() {
                //cylinder(d=2*wall+spokeID,h=spokeL,center=true);
                r=8; // rounding radius
                linear_extrude(height=spokeL,center=true,convexity=4)
                offset(r=-r) offset(r=+r)
                {
                    w=5; h=15;
                    translate([-spokeID/2-w/2-0.5,0])
                        square([w,h],center=true);
                    circle(r=spokeID/2+spokeW);
                }
            }
        }
        
        // Mounting holes are M3 thru bolts
        motor_coords() {
            for (angle=[360/motor_insideN/2:360/motor_insideN:360-1]) rotate([0,0,angle])
                translate([motor_insideR,0,motor_outsideZ])
                    cylinder(d=3.1,h=10,center=true);
        
            
            // Space in middle for counterweight or axle or such
            cylinder(r=motor_insideR-4,h=100,center=true);
        }
        
        // Space for actual motor and spokes
        motor_outside();
        spoke_models();
        
        // Trim bottom and top flat
        motor_coords() {
            // bottom
            translate([0,0,motor_outsideZ-trimZ-200]) 
                cube([400,400,400],center=true);
            
            // top
            translate([0,0,motor_outsideZ+floor+200]) 
                cube([400,400,400],center=true);            
        }
    }
}

// 3D printable version of central hub
module hub_printable() {
    rotate([180,0,0]) hub();
}

// Tire rim with all mounting holes
module tire_rim_holy(solid=0,grouserbolts=1,spokes=1)
{
    difference() {
        union() {
            tire_rim(solid=solid)
                if (spokes) spoke_reinforcing(); // below rim
            spoke_reinforcing_surface(); // sticking up from rim
            
            if (grouserbolts) grouser_bolthole_centers() cylinder(d1=10,d2=5,h=ribH);
            
        }
        if (grouserbolts) grouser_bolthole_centers() cylinder(d=grouser_hole_ID,h=20,center=true);
        if (spokes) spoke_models(0.0,extraLen=10,Zshift=10); // with insert hole
        
        // Trim off top and bottom
        for (topbot=[-1,+1]) translate([0,0,wheelZ/2+topbot*wheelZ/2])
            cube([wheelOD,wheelOD,1],center=true);

        // Cutaway
        //translate([0,0,wheelZ/2]) cube([1000,1000,1000]);
    }
}



//#spoke_models();
//#motor_outside();

// Cutaway of tire rim
module rim_section()
{
    intersection() {
        if (1) intersection() { // print one test sector
            rotate([0,0,10]) cube([1000,1000,1000]);
            rotate([0,0,-10]) cube([1000,1000,1000]);
        }
        
        tire_rim_holy(solid=0);
    }
}

// Manufacturing tool for the grouser segments
module grouser_tool() 
{
    wid = 10.3;
    ht = 12.5;
    len=grouserZ; 
    wall=2.4;
    
    difference() {
        intersection() {
            // Block outside the grouser
            translate([0,0,(ht+wall)/2])
                cube([wid+2*wall,len+2*wall,ht+wall],center=true);
            
            // Rolling perimeter of wheel
            translate([0,0,-wheelOD/2])
                rotate([0,0,-grouserA]) rotate([90,0,0])
                union() {
                    bevelcylinder(d=wheelOD+2*wall+2*ht,h=wheelZ,bevel=wheelB+ht,center=true);
                    cylinder(d=wheelOD+2*wall,h=wheelZ,center=true);
                }
        }
        
        // Slot for the grouser
        translate([0,0,wall+(ht)/2])
            cube([wid,len,ht],center=true);
        
        // Holes for drilling grouser bolt holes
        for (shift=[+1,0,-1]) translate([0,shift*grouserboltZ,0])
            cylinder(d=3.2,h=50,center=true);
        
    }
}

// 3D demo of wheel parts
module wheel_demo() {
    tire_rim_holy(solid=1);
    hub();
    #spoke_models();
}

wheel_demo();

//tire_rim_holy(solid=draw_solid); // full rim
//hub_printable(); // hub

//rim_section(); // development section (faster print with less plastic used)
//grouser_tool(); // CAM tool for drilling and cutting aluminumg grousers




