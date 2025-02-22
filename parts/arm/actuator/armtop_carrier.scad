/*

*/
include <planetary_bearing.scad>;


hole_circle_OD=141.5; // bolt circle center diameter
holeID = 0.2*inch; //  #10-24 bolts
holeCap = 0.4*inch; // cap on top of bolts
holeN = 8; // taking only the tapped holes in the robot front mount

trimZ = 3; // Z thickness to trim off bottom
holeZ = 6+trimZ; // material around 1/2" bolts

module mountbolt_centers() {
    for (side=[-1,+1]) scale([side,1,1])
        for (angle=[-1,+1]) 
            rotate([0,0,angle*360/holeN/2])
                translate([hole_circle_OD/2,0,holeZ])
                    scale([1,1,-1])
                        children();
}

module mountbolt_xform(sign=+1) {
    if (sign==+1) 
        rotate([180,0,0]) translate([0,0,-frame_TZ]) children();
    else
        translate([0,0,frame_TZ]) rotate([-180,0,0]) children();
}

// Frame_T with mounting bolts on the bolt circle
module frame_T_mounted() {
    difference() {
        mountbolt_xform(-1) {
            frame_T_supports();
            frame_T(50)
            hull() {
                mountbolt_xform() mountbolt_centers() cylinder(d=holeCap,h=holeZ);
                translate(gearTC+[0,0,-BTspaceZ]) cylinder(d=gear_OD(gearplane_Rgear(gearplaneT)),h=gearZ);
            }
        }
        
        mountbolt_centers() {
            cylinder(d=holeID,h=100,center=true);
            scale([1,1,-1]) cylinder(d=holeCap,h=20);
        }
        
        // lighten holes
        //for (side=[-1,+1]) translate([side*50,0,0])
        //    cylinder(d=32,h=16);
        
        translate([0,0,trimZ-200]) cube([400,400,400],center=true);
    }
}

frame_T_mounted();

