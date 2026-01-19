/*
 Interface using toothed gears to transmit torque. 
 
*/

gearZ = 12.0; // vertical height of gear teeth 

nring=25; // teeth in planet gear

planet_gearclear=0.05; // + gives planet gear clearance: printer adds enough for assembly


/*
Another option:
 3mm metric gear module = 9.425mm circular pitch
 ANSI #35 roller chain = 3/8 inch = 9.525mm pitch
    Roller diameter 0.200" = 5.08mm diameter
*/
geartype = geartype_create(1.75,gearZ-0.5,20,0.2,0.25); // Stubby module 1.0 gears
tooth_clearance=0.0; // clearance between gear teeth

// bottom (B) and top (T) gearplanes
space=3; // fewer teeth in sun gear (gives space for gear tips to clear)
skip=2; // more teeth in output plane (sets ratio)
gB = [geartype,-nring+space,nring,1]; // bottom (B) gearplane
gRB = gearplane_Rgear(gB); // ring gear
gPB = gearplane_Pgear(gB); // planet gear

gT = [geartype,-nring-skip+space,nring+skip,1]; // top (T) gearplane
gRT = gearplane_Rgear(gT); // ring gear
gPT = gearplane_Pgear(gT); // planet gear



// Print the orbit radius values for the bottom and top.  
//   The orbit radii need to match.
module echo_orbits() {
    gearplane_print(gB, "Bottom (input) gears");
    gearplane_print(gT, "Top (output) gears");
    
    rB = gearplane_ratio_Rfixed(gB);
    rT = gearplane_ratio_Rfixed(gT);
    echo("OrbitB = ",gearplane_Oradius(gB),"  ratioB ",rB);
    echo("OrbitT = ",gearplane_Oradius(gT),"  ratioT ",rT);
    
    // Actual gear reduction ratio as measured: about 1:448
    //echo("Overall axle reduction ratio: ",1/(rB-rT));
    eRatio = nring / space; // reduction ratio axle to eccentric revs
    rRatio = nring / skip; // reduction ratio eccentric to ring shift
    echo("Approx ratio: ",eRatio," * ",rRatio," = ",eRatio*rRatio);
}


/* Demo gearplanes in 2D */
module demo2D()
{
    // Bottom gearplane (yellow)
    difference() { 
        gearplane_2D(gB,clearance=tooth_clearance); 
        gearplane_planets(gB) circle(d=bearingOD(axle_bearing)); 
    }

    // Top gearplane (purple)
    translate([0,0,gearZ]) color([1,0,1]) 
        difference() { 
            gearplane_2D(gT,clearance=tooth_clearance); 
            gearplane_planets(gT) circle(d=bearingOD(axle_bearing)); 
        }
    
    #mainbolt_centersI() circle(d=mainheadODI);
    #translate([0,0,botZ]) bearing3D(main_bearing);
    //#cube([75,25.4,1],center=true); // 1x1 steel frame
}


/* Small parts to demonstrate principle of operation */    
module printable_demo() {
    gearclear=0.1; // planet gear clearance
    clearance=0.15; // space for moving parts
    rB = gearplane_Rgear(gB);
    rT = gearplane_Rgear(gT);
    
    // Eccentric planet gears
    difference() {
        union() {
            toothspot_bottom() tooth_planet_bottom(gearZ);
            translate([0,0,gearZ])
                toothspot_top() tooth_planet_top(gearZ);
        }
        // Thru clearance for bearing (etc)
        toothspot_bottom() 
            cylinder(d=bearingOD(axle_bearing),h=100,center=true);
    }
    
    // Bottom ring
    translate([0,-50,0]) {
        linear_extrude(height=gearZ) {
            gearplane_ring_2D(gB);
            translate([gear_OR(rB),0,0]) square([5,2]);
        }
        linear_extrude(height=2*gearZ) 
        difference() { // shell to hold bottom and top together
            rim=2; // baked into gearplane_ring_2D
            circle(r=gear_OR(rB)+rim+0.5);
            circle(r=gear_OR(rT)+rim+clearance);
        }
    }
    
    // Top ring (fits in top of bottom ring)
    translate([0,+50,0])
    linear_extrude(height=gearZ) {
        gearplane_ring_2D(gT);
        //translate([gear_OR(rT),0,0]) square([4,2]);
    }
    
    
}

// Accessors for properties 
function tooth_ID_gRT() = gear_ID(gRT);
function solidOD() = min(gear_ID(gPB),gear_ID(gPT));

// Geometry: negative space for teeth
module toothcut_ring_top(height) { gear_3D(gRT,height=height); }
module toothcut_ring_bottom(height) { gear_3D(gRB,height=height); }

// Translation: shift to center point
module toothspot_bottom() { gearplane_planets(gB) children(); }
module toothspot_top() { gearplane_planets(gT) children(); }

// Geometry: positive space for spur teeth (solid)
module tooth_planet_bottom(height) { gear_3D(gPB,height=height,clearance=planet_gearclear); }
module tooth_planet_top(height) { gear_3D(gPT,height=height,clearance=planet_gearclear); }









