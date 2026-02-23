/*
 Interface using sprocket-style circular projections to transmit torque. 
 
*/
include <AuroraSCAD/sprocket.scad>


timing=0.0; // tooth advance, for animation


/*
Drive pin diameter is approx 0.2 inches or 5mm here.
 3mm metric gear module = 9.425mm circular pitch
 ANSI #35 roller chain = 3/8 inch = 9.525mm pitch
    Roller diameter 0.200" = 5.08mm diameter
*/
size=35; // Similar to ANSI #35 roller sprocket

sprocket_scale=0.9375; //<- only for 3D printing, lets us dial in spacing
echo("Roller diameter = ",sprocket_scale*get_roller_diameter_inch(size)," inch");


overallOD=65; // basic part diameter (used for cuts)


gearZ = 12.0; // vertical height of gear teeth 
helixPer = 2.0; // degrees of rotation per mm of gear height

nring=18; // teeth in bottom ring gear

// Vaguely similar spur gear shape:
geartype = geartype_create(3,gearZ-0.5,20,0.15,0.20); // Stubby gears
tooth_clearance=0.1; // clearance between gear teeth (applied to both sides)

// bottom (B) and top (T) gearplanes
space=2; // fewer teeth in sun gear (gives space for gear tips to clear)
skip=1; // more teeth in output plane (sets ratio)
gB = [geartype,-nring+space,nring,1]; // bottom (B) gearplane
gRB = gearplane_Rgear(gB); // ring gear
gPB = gearplane_Pgear(gB); // planet gear

gT = [geartype,-nring-skip+space,nring+skip,1]; // top (T) gearplane
gRT = gearplane_Rgear(gT); // ring gear
gPT = gearplane_Pgear(gT); // planet gear

teethPB=nring-space; // planet bottom
teethRB=nring; // ring bottom
teethPT=nring+skip-space; // planet top
teethRT=nring+skip; // ring top

pitchRIB = get_pitch_radius_mm(size,teethPB);
pitchROB = get_pitch_radius_mm(size,teethRB);
pitchRIT = get_pitch_radius_mm(size,teethPT);
pitchROT = get_pitch_radius_mm(size,teethRT);

Pshift = pitchROB-pitchRIB; // planet center shifted this far
echo("Bottom planet shift: ",Pshift);
echo("Top planet shift: ",pitchROT-pitchRIT);

mateR = 2.0; // amount of tooth engagement on each side of centerline
handoffInset=1.0; // shift teeth this far (bigger=easier clearance, but more axial force)
handoffRB=pitchROB-handoffInset; // bottom teeth are centered around this radius
handoffRT=pitchROT-handoffInset; // top teeth are centered around this radius 
handoffS=0.3; // space after handoff (for debris / air to freely escape)

roundI=1.0; // rounding on inside corners (for strength)
roundO=1.2; // rounding on outside corners (to clear edges)

// Convert tooth count to outside radius (OR).
function OR_from_tooth(teeth,isring=0) = get_pitch_radius_mm(size,teeth)-handoffInset+mateR+isring*handoffS+tooth_clearance;


// Planet gear sprocket
module planet_sprocket2D(teeth,handoffR)
{
    difference() {
        offset(r=-roundI) offset(r=+roundI-tooth_clearance)
        offset(r=+roundO) offset(r=-roundO)
        intersection() {
            union() {
                rotate([0,0,360/teeth*timing])
                sprocket_plate2D(size,teeth);
                circle(r=handoffR-mateR-handoffS-Pshift); // fill in gullets
            }
            
            circle(r=handoffR+mateR-Pshift); // trim circle
        }
        // circle(d=37+0.2); // 6706 bearing OD
    }
}

// Ring gear pegs (negative space cut)
module ring_pegs2D(teeth,handoffR)
{
    pR=get_pitch_radius_mm(size,teeth); // pitch radius
    rR=inches2mm(get_roller_diameter_inch(size)/2); // roller radius
    
    offset(r=-roundO) offset(r=+roundO+tooth_clearance)
    offset(r=+roundI) offset(r=-roundI)
    intersection()
    {
        difference() {
            circle(d=overallOD);
            
            difference() {
                rotate([0,0,360/teeth*timing])
                for (ang=[0:360/teeth:360-1]) rotate([0,0,ang])
                    translate([0,pR,0]) circle(r=rR);
                circle(r=handoffR-mateR); // trim circles
            }
        }
        
        circle(r=handoffR+mateR+handoffS); // trim back gullets
    }
}




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
    translate([0,0,gBZ]) { // bottom gearplane
        translate([0,Pshift,0])
            planet_sprocket2D(teethPB,handoffRB);

        difference() {
            circle(d=overallOD);
            ring_pegs2D(teethRB,handoffRB);
        }
    }
    color([1,0,1])
    translate([0,0,gTZ]) { // top gearplane
        translate([0,Pshift,0])
            planet_sprocket2D(teethPT,handoffRT);

        difference() {
            circle(d=overallOD);
            ring_pegs2D(teethRT,handoffRT);
        }
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
function tooth_ID_gRT() = sprocket_scale*2*(handoffRT-mateR-handoffS-tooth_clearance);
function tooth_OD_gRT() = sprocket_scale*2*(handoffRT+mateR+handoffS+tooth_clearance);
function solidOD() = sprocket_scale*2*(handoffRB-Pshift-mateR-handoffS-tooth_clearance);

// 3D geometry generation
module envelope_sprocket3D(teeth,height,isring) {
    OD=2*OR_from_tooth(teeth,isring);
    
    bevelcylinder(d=OD,h=height,bevel=isring?0.75*extendGZ:extendGZ);
}


// 3D planet gear (positive teeth)
module planet_sprocket3D(teeth,handoff,height) {
    scale(sprocket_scale*[1,1,0]+[0,0,1])
    intersection() {
        linear_extrude(height=height,convexity=4) {
            rotate([0,0,-90]) planet_sprocket2D(teeth,handoff);
        }
        envelope_sprocket3D(teeth,height,isring=0);
    }
}

// 3D ring gear cut (negative space)
module ring_pegs3D(teeth,handoff,height) {
    scale(sprocket_scale*[1,1,0]+[0,0,1])
    intersection() {
        linear_extrude(height=height,convexity=4) {
            rotate([0,0,-90]) ring_pegs2D(teeth,handoff);
        }
        envelope_sprocket3D(teeth,height,isring=1);
    }
}




// Geometry: negative space for teeth
module toothcut_ring_bottom(height) { ring_pegs3D(teethRB,handoffRB,height); }
module toothcut_ring_top(height) { ring_pegs3D(teethRT,handoffRT,height); }

// Translation: shift to center point
module toothspot_bottom() { translate([sprocket_scale*Pshift,0,0]) children(); }
module toothspot_top() { translate([sprocket_scale*Pshift,0,0]) children(); }

// Geometry: positive space for spur teeth (solid)
module tooth_planet_bottom(height) { planet_sprocket3D(teethPB,handoffRB,height); }
module tooth_planet_top(height) { planet_sprocket3D(teethPT,handoffRT,height); }









