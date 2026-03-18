/*
 Nanomoose frame-to-tool interface

*/
overscale=1/4; // scale factor to real version
sinch=25.4*overscale; // scaled inches

frameW = 1.0*sinch; // width
frameH = 2.0*sinch; // Z height (thicker for better stiffness)
frameN = 0.5*sinch; // narrow width for selected frame areas

armclear=0.3; // clearance between moving parts of arm

mainframeDY=19*sinch; // half the front-back edge-to-edge length
mainframeDX=14*sinch; // half the left-right edge-to-edge width
subframeDX=6.5*sinch; // half the left-right subframe width (outside to outside)


gearZ=8.0; // Z height of gear teeth

armAxleOD=3.0; // diameter of arm pivot bolts
armAxleTap=2.6; // tap diameter

armTiltDY=18; // distance up on gear to arm tilt linkage
armpivotOD = 6; // size of arm pivot points (same as M3 head)




// Make a frame-type box, centered, with these dimensions
module mainframeStick(dim,enlarge=0,bevel=0,threeD=0) {    
    if (threeD) {
        // Direct to 3D version:
        bevelcube(dim-2*enlarge*[1,1,1],center=true,bevel=bevel);
    }
    else {
        // 2D version, allowing bevels
        square([dim[0],dim[1]]-2*enlarge*[1,1],center=true);
    }
}


