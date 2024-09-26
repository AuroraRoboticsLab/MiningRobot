/*
Print-in-place origami truss

Print to z==1, add a sheet of plastic window screen, resume printing.

Original design by Andrew Mattson

*/
$fs=0.1; $fa=5;



dY = 32; // change in Y per half-diamond
dX = 12; // change in X per half-diamond

// Thickness of various line types
valleyOD = 2;
ridgeOD = 2;
diagOD = 1.6;

// Endpoint inset of various line types
valleyIN = 3; 
ridgeIN = 2;
diagIN = 2;
valley_space=2;
border_space=2;

// Draw a rounded line between these two points
module make_line(p1,p2,OD,inset=0)
{
    fullspan=p2-p1;
    dir=fullspan/norm(fullspan);
    
    hull() {
        for (p=[p1+inset*dir,p2-inset*dir]) translate(p) circle(d=OD);
    }
}


// Draw one full diamond shape around the origin
module make_diamond() {
    // Top, left, center, right, bottom:
                T = [0,dY];
    L = [-dX,0]; C=[0,0];  R=[+dX,0];
                B = [0,-dY];
    
    round=0.6;
    offset(r=+round) offset(r=-round)
    difference() {
        intersection() {
            union() {
                for (p=[ [T,L], [L,B], [B,R], [R,T] ])
                    make_line(p[0], p[1], ridgeOD, ridgeIN);
                
                for (p=[ [L,C], [C,R] ])
                    make_line(p[0], p[1], diagOD, diagIN);
            }
            
            // Keep away from adjacent patterns
            square([2*dX-valleyOD-2*border_space,2*dY-2*border_space],center=true);
        }
        
        // Cut the valley out of the other lines
        make_line(T,B, valleyOD+2*valley_space, valleyIN);
    }
    
    // solid valley
    make_line(T,B, valleyOD, valleyIN);
}

// Make children at a grid of these dimensions
module make_grid(nX, nY) {
    for (y=[0:nY])
        for (x=[0:nX])
        if (y%2==0)
            translate([(2*x)*dX, y*dY])  children();
        else if (x<nX)
            translate([(2*x+1)*dX, y*dY])  children();
}

linear_extrude(height=2,convexity=10) make_grid(4,4) make_diamond();




