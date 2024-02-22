/**
 Aurora Robotics: Excahauler Collision Configurations and Math Functions

 Andrew C. Mattson, acmattson3@alaska.edu
 Orion Sky Lawlor, lawlor@alaska.edu, 2014--2023 (Public Domain)
*/

/*** CONSTANTS ***/
// Buffer distance between moving parts
const static float SAFE_DIST = 0.03f; // Safety gap between moving parts

// Part parameters
const static float MINING_HEAD_R = 0.09f; // Radius of mining head

// Parent-relative offset points
const static vec3 TOOL_BACK_LOWER = vec3(0,-0.442f,0);
const static vec3 TOOL_BACK_UPPER = vec3(0,-0.502f,0.24f); 
const static vec3 MINING_HEAD_MID = vec3(0,-0.05f, 0.03f); // Tip-relative head center

// Hazardous points (scoop relative)
const static vec3 SCOOP_HAZ_UPPER = vec3(0,0.02f,0.275f);
const static vec3 SCOOP_HAZ_MID = vec3(0,-0.015f,-0.122f);
const static vec3 SCOOP_HAZ_LOWER = vec3(0,0.333f,-0.09f);
const static vec3 SCOOP_HAZ_OUTER = vec3(0,0.142f,0.243f); // Only used for spin

// Hazardous points (boom relative)
const static vec3 BOOM_HAZ_LOWER = vec3(0,0,0); // Base of boom
const static vec3 BOOM_HAZ_UPPER = vec3(0,0,0.25f); // Upper boom


/*** FUNCTIONS ***/

// Some of the following functions could be included in vec2.h
float dist_squared(vec2 v, vec2 w) {
    return (v.x-w.x)*(v.x-w.x)+(v.y-w.y)*(v.y-w.y);
}

float dist(vec2 v, vec2 w) {
    return sqrt(dist_squared(v, w));
}

// Distance between (line between v and w) and (point p)
// Code derived from code by Grumdrig, 2021
float point_to_line_dist_2D(vec2 v, vec2 w, vec2 p) {
    float len2 = dist_squared(v, w);
    if (len2 < 0.0001f) return dist(p, v);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const float t_res = dot(p - v, w - v) / len2;
    const float t_min = t_res>1.0f ? 1.0f : t_res;
    const float t = t_min<0 ? 0 : t_min;
    const vec2 projection = v + (w - v)*t;  // Projection falls on the segment
    return dist(p, projection);
}

// We don't use the x-axis for collision detection (yet).
float point_to_line_dist(vec3 v, vec3 w, vec3 p) {
    return point_to_line_dist_2D(vec2(v.y,v.z), vec2(w.y,w.z), vec2(p.y,p.z));
}