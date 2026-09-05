// GelSight-style optical finger module for the Tactile-UMI gripper.
//
// Coordinate frame: gel surface is the +Z face at z=0 (contact plane),
// optical axis is -Z, camera lens sits at z = -cam_standoff. The male
// quick-swap dovetail is on the back (-Y) face so the module slides onto the
// carriage adapter and locks with an M4 thumb screw.
//
//   openscad -o stl/finger_module.stl finger_module.scad
//   openscad -o stl/finger_module.stl -D part=\"body\" finger_module.scad
//   part = "body" | "camera_cap" | "gel_frame" | "all"

include <common.scad>

part = "all";

// Derived dimensions ---------------------------------------------------------
body_w   = gel_w + 2 * (wall + 4);            // X: room for LED channels either side
body_d   = gel_h + 2 * (wall + 4);            // Y
body_h   = cam_standoff + picam_lens_h + picam_pcb_t + 2;  // gel plane -> PCB back
cavity_top_w = gel_w + 2 * gel_lip;           // gel pocket
cavity_top_d = gel_h + 2 * gel_lip;
cavity_bot_w = picam_lens_d + 2;              // aperture around the lens barrel
cavity_bot_d = picam_lens_d + 2;
pcb_pocket_z = -(cam_standoff + picam_lens_h);  // PCB top face

// Tapered optical cavity: gel window down to the lens aperture.
module optical_cavity() {
    hull() {
        translate([0, 0, -eps]) linear_extrude(eps)
            square([cavity_top_w, cavity_top_d], center = true);
        translate([0, 0, pcb_pocket_z]) linear_extrude(eps)
            square([cavity_bot_w, cavity_bot_d], center = true);
    }
    // Gel + acrylic pocket (stepped) at the top.
    translate([0, 0, -(gel_t + acrylic_t)])
        linear_extrude(gel_t + acrylic_t + eps)
            square([gel_w + 2 * gel_lip + 0.3, gel_h + 2 * gel_lip + 0.3], center = true);
    // Contact window through the lip (gel protrudes here).
    translate([0, 0, -(gel_t + acrylic_t) - eps])
        linear_extrude(gel_t + acrylic_t + 2)
            square([gel_w, gel_h], center = true);
}

// One LED channel aimed at the gel centre from a grazing angle.
// dir: unit vector in the XY plane pointing from the gel centre toward the LED.
module led_channel(dir) {
    ang = atan2(dir[1], dir[0]);
    // Channel starts just below the gel underside, outside the cavity.
    r0 = max(cavity_top_w, cavity_top_d) / 2 + 1.5;
    rotate([0, 0, ang])
        translate([r0, 0, -(gel_t + acrylic_t) - 1.0])
            rotate([0, -(90 - led_angle_deg), 0])   // tilt so the beam grazes at led_angle_deg
                translate([0, 0, -led_channel_len]) cylinder(d = led_d, h = led_channel_len + 6);
    // Wire exit to the back face.
    rotate([0, 0, ang])
        translate([r0 + 2, 0, -(gel_t + acrylic_t) - 6])
            cylinder(d = 2.2, h = 8);
}

module led_channels() {
    led_channel([ 1,  0]);   // Red   +X
    led_channel([ 0,  1]);   // Green +Y
    led_channel([-1,  0]);   // Blue  -X
    led_channel([ 0, -1]);   // Blue  -Y
}

module camera_pocket() {
    // PCB pocket open towards -Z (camera slides in from below, held by cap).
    translate([0, 0, pcb_pocket_z - picab_pcb_t_safe() - 6])
        linear_extrude(picab_pcb_t_safe() + 6 + eps)
            square([picam_pcb_w + 0.4, picam_pcb_h + 0.4], center = true);
    // FPC connector relief along the -Y edge.
    translate([-picam_conn_w / 2, -picam_pcb_h / 2 - 0.2 - picam_conn_depth, pcb_pocket_z - 8])
        cube([picam_conn_w, picam_conn_depth + 1, 8]);
    // Heat-set inserts for the 4 PCB screws (M2.5 inserts; PCB holes are 2.2 mm,
    // use M2 screws into M2.5 inserts or drill PCB - inserts are specified by
    // the blueprint).
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * picam_hole_pitch_x / 2, sy * picam_hole_pitch_y / 2 + picam_lens_offset_y, pcb_pocket_z + eps])
            heatset_m25();
}
function picab_pcb_t_safe() = picam_pcb_t + 0.3;

module thumbscrew_boss() {
    // M4 tapped hole through the dovetail root from the +Z side... on the finger
    // the screw is in the female part (carriage adapter); the male side carries
    // a shallow detent so the screw tip seats.
    translate([0, -body_d / 2 - dt_depth + 1.2, -body_h / 2])
        rotate([90, 0, 0]) cylinder(d = 3, h = 2);
}

module finger_body() {
    difference() {
        union() {
            // Main block, gel plane at z=0, extends to -body_h.
            translate([-body_w / 2, -body_d / 2, -body_h]) cube([body_w, body_d, body_h]);
            // Male dovetail on the back (-Y) face, running along Z.
            translate([0, -body_d / 2, -body_h / 2])
                rotate([90, 0, 0]) rotate([0, 0, 90]) dovetail_male(len = body_h - 10);
        }
        optical_cavity();
        led_channels();
        camera_pocket();
        thumbscrew_boss();
        // Chamfer the fingertip edges for grasping clearance.
        for (sx = [-1, 1])
            translate([sx * body_w / 2, 0, 0]) rotate([0, 45, 0]) cube([3, body_d + 2, 3], center = true);
    }
}

// Bottom cap retaining the camera PCB (screws into the heat-set inserts).
module camera_cap() {
    difference() {
        translate([-body_w / 2, -body_d / 2, 0]) cube([body_w, body_d, 3]);
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx * picam_hole_pitch_x / 2, sy * picam_hole_pitch_y / 2 + picam_lens_offset_y, -eps])
                cylinder(d = m25_clearance_d, h = 4);
        // FPC cable slot
        translate([-picam_conn_w / 2 - 1, -picam_pcb_h / 2 - 3, -eps]) cube([picam_conn_w + 2, 4, 4]);
    }
}

// Thin frame clamping the gel/acrylic stack from the contact side.
module gel_frame() {
    difference() {
        translate([-cavity_top_w / 2 - 2, -cavity_top_d / 2 - 2, 0]) cube([cavity_top_w + 4, cavity_top_d + 4, 1.2]);
        translate([0, 0, -eps]) linear_extrude(2) square([gel_w, gel_h], center = true);
    }
}

if (part == "body" || part == "all") finger_body();
if (part == "camera_cap") camera_cap();
if (part == "gel_frame") gel_frame();
if (part == "all") {
    translate([body_w + 10, 0, 0]) camera_cap();
    translate([-(body_w + 10), 0, 0]) gel_frame();
}
