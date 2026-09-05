// Tactile-UMI parallel-jaw gripper housing.
//
// Frame: rail runs along X (jaw travel axis), Z is up (toward the F/T sensor
// and wrist camera), fingers point in -Z... in this model the fingers hang
// below the rail (-Z) and the handle/trigger is behind (-Y).
//
// Parts (select with -D part="..."):
//   rail_base        MGN7H rail bracket with F/T-plate dovetail and rod bores
//   carriage_adapter finger mount for one MGN7H carriage, female dovetail + M4 thumb screw
//   trigger          spring-loaded index-finger trigger with pivot
//   ft_plate         wrist F/T sensor adapter plate (female dovetail)
//   all              exploded layout of every part
//
//   openscad -o stl/rail_base.stl -D part=\"rail_base\" gripper_housing.scad

include <common.scad>

part = "all";

// ---- Dimensions -------------------------------------------------------------
rail_len      = stroke_mm + 2 * mgn7h_carriage_l + 10;   // both carriages fully apart
base_w        = 34;                                       // Y
base_t        = 6;                                        // Z, under the rail
rod_spacing   = 22;                                       // carbon rods flanking the rail
handle_len    = 100;                                      // -Y handle
handle_d      = 30;
trigger_len   = 60;
trigger_w     = 20;
trigger_t     = 8;
pivot_d       = 4.1;                                      // M4 pivot pin
adapter_h     = 14;                                       // carriage top -> finger dovetail face

// ---- Rail base -------------------------------------------------------------
module rail_holes() {
    n = floor((rail_len - 10) / mgn7_rail_hole_pitch);
    for (i = [0 : n])
        translate([-rail_len / 2 + 5 + i * mgn7_rail_hole_pitch, 0, -eps]) {
            cylinder(d = mgn7_rail_hole_d, h = base_t + 1);
            // M2 head counterbore from below
            cylinder(d = m2_head_d, h = 2.2);
        }
}

module rail_base() {
    difference() {
        union() {
            // Plate under the rail
            translate([-rail_len / 2, -base_w / 2, 0]) cube([rail_len, base_w, base_t]);
            // Rail seating ridge (locates the 7 mm rail)
            translate([-rail_len / 2, -(mgn7_rail_w + 4) / 2, base_t]) cube([rail_len, mgn7_rail_w + 4, 1.5]);
            // Centre tower up to the F/T sensor plate
            translate([-20, -base_w / 2, base_t]) cube([40, base_w, 18]);
        }
        // Rail channel through the ridge
        translate([-rail_len / 2 - 1, -mgn7_rail_w / 2 - 0.1, base_t - eps]) cube([rail_len + 2, mgn7_rail_w + 0.2, 3]);
        rail_holes();
        // Carbon rod bores (stiffening rods along X, either side of the rail)
        for (sy = [-1, 1])
            translate([-rail_len / 2 - 1, sy * rod_spacing / 2, base_t / 2 + 6])
                rotate([0, 90, 0]) cylinder(d = cf_rod_d, h = rail_len + 2);
        // Female dovetail for the F/T sensor plate on the tower top face (z = base_t + 18)
        translate([0, 0, base_t + 18]) rotate([0, 0, 90]) dovetail_female_cutter(len = base_w + 2);
        // M4 thumb screw entering the dovetail from +X
        translate([-25, 0, base_t + 18 - dt_depth / 2]) rotate([0, 90, 0]) cylinder(d = m4_thumbscrew_tap_d, h = 30);
        // Handle attachment inserts (M2.5) on the -Y face of the tower
        for (sx = [-1, 1], z = [base_t + 5, base_t + 13])
            translate([sx * 12, -base_w / 2 + eps, z]) rotate([90, 0, 0]) heatset_m25();
        // Encoder mount: AS5048A board (magnet on the jaw carriage) inserts, +Y face
        for (sx = [-1, 1])
            translate([sx * 7, base_w / 2 - eps, base_t + 9]) rotate([-90, 0, 0]) heatset_m25();
        // Lightening pockets under the plate
        for (i = [-2 : 2])
            translate([i * 28, 0, -eps]) linear_extrude(base_t / 2) square([18, base_w - 2 * wall - 2 * cf_rod_d], center = true);
    }
}

// ---- Carriage adapter: MGN7H carriage -> finger dovetail --------------------
module carriage_adapter() {
    w = mgn7h_carriage_w + 2 * wall;
    l = mgn7h_carriage_l;
    difference() {
        union() {
            // Block over the carriage
            translate([-l / 2, -w / 2, 0]) cube([l, w, adapter_h]);
            // Downward arm carrying the finger (fingers hang below the rail)
            translate([-l / 2, -w / 2, -22]) cube([l, wall + 4, 22 + eps]);
        }
        // Carriage screw holes (M2, 12 x 8 pattern) from the top
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx * mgn7h_hole_pitch_l / 2, sy * mgn7h_hole_pitch_w / 2, -eps]) {
                cylinder(d = m2_clearance_d, h = adapter_h + 1);
                translate([0, 0, adapter_h - 2.5]) cylinder(d = m2_head_d, h = 3);
            }
        // Female dovetail on the arm's -Y face... finger module dovetail runs
        // vertically (Z) so the finger slides down onto the arm.
        translate([0, -w / 2 + eps, -12]) rotate([90, 0, 0]) rotate([0, 0, 90]) dovetail_female_cutter(len = 26);
        // Thumb screw (M4) from the +Y side of the arm into the dovetail
        translate([0, -w / 2 + wall + 4 + 1, -12]) rotate([90, 0, 0]) cylinder(d = m4_thumbscrew_tap_d, h = 10);
        // Encoder magnet pocket (6 mm diametric magnet) on the +Y face
        translate([0, w / 2 - 3, adapter_h / 2]) rotate([-90, 0, 0]) cylinder(d = 6.2, h = 4);
        // Spring anchor
        translate([0, 0, adapter_h - 3]) rotate([0, 90, 0]) cylinder(d = 2.5, h = l + 2, center = true);
    }
}

// ---- Trigger ---------------------------------------------------------------
module trigger() {
    difference() {
        union() {
            hull() {
                cylinder(d = trigger_w, h = trigger_t);                    // pivot boss
                translate([trigger_len - 8, -6, 0]) cylinder(d = 16, h = trigger_t);  // finger pad
            }
            // Cable/tendon tab connecting to the carriage adapter
            translate([-4, 6, 0]) cube([8, 10, trigger_t]);
        }
        translate([0, 0, -eps]) cylinder(d = pivot_d, h = trigger_t + 1);   // pivot
        translate([0, 14, trigger_t / 2]) rotate([90, 0, 0]) cylinder(d = 2.2, h = 8, center = true); // tendon hole
        // Finger-pad scallop
        translate([trigger_len - 8, -6 - 12, -1]) cylinder(d = 18, h = trigger_t + 2);
        // Return-spring hook
        translate([12, 0, trigger_t / 2]) rotate([0, 90, 0]) cylinder(d = 2.5, h = 6, center = true);
    }
}

// ---- F/T sensor plate -----------------------------------------------------
module ft_plate() {
    d = 50;   // typical compact 6-axis F/T flange
    difference() {
        union() {
            cylinder(d = d, h = 6);
            // Male dovetail on the underside, mates with the rail-base tower
            translate([0, 0, 0]) mirror([0, 0, 1]) rotate([0, 0, 90]) dovetail_male(len = base_w - 2);
        }
        // Generic 4 x M3 flange pattern on a 36 mm circle (adjust to sensor datasheet)
        for (a = [45 : 90 : 315])
            translate([18 * cos(a), 18 * sin(a), -eps]) cylinder(d = 3.4, h = 7);
        translate([0, 0, -eps]) cylinder(d = 10, h = 7);   // cable pass-through
    }
}

// ---- Layout ----------------------------------------------------------------
if (part == "rail_base") rail_base();
if (part == "carriage_adapter") carriage_adapter();
if (part == "trigger") trigger();
if (part == "ft_plate") ft_plate();
if (part == "all") {
    rail_base();
    translate([-stroke_mm / 2 - mgn7h_carriage_l / 2, 0, mgn7h_carriage_h + base_t]) carriage_adapter();
    translate([ stroke_mm / 2 + mgn7h_carriage_l / 2, 0, mgn7h_carriage_h + base_t]) mirror([0, 1, 0]) carriage_adapter();
    translate([0, -base_w / 2 - 25, 0]) rotate([90, 0, 0]) trigger();
    translate([0, 0, base_t + 18 + dt_depth + 15]) ft_plate();
}
