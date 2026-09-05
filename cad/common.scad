// Tactile-UMI shared CAD parameters and primitives (OpenSCAD, millimetres).
// Everything structural is designed for PETG / PA-CF FDM printing.

$fn = 48;
eps = 0.01;

// ---- Fasteners --------------------------------------------------------------
m25_heatset_d      = 3.5;   // hole for M2.5 heat-set insert (3.5 x 4 mm inserts)
m25_heatset_depth  = 4.2;
m25_clearance_d    = 2.8;   // through-hole for M2.5 screw
m2_clearance_d     = 2.3;   // MGN7 rail / carriage use M2 screws
m2_head_d          = 3.9;   // M2 socket head counterbore
m4_thumbscrew_tap_d = 3.3;  // M4 heat-set/tapped for the dovetail thumb screw

module heatset_m25(depth = m25_heatset_depth) {
    // Cutter, oriented along -Z from the origin surface.
    translate([0, 0, -depth]) cylinder(d = m25_heatset_d, h = depth + eps);
}

// ---- MGN7H miniature linear rail --------------------------------------------
mgn7_rail_w        = 7;
mgn7_rail_h        = 4.8;
mgn7_rail_hole_pitch = 15;
mgn7_rail_hole_d   = 2.4;   // M2 through
mgn7_rail_cbore_d  = 4.2;   // countersunk/cbore in the rail itself (reference only)
mgn7h_carriage_w   = 17;
mgn7h_carriage_l   = 30.8;
mgn7h_carriage_h   = 8;     // total height rail bottom -> carriage top
mgn7h_hole_pitch_w = 12;    // carriage tapped holes, across
mgn7h_hole_pitch_l = 8;     // carriage tapped holes, along

// ---- Kinematics ---------------------------------------------------------------
stroke_mm          = 85;    // 0..85 mm parallel opening (blueprint)
finger_pitch_closed = 0;    // jaws touch at 0

// ---- Quick-swap dovetail ----------------------------------------------------
// Male profile on the finger module, female pocket on the carriage adapter and
// the F/T plate. 60 degree flanks, self-centering, locked by an M4 thumb screw.
dt_top_w      = 12;         // width at the narrow (outer) face
dt_depth      = 4;          // engagement depth
dt_angle      = 60;         // flank angle from the base plane
dt_len        = 14;         // sliding length
dt_clearance  = 0.15;       // per-side printing clearance for the female side
dt_base_w     = dt_top_w + 2 * dt_depth / tan(dt_angle);

// 2D trapezoid, wide side at y=0 (against the part), narrow side at y=depth.
module dovetail_profile(clr = 0) {
    w0 = dt_base_w + 2 * clr;
    w1 = dt_top_w + 2 * clr;
    polygon([[-w0 / 2, 0], [w0 / 2, 0], [w1 / 2, dt_depth + clr], [-w1 / 2, dt_depth + clr]]);
}

// Male dovetail: extruded along X, protruding in +Z from z=0.
module dovetail_male(len = dt_len) {
    rotate([90, 0, 90]) translate([0, 0, -len / 2])
        linear_extrude(len) dovetail_profile(0);
}

// Female dovetail cutter (with clearance), extruded along X, cut into a body
// whose mating face is at z=0 (pocket goes into -Z).
module dovetail_female_cutter(len = dt_len + 1) {
    mirror([0, 0, 1]) rotate([90, 0, 90]) translate([0, 0, -len / 2])
        linear_extrude(len) dovetail_profile(dt_clearance);
}

// ---- Camera: Raspberry Pi Camera Module 3 (also fits NoIR) -----------------
picam_pcb_w        = 25;
picam_pcb_h        = 24;
picam_pcb_t        = 1.0;
picam_hole_pitch_x = 21;    // 4 holes, 2.2 mm dia (M2), pattern 21 x 12.5
picam_hole_pitch_y = 12.5;
picam_hole_d       = 2.2;
picam_lens_d       = 8.5;   // lens barrel housing diameter
picam_lens_h       = 4.5;   // barrel height above PCB
picam_lens_offset_y = -1.5; // lens centre relative to PCB centre (towards connector side)
picam_conn_w       = 16;    // FPC connector footprint along the bottom edge
picam_conn_depth   = 6;

// ---- GelSight optical finger -------------------------------------------------
gel_w              = 20;    // touch surface (X)
gel_h              = 15;    // touch surface (Y)
gel_t              = 4;     // cast silicone thickness
gel_lip            = 1.5;   // retaining lip around the gel window
acrylic_t          = 2;     // clear backing plate under the gel
cam_standoff       = 35;    // gel surface -> camera lens (blueprint)
led_angle_deg      = 30;    // grazing angle relative to gel surface
led_d              = 3.2;   // 3 mm LED (or 2 mm channel for SMD on flex ring)
led_channel_len    = 14;

// ---- Structure ---------------------------------------------------------------
wall               = 3;
cf_rod_d           = 6.1;   // 6 mm carbon fibre rod bores
