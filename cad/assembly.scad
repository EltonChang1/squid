// Visual assembly of the Tactile-UMI gripper: rail base, two carriage adapters
// at a given jaw opening, two GelSight finger modules, trigger and F/T plate.
//
//   openscad -o stl/assembly.stl -D opening=40 assembly.scad
//   (preview in the GUI with F5; opening is the jaw gap in mm, 0..85)

include <common.scad>
use <gripper_housing.scad>
use <finger_module.scad>

opening = 40;

// Mirrors of module-local dimensions (use<> does not import variables).
base_w    = 34;
base_t    = 6;
adapter_w = mgn7h_carriage_w + 2 * wall;                       // 23
finger_h  = cam_standoff + picam_lens_h + picam_pcb_t + 2;     // 42.5, gel plane -> PCB back
finger_d  = gel_h + 2 * (wall + 4);                            // 29, along the dovetail normal
dt_z      = -12;                                               // dovetail centre on the adapter arm

color("SlateGray") rail_base();

for (side = [-1, 1]) {
    ax = side * (opening / 2 + finger_h / 2);   // adapter x so the gel plane sits at +/- opening/2
    translate([ax, 0, mgn7h_carriage_h + base_t]) {
        color("DimGray") carriage_adapter();
        // Finger: gel normal toward the centre line, dovetail (finger -Y face)
        // pointing +Y into the adapter arm's female pocket.
        translate([-side * finger_h / 2, -adapter_w / 2 - finger_d / 2, dt_z])
            rotate([180, 0, 0]) rotate([0, -side * 90, 0])
                color("Tomato") finger_body();
    }
}

color("SteelBlue") translate([0, -base_w / 2 - 25, 0]) rotate([90, 0, 0]) trigger();
color("Gold") translate([0, 0, base_t + 18 + dt_depth]) ft_plate();

// Wrist camera stand-in (RealSense D405-ish box) above the F/T plate
color("Black", 0.6) translate([-21, -10, base_t + 18 + dt_depth + 6]) cube([42, 20, 23]);
