$fn = 48;

pcb_w = 92;
pcb_l = 108.5;
pcb_t = 1.6;

mount_holes = [[4.0, 73.5], [45.5, 46.0], [88.5, 9.0]];
insert_hole_d = 4.7;
insert_hole_depth = 4;
boss_od = 9;

wall_t = 4.8;
gap = 0.4;
outer_w = pcb_w + 2*wall_t + 2*gap;
outer_l = pcb_l + 2*wall_t + 2*gap;
pcb_origin = [wall_t + gap, wall_t + gap];

tilt_deg = 7;
tilt = tan(tilt_deg);

floor_t_front = 3;
floor_t_back = floor_t_front + outer_l * tilt;

cap_clearance = 7;
standoff_margin = 0.5;
standoff_h = cap_clearance + standoff_margin;

switch_clear = 5;
plate_t = 1.5;
lid_gap = 1;
wall_h = standoff_h + pcb_t + switch_clear + plate_t + lid_gap;

// Plate is held on by magnets instead of screws (2026-09-25) — no boss/insert
// needed at all, which sidesteps the boss_od-vs-wall_t clearance problem
// entirely: a magnet pocket only needs to be recessed a shallow depth DOWN
// into the wall's top face, not bored radially THROUGH the wall's thickness,
// so it never has to intrude on the board cavity or bulge past the case's
// outer footprint. Pockets sit centered in the wall band at the same 3
// positions the screw posts used to occupy.
cav_x0 = pcb_origin[0];
cav_x1 = outer_w - pcb_origin[0];
cav_y0 = pcb_origin[1];
cav_y1 = outer_l - pcb_origin[1];
magnet_d = 4;
magnet_fit = 0.2;           // extra diameter for a snug press/glue fit
magnet_h_case = 1.5;        // pocket depth into the case wall's top face
magnet_h_plate = 1.0;       // pocket depth into the plate's underside
magnet_positions = [
    [outer_w/2, cav_y0/2],                 // front wall, centered in the wall band
    [cav_x0/2, cav_y1 + (outer_l-cav_y1)/2],       // back-left corner, centered
    [cav_x1 + (outer_w-cav_x1)/2, cav_y1 + (outer_l-cav_y1)/2]  // back-right corner, centered
];

usbc_w = 10;
usbc_h = 4.5;
usbc_x = pcb_origin[0] + pcb_w/2;
usbc_z = standoff_h - 4.5;

module wedge_floor(w, l, t_front, t_back) {
    polyhedron(
        points = [
            [0,0,0], [w,0,0], [w,l,0], [0,l,0],
            [0,0,t_front], [w,0,t_front], [w,l,t_back], [0,l,t_back]
        ],
        faces = [
            [0,1,2,3],
            [0,4,5,1],
            [1,5,6,2],
            [2,6,7,3],
            [3,7,4,0],
            [4,7,6,5]
        ]
    );
}

module sheared() {
    multmatrix(m = [
        [1,0,0,0],
        [0,1,0,0],
        [0,tilt,1,0],
        [0,0,0,1]
    ])
    translate([0, 0, floor_t_front])
    children();
}

// USB-C port cut as an actual stadium/oval slot (two rounded ends + straight
// sides), not a sharp rectangle — matches the real connector shape instead of
// a generic cutout. Built as a hull of two cylinders whose axis runs through
// the wall (Y), so it extrudes cleanly through wall_shell()'s thickness.
module usbc_slot_cutter(depth) {
    r = usbc_h / 2;
    hull() {
        translate([r, 0, r]) rotate([-90, 0, 0]) cylinder(r = r, h = depth);
        translate([usbc_w - r, 0, r]) rotate([-90, 0, 0]) cylinder(r = r, h = depth);
    }
}

// Shallow recessed ledge around the port on the OUTSIDE face only (not a
// through-cut) — gives the cable's plastic shell somewhere to seat instead of
// butting flat against the wall, like a real device's USB-C surround.
usbc_recess_margin = 3;
usbc_recess_depth = 2.5;
module usbc_recess_cutter(depth) {
    w = usbc_w + 2 * usbc_recess_margin;
    h = usbc_h + 2 * usbc_recess_margin;
    r = h / 2;
    hull() {
        translate([r, 0, r]) rotate([-90, 0, 0]) cylinder(r = r, h = depth);
        translate([w - r, 0, r]) rotate([-90, 0, 0]) cylinder(r = r, h = depth);
    }
}

module wall_shell() {
    difference() {
        cube([outer_w, outer_l, wall_h]);
        translate([wall_t, wall_t, -1])
            cube([outer_w - 2*wall_t, outer_l - 2*wall_t, wall_h + 2]);
        translate([usbc_x - usbc_w/2, outer_l - wall_t - 1, usbc_z])
            usbc_slot_cutter(wall_t + 2);
        translate([
            usbc_x - usbc_w/2 - usbc_recess_margin,
            outer_l - usbc_recess_depth,
            usbc_z - usbc_recess_margin
        ])
            usbc_recess_cutter(usbc_recess_depth + 1);
        // magnet pockets, recessed straight down from the wall's top face —
        // purely vertical, so they never touch the cavity or outer boundary
        for (m = magnet_positions)
            translate([m[0], m[1], wall_h - magnet_h_case])
                cylinder(d = magnet_d + magnet_fit, h = magnet_h_case + 1);
        // underglow light channels — full-height vertical cuts running from
        // the LED's real height down to near the floor, cut through BOTH
        // walls that meet at each corner (not just one), so the glow wraps
        // the actual corner instead of only showing on one face
        for (u = underglow_centers) {
            is_front = u[1] < outer_l/2;
            is_left = u[0] < outer_w/2;
            y_pos = is_front ? -1 : outer_l - wall_t - 1;
            x_pos = is_left ? -1 : outer_w - wall_t - 1;
            // front/back wall cut
            translate([u[0] - underglow_channel_w/2, y_pos, underglow_channel_bottom])
                underglow_channel_cutter(wall_t + 2);
            // left/right wall cut (wraps the same corner on the other face)
            translate([x_pos, u[1] - underglow_channel_w/2, underglow_channel_bottom])
                cube([wall_t + 2, underglow_channel_w, underglow_channel_h]);
            // the corner post itself — without this, the two cuts above each
            // stop right at the wall edge and leave a solid sliver standing
            // at the actual vertex between them; this removes that post so
            // the whole thing is one continuous wraparound opening, not two
            // separate slots that happen to meet nearby
            translate([x_pos, y_pos, underglow_channel_bottom])
                cube([wall_t + 2, wall_t + 2, underglow_channel_h]);
        }
    }
}

module pcb_standoffs() {
    for (h = mount_holes)
        translate([h[0] + pcb_origin[0], h[1] + pcb_origin[1], 0])
            difference() {
                cylinder(d = boss_od, h = standoff_h);
                translate([0, 0, standoff_h - insert_hole_depth])
                    cylinder(d = insert_hole_d, h = insert_hole_depth + 1);
            }
}

module case_bottom() {
    wedge_floor(outer_w, outer_l, floor_t_front, floor_t_back);
    sheared() {
        wall_shell();
        pcb_standoffs();
    }
}

switch_pitch = 20;
switch_cut = 14;
switch_grid_center = [pcb_origin[0] + 45.5, pcb_origin[1] + 46.5];

oled_w = 26.0;
oled_h = 11.0;
oled_center = [pcb_origin[0] + 46.12, pcb_origin[1] + 95.5];

encoder_d = 6.5;
encoder_centers = [
    [pcb_origin[0] + 14.75, pcb_origin[1] + 95.5],
    [pcb_origin[0] + 76.25, pcb_origin[1] + 95.5]
];

// Underglow LEDs (LED17-20 in KiCad) are front-face-mounted at the 4 board
// corners (extracted from the STEP component tree, not assumed) — they shine
// up toward the plate, not down through the floor. Real extracted centers:
underglow_centers = [
    [pcb_origin[0] + 87.5, pcb_origin[1] + 105.63],  // LED17, back-right
    [pcb_origin[0] + 4.5,  pcb_origin[1] + 105.63],  // LED18, back-left
    [pcb_origin[0] + 87.5, pcb_origin[1] + 2.63],    // LED19, front-right
    [pcb_origin[0] + 4.5,  pcb_origin[1] + 2.63]     // LED20, front-left
];

// Underglow re-done as a real side-wall light CHANNEL (2026-09-25, rev 2),
// replacing both the original vertical plate pinholes and the first side-slot
// attempt (which only opened a window right at LED height, ~60% up the
// wall — not real desk-level glow). The LED itself still physically sits at
// Z = standoff_h+pcb_t+0.67 to +2.46 (per STEP extraction) and can't move,
// but instead of a small window only there, the wall is opened as one
// continuous vertical channel running from that LED height all the way DOWN
// to just above the floor — an open-air light guide, not a literal optical
// light pipe (no reflective walls/lens), so it'll read brightest near the
// LED and dimmer toward the bottom, but it genuinely puts light output at
// desk level instead of stopping at the LED's own height.
underglow_channel_w = 10;
underglow_channel_top = standoff_h + pcb_t + 3;   // ~12.6mm: a few mm above the real 11.56mm LED top, so light has room to enter the channel
underglow_channel_bottom = 1.0;                   // 1mm above the floor — as low as it goes while leaving the wall-to-floor joint some strength
underglow_channel_h = underglow_channel_top - underglow_channel_bottom;
module underglow_channel_cutter(depth) {
    cube([underglow_channel_w, depth, underglow_channel_h]);
}

module top_plate() {
    difference() {
        cube([outer_w, outer_l, plate_t]);

        // magnet pockets, recessed up into the plate's underside, same XY as
        // the case-side pockets so the two faces meet
        for (m = magnet_positions)
            translate([m[0], m[1], -1])
                cylinder(d = magnet_d + magnet_fit, h = magnet_h_plate + 1);

        for (r = [0:3])
            for (c = [0:3])
                translate([
                    switch_grid_center[0] + (c - 1.5) * switch_pitch - switch_cut/2,
                    switch_grid_center[1] + (r - 1.5) * switch_pitch - switch_cut/2,
                    -1
                ])
                    cube([switch_cut, switch_cut, plate_t + 2]);

        translate([oled_center[0] - oled_w/2, oled_center[1] - oled_h/2, -1])
            cube([oled_w, oled_h, plate_t + 2]);

        for (e = encoder_centers)
            translate([e[0], e[1], -1])
                cylinder(d = encoder_d, h = plate_t + 2);

        // no plate cutout for underglow anymore — the LEDs vent sideways
        // through wall_shell()'s slots instead (see underglow_slot_cutter)
    }
}

part = "both";

if (part == "bottom")
    case_bottom();
else if (part == "top")
    top_plate();
else {
    case_bottom();
    translate([outer_w + 15, 0, 0]) top_plate();
}
