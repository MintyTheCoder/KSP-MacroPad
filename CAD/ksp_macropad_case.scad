$fn = 64;

tilt = 7;
pcb_w = 100;
pcb_l = 100;
pcb_t = 1.510;
wall_t = 4.8;
gap = 0.4;
standoff_h = 7.5;
plate_t = 1.5;
plate_z0 = pcb_t + 3.5;
floor_front = 6.60;

ox = wall_t + gap;
oy = wall_t + gap + pcb_t * sin(tilt);
yp_front = (-oy - standoff_h * sin(tilt)) / cos(tilt);
oz = floor_front - yp_front * sin(tilt) + standoff_h * cos(tilt);

outer_w = pcb_w + 2 * (gap + wall_t);
cav_y1 = oy + pcb_l * cos(tilt) + gap;
outer_l = cav_y1 + wall_t;
height = 80;

mount_holes = [[5.0, 66.25], [50.0, 41.25], [91.0, 5.25]];
boss_d = 9;
insert_d = 4.7;
insert_depth = 4;
screw_clear_d = 3.4;
screw_below = 14.49;
relief_boxes = [[46.92, 49.08, 29.44, 37.68, -2.67, 2.42], [91.65, 94.35, 9.19, 20.15, -2.69, 2.92]];
relief_margin = 0.4;

switch_centers = [[19.5, 12.25], [19.5, 32.25], [19.5, 52.25], [19.5, 72.25], [39.5, 12.25], [39.5, 32.25], [39.5, 52.25], [39.5, 72.25], [59.5, 12.25], [59.5, 32.25], [59.5, 52.25], [59.5, 72.25], [79.5, 12.25], [79.5, 32.25], [79.5, 52.25], [79.5, 72.25]];
switch_cut = 14.0;
encoder_centers = [[19.0, 91.25], [80.5, 91.25]];
encoder_body = [12.4, 12.3];
encoder_body_top = 5.7;
encoder_collar_d = 17;
encoder_collar_h = 1.6;
encoder_bushing_d = 7.4;

oled_module = [30.06, 68.67, 84.95, 97.55];
oled_pocket_depth = 0.4;
oled_window = [31.5, 63.52, 85.35, 97.15];
oled_pin_x = 67.085;
oled_pin_ys = [87.465, 90.005, 92.545, 95.085];
oled_pin_d = 1.3;

usb_center = [50.11, -2.39];
usb_opening = [12.5, 7.0];
usb_start_y = 100.4;
xiao_pocket = [40.9, 59.3];

magnet_d = 4.2;
magnet_case_depth = 1.5;
magnet_plate_depth = 1.0;

ug_leds = [[1.72, 4.53, 4.81, 10.69, 0.67, 2.46], [1.47, 4.28, 89.31, 95.19, 0.67, 2.46], [95.47, 98.28, 4.81, 10.69, 0.67, 2.46], [95.72, 98.53, 89.31, 95.19, 0.67, 2.46]];
ug_margin = 5;
ug_top = 3.5;

module rigid() {
    translate([ox, oy, oz]) rotate([tilt, 0, 0]) children();
}

function to_yp(world_y, zp) = (world_y - oy + zp * sin(tilt)) / cos(tilt);

module outer_prism() { cube([outer_w, outer_l, height]); }
module cavity_prism() { translate([wall_t, wall_t, -1]) cube([outer_w - 2 * wall_t, outer_l - 2 * wall_t, height + 2]); }
module below_plane(zp) { rigid() translate([-500, -500, zp - 1000]) cube([1000, 1000, 1000]); }
module above_plane(zp) { rigid() translate([-500, -500, zp]) cube([1000, 1000, 1000]); }

module standoffs() {
    intersection() {
        outer_prism();
        rigid() for (h = mount_holes)
            translate([h[0], h[1], -standoff_h - 3]) cylinder(d = boss_d, h = standoff_h + 3);
    }
}

module standoff_cuts() {
    rigid() {
        for (h = mount_holes) {
            translate([h[0], h[1], -insert_depth]) cylinder(d = insert_d, h = insert_depth + 0.01);
            translate([h[0], h[1], -screw_below - 1]) cylinder(d = screw_clear_d, h = screw_below + 1);
        }
        for (b = relief_boxes)
            translate([b[0] - relief_margin, b[2] - relief_margin, b[4] - relief_margin])
                cube([b[1] - b[0] + 2 * relief_margin, b[3] - b[2] + 2 * relief_margin, -b[4] + relief_margin]);
    }
}

module stadium_y(w, h, len) {
    r = h / 2;
    hull() {
        translate([-w / 2 + r, 0, 0]) rotate([-90, 0, 0]) cylinder(r = r, h = len);
        translate([w / 2 - r, 0, 0]) rotate([-90, 0, 0]) cylinder(r = r, h = len);
    }
}

module usb_cuts() {
    rigid() {
        translate([usb_center[0], usb_start_y, usb_center[1]]) stadium_y(usb_opening[0], usb_opening[1], 30);
        translate([xiao_pocket[0], 99.5, -5]) cube([xiao_pocket[1] - xiao_pocket[0], 3, 5]);
    }
}

module underglow_cuts() {
    rigid() for (l = ug_leds) {
        left = l[0] < pcb_w / 2;
        front = l[2] < pcb_l / 2;
        x0 = left ? -30 : l[0] - ug_margin;
        x1 = left ? l[1] + ug_margin : pcb_w + 30;
        y0 = front ? -30 : l[2] - ug_margin;
        y1 = front ? l[3] + ug_margin : pcb_l + 30;
        sx0 = left ? -30 : pcb_w;
        sx1 = left ? 0 : pcb_w + 30;
        sy0 = front ? -30 : pcb_l;
        sy1 = front ? 0 : pcb_l + 30;
        z0 = -standoff_h + 1;
        translate([sx0, y0, z0]) cube([sx1 - sx0, y1 - y0, ug_top - z0]);
        translate([x0, sy0, z0]) cube([x1 - x0, sy1 - sy0, ug_top - z0]);
    }
}

function magnet_positions() = [
    [outer_w / 2, wall_t / 2],
    [outer_w * 0.25, outer_l - wall_t / 2],
    [outer_w * 0.75, outer_l - wall_t / 2]
];

module magnet_cuts(depth, below) {
    rigid() for (m = magnet_positions())
        translate([m[0] - ox, to_yp(m[1], plate_z0), below ? plate_z0 - depth : plate_z0 - 0.01])
            cylinder(d = magnet_d, h = depth + 0.01);
}

module case_bottom() {
    difference() {
        union() {
            difference() {
                intersection() { outer_prism(); below_plane(plate_z0); }
                intersection() { cavity_prism(); above_plane(-standoff_h); }
            }
            standoffs();
        }
        standoff_cuts();
        usb_cuts();
        underglow_cuts();
        magnet_cuts(magnet_case_depth, true);
    }
}

module plate_world() {
    difference() {
        union() {
            intersection() {
                outer_prism();
                rigid() translate([-100, -100, plate_z0]) cube([400, 400, plate_t]);
            }
            rigid() for (e = encoder_centers)
                translate([e[0], e[1], plate_z0 + plate_t - 0.01]) cylinder(d = encoder_collar_d, h = encoder_collar_h + 0.01);
        }
        rigid() {
            for (s = switch_centers)
                translate([s[0] - switch_cut / 2, s[1] - switch_cut / 2, plate_z0 - 1]) cube([switch_cut, switch_cut, plate_t + 2]);
            for (e = encoder_centers) {
                translate([e[0] - encoder_body[0] / 2, e[1] - encoder_body[1] / 2, plate_z0 - 1])
                    cube([encoder_body[0], encoder_body[1], pcb_t + encoder_body_top - plate_z0 + 1]);
                translate([e[0], e[1], plate_z0]) cylinder(d = encoder_bushing_d, h = 10);
            }
            translate([oled_module[0], oled_module[2], plate_z0 - 1])
                cube([oled_module[1] - oled_module[0], oled_module[3] - oled_module[2], oled_pocket_depth + 1]);
            translate([oled_window[0], oled_window[2], plate_z0 - 1])
                cube([oled_window[1] - oled_window[0], oled_window[3] - oled_window[2], plate_t + 2]);
            for (y = oled_pin_ys)
                translate([oled_pin_x, y, plate_z0 - 1]) cylinder(d = oled_pin_d, h = plate_t + 2);
        }
        magnet_cuts(magnet_plate_depth, false);
    }
}

module top_plate_pcb_frame() {
    rotate([-tilt, 0, 0]) translate([-ox, -oy, -oz]) plate_world();
}

part = "bottom";
if (part == "bottom") case_bottom();
else if (part == "plate") top_plate_pcb_frame();
else if (part == "plate_world") plate_world();
else if (part == "print_plate") translate([0, 0, -plate_z0]) top_plate_pcb_frame();
