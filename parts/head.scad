module screw_hole(radius = 1.7) {
    cylinder(h = 5, r1 = radius, r2 = radius);
}

module stud(radius = 5) {
    cylinder(h = 3, r1 = radius, r2 = radius);
}

module chassis_holes(radius = 1.6) {
    x = 17.5;
    y = 36;
    translate([-x, y, 0]) screw_hole(radius);
    translate([x, y, 0]) screw_hole(radius);
    translate([-x, -y, 0]) screw_hole(radius);
    translate([x, -y, 0]) screw_hole(radius);
}

module chassis_studs() {
    x = 17.5;
    y = 36;
    translate([-x, y, 0]) stud();
    translate([x, y, 0]) stud();
    translate([-x, -y, 0]) stud();
    translate([x, -y, 0]) stud();
}

module chassis() {
    difference() {
        chassis_studs();
        translate([0,0,-1]) chassis_holes();
    }
}

module mega_holes(x = 0, y = 0) {
    translate([x - 24.1, y - 39.95, 2])  screw_hole();
    translate([x + 24.1, y - 41.25, 2])  screw_hole();
    translate([x - 24.1, y + 34.95, 2])  screw_hole();
    translate([x - 8.9,  y + 10.85, 2])  screw_hole();
    translate([x + 19,   y + 10.85, 2])  screw_hole();
    translate([x + 24.1, y + 41.25, 2])  screw_hole();
}

module sensor_holes(x = 0, y = 0) {
    translate([x + 9, y + 16.5, 2]) screw_hole();
    translate([x + 9, y - 16.5, 2]) screw_hole();
    translate([x - 9, y + 16.5, 2]) screw_hole();
    translate([x - 9, y - 16.5, 2]) screw_hole();
}

module platform_trimming() {
    translate([-43.5, 21, 2]) cube([21, 26, 5]);
    translate([-43.5, -56, 2]) cube([21, 26, 5]);
    translate([-43, -18, 2]) cube([10, 26, 5]);
}

module platform() {
    difference() {
        translate([-42.5, -55, 3]) cube([85, 100, 3]);
        translate([0,0,2]) chassis_holes(3.5);
        rotate(180) mega_holes(x = -15, y = 10);
        sensor_holes(x = -27, y = -5);
        platform_trimming();
    }
}

chassis();
platform();
