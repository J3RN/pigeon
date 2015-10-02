module screw_hole(radius = 1.8, height = 5) {
    cylinder(h = height, r1 = radius, r2 = radius);
}

module stud(radius = 5) {
    cylinder(h = 7, r1 = radius, r2 = radius);
}

module chassis_holes(radius = 1.8) {
    x = 16;
    y = 36;
    translate([-x, y, 0])   screw_hole(radius = radius, height = 9);
    translate([x, y, 0])    screw_hole(radius = radius, height = 9);
    translate([-x, -y, 0])  screw_hole(radius = radius, height = 9);
    translate([x, -y, 0])   screw_hole(radius = radius, height = 9);
}

module chassis_studs() {
    x = 16;
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
    translate([x - 24.1, y - 39.95, 0])  screw_hole();
    translate([x + 24.1, y - 41.25, 0])  screw_hole();
    translate([x - 24.1, y + 34.95, 0])  screw_hole();
    translate([x - 8.9,  y + 10.85, 0])  screw_hole();
    translate([x + 19,   y + 10.85, 0])  screw_hole();
    translate([x + 24.1, y + 41.25, 0])  screw_hole();
}

module sensor_holes(x = 0, y = 0) {
    translate([x + 9, y + 16.5, 0]) screw_hole();
    translate([x + 9, y - 16.5, 0]) screw_hole();
    translate([x - 9, y + 16.5, 0]) screw_hole();
    translate([x - 9, y - 16.5, 0]) screw_hole();
}

module platform_trimming() {
    translate([-43.5,  21, 0]) cube([21, 26, 5]);
    translate([-43.5, -56, 0]) cube([21, 26, 5]);
    translate([-43,   -18, 0]) cube([10, 26, 5]);
}

module platform() {
    difference() {
        translate([-50, -50, 7]) cube([100, 100, 3]);
        translate([0, 0, 6]) chassis_holes(radius = 3.5);
        translate([0, 0, 6]) rotate(180) mega_holes(x = -15, y = 0);
        translate([0, 0, 6]) sensor_holes(x = -30, y = 0);
    }
}

chassis();
platform();
