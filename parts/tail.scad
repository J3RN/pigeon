module screw_hole(radius = 1.6) {
    cylinder(h = 5, r1 = radius, r2 = radius);
}

module stud(radius = 5) {
    cylinder(h = 3, r1 = radius, r2 = radius);
}

module chassis_holes(radius = 1.6) {
    x = 17.5;
    y = 38;
    translate([-x, y, 0]) screw_hole(radius);
    translate([x, y, 0]) screw_hole(radius);
    translate([-x, -y, 0]) screw_hole(radius);
    translate([x, -y, 0]) screw_hole(radius);
}

module chassis_studs() {
    x = 17.5;
    y = 38;
    translate([-x, y, 0]) stud();
    translate([x, y, 0]) stud();
    translate([-x, -y, 0]) stud();
    translate([x, -y, 0]) stud();
}

module chassis() {
    difference() {
        union() {
        translate([-25, -50, 3]) cube([50, 100, 3]);
        chassis_studs();
        }
        translate([0,0,-1]) chassis_holes();
    }
}

chassis();
