module screw_hole(radius = 1.8) {
    cylinder(h = 5, r1 = radius, r2 = radius);
}

module stud(radius = 1.5) {
    smaller_r = radius / 2;

    difference() {
        union() {
            cylinder(h = 3, r1 = radius, r2 = radius);
            translate([0, 10, 0]) cylinder(h = 3, r1 = radius, r2 = radius);
            translate([-radius, 0, 0]) cube([2 * radius, 10, 3]);
        }

        translate([0, 0, -1])   cylinder(h = 5, r1 = smaller_r, r2 = smaller_r);
        translate([0, 10, -1])  cylinder(h = 5, r1 = smaller_r, r2 = smaller_r);
        translate([-smaller_r, 0, -1]) cube([2 * smaller_r, 10, 5]);
    }
}

module ramp() {
    
}

module esc_slot() {
    cube([47, 28, 8]);
}

module shared_esc_slot() {
    difference() {
        cube([45, 31, 20.5]);
        translate([-1, 1.5, 1.5]) esc_slot();
        translate([-1, 1.5, 11]) esc_slot();
    }
}

module platform() {
    translate([0, 0, 5]) cube([100, 100,3]);
}

module rig_hook() {
    union() {
        difference() {
            cube([15, 15, 10]);
            translate([-1, 2.5, 2.5]) cube([17, 10, 8]);
        }
        
        difference() {
            translate([0, 7.5, 10]) rotate([0, 90, 0]) cylinder(15, 7.5, 7.5);
            translate([-1, 7.5, 10]) rotate([0, 90, 0]) cylinder(17, 5, 5);
            translate([-1, -1, 0]) cube([17, 16, 10]);
        }
    }
}

//stud();
//shared_esc_slot();
//platform();
rig_hook();