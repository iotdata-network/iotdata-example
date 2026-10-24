// Plug-mount base for gen_lora_i2c_case: a rectangular SLEEVE that slips over a
// Europlug-style USB charger body, presenting the case on top -- powered by a short
// USB-A -> USB-C jumper. The cavity keys the charger in ONE orientation; a raised
// upstand (the same block the normal base uses) sits on EACH of the four faces, so
// whichever face lands up -- whether the socket pins are horizontal or vertical --
// takes the case. Case rim lands on the flat face, upstand enters the case cavity.
//
// PRINT ORIENTATION: box standing on the bed, axis = +Z (the push-in direction);
// the walls print vertical and the four upstands print flat on top of each face.

// === FLAGS ===
part = "plugbase";   // "plugbase" | "demo" (+ charger + case ghosts)

// === Case interface (must match gen_lora_i2c_case.scad) ===
cav_w       = 26;    // case cavity X
cav_d       = 16;    // case cavity Y
case_wall   = 2.5;
ext_w       = cav_w + 2 * case_wall;   // 31 -> box length (axial) AND case rim X
ext_d       = cav_d + 2 * case_wall;   // 21 -> case rim Y
shell_round = 3;
fit_clear   = 0.075; // per-side slip of the case cavity over the upstand
pcb_t       = 1.6;
pcb_face_gap = 0.5;

// derived block (the case-mating upstand), exactly as the base computes it
block_x  = cav_w - 2 * fit_clear;                                   // 25.85 (axial)
block_y  = cav_d - fit_clear - pcb_t - pcb_face_gap - (-fit_clear); // 13.75 (across face)
block_yc = (pcb_t + pcb_face_gap) / 2;                              // 1.05 offset (leaves board pocket)
upstand_h = 5;

// === Charger (Europlug body) ===
chg_w     = 35.3;    // body width (tapered point-to-point) -> the WIDE cavity dim
chg_t     = 13.7;    // body thickness (flat-to-flat)       -> the NARROW cavity dim
chg_clear = 0.6;     // clearance around the charger (bumped: it only wedged in slightly)

// === Sleeve box ===
wall       = 5;     // fitted-shell wall around the charger (1-2 cm -> solid, stable)
sleeve_len = 40;     // covers the WIDER Pi case (pi_ext_w 38 + margin); lora case (31) leaves sleeve proud
box_w      = chg_w + 2 * wall;   // wide faces
box_h      = chg_t + 2 * wall;   // narrow faces
box_round  = 3;

$fn = $preview ? 48 : 256;

echo(str("box ", box_w, " x ", box_h, " x ", sleeve_len, "; cavity ", chg_w, " x ", chg_t,
         " (+", chg_clear, "); wall ", wall, "; upstand ", block_x, " x ", block_y, " x ", upstand_h));

// ---------------------------------------------------------------------------
module rrect_c(w, h, r) { if (r > 0) offset(r) offset(-r) square([w, h], center = true);
                          else square([w, h], center = true); }

// Charger cross-section: elongated hexagon (35.3 x 13.7 with 45-deg tapered ends).
module charger_2d() polygon([[chg_w/2, 0], [chg_w/2 - chg_t/2, chg_t/2],
    [-(chg_w/2 - chg_t/2), chg_t/2], [-chg_w/2, 0],
    [-(chg_w/2 - chg_t/2), -chg_t/2], [chg_w/2 - chg_t/2, -chg_t/2]]);

module box_2d()    rrect_c(box_w, box_h, box_round);
module cavity_2d() offset(chg_clear) charger_2d();   // fitted to the tapered profile

// One upstand on a face at distance d (outward normal +Y, across-face = X).
// Sunk 0.8 mm into the wall so it fuses into the box (one solid, clean weld).
module face_upstand(d)
    translate([block_yc, d + upstand_h / 2 - 0.4, sleeve_len / 2])
        cube([block_y, upstand_h + 0.8, block_x], center = true);

module upstands() {
    face_upstand(box_h / 2);                    // +Y face (wide)
    rotate([0, 0, 180]) face_upstand(box_h / 2);// -Y face (wide)
    rotate([0, 0, -90]) face_upstand(box_w / 2);// +X face (narrow)
    rotate([0, 0,  90]) face_upstand(box_w / 2);// -X face (narrow)
}

module plugbase() {
    difference() {
        linear_extrude(sleeve_len) box_2d();
        translate([0, 0, -1]) linear_extrude(sleeve_len + 2) cavity_2d();
    }
    upstands();
}

// ghosts
module charger_ghost() color("DimGray", 0.6) linear_extrude(sleeve_len) charger_2d();
module case_ghost()   // case sitting on the +Y (top) upstand
    color("SteelBlue", 0.5) translate([0, box_h/2, sleeve_len/2]) rotate([-90,0,0])
        linear_extrude(40) rrect_c(ext_w, ext_d, shell_round);

// ===========================================================================
//  Raspberry Pi Zero (W) case -- the plug-mounted remote sibling of the lora case.
//  Same family (rectangular cup, roof-DOWN print, AE logo), but: NO antenna hole,
//  NO vent grill, X grown for the Pi's width, and TWO micro-USB slots stacked on
//  one narrow face. Chain: charger USB-A -> Pi micro-USB (PWR); Pi micro-USB (OTG)
//  -> lora case USB-C. Seats on the plug base's opposite upstand (base = next step).
// ===========================================================================
pi_cav_w = 31.5;    // X  (Pi ~31 incl connectors + a bit of clearance; lora was 26)
pi_cav_d = 16;    // Y  (same as lora)
pi_cav_h = 77;    // Z  (Pi ~70 + 5 mm for the SD card standing proud of the board)
pi_wall  = 1.5;
pi_roof  = 2.5;
pi_ext_w = pi_cav_w + 2 * pi_wall;   // 36
pi_ext_d = pi_cav_d + 2 * pi_wall;   // 21

// Base pad: the lora upstand grips because the springy PCB fills its Y pocket; the Pi
// has no PCB there, so fill that pocket with a printed pad -> the shared upstand seats
// snug in Y (X stays a touch loose, which is fine).
pi_pad_on = true;
pi_pad_t  = pcb_t + pcb_face_gap + 0.3;   // 2.4 -> pocket + a smidge, so the upstand grips tight
pi_pad_h  = upstand_h + 3;                // covers the upstand's engagement up from the base rim

// two micro-USB slots on the +X narrow face (PWR at the lora USB-C height, DATA +12.5)
pi_usb_z1    = 20.5;   // centre height of slot 1 -- matches the lora USB-C (measure/tune)
pi_usb_gap   = 12.5;   // slot 2 this much higher (toward the top)
pi_usb_zsize = 8;      // micro-USB opening along Z  (MEASURE)
pi_usb_ysize = 4;      // opening along Y            (MEASURE)
pi_usb_y     = 0;
pi_usb_side  = 1;      // +1 = +X narrow face

// logo (same stencil, raised on the -Y wide face)
pi_logo_w = 18; pi_logo_raise = 1.25; pi_logo_sink = 0.8;
pi_logo_pos = 6; pi_logo_z = 14; pi_logo_spin = 90;

// helpers mirrored from gen_lora_i2c_case.scad
module xslot(len, sz_z, sz_y) { r = sz_y/2; off = sz_z/2 - r;
    hull() for (s = [-1,1]) translate([0,0,s*off]) rotate([0,90,0]) cylinder(h = len, d = sz_y, center = true); }
pi_logo_cx = 0.5112; pi_logo_cy = 0.1092;
module pi_logo2d() rotate([0,0,pi_logo_spin])
    translate([-pi_logo_cx*pi_logo_w, -pi_logo_cy*pi_logo_w])
        resize([pi_logo_w,0,0], auto = true) import("ae_stencil_one.svg", center = true);
module pi_flat_logo() { T = pi_logo_sink + pi_logo_raise;
    translate([pi_logo_pos, -pi_ext_d/2 + pi_logo_sink, pi_logo_z]) rotate([90,0,0]) linear_extrude(T) pi_logo2d(); }

module pi_top_body() {
    H = pi_cav_h + pi_roof;
    difference() {
        linear_extrude(H) rrect_c(pi_ext_w, pi_ext_d, shell_round);
        translate([0,0,-1]) linear_extrude(pi_cav_h + 1) square([pi_cav_w, pi_cav_d], center = true);
        for (z = [pi_usb_z1, pi_usb_z1 + pi_usb_gap])
            translate([pi_usb_side*(pi_ext_w/2), pi_usb_y, z]) xslot(pi_wall + 4, pi_usb_zsize, pi_usb_ysize);
    }
    pi_flat_logo();
    // base pad on the +Y wall (the orientation you want), fused 0.5 into the wall, up from the base rim
    if (pi_pad_on)
        translate([0, pi_cav_d/2 + 0.5 - (pi_pad_t + 0.5)/2, pi_pad_h/2])
            cube([block_x, pi_pad_t + 0.5, pi_pad_h], center = true);
}
module pi_top_print() translate([0,0,pi_cav_h + pi_roof]) rotate([180,0,0]) pi_top_body();

// === Dispatch ===
if (part == "plugbase") plugbase();
else if (part == "demo") { plugbase(); charger_ghost(); case_ghost(); }
else if (part == "pi_top") pi_top_print();
else if (part == "pi_body") pi_top_body();   // unflipped, for viewing
