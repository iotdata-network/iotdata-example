// Enclosure for the gen_lora_i2c board — a small, light vertical case.
//
// The board is an integrated PCB (~85 x 16 x 26 mm) carrying a USB-C receptacle
// on one edge and a LoRa module whose antenna SMA points out of the top. The
// case stands on its short end so the antenna is up. Assembly, the way it goes
// together on the bench:
//
//   1. drop the board into the TOP (the tall cup) from the open bottom,
//   2. feed the SMA barrel through the hole in the closed roof and run the
//      bulkhead nut down — this clamps the board to the roof and fixes it,
//   3. the USB-C receptacle now lines up with its slot in the side wall,
//   4. push the TOP straight down onto the BASE: the base upstand enters the
//      cup and grips by friction, and the board's bottom edge drops into the
//      base slot so the cable can't waggle it about on plug / unplug.
//
// Two printable parts in this file (export one at a time with -D part=...):
//
//   * top  — the rectangular cup: four walls + a closed roof, open at the
//     bottom. The roof carries the SMA pass-through; one narrow (85 x 16) side
//     wall carries the USB-C slot. Printed ROOF DOWN (flat solid roof on the
//     bed, open rim up — no bridging over the cavity, clean top face).
//
//   * base — a flat foot the cup seats onto. A 5 mm rectangular upstand in the
//     middle enters the cup and is gripped by friction; the cup's wall rim
//     lands on the perimeter ledge as a positive stop. Stability flanges reach
//     outwards from the foot so the tall, narrow case will not topple. A
//     shallow slot across the upstand locates the board's (unused) bottom edge.
//     Printed AS MODELLED (foot flat on the bed, upstand + slot opening up).
//
// EVERYTHING that depends on the (approximate, will-be-measured-again) board is
// a tunable at the top: wall thickness, the press-fit clearance, and above all
// the SMA and USB-C positions. Expect to nudge sma_inset_w / sma_y and
// usbc_from_top / usbc_y after the first test print.
//
// Shared frame (used by "assembly"/"section"; each part also prints upright):
//   X = WIDTH  axis — the 26 mm cavity dimension
//   Y = DEPTH  axis — the 16 mm cavity dimension
//   Z = HEIGHT axis — the long (vertical) cavity dimension, +Z up
//   origin: centred in X-Y, Z = 0 at the underside of the base foot.

// === FLAGS ===
part = "all";   // "all" | "top" | "base" | "assembly" | "section"

// === Internal cavity (the board envelope) ===
cav_w  = 26;     // internal WIDTH  (X) — the 26 mm axis
cav_d  = 16;     // internal DEPTH  (Y) — the 16 mm axis
cav_h  = 77.5;   // internal HEIGHT (Z) — inside-of-roof down to the open rim
                 // (total case height = cav_h + roof_t; trims the bottom)

wall   = 2.5;    // side-wall thickness
roof_t = 2.5;    // closed-top (roof) thickness

shell_round = 3; // outer vertical-edge radius (cosmetic; cavity stays square)

// derived outer footprint
ext_w = cav_w + 2 * wall;   // 31
ext_d = cav_d + 2 * wall;   // 21

// === Top <-> base press-fit joint ===
// The upstand enters the cup; the cup slides over it. Outer upstand = cavity
// inner minus a per-side slip clearance, so it grips by friction. Drop
// fit_clear if a test print is loose, raise it if it binds.
upstand_h = 5;     // block height (how far it enters the cup)
fit_clear = 0.075;   // per-side clearance, cup cavity over the block
                   // (smaller = tighter; 0.15 was loose, 0 -> press fit)

// === Base foot + stability flanges ===
// The foot core matches the cup footprint (the rim seats on it); the flanges
// are arms reaching out beyond the walls to widen the footprint so the tall
// case will not tip. The depth (Y, 21 mm) is the tippy axis, so the +/-Y arms
// matter most — keep both pairs on unless space is tight.
plate_t      = 4;     // foot / flange thickness
flange_out   = 14;    // how far each arm reaches beyond the case wall
flange_round = 4;     // arm corner radius
flange_x     = true;  // arms on the +/-X (narrow) faces
flange_y     = true;  // arms on the +/-Y (wide) faces
flange_hole_d = 0;    // >0 puts a mounting hole near each arm tip

// === Board location in the base ===
// The board does NOT sit centred — it lies flat against one wide (26 x 85)
// face with a small gap. So the base block is full size in the cavity EXCEPT
// it stops short of that face by (board + gap), leaving an open pocket the
// board drops into. Sliding the cup on then braces across the two narrow ends
// (X) and presses home between the two wide ends (Y).
pcb_t        = 1.6;   // board THICKNESS (Y) — standard 1.6 mm
pcb_face_gap = 0.5;   // gap from the board to the wide face it sits against
pcb_side     = -1;    // which wide face the board sits against: -1 = -Y, +1 = +Y

// === SMA antenna hole (through the closed roof) ===
// Centre sits sma_inset_w in from one internal WIDTH wall, centred on depth.
sma_hole_d  = 6.5;  // 1/4-36 bulkhead barrel (~6.35; drill 6.5)
sma_inset_w = 6;    // mm in from an internal WIDTH wall to the hole centre (TUNE)
sma_y       = 0;    // depth-axis position, 0 = centred (TUNE)
sma_side    = 1;    // which width wall to measure from: -1 = -X wall, +1 = +X wall

// === USB-C pass-through (through a narrow 85 x 16 side wall) ===
// The TOP of the slot sits usbc_from_top below the inside of the roof. The
// receptacle stands tall here: its WIDE aperture runs along the 85 mm axis (Z),
// its small aperture along the 16 mm axis (Y) — so the slot is tall and narrow.
usbc_from_top = 52.25;   // mm: inside-of-roof down to the TOP of the slot (TUNE)
usbc_z_size   = 9.5;  // opening along the vertical axis (Z) — the WIDE aperture
usbc_y_size   = 3.5;  // opening along the depth axis (Y) — the small aperture
usbc_y        = -0.5;  // depth-axis centre; +ve = toward the face OPPOSITE the
                      // board (the receptacle aperture is offset off the board) (TUNE)
usbc_side     = 1;    // which narrow face: +1 = +X face, -1 = -X face

// === Embossed "AE" logo (raised, on one outside face) ===
// Same artwork as sensor_ice_depth_ntc_mount.scad, but FLAT — no cylinder wrap.
// The SVG is resized to logo_w wide (height follows its aspect), stood upright
// on the chosen face, sunk logo_sink into the skin for a clean weld and raised
// logo_raise proud. Fix orientation after a test render with logo_spin /
// logo_mirror; move it with logo_pos (along the face) and logo_z (up the face).
logo_emboss = true;
logo_file   = "ae_stencil_one.svg";
logo_face   = "-Y";   // outside face: "+Y"/"-Y" (wide, 31 mm) or "+X"/"-X" (narrow; +X has USB-C)
logo_w      = 18;     // artwork width (mm) BEFORE rotation (height follows aspect)
logo_raise  = 1.25;    // mm proud of the face
logo_sink   = 0.8;    // mm buried for a clean weld
logo_pos    = 5;      // shift along the face (0 = centred)   -- TUNE placement
logo_z      = 12.5;     // height up the face (0..80; 40 = centred) -- TUNE placement
logo_spin   = 90;     // in-plane rotation (90 = "ae" runs up the face, a low / e high)
logo_mirror = false;  // flip left-right (toggle if it reads backwards)

$fn = 128;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// Centred rectangle with rounded outer corners (square if r <= 0).
module rrect_c(w, h, r) {
    if (r > 0) offset(r) offset(-r) square([w, h], center = true);
    else square([w, h], center = true);
}

// A horizontal slot through the wall: through-axis = X (length len), cross
// section sz_z (along Z) x sz_y (along Y), with rounded ends (stadium).
module xslot(len, sz_z, sz_y) {
    r   = sz_y / 2;
    off = sz_z / 2 - r;
    hull()
        for (s = [-1, 1])
            translate([0, 0, s * off])
                rotate([0, 90, 0]) cylinder(h = len, d = sz_y, center = true);
}

// SMA hole centre on the WIDTH axis: sma_inset_w in from the chosen wall.
sma_x = sma_side * (cav_w / 2 - sma_inset_w);

// USB-C slot centre Z, in the TOP's own frame (rim at z = 0, roof inside at cav_h).
usbc_cz = (cav_h - usbc_from_top) - usbc_z_size / 2;

// Base block geometry. Full width (braces on the narrow ends); in depth it runs
// from fit_clear off the no-board wall to (board + gap) short of the pcb_side
// wall, leaving the open board pocket against that face.
block_x  = cav_w - 2 * fit_clear;
block_f1 = -pcb_side * (cav_d / 2 - fit_clear);                          // no-board face
block_f2 =  pcb_side * (cav_d / 2 - fit_clear - pcb_t - pcb_face_gap);   // pocket face
block_yc = (block_f1 + block_f2) / 2;
block_y  = abs(block_f1 - block_f2);

// ---------------------------------------------------------------------------
// Embossed logo
// ---------------------------------------------------------------------------

// 2-D artwork, normalised to logo_w wide (aspect kept), centred on the origin.
// import(center=true) centres on the SVG viewBox, NOT the ink, so the artwork
// lands off to one side. We resize to logo_w, then translate by the measured
// ink-centre (as a fraction of logo_w) to put it truly on the origin BEFORE
// rotating — so it stays centred at any logo_spin. (Fractions are for
// ae_stencil_one.svg; remeasure if the artwork changes.)
logo_cx = 0.5112;   // ink centre X / logo_w, after resize, before rotation
logo_cy = 0.1092;   // ink centre Y / logo_w
module logo2d() {
    mirror([logo_mirror ? 1 : 0, 0, 0])
        rotate([0, 0, logo_spin])
            translate([-logo_cx * logo_w, -logo_cy * logo_w])
                resize([logo_w, 0, 0], auto = true)
                    import(logo_file, center = true);
}

// Stand the artwork upright on the chosen outside face, sunk logo_sink into the
// skin (weld) and raised logo_raise proud. Opposite faces read mirrored — use
// logo_mirror to correct. Added to the cup AFTER its cuts (always solid).
module flat_logo() {
    T = logo_sink + logo_raise;
    if (logo_face == "-Y")
        translate([logo_pos, -ext_d / 2 + logo_sink, logo_z])
            rotate([90, 0,   0]) linear_extrude(T) logo2d();
    else if (logo_face == "+Y")
        translate([logo_pos,  ext_d / 2 - logo_sink, logo_z])
            rotate([90, 0, 180]) linear_extrude(T) logo2d();
    else if (logo_face == "-X")
        translate([-ext_w / 2 + logo_sink, logo_pos, logo_z])
            rotate([90, 0, -90]) linear_extrude(T) logo2d();
    else if (logo_face == "+X")
        translate([ ext_w / 2 - logo_sink, logo_pos, logo_z])
            rotate([90, 0,  90]) linear_extrude(T) logo2d();
}

// ---------------------------------------------------------------------------
// Top (the cup) — modelled rim at z = 0, roof up
// ---------------------------------------------------------------------------
module top_body() {
    H = cav_h + roof_t;
    difference() {
        // Outer shell, rounded vertical edges.
        linear_extrude(H) rrect_c(ext_w, ext_d, shell_round);

        // Cavity: open at the bottom rim, closed at the top by the roof.
        translate([0, 0, -1])
            linear_extrude(cav_h + 1) square([cav_w, cav_d], center = true);

        // SMA pass-through, through the roof.
        translate([sma_x, sma_y, cav_h - 1])
            cylinder(h = roof_t + 2, d = sma_hole_d);

        // USB-C slot, through a narrow side wall.
        translate([usbc_side * (ext_w / 2), usbc_y, usbc_cz])
            xslot(wall + 4, usbc_z_size, usbc_y_size);
    }

    // Embossed logo, added after the cuts so it is always solid.
    if (logo_emboss) flat_logo();
}

// Print-ready: roof flat on the bed, open rim up (no cavity bridging).
module top_print() {
    translate([0, 0, cav_h + roof_t]) rotate([180, 0, 0]) top_body();
}

// ---------------------------------------------------------------------------
// Base (the foot) — modelled foot on the bed, z = 0 underside
// ---------------------------------------------------------------------------

// 2-D foot footprint: core plate the cup seats on, plus the flange arms.
module foot_2d() {
    union() {
        rrect_c(ext_w, ext_d, shell_round);                       // core (cup seat)
        if (flange_y) rrect_c(ext_w, ext_d + 2 * flange_out, flange_round);  // +/-Y arms
        if (flange_x) rrect_c(ext_w + 2 * flange_out, ext_d, flange_round);  // +/-X arms
    }
}

// Optional mounting holes near each arm tip (axis Z, through the foot).
module foot_holes() {
    if (flange_hole_d > 0) {
        edge = flange_round + flange_hole_d / 2 + 1;   // sit clear of the rounded tip
        if (flange_y) for (s = [-1, 1])
            translate([0, s * (ext_d / 2 + flange_out - edge), -1])
                cylinder(h = plate_t + 2, d = flange_hole_d);
        if (flange_x) for (s = [-1, 1])
            translate([s * (ext_w / 2 + flange_out - edge), 0, -1])
                cylinder(h = plate_t + 2, d = flange_hole_d);
    }
}

module base() {
    difference() {
        union() {
            // Foot + flanges.
            linear_extrude(plate_t) foot_2d();
            // Locating block — full width, offset to one side so the open
            // pocket against the pcb_side face receives the board.
            translate([0, block_yc, plate_t + upstand_h / 2])
                cube([block_x, block_y, upstand_h], center = true);
        }

        foot_holes();
    }
}

// ---------------------------------------------------------------------------
// Reports
// ---------------------------------------------------------------------------
echo(str("cavity = ", cav_w, " x ", cav_d, " x ", cav_h,
         " mm; outer = ", ext_w, " x ", ext_d, " mm; wall ", wall, " / roof ", roof_t));
echo(str("assembled height ~ ", plate_t + cav_h + roof_t,
         " mm; footprint with flanges = ",
         ext_w + (flange_x ? 2 * flange_out : 0), " x ",
         ext_d + (flange_y ? 2 * flange_out : 0), " mm"));
echo(str("SMA hole = ", sma_hole_d, " mm, ", sma_inset_w,
         " mm in from the ", sma_side < 0 ? "-X" : "+X",
         " width wall (x=", sma_x, "), y=", sma_y));
echo(str("USB-C slot = ", usbc_z_size, " (Z) x ", usbc_y_size,
         " (Y) mm, top ", usbc_from_top, " mm below the roof, on the ",
         usbc_side < 0 ? "-X" : "+X", " face, y=", usbc_y));
echo(str("joint: block ", block_x, " (X) x ", block_y, " (Y) x ", upstand_h,
         " mm, centred y=", block_yc, ", ", fit_clear, " mm/side clearance"));
echo(str("board pocket: ", pcb_t + pcb_face_gap, " mm against the ",
         pcb_side < 0 ? "-Y" : "+Y", " face (", pcb_face_gap, " gap + ", pcb_t, " board)"));
if (logo_emboss)
    echo(str("logo = ", logo_file, ", ", logo_w, " mm wide on the ", logo_face,
             " face, raised ", logo_raise, " mm, z=", logo_z));

// === Dispatch ===
if (part == "top")
    top_print();
else if (part == "base")
    base();
else if (part == "assembly") {
    color("Gainsboro") base();
    color("LightSteelBlue", 0.55) translate([0, 0, plate_t]) top_body();
}
else if (part == "section") {
    difference() {
        union() {
            color("Gainsboro") base();
            color("LightSteelBlue") translate([0, 0, plate_t]) top_body();
        }
        translate([0, -100, -1]) cube(200);   // remove +X half -> reveal the depth (Y-Z) section: block + board pocket
    }
}
else if (part == "all") {
    color("Gainsboro") base();
    translate([ext_w + 2 * flange_out + 20, 0, 0])
        color("LightSteelBlue") top_body();
}
