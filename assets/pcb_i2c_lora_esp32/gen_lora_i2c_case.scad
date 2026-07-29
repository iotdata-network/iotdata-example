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

// === Base foot ===
plate_t      = 5;     // foot thickness

// --- House style: aperiodic monotile foot (the "hat" einstein tile) ---------
// The foot footprint is the hat monotile rather than a plain plate. It still
// widens the base well beyond the walls (so the tall case will not tip) but
// does it with the monotile silhouette. The native polygon (MONO_PTS, below)
// is 52.75 x 77.22 mm; its long axis runs along Y, which is the tippy DEPTH
// axis — exactly where the extra reach is wanted, so the default is unrotated.
// The case is seated in the hat's wide "head" (mono_seat, max-clearance spot);
// mono_dx/dy/rot/scale nudge it from there. The cup-seat rectangle is unioned
// in regardless, so the rim is always fully supported even if you tune it off.
base_monotile = true;   // true = hat-monotile foot; false = legacy rect+flanges
mono_scale    = 1.0;    // scale on the native polygon (1.0 -> 52.75 x 77.22 mm)
mono_dx       = 5;      // nudge the foot under the case, X (mm)        -- TUNE
mono_dy       = 5;      // nudge the foot under the case, Y (mm)        -- TUNE
mono_rot      = -50;      // rotate the foot under the case (deg; 0 = long axis up Y)

// --- Legacy stability flanges (used only when base_monotile = false) ---------
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
// The inlined artwork is resized to logo_w wide (height follows its aspect), upright
// on the chosen face, sunk logo_sink into the skin for a clean weld and raised
// logo_raise proud. Fix orientation after a test render with logo_spin /
// logo_mirror; move it with logo_pos (along the face) and logo_z (up the face).
logo_emboss = true;
// artwork: "ae" stencil, inlined below as ae_logo_raw() (no external SVG)
logo_face   = "-Y";   // outside face: "+Y"/"-Y" (wide, 31 mm) or "+X"/"-X" (narrow; +X has USB-C)
logo_w      = 18;     // artwork width (mm) BEFORE rotation (height follows aspect)
logo_raise  = 1.25;    // mm proud of the face
logo_sink   = 0.8;    // mm buried for a clean weld
logo_pos    = 5;      // shift along the face (0 = centred)   -- TUNE placement
logo_z      = 20.5;     // height up the face (0..80; 40 = centred) -- TUNE placement
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

// Inlined artwork from ae_stencil_one.svg (single even-odd path, 3 subpaths,
// straight segments only). Coordinates are the raw SVG path points; the group
// transform scale(1,-1)/translate(0,-510) and OpenSCAD's viewBox Y-flip cancel,
// so raw path coords equal the model coords that import() produced. The viewBox
// centre (77.25, 255.0) is subtracted in logo2d() to match import(center=true).
ae_logo_vb_cx = 77.25;   // SVG viewBox centre X (model coords)
ae_logo_vb_cy = 255.0;   // SVG viewBox centre Y (model coords)
ae_logo_pts = [
    [-183.4,195.9],[-183.3,191.6],[-183.2,187.4],[-183.0,183.4],[-182.8,179.5],[-182.6,175.7],
    [-182.3,172.1],[-182.0,168.6],[-181.6,165.3],[-181.2,162.1],[-180.8,159.0],[-180.3,156.1],
    [-179.8,153.3],[-179.2,150.6],[-178.6,148.1],[-178.0,145.7],[-177.3,143.5],[-176.6,141.4],
    [-175.9,139.4],[-175.1,137.6],[-174.3,135.9],[-173.4,134.4],[-172.5,133.0],[-172.0,132.3],
    [-171.5,131.5],[-170.9,130.9],[-170.4,130.2],[-169.8,129.5],[-169.2,128.9],[-168.7,128.3],
    [-168.1,127.7],[-167.4,127.2],[-166.8,126.6],[-166.2,126.1],[-165.5,125.6],[-164.8,125.2],
    [-164.1,124.7],[-163.4,124.3],[-162.7,123.9],[-162.0,123.5],[-161.2,123.2],[-160.5,122.8],
    [-159.7,122.5],[-158.9,122.2],[-158.1,122.0],[-157.3,121.7],[-156.5,121.5],[-155.7,121.3],
    [-154.8,121.1],[-153.9,120.9],[-152.9,120.7],[-151.9,120.6],[-150.9,120.4],[-149.9,120.3],
    [-148.8,120.1],[-147.7,120.0],[-146.6,119.9],[-145.4,119.7],[-144.2,119.6],[-143.0,119.5],
    [-141.8,119.4],[-140.5,119.4],[-139.2,119.3],[-137.8,119.2],[-136.4,119.2],[-135.0,119.1],
    [-133.6,119.1],[-132.1,119.0],[-130.6,119.0],[-129.1,119.0],[-127.5,119.0],[-113.5,119.0],
    [-113.5,-8.0],[-127.5,-8.0],[-134.6,-8.0],[-141.6,-7.9],[-148.4,-7.8],[-155.1,-7.6],
    [-161.6,-7.4],[-168.0,-7.1],[-174.3,-6.8],[-180.4,-6.4],[-186.4,-6.0],[-192.3,-5.6],
    [-198.0,-5.1],[-203.6,-4.5],[-209.1,-3.9],[-214.4,-3.2],[-219.6,-2.5],[-224.6,-1.8],
    [-229.5,-1.0],[-234.3,-0.1],[-238.9,0.8],[-243.4,1.7],[-247.8,2.7],[-252.0,3.8],
    [-256.1,4.9],[-260.0,6.0],[-263.8,7.2],[-267.6,8.5],[-271.2,9.8],[-274.8,11.2],
    [-278.3,12.6],[-281.7,14.1],[-285.0,15.7],[-288.3,17.3],[-291.4,19.0],[-294.5,20.8],
    [-297.5,22.6],[-300.4,24.5],[-303.2,26.4],[-305.9,28.5],[-308.6,30.5],[-311.1,32.7],
    [-313.6,34.9],[-316.0,37.1],[-318.3,39.4],[-320.5,41.8],[-322.6,44.3],[-324.7,46.8],
    [-326.6,49.4],[-328.5,52.0],[-330.1,54.4],[-331.7,56.8],[-333.3,59.2],[-334.7,61.8],
    [-336.2,64.4],[-337.5,67.0],[-338.8,69.7],[-340.1,72.4],[-341.3,75.2],[-342.5,78.1],
    [-343.6,81.0],[-344.6,84.0],[-345.6,87.0],[-346.6,90.1],[-347.4,93.2],[-348.3,96.4],
    [-349.1,99.7],[-349.8,103.0],[-350.5,106.4],[-351.1,109.8],[-351.6,113.2],[-352.1,116.8],
    [-352.6,120.4],[-353.0,124.0],[-353.4,127.7],[-353.7,131.6],[-354.1,135.7],[-354.4,139.9],
    [-354.7,144.2],[-355.0,148.8],[-355.2,153.4],[-355.5,158.2],[-355.7,163.2],[-356.0,168.3],
    [-356.2,173.6],[-356.4,179.0],[-356.6,184.6],[-356.7,190.3],[-356.9,196.2],[-357.0,202.2],
    [-357.1,208.4],[-357.2,214.8],[-357.3,221.2],[-357.4,227.9],[-357.4,234.7],[-357.5,241.6],
    [-357.5,248.7],[-357.5,256.0],[-357.5,296.0],[-113.5,296.0],[-113.5,205.0],[-183.5,205.0],
    [-183.5,200.4],[76.3,454.1],[78.2,456.6],[80.2,459.0],[82.2,461.4],[84.2,463.7],
    [86.4,466.0],[88.6,468.2],[90.9,470.4],[93.2,472.5],[95.6,474.6],[98.1,476.6],[100.7,478.6],
    [103.3,480.5],[106.0,482.3],[108.7,484.2],[111.5,485.9],[114.4,487.6],[117.4,489.3],
    [120.4,490.9],[123.5,492.5],[126.7,494.0],[129.9,495.5],[133.3,496.9],[136.8,498.3],
    [140.3,499.6],[144.0,500.9],[147.7,502.1],[151.6,503.3],[155.5,504.5],[159.5,505.6],
    [163.7,506.6],[167.9,507.6],[172.2,508.6],[176.6,509.5],[181.1,510.4],[185.7,511.2],
    [190.4,511.9],[195.2,512.7],[200.1,513.3],[205.1,514.0],[210.2,514.5],[215.4,515.1],
    [220.6,515.6],[226.0,516.0],[226.0,214.0],[226.0,211.2],[226.0,208.5],[226.0,205.8],
    [226.1,203.2],[226.1,200.7],[226.2,198.2],[226.3,195.8],[226.3,193.5],[226.4,191.2],
    [226.5,189.1],[226.6,186.9],[226.8,184.9],[226.9,182.9],[227.0,181.0],[227.2,179.1],
    [227.3,177.3],[227.5,175.6],[227.7,174.0],[227.9,172.4],[228.1,170.9],[228.3,169.4],
    [228.5,168.1],[228.8,166.7],[229.0,165.5],[229.3,164.3],[229.6,163.1],[229.9,162.0],
    [230.2,160.9],[230.6,159.8],[231.0,158.7],[231.4,157.7],[231.9,156.7],[232.4,155.7],
    [232.9,154.7],[233.4,153.8],[234.0,152.9],[234.6,152.0],[235.2,151.1],[235.9,150.3],
    [236.6,149.5],[237.3,148.7],[238.0,148.0],[238.8,147.2],[239.6,146.5],[240.4,145.9],
    [241.2,145.2],[242.1,144.6],[243.0,144.0],[244.0,143.3],[245.1,142.7],[246.2,142.1],
    [247.4,141.5],[248.6,141.0],[249.8,140.4],[251.1,139.9],[252.4,139.4],[253.8,139.0],
    [255.3,138.5],[256.7,138.1],[258.2,137.8],[259.8,137.4],[261.4,137.0],[263.1,136.7],
    [264.8,136.4],[266.5,136.2],[268.3,135.9],[270.1,135.7],[272.0,135.5],[274.0,135.4],
    [275.9,135.2],[277.9,135.1],[280.0,135.0],[282.7,134.8],[285.4,134.7],[288.1,134.5],
    [290.8,134.4],[293.5,134.3],[296.2,134.1],[299.0,134.0],[301.8,133.9],[304.6,133.8],
    [307.4,133.7],[310.2,133.6],[313.0,133.5],[315.8,133.4],[318.7,133.3],[321.6,133.3],
    [324.4,133.2],[327.3,133.2],[330.2,133.1],[333.2,133.1],[336.1,133.1],[339.1,133.0],
    [342.0,133.0],[345.0,133.0],[348.0,133.0],[353.3,133.0],[358.4,133.0],[363.5,133.0],
    [368.4,133.1],[373.1,133.1],[377.8,133.2],[382.4,133.3],[386.8,133.3],[391.1,133.4],
    [395.3,133.5],[399.3,133.6],[403.2,133.8],[407.1,133.9],[410.8,134.0],[414.3,134.2],
    [417.8,134.3],[421.1,134.5],[424.3,134.7],[427.4,134.9],[430.4,135.1],[433.2,135.3],
    [435.9,135.5],[438.5,135.8],[441.0,136.0],[492.0,140.0],[492.0,11.0],[488.6,10.0],
    [485.1,9.1],[481.5,8.1],[477.8,7.2],[474.0,6.3],[470.1,5.5],[466.1,4.7],[462.0,3.9],
    [457.8,3.1],[453.5,2.4],[449.0,1.7],[444.5,1.0],[439.9,0.3],[435.1,-0.3],[430.3,-0.9],
    [425.3,-1.4],[420.3,-2.0],[415.1,-2.5],[409.9,-3.0],[404.5,-3.4],[399.0,-3.9],[393.5,-4.3],
    [387.8,-4.7],[382.0,-5.0],[379.1,-5.2],[376.2,-5.5],[373.3,-5.7],[370.3,-5.9],[367.3,-6.1],
    [364.2,-6.3],[361.1,-6.5],[358.0,-6.7],[354.8,-6.8],[351.6,-7.0],[348.3,-7.1],[345.0,-7.2],
    [341.6,-7.4],[338.2,-7.5],[334.8,-7.6],[331.3,-7.7],[327.8,-7.7],[324.2,-7.8],[320.6,-7.9],
    [317.0,-7.9],[313.3,-8.0],[309.6,-8.0],[305.8,-8.0],[302.0,-8.0],[294.4,-8.0],[286.9,-7.9],
    [279.6,-7.8],[272.4,-7.6],[265.4,-7.3],[258.5,-7.0],[251.7,-6.7],[245.1,-6.3],[238.6,-5.8],
    [232.2,-5.3],[226.0,-4.7],[219.9,-4.1],[213.9,-3.5],[208.1,-2.7],[202.4,-1.9],[196.9,-1.1],
    [191.5,-0.2],[186.2,0.7],[181.1,1.7],[176.1,2.8],[171.2,3.9],[166.5,5.0],[161.9,6.2],
    [157.5,7.5],[153.2,8.8],[148.9,10.2],[144.8,11.7],[140.7,13.2],[136.8,14.8],[132.9,16.4],
    [129.1,18.1],[125.4,19.9],[121.9,21.7],[118.4,23.6],[114.9,25.6],[111.6,27.6],[108.4,29.7],
    [105.3,31.9],[102.2,34.1],[99.3,36.4],[96.4,38.7],[93.7,41.2],[91.0,43.6],[88.4,46.2],
    [85.9,48.8],[83.5,51.5],[81.2,54.2],[79.0,57.0],[78.2,55.9],[76.3,53.4],[74.3,51.0],
    [72.3,48.6],[70.3,46.3],[68.1,44.0],[65.9,41.8],[63.6,39.6],[61.3,37.5],[58.9,35.4],
    [56.4,33.4],[53.8,31.4],[51.2,29.5],[48.5,27.7],[45.8,25.8],[43.0,24.1],[40.1,22.4],
    [37.1,20.7],[34.1,19.1],[31.0,17.5],[27.8,16.0],[24.6,14.5],[21.2,13.1],[17.7,11.7],
    [14.2,10.4],[10.5,9.1],[6.8,7.9],[2.9,6.7],[-1.0,5.5],[-5.0,4.4],[-9.2,3.4],[-13.4,2.4],
    [-17.7,1.4],[-22.1,0.5],[-26.6,-0.4],[-31.2,-1.2],[-35.9,-1.9],[-40.7,-2.7],[-45.6,-3.3],
    [-50.6,-4.0],[-55.7,-4.5],[-60.9,-5.1],[-66.1,-5.6],[-71.5,-6.0],[-71.5,296.0],
    [-71.5,298.8],[-71.5,301.5],[-71.5,304.2],[-71.6,306.8],[-71.6,309.3],[-71.7,311.8],
    [-71.8,314.2],[-71.8,316.5],[-71.9,318.8],[-72.0,320.9],[-72.1,323.1],[-72.2,325.1],
    [-72.4,327.1],[-72.5,329.0],[-72.7,330.9],[-72.8,332.7],[-73.0,334.4],[-73.2,336.0],
    [-73.4,337.6],[-73.6,339.1],[-73.8,340.6],[-74.0,341.9],[-74.3,343.3],[-74.5,344.5],
    [-74.8,345.7],[-75.1,346.9],[-75.4,348.0],[-75.7,349.1],[-76.1,350.2],[-76.5,351.3],
    [-76.9,352.3],[-77.4,353.3],[-77.9,354.3],[-78.4,355.3],[-78.9,356.2],[-79.5,357.1],
    [-80.1,358.0],[-80.7,358.9],[-81.4,359.7],[-82.1,360.5],[-82.8,361.3],[-83.5,362.0],
    [-84.3,362.8],[-85.1,363.5],[-85.9,364.1],[-86.7,364.8],[-87.6,365.4],[-88.5,366.0],
    [-89.5,366.7],[-90.6,367.3],[-91.7,367.9],[-92.9,368.5],[-94.1,369.0],[-95.3,369.6],
    [-96.6,370.1],[-97.9,370.6],[-99.3,371.0],[-100.8,371.5],[-102.2,371.9],[-103.8,372.2],
    [-105.3,372.6],[-106.9,373.0],[-108.6,373.3],[-110.3,373.6],[-112.0,373.8],[-113.8,374.1],
    [-115.6,374.3],[-117.5,374.5],[-119.5,374.6],[-121.4,374.8],[-123.4,374.9],[-125.5,375.0],
    [-128.2,375.2],[-130.9,375.3],[-133.6,375.5],[-136.3,375.6],[-139.0,375.7],[-141.8,375.9],
    [-144.5,376.0],[-147.3,376.1],[-150.1,376.2],[-152.9,376.3],[-155.7,376.4],[-158.5,376.5],
    [-161.3,376.6],[-164.2,376.7],[-167.1,376.7],[-169.9,376.8],[-172.8,376.8],[-175.8,376.9],
    [-178.7,376.9],[-181.6,376.9],[-184.6,377.0],[-187.5,377.0],[-190.5,377.0],[-193.5,377.0],
    [-198.8,377.0],[-203.9,377.0],[-209.0,377.0],[-213.9,376.9],[-218.6,376.9],[-223.3,376.8],
    [-227.9,376.7],[-232.3,376.7],[-236.6,376.6],[-240.8,376.5],[-244.8,376.4],[-248.8,376.2],
    [-252.6,376.1],[-256.3,376.0],[-259.8,375.8],[-263.3,375.7],[-266.6,375.5],[-269.8,375.3],
    [-272.9,375.1],[-275.9,374.9],[-278.7,374.7],[-281.4,374.5],[-284.0,374.2],[-286.5,374.0],
    [-337.5,370.0],[-337.5,499.0],[-334.1,500.0],[-330.6,500.9],[-327.0,501.9],[-323.3,502.8],
    [-319.5,503.7],[-315.6,504.5],[-311.6,505.3],[-307.5,506.1],[-303.3,506.9],[-299.0,507.6],
    [-294.5,508.3],[-290.0,509.0],[-285.4,509.7],[-280.6,510.3],[-275.8,510.9],[-270.8,511.4],
    [-265.8,512.0],[-260.6,512.5],[-255.4,513.0],[-250.0,513.4],[-244.5,513.9],[-239.0,514.3],
    [-233.3,514.7],[-227.5,515.0],[-224.6,515.2],[-221.8,515.5],[-218.8,515.7],[-215.8,515.9],
    [-212.8,516.1],[-209.8,516.3],[-206.6,516.5],[-203.5,516.7],[-200.3,516.8],[-197.1,517.0],
    [-193.8,517.1],[-190.5,517.2],[-187.1,517.4],[-183.8,517.5],[-180.3,517.6],[-176.8,517.7],
    [-173.3,517.7],[-169.8,517.8],[-166.1,517.9],[-162.5,517.9],[-158.8,518.0],[-155.1,518.0],
    [-151.3,518.0],[-147.5,518.0],[-139.9,518.0],[-132.4,517.9],[-125.1,517.8],[-117.9,517.6],
    [-110.9,517.3],[-104.0,517.0],[-97.2,516.7],[-90.6,516.3],[-84.1,515.8],[-77.7,515.3],
    [-71.5,514.7],[-65.4,514.1],[-59.4,513.5],[-53.6,512.7],[-47.9,511.9],[-42.4,511.1],
    [-37.0,510.2],[-31.7,509.3],[-26.6,508.3],[-21.6,507.2],[-16.7,506.1],[-12.0,505.0],
    [-7.4,503.8],[-3.0,502.5],[1.3,501.2],[5.6,499.8],[9.7,498.3],[13.8,496.8],[17.7,495.2],
    [21.6,493.6],[25.4,491.9],[29.1,490.1],[32.6,488.3],[36.1,486.4],[39.6,484.4],[42.9,482.4],
    [46.1,480.3],[49.2,478.1],[52.3,475.9],[55.2,473.6],[58.1,471.3],[60.8,468.8],[63.5,466.4],
    [66.1,463.8],[68.6,461.2],[71.0,458.5],[73.3,455.8],[75.5,453.0],[337.9,314.1],
    [337.8,318.4],[337.7,322.6],[337.5,326.6],[337.3,330.5],[337.1,334.3],[336.8,337.9],
    [336.5,341.4],[336.1,344.7],[335.7,347.9],[335.2,351.0],[334.8,353.9],[334.3,356.7],
    [333.7,359.4],[333.1,361.9],[332.5,364.3],[331.8,366.5],[331.1,368.6],[330.4,370.6],
    [329.6,372.4],[328.8,374.1],[327.9,375.6],[327.0,377.0],[326.5,377.7],[326.0,378.5],
    [325.4,379.1],[324.9,379.8],[324.3,380.5],[323.8,381.1],[323.2,381.7],[322.6,382.3],
    [321.9,382.8],[321.3,383.4],[320.7,383.9],[320.0,384.4],[319.3,384.8],[318.6,385.3],
    [317.9,385.7],[317.2,386.1],[316.5,386.5],[315.8,386.8],[315.0,387.2],[314.2,387.5],
    [313.4,387.8],[312.6,388.0],[311.8,388.3],[311.0,388.5],[310.2,388.7],[309.3,388.9],
    [308.4,389.1],[307.4,389.3],[306.4,389.4],[305.4,389.6],[304.4,389.7],[303.3,389.9],
    [302.2,390.0],[301.1,390.1],[299.9,390.3],[298.8,390.4],[297.5,390.5],[296.3,390.6],
    [295.0,390.6],[293.7,390.7],[292.3,390.8],[290.9,390.8],[289.5,390.9],[288.1,390.9],
    [286.6,391.0],[285.1,391.0],[283.6,391.0],[282.0,391.0],[268.0,391.0],[268.0,518.0],
    [282.0,518.0],[289.1,518.0],[296.1,517.9],[302.9,517.8],[309.6,517.6],[316.1,517.4],
    [322.5,517.1],[328.8,516.8],[334.9,516.4],[340.9,516.0],[346.8,515.6],[352.5,515.1],
    [358.1,514.5],[363.6,513.9],[368.9,513.2],[374.1,512.5],[379.1,511.8],[384.0,511.0],
    [388.8,510.1],[393.4,509.2],[397.9,508.3],[402.3,507.3],[406.5,506.2],[410.6,505.1],
    [414.5,504.0],[418.3,502.8],[422.1,501.5],[425.7,500.2],[429.3,498.8],[432.8,497.4],
    [436.2,495.9],[439.5,494.3],[442.8,492.7],[445.9,491.0],[449.0,489.2],[452.0,487.4],
    [454.9,485.5],[457.7,483.6],[460.4,481.5],[463.1,479.5],[465.6,477.3],[468.1,475.1],
    [470.5,472.9],[472.8,470.6],[475.0,468.2],[477.1,465.7],[479.2,463.2],[481.1,460.6],
    [483.0,458.0],[484.6,455.6],[486.2,453.2],[487.8,450.8],[489.2,448.2],[490.7,445.6],
    [492.0,443.0],[493.3,440.3],[494.6,437.6],[495.8,434.8],[497.0,431.9],[498.1,429.0],
    [499.1,426.0],[500.1,423.0],[501.1,419.9],[501.9,416.8],[502.8,413.6],[503.6,410.3],
    [504.3,407.0],[505.0,403.6],[505.6,400.2],[506.1,396.8],[506.6,393.2],[507.1,389.6],
    [507.5,386.0],[507.9,382.3],[508.2,378.4],[508.6,374.3],[508.9,370.1],[509.2,365.8],
    [509.5,361.2],[509.7,356.6],[510.0,351.8],[510.2,346.8],[510.5,341.7],[510.7,336.4],
    [510.9,331.0],[511.1,325.4],[511.2,319.7],[511.4,313.8],[511.5,307.8],[511.6,301.6],
    [511.7,295.2],[511.8,288.8],[511.9,282.1],[511.9,275.3],[512.0,268.4],[512.0,261.3],
    [512.0,254.0],[512.0,214.0],[268.0,214.0],[268.0,305.0],[338.0,305.0],[338.0,309.6]
];
ae_logo_paths = [
    [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
     33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,
     63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,
     93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,
     117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,
     139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,
     161,162,163,164,165,166,167,168,169,170,171,172,173,174],
    [175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,
     197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,
     219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,
     241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,256,257,258,259,260,261,262,
     263,264,265,266,267,268,269,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,
     285,286,287,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305,306,
     307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323,324,325,326,327,328,
     329,330,331,332,333,334,335,336,337,338,339,340,341,342,343,344,345,346,347,348,349,350,
     351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,
     373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,
     395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,
     417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,
     439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,
     461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,
     483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,
     505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,
     527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,
     549,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,
     571,572,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,589,590,591,592,
     593,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,
     615,616,617,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,
     637,638,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,
     659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,
     681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700],
    [701,702,703,704,705,706,707,708,709,710,711,712,713,714,715,716,717,718,719,720,721,722,
     723,724,725,726,727,728,729,730,731,732,733,734,735,736,737,738,739,740,741,742,743,744,
     745,746,747,748,749,750,751,752,753,754,755,756,757,758,759,760,761,762,763,764,765,766,
     767,768,769,770,771,772,773,774,775,776,777,778,779,780,781,782,783,784,785,786,787,788,
     789,790,791,792,793,794,795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,
     811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,
     833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,
     855,856,857,858,859,860,861,862,863,864,865,866,867,868,869,870,871,872,873,874,875],
];
module ae_logo_raw() { polygon(points = ae_logo_pts, paths = ae_logo_paths); }

// 2-D artwork, normalised to logo_w wide (aspect kept), centred on the origin.
// ae_logo_raw() is in the raw SVG viewBox coordinate space; subtracting the
// viewBox centre reproduces the old import(center=true) exactly (which centred on
// the viewBox, NOT the ink). We then resize to logo_w and translate by the
// measured ink-centre (as a fraction of logo_w) to put the ink truly on the
// origin BEFORE rotating — so it stays centred at any logo_spin.
logo_cx = 0.5112;   // ink centre X / logo_w, after resize, before rotation
logo_cy = 0.1092;   // ink centre Y / logo_w
module logo2d() {
    mirror([logo_mirror ? 1 : 0, 0, 0])
        rotate([0, 0, logo_spin])
            translate([-logo_cx * logo_w, -logo_cy * logo_w])
                resize([logo_w, 0, 0], auto = true)
                    translate([-ae_logo_vb_cx, -ae_logo_vb_cy])
                        ae_logo_raw();
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

// The "hat" aperiodic monotile (einstein tile), as digitised — 13 vertices,
// twelve edges of one unit and one doubled edge. Native bbox 52.75 x 77.22 mm.
MONO_PTS = [
    [40.04, 78.63], [54.16, 70.45], [54.15, 37.83], [40.01, 29.67],
    [48.15, 15.53], [40.00,  1.41], [25.87,  9.58], [25.88, 25.89],
    [ 9.56, 25.90], [ 1.41, 40.04], [15.54, 48.19], [15.55, 64.51],
    [31.86, 64.50]
];
// Seat point in native polygon coords: where the case footprint sits in the
// hat's wide head with the most clearance (31 x 21 fits with ~3.6 mm at 1.0).
// Translate-then-scale about this point, so the case stays put at any scale.
mono_seat_x = 34.91;
mono_seat_y = 50.41;

// Monotile foot outline, centred so mono_seat lands on the case origin.
module monotile_2d() {
    scale(mono_scale)
        translate([-mono_seat_x, -mono_seat_y])
            polygon(points = MONO_PTS);
}

// 2-D foot footprint: the cup-seat rectangle (always, so the rim is supported)
// unioned with either the monotile or the legacy flange arms.
module foot_2d() {
    union() {
        rrect_c(ext_w, ext_d, shell_round);                       // core (cup seat)
        if (base_monotile)
            translate([mono_dx, mono_dy]) rotate([0, 0, mono_rot]) monotile_2d();
        else {
            if (flange_y) rrect_c(ext_w, ext_d + 2 * flange_out, flange_round);  // +/-Y arms
            if (flange_x) rrect_c(ext_w + 2 * flange_out, ext_d, flange_round);  // +/-X arms
        }
    }
}

// Optional mounting holes near each arm tip (axis Z, through the foot).
// Legacy-flange only — the monotile foot stays a clean silhouette.
module foot_holes() {
    if (!base_monotile && flange_hole_d > 0) {
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
echo(str("assembled height ~ ", plate_t + cav_h + roof_t, " mm; foot = ",
         base_monotile
           ? str("hat monotile x", mono_scale, " (~", round(52.75 * mono_scale),
                 " x ", round(77.22 * mono_scale), " mm)")
           : str("flanged ", ext_w + (flange_x ? 2 * flange_out : 0), " x ",
                 ext_d + (flange_y ? 2 * flange_out : 0), " mm")));
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
    echo(str("logo = ae (inlined), ", logo_w, " mm wide on the ", logo_face,
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
    foot_span = base_monotile ? 52.75 * mono_scale : ext_w + 2 * flange_out;
    color("Gainsboro") base();
    translate([foot_span + 25, 0, 0])
        color("LightSteelBlue") top_body();
}
