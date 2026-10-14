"""Servo tool v1: the printed and machined parts as STEP files (cadquery, run in the stepenv interpreter), sized from
docs/research/2026-10/research_notes/servo_tool_v1_design_numbers.md (drive layout 2026-10-13 night, revised after the schematic's interference check)
and the twin's GRIPPERS["servo_v1"] entry.
CAD frame (the twin's assembly frame shifted): X along the body axis with the jaw end face at X = 61 (assembly X + 59), Y across (assembly Y - 26),
Z along the stroke (assembly Z - 20: the stroke centreline is Z = 0, the flange side is +Z).
Drive: the XL330-M077-T lies flat in the back of the body (20 across, 34 along the axis, 26 tall: its full height), horn up; a 3 mm GT2 belt over the
top (under a 6 mm cover) turns the M4 x 1 lead screw 1:1 through two 12-tooth pulleys; the screw runs down through the body's top wall at X = 50 into
the PEEK nut in the upper carrier and on through a clearance hole in the lower carrier. The two carriers (16 x 14 x 4) run in the printed slot through
the front 16 mm of the body (the screw is the master guide for the upper one); the rocker (3 x 3 x 22, along the stroke) pivots on a 2 mm pin through
the end block's middle web at the carriers' front-west corner and rides in a notch in each carrier. The front 6 mm (the end block) is open above and
below the carriers so the fingers and cell P sit on their outer faces. Bought parts are not modelled.
usage: <stepenv>/bin/python -I scripts/ffc_servo_tool_cad.py <out dir>"""
import sys
from pathlib import Path
import cadquery as cq
OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "out/servo_v1_cad"); OUT.mkdir(parents=True, exist_ok=True)
BODY_L, BODY_W, BODY_H = 55.0, 22.0, 26.0        # along the axis, across, along the stroke
GUIDE_L = 6.0                                     # the end block ahead of the body
L = BODY_L + GUIDE_L                              # 61: the jaw end face
SERVO = (34.0, 20.0, 26.0)                        # XL330-M077-T lying flat: 34 along the axis, 20 across, 26 along the stroke; horn axis along the 26
SCREW_X = L - 11.0                                # the lead screw, 50 from the back end (assembly X = -9)
JAW_T, JAW_D, JAW_W = 4.0, 16.0, 14.0             # each carrier: 4 along the stroke, 16 along the axis (6 exposed in the end block, 10 inside the body), 14 across
NUT_D = 8.0                                       # PEEK nut: 8 dia, 8 long, bonded in the upper carrier, protruding into the gap
STROKE, OPEN_GAP = 1.5, 14.0
SLOT_H = OPEN_GAP + 2 * JAW_T + 2 * STROKE + 0.4   # 25.4: the carriers' travel zone
ROCK_X, ROCK_Y, ROCK_T = L - 2.0 - 2.5 + 1.0, -5.5, 3.0    # rocker centre: 2.5 behind the jaw face region... see note; 5.5 west of the centreline
COVER_H = 6.0
rows = []


def save(name, shape, note):
    cq.exporters.export(shape, str(OUT / f"{name}.step"))
    bb = shape.val().BoundingBox(); rows.append((name, f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}", note)); print("wrote", name, f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")


def cyl_z(r, z0, z1, x, y):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def cyl_y(r, y0, y1, x, z):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


ROCK_X = L - 2.0                                  # rocker centreline 2 mm behind the jaw end face (assembly X = -2), inside the end block's middle web
# 1. body
body = cq.Workplane("XY").box(L, BODY_W, BODY_H).translate((L / 2, 0, 0))
pocket = cq.Workplane("XY").box(SERVO[0] + 0.6, SERVO[1] + 0.6, SERVO[2] + 2).translate((2.0 + (SERVO[0] + 0.6) / 2, 0, 1.0))   # servo lies flat, 2 mm front wall, open at the back and the top
body = body.cut(pocket)
slot = cq.Workplane("XY").box(JAW_D + 0.4, JAW_W + 0.4, SLOT_H).translate((L - (JAW_D + 0.4) / 2 + 0.4, 0, 0))                     # the carriers' slot, open at the front face
body = body.cut(slot)
# the end block: open above and below the carriers for the front 6 mm, except the middle web between the jaws and the side cheeks
for z0, z1 in ((OPEN_GAP / 2 + 0.2, BODY_H / 2 + 1), (-BODY_H / 2 - 1, -OPEN_GAP / 2 - 0.2)):
    body = body.cut(cq.Workplane("XY").box(GUIDE_L + 0.4, JAW_W + 0.4, z1 - z0).translate((L - (GUIDE_L + 0.4) / 2 + 0.4, 0, (z0 + z1) / 2)))
web = cq.Workplane("XY").box(GUIDE_L, JAW_W + 0.4, OPEN_GAP - 2 * STROKE - 0.6).translate((L - GUIDE_L / 2, 0, 0))                  # the middle web between the carriers (10.4 tall: clear of both at full stroke)
web = web.cut(cyl_z(NUT_D / 2 + 0.3, -20, 20, SCREW_X, 0)) if SCREW_X > L - GUIDE_L else web
body = body.union(web)
body = body.cut(cyl_z(2.15, 0, BODY_H / 2 + 1, SCREW_X, 0))                                                                       # screw bore through the top wall (its seat) and the slot
body = body.cut(cyl_y(1.05, -BODY_W / 2 - 1, BODY_W / 2 + 1, ROCK_X, 0))                                                           # rocker pivot pin across the end block (through the cheeks and the web)
body = body.cut(cq.Workplane("XY").box(ROCK_T + 0.6, ROCK_T + 0.6, OPEN_GAP - 2 * STROKE).translate((ROCK_X, ROCK_Y, 0)))             # rocker channel through the web
belt = cq.Workplane("XY").box(SCREW_X - 28.0 + 12.0, 12.0, 1.0).translate(((SCREW_X + 28.0) / 2, 0, BODY_H / 2 - 0.5))              # shallow belt channel on top, from the horn pulley to the screw pulley
body = body.cut(belt)
for x in (BODY_L - 12.0, BODY_L - 22.0):                                                                                        # heat-set inserts at CAD X 43 and 33 (assembly -16, -26): behind the carriers' slot, under the side plate
    for z in (BODY_H / 2 - 5.0, -(BODY_H / 2 - 5.0)):
        body = body.cut(cq.Workplane("XZ").circle(1.25).extrude(-8.0).translate((x, BODY_W / 2, z)))                                # 4 x M3 heat-set, east face, behind the slot
save("servo_body", body, "PAHT-CF print, 0.2 mm layers, 4 walls. Servo pocket (flat, horn up, open at the back and top); carriers' slot through the front 16 mm with 2 mm top and bottom walls, open above and below the carriers for the front 6 mm; middle web between the jaws; 4.3 mm screw bore in the top wall; 2 mm rocker pivot across the end block; belt channel on top; 4 x M3 heat-set in the east face")

# 2. belt cover
cover = cq.Workplane("XY").box(L, BODY_W, COVER_H).translate((L / 2, 0, BODY_H / 2 + COVER_H / 2))
cover = cover.cut(cq.Workplane("XY").box(SCREW_X - 28.0 + 14.0, 13.0, COVER_H - 1.5).translate(((SCREW_X + 28.0) / 2, 0, BODY_H / 2 + (COVER_H - 1.5) / 2)))
save("servo_belt_cover", cover, "PAHT-CF print; hollow over the two 12T GT2 pulleys and the 3 mm belt; 4 x M2 into the body")

# 3. carriers (CAD X 0..16 = assembly -14..2): nut pocket or screw clearance at X 5 (the screw), the rocker notch at the front-west corner, the MEGP finger pattern at X 13
def carrier(upper):
    c = cq.Workplane("XY").box(JAW_D, JAW_W, JAW_T).translate((JAW_D / 2, 0, 0))
    xs = SCREW_X - (L - JAW_D)                                                                                                      # 5.0
    if upper:
        c = c.cut(cyl_z(NUT_D / 2 + 0.1, -JAW_T / 2 - 1, JAW_T / 2 + 1, xs, 0))                                                      # nut pocket (the nut is 8 long: it protrudes into the gap)
    else:
        c = c.cut(cyl_z(2.2, -JAW_T / 2 - 1, JAW_T / 2 + 1, xs, 0))                                                                  # screw clearance
    c = c.cut(cq.Workplane("XY").box(ROCK_T + 1.8, ROCK_T + 0.6, JAW_T + 1).translate((ROCK_X - (L - JAW_D), ROCK_Y, 0)))              # rocker notch (the end swings 0.1 mm along X)
    if upper:
        for y in (5.0, -5.0):
            c = c.cut(cyl_z(2.2, -JAW_T / 2 - 1, JAW_T / 2 + 1, 13.0, y))                                                            # 2 x M4 through bolts: cell P's fixed-end holes (10 mm apart), nuts below
    else:
        c = c.cut(cyl_z(2.0, -JAW_T / 2 - 1, JAW_T / 2 + 1, 13.0, 0))                                                                # M4 through bolt for the lower finger, nut above
        for y in (3.0, -3.0):
            c = c.cut(cyl_z(0.75, -JAW_T / 2 - 1, JAW_T / 2 + 1, 13.0, y))                                                           # 1.5 H7 finger pins
    return c


save("servo_jaw_carrier_upper", carrier(True), "PAHT-CF print; PEEK nut bonded in the pocket (the screw is its guide); rocker notch at the front-west corner; 2 x M4 through bolts on the exposed top face for cell P's fixed end (nuts in the gap below)")
save("servo_jaw_carrier_lower", carrier(False), "PAHT-CF print; 4.4 mm screw clearance; rocker notch; MEGP finger pattern for the lower finger; guided by the slot and the rocker")

# 4. rocker: 3 x 3 x 22 along the stroke, 2 mm pivot through its centre (across); its ends ride in the carriers' notches 9 mm above and below
rocker = cq.Workplane("XY").box(ROCK_T, ROCK_T, 22.0)
rocker = rocker.cut(cyl_y(1.05, -3.0, 3.0, 0, 0))
save("servo_rocker", rocker, "PAHT-CF print; 2 mm steel pivot pin through the end block's cheeks and web; ends in the notches of both carriers: 1.5 mm down on the upper is 1.5 mm up on the lower")

# 5. standoffs
for name, d, h in (("servo_standoff_meca", 28.0, 22.0), ("servo_standoff_lrmate", 36.0, 34.0)):
    so = cq.Workplane("XY").circle(d / 2).extrude(h)
    for x, y in ((7.75, 7.75), (-7.75, 7.75), (7.75, -7.75), (-7.75, -7.75)):
        so = so.cut(cyl_z(1.7, -0.5, h + 0.5, x, y))
    so = so.cut(cyl_z(2.1, -0.5, h + 0.5, 0, -8.0)).cut(cyl_z(2.1, -0.5, h + 0.5, 0, 2.0))
    save(name, so, "PAHT-CF print, solid; 4 x M3 clearance on the 15.5 mm square to the adapter plate; 2 x M4 through for cell A's fixed end")

# 6. cell bridge plates (6061, 4 mm)
PLATE_T, CELL_W = 4.0, 12.7
corner = cq.Workplane("XY").box(CELL_W, 2 * CELL_W + 1.0 + 15.0, PLATE_T).translate((0, 0, PLATE_T / 2))
for y in (-10.0, -4.0, 5.0, 11.0):
    corner = corner.cut(cyl_z(2.3, -0.5, PLATE_T + 0.5, 0, y))
save("cell_corner_plate", corner, "6061, 4 mm: on top of cell A's free end (2 x M5) and cell B's fixed end (2 x M4)")
tab = cq.Workplane("XY").box(15.0, CELL_W + 8.0, PLATE_T).translate((0, 0, PLATE_T / 2))
for x in (-4.0, 4.0):
    tab = tab.cut(cyl_z(2.3, -0.5, PLATE_T + 0.5, x, 4.0))
side = cq.Workplane("XY").box(15.0, PLATE_T, 32.0).translate((0, -(CELL_W + 8.0) / 2 + PLATE_T / 2, -16.0 + PLATE_T))
for x in (-5.15, 4.85):
    for z in (-6.0, -24.0):
        side = side.cut(cyl_y(1.7, -(CELL_W + 8.0) / 2 - 1, -(CELL_W + 8.0) / 2 + PLATE_T + 1, x, z))
save("cell_side_plate", side.union(tab), "6061, 4 mm, one piece: the tab on top of cell B's free end (2 x M5) and the plate down the body's east face to the stroke centreline (4 x M3 into the heat-set inserts, behind the carriers' slot)")

(OUT / "index.html").write_text("<!doctype html><meta charset=utf-8><title>Servo tool v1 parts</title><style>body{font:14px/1.5 -apple-system,Helvetica,sans-serif;margin:24px;background:#FDF9F2}td,th{padding:6px 10px;text-align:left;border-bottom:1px solid #EFE6D6}</style><h1>Servo tool v1: STEP files (drive layout 2026-10-13 night)</h1><p>Bought parts not modelled: XL330-M077-T (lying flat, horn up), two 12T GT2 pulleys and a 3 mm GT2 belt, M4 x 1 lead screw (McMaster 90500A160) with the PEEK nut (90500A310), a 2 mm rocker pivot pin, three TAL220B 5 kg cells, two ADS122C04 boards, the QT Py RP2040, the Pololu buck. The fingers, shoe and pad are the same STEP files as the bought stacks; the upper finger's cross plate is cell P. The working 3D schematic with the interference check: FFC-servo-tool-3d.html.</p><table><tr><th>part</th><th>size mm</th><th>notes</th></tr>" + "".join(f"<tr><td><a href='{n}.step'>{n}.step</a></td><td>{s}</td><td>{t}</td></tr>" for n, s, t in rows) + "</table>")
print("index written to", OUT / "index.html")
