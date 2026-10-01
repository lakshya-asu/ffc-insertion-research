"""Parametric holder for the two 5-pin connector housings of the SANDAL plug_5pin cell, for resin printing on a Form 4.

One base per part: one for the male (pins) housing, one for the female (sockets) housing. Each housing stands in a
pocket MATING FACE DOWN, wires up. The gripper comes from above with its fingers pointing down, closes on the
housing's two end faces, and lifts: the mating face then points out past the fingertips and the wires run back
toward the palm, which is exactly how the connect policy's demonstrations hold both parts (checked on the wrist
camera frames of episode c3e3cb5e, 2026-09-24: left arm holds the male, right arm holds the female). A pose move
to the handover pose then turns the wrist; the part's position in the fingers is what has to match, not its
orientation in the room. The male's pins hang into a slot in the pocket floor so they never carry the part's weight.

Everything that depends on the actual parts is a parameter at the top. Defaults are a standard 1x5 2.54 mm
"Dupont" crimp housing (14.6 x 2.6 x 14.5 mm). MEASURE the real housings with calipers and set the four *_mm
values before printing; the tolerance coupon (second STL) exists so the pocket width can be chosen empirically.

Run:  cadenv/bin/python connector_holder.py [outdir]   -> holder_pins.stl/.step, holder_sockets.stl/.step, coupon.stl
"""
from __future__ import annotations

import cadquery as cq

# ------------------------------------------------------------------ measure these on the real parts (mm)
# Two separate holders, one per part. Each gets its own numbers; defaults are a 1x5 2.54 mm crimp housing.
# Named by what the part looks like, because the team's notes call the right arm's part "the male" while the
# recordings show the RIGHT arm holding the part with SOCKETS and the LEFT arm holding the part with PINS.
#   pins    = housing with five protruding pins, held by the LEFT arm   (label P)
#   sockets = housing with five socket openings, held by the RIGHT arm  (label S)
PARTS = {
    "pins":    dict(L=14.6, W=2.6, H=14.5, pin_len=6.0, pin_row=11.0, pin_t=0.64, label="P"),
    "sockets": dict(L=14.6, W=2.6, H=14.5, pin_len=0.0, pin_row=0.0, pin_t=0.0, label="S"),
}
# L: along the row of 5 contacts. W: single-row housing thickness. H: body height without pins.
# pin_len: how far the male pins stick out of the mating face (0 for the female, whose face is flat).
# pin_row: end-to-end span of the five pins (2.54 mm pitch x 4 + one pin width). pin_t: pin thickness.
# The pins hang into a blind slot below the pocket floor; the housing rests on the floor either side of the slot.
# Wires point UP out of the pocket and sit between the finger pads; they need no room in the holder.
HOUSING_L, HOUSING_W, HOUSING_H = PARTS["pins"]["L"], PARTS["pins"]["W"], PARTS["pins"]["H"]   # used by the coupon

# ------------------------------------------------------------------ fit and geometry choices
CLEARANCE = 0.25      # per side, pocket to housing; Form 4 Grey resin holds about 0.1 mm, so 0.25 gives a loose drop-in
POCKET_DEPTH = 6.0    # how deep the housing sits (mating face on the floor); HOUSING_H - POCKET_DEPTH stands proud
LEADIN = 1.2          # chamfer at the pocket mouth so a slightly-off part slides in and self-centres
FINGER_CUT_W = 20.0   # width (across Y) of the clearance trench at each END face, so the finger pads can reach below the top
FINGER_CUT_LEN = 12.0  # how far the trench extends outward from each end face (along X)
FINGER_CUT_DEPTH = 2.0  # how far below the top surface the trench goes; the pad tips then sit POCKET_DEPTH - this = 4 mm
#                        above the mating face, which is what the demonstration frames show (pads end 4 to 6 mm behind it)
PIN_SLOT_MAX = max(p["pin_len"] for p in PARTS.values())
FLOOR = PIN_SLOT_MAX + 1.0 + 2.0   # blind pin slot (pin length + 1 mm) plus 2 mm of solid floor under it
BASE_T = FLOOR + POCKET_DEPTH
BASE_L = 84.0         # single-pocket base, wide and low so an arm bump cannot tip it
BASE_W = 60.0
SCREW_D = 5.5         # through hole: M5 machine screw into a T-slot/threaded insert, or a #10 wood screw into the bench
CBORE_D = 10.5        # counterbore so the screw head or washer sits below the top surface
CBORE_DEPTH = 5.0
DOWEL_D = 3.1         # two 3 mm dowel holes: put pins in the bench once, and the holder always goes back to the same spot
TAPE_RECESS = 0.8     # underside recesses for 3M VHB pads if screws are not wanted
FILLET = 3.0
BOLT_D = 5.5          # M5 clearance for clamping to the bench; or ignore and use tape
LABEL_DEPTH = 0.6


def pocket_cut(part: dict) -> cq.Workplane:
    """Everything to subtract for one pocket, centred at the origin, top face at z=0 (base top)."""
    L = part["L"] + 2 * CLEARANCE
    W = part["W"] + 2 * CLEARANCE
    # the pocket itself, from the top face down POCKET_DEPTH; the mating face rests on its floor
    pocket = cq.Workplane("XY").box(L, W, POCKET_DEPTH, centered=(True, True, False)).translate((0, 0, -POCKET_DEPTH))
    # lead-in: a truncated pyramid at the mouth
    mouth = (
        cq.Workplane("XY")
        .rect(L + 2 * LEADIN, W + 2 * LEADIN)
        .workplane(offset=-LEADIN)
        .rect(L, W)
        .loft(combine=True)
    )
    cut = pocket.union(mouth)
    # male only: a blind slot below the pocket floor for the pins, 0.4 mm wider than the pin row each way and 0.8 mm
    # wider than a pin, so the housing's shoulders (about 0.5 mm each side) rest on the floor and the pins touch nothing
    if part["pin_len"] > 0:
        slot = cq.Workplane("XY").box(part["pin_row"] + 0.8, part["pin_t"] + 0.8, part["pin_len"] + 1.0, centered=(True, True, False))
        cut = cut.union(slot.translate((0, 0, -POCKET_DEPTH - part["pin_len"] - 1.0)))
    # finger clearance at the two END faces (+/- X) only: two trenches outside the pocket ends, so the pads can close
    # below the top surface while the pocket's long walls (+/- Y) stay full height and keep locating the housing
    for sign in (-1, 1):
        trench = cq.Workplane("XY").box(FINGER_CUT_LEN, FINGER_CUT_W, FINGER_CUT_DEPTH, centered=(True, True, False))
        cut = cut.union(trench.translate((sign * (L / 2 + FINGER_CUT_LEN / 2), 0, -FINGER_CUT_DEPTH)))
    return cut


def holder(part: dict) -> cq.Workplane:
    """One base with one pocket for one part."""
    base = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(FILLET)
        .translate((0, 0, -BASE_T))
    )
    base = base.cut(pocket_cut(part))
    # four counterbored screw holes in the corners, clear of the finger trenches (x within +/-20, y within +/-10)
    corners = [(sx * (BASE_L / 2 - 9), sy * (BASE_W / 2 - 9)) for sx in (-1, 1) for sy in (-1, 1)]
    base = base.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(corners).cboreHole(SCREW_D, CBORE_D, CBORE_DEPTH)
    # two dowel holes on the long axis, outside the trenches
    base = base.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints([(-BASE_L / 2 + 9, 0), (BASE_L / 2 - 9, 0)]).hole(DOWEL_D)
    # underside recesses for adhesive pads, one per corner, 20 x 20 mm, around the screw holes
    for (x, y) in corners:
        base = base.cut(cq.Workplane("XY").box(20, 20, TAPE_RECESS, centered=(True, True, False)).translate((x, y, -BASE_T - 0.01)))
    # a chamfer on the top outside edge so the arms and hands do not catch it
    base = base.faces(">Z").edges(">Y or <Y or >X or <X").chamfer(1.0)
    # engraved label
    base = base.cut(cq.Workplane("XY").text(part["label"], 6, LABEL_DEPTH, combine=False).translate((0, -BASE_W / 2 + 6, -LABEL_DEPTH)))
    return base


def coupon() -> cq.Workplane:
    """Five pockets of increasing width so the best fit can be chosen by hand: clearance 0.10 to 0.40 per side."""
    n = 5
    step = 16.0
    length = n * step + 10
    base = cq.Workplane("XY").box(length, 30, BASE_T, centered=(True, True, False)).edges("|Z").fillet(2).translate((0, 0, -BASE_T))
    for i, c in enumerate((0.10, 0.175, 0.25, 0.325, 0.40)):
        x = -length / 2 + 10 + i * step
        L = HOUSING_L + 2 * c
        W = HOUSING_W + 2 * c
        p = cq.Workplane("XY").box(L, W, POCKET_DEPTH, centered=(True, True, False)).translate((x, 0, -POCKET_DEPTH))
        m = cq.Workplane("XY").rect(L + 2, W + 2).workplane(offset=-1).rect(L, W).loft(combine=True).translate((x, 0, 0))
        base = base.cut(p).cut(m)
        # same pin slot as the pins holder, so the pins part can be tried mating face down too
        pp = PARTS["pins"]
        slot = cq.Workplane("XY").box(pp["pin_row"] + 0.8, pp["pin_t"] + 0.8, pp["pin_len"] + 1.0, centered=(True, True, False))
        base = base.cut(slot.translate((x, 0, -POCKET_DEPTH - pp["pin_len"] - 1.0)))
        base = base.cut(cq.Workplane("XY").text(f"{c:.2f}", 3, 0.5, combine=False).translate((x, -10, -0.5)))
    return base


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
    out.mkdir(parents=True, exist_ok=True)
    for name, part in PARTS.items():
        h = holder(part)
        cq.exporters.export(h, str(out / f"holder_{name}.stl"), tolerance=0.01, angularTolerance=0.1)
        cq.exporters.export(h, str(out / f"holder_{name}.step"))
        bb = h.val().BoundingBox()
        print(f"holder_{name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm; pocket {part['L'] + 2*CLEARANCE:.2f} x {part['W'] + 2*CLEARANCE:.2f} x {POCKET_DEPTH} mm; exposed {part['H'] - POCKET_DEPTH:.1f} mm")
    c = coupon()
    cq.exporters.export(c, str(out / "coupon.stl"), tolerance=0.01, angularTolerance=0.1)
    print("wrote", sorted(p.name for p in out.iterdir()))
