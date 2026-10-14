"""Visual meshes for the FFC/ZIF twin, built from the same dimensions as the physics boxes: connector housing with chamfered mouth,
side ears and pivot bosses; the white actuator bar with pivot pins and a finger notch; the cable stiffener with its label; the printed
finger pair (lower: legs, cross bar, shim-steel shoe, lip; upper: legs, pad). Written as OBJ into assets/ffc/ (metres). The physics
stays on the simple boxes; these are drawn in their place. Run with the CAD interpreter that has trimesh:
  <cadenv>/bin/python scripts/ffc_zif_meshes.py"""
import sys
from pathlib import Path
import numpy as np, trimesh
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
OUT = Path(__file__).resolve().parent.parent / "assets" / "ffc"; OUT.mkdir(parents=True, exist_ok=True)

# dimensions (m), mirrored from cell_twin/ffc_zif.py so this file has no mujoco dependency
PITCH, N = 0.8e-3, 32
CABLE_W = 0.028; HOUSING_L = PITCH * (N + 1) + 0.0050; HOUSING_D = 0.0060; HOUSING_H = 0.0020
SLOT_FLOOR, SLOT_GAP, INSERT_DEPTH, ROOF_D = 0.0006, 0.00040, 0.0035, 0.0015
LATCH_CLAMP_GAP, LID_PROUD = 0.00027, 0.0002
LID_T = HOUSING_H + LID_PROUD - SLOT_FLOOR - LATCH_CLAMP_GAP; LID_AXIS_BELOW_TOP = 0.0003; LID_D = HOUSING_D - ROOF_D - 0.0002
STIFF_T, STIFF_L, BODY_T = 0.00030, 0.0130, 0.00014
BLADE_L, BLADE_T, BLADE_W = 0.0015, 0.0004, 0.020
SHOE_D, SHOE_W, PAD_W, PAD_D, PAD_T_UP, PAD_OPEN = 0.007, 0.020, 0.046, 0.006, 0.002, 0.003
JAW_DROP, LEG_W, BLOCK_CLEAR, FINGER_OFF = 0.030, 0.004, 0.015, 0.020
LEG_Y = CABLE_W / 2 + 0.001 + LEG_W / 2; PLEG_Y = LEG_Y + LEG_W + 0.0005


def box(size, at, color):
    m = trimesh.creation.box(extents=size); m.apply_translation(at); m.visual.face_colors = color; return m


def cyl(r, h, at, axis, color):
    m = trimesh.creation.cylinder(radius=r, height=h, sections=24)
    if axis == "x":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    m.apply_translation(at); m.visual.face_colors = color; return m


def wedge(w, d, h, at, color, flip=False):
    """the chamfer at the mouth: a thin slab tilted so it slopes down toward +y (no booleans needed for a visual)"""
    L = (d ** 2 + h ** 2) ** 0.5; ang = np.arctan2(h, d)
    m = trimesh.creation.box(extents=[w, L, 0.0002])
    m.apply_transform(trimesh.transformations.rotation_matrix(-ang if not flip else ang, [1, 0, 0]))
    m.apply_translation([at[0], at[1] + d / 2, at[2] + h / 2]); m.visual.face_colors = color; return m


def save(name, parts):
    m = trimesh.util.concatenate(parts); m.export(OUT / f"{name}.obj"); print(name, len(m.faces), "faces")


black = [40, 40, 44, 255]; dark = [70, 70, 74, 255]; white = [235, 232, 222, 255]; gold = [215, 178, 60, 255]
blue = [50, 90, 190, 255]; amber = [215, 155, 60, 255]; steel = [190, 195, 205, 255]; printed = [238, 140, 50, 255]; label = [245, 245, 240, 255]

# ---- connector housing (frame: origin at the mouth line on the board, +y toward the cable, -y toward the rear, z up)
hl = HOUSING_L / 2; rear = -INSERT_DEPTH; gw = (HOUSING_L - CABLE_W - 0.0006) / 2
parts = [box([HOUSING_L, HOUSING_D, SLOT_FLOOR], [0, -HOUSING_D / 2, SLOT_FLOOR / 2], black),                              # floor
         box([HOUSING_L, HOUSING_D - INSERT_DEPTH, HOUSING_H], [0, rear - (HOUSING_D - INSERT_DEPTH) / 2, HOUSING_H / 2], black),   # rear stop
         box([HOUSING_L - 2 * gw, ROOF_D, HOUSING_H - SLOT_FLOOR - SLOT_GAP], [0, rear + ROOF_D / 2, SLOT_FLOOR + SLOT_GAP + (HOUSING_H - SLOT_FLOOR - SLOT_GAP) / 2], black)]
for sgn in (-1, 1):   # side guides with a chamfered lead-in at the mouth, and pivot bosses at the rear
    parts.append(box([gw, INSERT_DEPTH, HOUSING_H], [sgn * (hl - gw / 2), -INSERT_DEPTH / 2, SLOT_FLOOR + HOUSING_H / 2], black))
    parts.append(box([gw, 0.0008, HOUSING_H + 0.0006], [sgn * (hl - gw / 2), 0.0004, SLOT_FLOOR + (HOUSING_H + 0.0006) / 2], black))
    parts.append(cyl(0.0007, gw, [sgn * (hl - gw / 2), rear + ROOF_D, SLOT_FLOOR + LATCH_CLAMP_GAP + LID_T - LID_AXIS_BELOW_TOP], "x", dark))
parts.append(wedge(HOUSING_L - 2 * gw, 0.0006, 0.0003, [0, 0, SLOT_FLOOR - 0.0003], black))        # chamfer below the mouth
for i in range(N):   # contact fingers on the floor
    parts.append(box([0.0004, 0.0012, 0.00006], [-(N - 1) / 2 * PITCH + i * PITCH, rear + ROOF_D + 0.0006, SLOT_FLOOR + 0.00003], gold))
save("zif_housing", parts)

# ---- actuator bar (frame: hinge axis at the origin, +y toward the mouth; closed pose)
parts = [box([HOUSING_L - 2 * gw - 0.0002, LID_D - 0.0005, LID_T], [0, 0.0005 + (LID_D - 0.0005) / 2, LID_AXIS_BELOW_TOP - LID_T / 2], white)]
for sgn in (-1, 1):   # pivot pins into the bosses
    parts.append(cyl(0.0005, 0.0012, [sgn * (hl - gw - 0.0001 + 0.0006), 0, 0], "x", white))
parts.append(box([0.008, 0.0006, 0.0003], [0, LID_D - 0.0008, LID_AXIS_BELOW_TOP + 0.00015], white))   # finger notch ridge at the front edge
save("zif_actuator", parts)

# ---- cable stiffener (frame: root at the origin, tip toward -y) with the label on top of the body end
parts = [box([CABLE_W, STIFF_L, STIFF_T], [0, -STIFF_L / 2, 0], blue),
         box([CABLE_W - 0.002, 0.004, 0.00005], [0, -STIFF_L + 0.0025, STIFF_T / 2 + 0.00003], [20, 30, 60, 255])]   # dark strip near the tip like the photo
save("ffc_stiffener", parts)
parts = [box([CABLE_W, 0.0050, BODY_T], [0, -0.0025, 0], amber)]
save("ffc_segment", parts)
parts = [box([CABLE_W, 0.0050, BODY_T], [0, -0.0025, 0], amber), box([0.017, 0.0046, 0.00008], [0, -0.0025, BODY_T / 2 + 0.00004], label)]
save("ffc_segment_label", parts)

# ---- fingers (jaw frame: x down, y across (+y east), z forward; origin on the shoe's upper surface at the jaw tip line). Both are C-shaped from the east.
tipz = PAD_D + 0.010; shoe_t = 0.0005
carr = [120, 122, 126, 255]
wing_y0, wing_y1 = -SHOE_W / 2, LEG_Y + LEG_W / 2
parts = [box([shoe_t, SHOE_W, SHOE_D], [shoe_t / 2, 0, tipz - SHOE_D / 2], steel),                                   # shim-steel shoe, centred on the cable
         box([BLADE_T, BLADE_W, BLADE_L], [shoe_t - BLADE_T / 2, 0, tipz + BLADE_L / 2], steel),                      # lip
         box([shoe_t, wing_y1 - wing_y0, 0.003], [shoe_t / 2, (wing_y0 + wing_y1) / 2, tipz - SHOE_D + 0.0015], steel),   # rear wing out to the east leg
         box([0.006, FINGER_OFF + 0.008 - (LEG_Y - LEG_W / 2), 0.011], [-BLOCK_CLEAR, (LEG_Y - LEG_W / 2 + FINGER_OFF + 0.008) / 2, 0.0075], printed),   # beam from the carriage to the leg
         box([0.006, 0.016, 0.003], [-BLOCK_CLEAR - 0.003, FINGER_OFF, 0.0035], carr),                                 # lower carriage (gripper part)
         box([BLOCK_CLEAR - 0.003, LEG_W, 0.004], [-(BLOCK_CLEAR - 0.003) / 2, LEG_Y, tipz - SHOE_D + 0.002], printed)]   # the one leg
save("finger_lower", parts)
save("finger_lower_steel", parts[:3]); save("finger_lower_body", parts[3:])
pz = PAD_D / 2 + 0.010; px0 = -PAD_OPEN - PAD_T_UP / 2
parts = [box([PAD_T_UP, PAD_W, PAD_D], [px0 + 0.0003, 0, pz], printed),
         box([0.0015, PAD_W - 0.004, PAD_D - 0.002], [px0 + 0.0003 + PAD_T_UP / 2 - 0.00075 + 0.0005, 0, pz], [60, 60, 64, 255]),   # compliant rubber face
         box([0.002, PLEG_Y + LEG_W / 2 - PAD_W / 2 + 0.0002, PAD_D], [px0 + 0.0003 - PAD_T_UP / 2 - 0.001, (PAD_W / 2 + PLEG_Y + LEG_W / 2) / 2, pz], printed),   # top plate out to the leg
         box([0.006, FINGER_OFF + 0.008 - (PLEG_Y - LEG_W / 2), 0.012], [-BLOCK_CLEAR - 0.010, (PLEG_Y - LEG_W / 2 + FINGER_OFF + 0.008) / 2, 0.0080], printed),   # beam on the upper carriage
         box([0.006, 0.016, 0.003], [-BLOCK_CLEAR - 0.013, FINGER_OFF, 0.0035], carr)]                                 # upper carriage (gripper part)
top = -BLOCK_CLEAR - 0.007; bot = px0 + 0.0003 - PAD_T_UP / 2
parts.append(box([bot - top, LEG_W, 0.004], [(top + bot) / 2, PLEG_Y, 0.012], printed))                                 # the one leg
save("finger_upper", parts)
save("finger_upper_body", [parts[0]] + parts[2:]); save("finger_upper_pad", [parts[1]])
print("written to", OUT)
