"""Self-overlap audit of the stack and the bench fixtures: every part drawn as a box, cylinder or mesh is tested against every other for
interpenetration at the home pose (parts on the same rigid body included, which MuJoCo's own contacts never report). Oriented boxes via the
separating axis test; cylinders and meshes by their bounding boxes. Pairs that belong to the same part (a CAD mesh over its own mass box,
a jaw over its mesh) are skipped; touching pairs (depth under 0.3 mm) are the designed joints and are listed as such.
usage: .venv/bin/python scripts/ffc_zif_selfcheck.py [arm tool case]   (default: all working variants)"""
import sys, itertools, math
from pathlib import Path
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F
VARIANTS = [("meca500", "megp_nano25", "laptop"), ("lrmate200id", "lehz16_mini40", "laptop"), ("meca500", "megp_nano25", "platform"), ("meca500", "servo_v1", "laptop"), ("lrmate200id", "servo_v1_lr", "laptop")]
SAME = [("gripper_mesh", "gripper_body"), ("gripper_mesh", "gripper_end"), ("gripper_mesh", "gripper_conn"), ("gripper_plate_mesh", "gripper_plate"), ("gripper_mesh", "gripper_plate_mesh"),   # the STAP01 plate's screws and pins enter the body by design ("jaw_lo_mesh", "jaw_lo"), ("jaw_up_mesh", "jaw_up"),
        ("stack_zaber_mesh", "stack_zaber"), ("gripper_mesh", "jaw_lo"), ("gripper_mesh", "jaw_up"), ("gripper_cover", "gripper_body"), ("gripper_mesh", "jaw_lo_mesh"), ("gripper_mesh", "jaw_up_mesh")]
# one physical part drawn as several boxes (a printed finger, a machined L, a bracket): their mutual laps are the part's own geometry
GROUPS = [("plate_up", "pad_leg_e", "pad_arm", "pad_g", "cell_p", "jaw_up", "plate_up_tongue", "plate_up_web"), ("plate_lo", "leg_e", "shoe_g", "shoe_bar", "blade"), ("arm_d405_y", "arm_d405", "arm_d405_drop", "arm_d405_plate"), ("bracket_h", "bracket_v", "cell_b", "cell_corner", "cell_tab"),
          ("fixcam_macro_e_g", "fixcam_macro_e_lens", "fixcam_macro_e_block", "fixcam_macro_e_post"), ("fixcam_macro_w_g", "fixcam_macro_w_lens", "fixcam_macro_w_block", "fixcam_macro_w_post"), ("light_g", "light_post")]
TOUCH = 0.0003


def same_part(a, b):
    return any(a in g and b in g for g in GROUPS)


def mesh_vertices_world(model, data, g):
    mid = model.geom_dataid[g]; v0 = model.mesh_vertadr[mid]; nv = model.mesh_vertnum[mid]; V = model.mesh_vert[v0:v0 + nv]
    return data.geom_xpos[g] + V @ data.geom_xmat[g].reshape(3, 3).T


def vertex_depth(P, box):
    """Deepest point of a vertex cloud inside an OBB (0 if none inside)."""
    c, R, h = box; L = (P - c) @ R; inside = np.all(np.abs(L) < h, axis=1)
    if not inside.any():
        return 0.0
    return float((h - np.abs(L[inside])).min(axis=1).max())


def obb(model, data, g):
    """centre, rotation, half extents of a geom's oriented bounding box (meshes: their vertex bounds in the geom frame)."""
    c = data.geom_xpos[g].copy(); R = data.geom_xmat[g].reshape(3, 3).copy(); t = model.geom_type[g]; sz = model.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        h = sz.copy()
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        h = np.array([sz[0], sz[0], sz[1]])
    elif t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[g]; v0 = model.mesh_vertadr[mid]; nv = model.mesh_vertnum[mid]; V = model.mesh_vert[v0:v0 + nv]
        lo, hi = V.min(0), V.max(0); h = (hi - lo) / 2; c = c + R @ ((hi + lo) / 2)
    else:
        return None
    return c, R, h


def penetration(a, b):
    """Smallest translation that separates two OBBs (0 if already apart), by the 15-axis separating axis test."""
    ca, Ra, ha = a; cb, Rb, hb = b; d = cb - ca
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)] + [np.cross(Ra[:, i], Rb[:, j]) for i in range(3) for j in range(3)]
    best = math.inf
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        ra = sum(ha[i] * abs(ax @ Ra[:, i]) for i in range(3)); rb = sum(hb[i] * abs(ax @ Rb[:, i]) for i in range(3))
        overlap = ra + rb - abs(d @ ax)
        if overlap <= 0:
            return 0.0
        best = min(best, overlap)
    return best


def check(arm, tool, case):
    spec = F.build(arm, case, tool); model = spec.compile(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    tool_b = model.body("tool").id
    def under(b):
        while b > 0:
            if b == tool_b: return True
            b = model.body_parentid[b]
        return False
    names = [model.geom(g).name for g in range(model.ngeom)]
    sel = [g for g in range(model.ngeom) if (under(model.geom_bodyid[g]) or names[g].startswith(("fixcam_", "light_", "cam_post", "arm_plate"))) and model.geom_rgba[g][3] > 0]
    boxes = {g: obb(model, data, g) for g in sel}
    hits, touches = [], []
    for g1, g2 in itertools.combinations(sel, 2):
        n1, n2 = names[g1], names[g2]
        if (n1, n2) in SAME or (n2, n1) in SAME or boxes[g1] is None or boxes[g2] is None:
            continue
        if same_part(n1, n2):
            continue
        m1 = model.geom_type[g1] == mujoco.mjtGeom.mjGEOM_MESH; m2 = model.geom_type[g2] == mujoco.mjtGeom.mjGEOM_MESH
        if m1 or m2:      # meshes: by their vertices inside the other's box (a bounding box would flag the jaw block's own recesses)
            p = 0.0
            if m1: p = max(p, vertex_depth(mesh_vertices_world(model, data, g1), boxes[g2]))
            if m2: p = max(p, vertex_depth(mesh_vertices_world(model, data, g2), boxes[g1]))
        else:
            p = penetration(boxes[g1], boxes[g2])
        if p > TOUCH:
            hits.append((p, n1, n2))
        elif p > 0:
            touches.append((p, n1, n2))
    tag = f"{arm}-{tool}-{case}"
    print(f"\n{tag}: {len(sel)} parts, {len(hits)} overlapping pairs, {len(touches)} touching pairs (designed joints)")
    for p, a, b in sorted(hits, reverse=True):
        print(f"  OVERLAP {p*1e3:5.2f} mm  {a} <-> {b}")
    return hits


if __name__ == "__main__":
    vs = [tuple(sys.argv[1:4])] if len(sys.argv) >= 4 else VARIANTS
    total = sum(len(check(*v)) for v in vs)
    print(f"\ntotal overlapping pairs: {total}")
