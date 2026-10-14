"""Joint registration of the right arm base and the wrist D405 mount from the real wrist frames + seated demos.

Unknowns: right base pose (xyz + roll/pitch/yaw; the left base defines the world) and the D405 mount in link_6
(y, z offset on the gripper's flat-base side at x=0, pitch toward the fingers, roll, yaw), shared by both arms,
plus the field of view (discrete). Observations: picked pixels in the last frame of one demo for both wrist
cameras (own pads, far pads, far carriage plate, far D405) and, for the base, the 16 seated demo end frames
where the two tool axes must be collinear and opposed.
writes out/register_cell.json and out/camgrid/cell_registered.png
"""
import sys, json, glob, numpy as np, mujoco, cv2
from scipy.optimize import least_squares
sys.path.insert(0, "."); import cell_twin.layout as LAY
ROOT = "."; EPS = "inputs/nuc/episodes/objects"; PRE = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"
EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"
PICKS = {"right": {"own_pads": (320, 415), "far_pads": (320, 345), "far_plate": (335, 225), "far_cam": (290, 70)},
         "left":  {"own_pads": (322, 400), "far_pads": (322, 345), "far_plate": (335, 215), "far_cam": (290, 60)}}
WEIGHT = {"own_pads": 1.0, "far_pads": 0.8, "far_plate": 0.5, "far_cam": 0.5}
# single-arm FK
spec = mujoco.MjSpec.from_file("assets/yam/yam_linear_4310.xml"); m = spec.compile(); d = mujoco.MjData(m)
def link6_T(q):
    d.qpos[:] = 0; d.qpos[:6] = q; mujoco.mj_forward(m, d)
    T = np.eye(4); T[:3, :3] = d.xmat[m.body("link_6").id].reshape(3, 3); T[:3, 3] = d.xpos[m.body("link_6").id]; return T
mujoco.mj_forward(m, d); T0i = np.linalg.inv(link6_T(np.zeros(6)))
def in_link6(g): p = d.geom_xpos[m.geom(g).id]; return T0i[:3, :3] @ p + T0i[:3, 3]
FEAT = {"pads": (in_link6("left_pad") + in_link6("right_pad")) / 2, "plate": in_link6("gripper_housing"), "cam": in_link6("d405")}
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx
def base_T(xyz, r, p, y): T = np.eye(4); T[:3, :3] = rpy(r, p, y); T[:3, 3] = xyz; return T
L = base_T(LAY.ARM_BASE["left"][0], 0, 0, np.radians(LAY.ARM_BASE["left"][1]))
def cam_T(y, z, pitch, roll, yaw):
    R0 = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], float)     # cam -z along link +z (fingers), cam +y along link +y
    T = np.eye(4); T[:3, :3] = R0 @ rpy(pitch, yaw, roll); T[:3, 3] = [0.0, y, z]; return T
# data
st = np.load(f"{EPS}/{EP}/frames.npz")["observation.state"][-1]; qL, qR = st[:6], st[7:13]
seated = []
for src in sorted(glob.glob(f"{EPS}/*/source.json")):
    meta = json.load(open(src))
    if meta.get("starting_pose") == PRE and not meta["title"].startswith("Correction"):
        s = np.load(src.replace("source.json", "frames.npz"))["observation.state"][-1]; seated.append((link6_T(s[:6]), link6_T(s[7:13])))
print(len(seated), "seated demos")
def project(Tcam_w, p_w, f):
    v = Tcam_w[:3, :3].T @ (p_w - Tcam_w[:3, 3])
    if v[2] > -0.01: return np.array([1e3, 1e3])
    return np.array([320 + f * v[0] / (-v[2]), 240 - f * v[1] / (-v[2])])
def unpack(x): return x[:3], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10]
def residuals(x, f):
    bxyz, br, bp, by, cy_, cz, cpitch, croll, cyaw = unpack(x)
    Rb = base_T(bxyz, br, bp, by); TL = L @ link6_T(qL); TR = Rb @ link6_T(qR); Tc = cam_T(cy_, cz, cpitch, croll, cyaw)
    res = []
    for arm, Town, Tfar in (("right", TR, TL), ("left", TL, TR)):
        Tcam = Town @ Tc
        for key, Tsrc, feat in (("own_pads", Town, "pads"), ("far_pads", Tfar, "pads"), ("far_plate", Tfar, "plate"), ("far_cam", Tfar, "cam")):
            pw = Tsrc[:3, :3] @ FEAT[feat] + Tsrc[:3, 3]
            res.extend(WEIGHT[key] * (project(Tcam, pw, f) - np.array(PICKS[arm][key], float)) / 10.0)   # 10 px units
    for TLl, TRl in seated:
        TLw, TRw = L @ TLl, Rb @ TRl; zL, zR = TLw[:3, 2], TRw[:3, 2]; v = TRw[:3, 3] - TLw[:3, 3]
        lat = v - (v @ zL) * zL; res.extend(lat * 1000 / 2.0)                        # 2 mm units
        res.append(np.degrees(np.arccos(np.clip(-zL @ zR, -1, 1))) / 1.0)            # 1 deg units
    return np.array(res)
x0 = np.array([*LAY.ARM_BASE["right"][0], 0, 0, np.radians(180.0), 0.075, 0.067, np.radians(-35), 0, 0])
best = None
for fovy in (58.0, 50.0, 42.0):
    f = 240 / np.tan(np.radians(fovy) / 2)
    for pitch0 in (-20, -35, -50):
        x0[8] = np.radians(pitch0)
        r = least_squares(residuals, x0, args=(f,), max_nfev=4000)
        if best is None or r.cost < best[0]: best = (r.cost, r.x, fovy)
    print(f"fovy {fovy}: cost {min(best[0], r.cost):.2f}")
cost, x, fovy = best; f = 240 / np.tan(np.radians(fovy) / 2); bxyz, br, bp, by, cy_, cz, cpitch, croll, cyaw = unpack(x)
res = residuals(x, f); npx = 16
print(f"fovy {fovy}: pixel rms {np.sqrt(np.mean((res[:npx]*10)**2)):.1f} px, seated lateral rms {np.sqrt(np.mean((res[npx:].reshape(-1,4)[:,:3]*2)**2)):.2f} mm, axis {np.mean(np.abs(res[npx:].reshape(-1,4)[:,3])):.2f} deg")
print(f"right base xyz {np.round(bxyz,4)} rpy {np.degrees([br,bp,by]).round(2)} (layout 0.6192,0,0.045 / 0,0,180)")
print(f"camera in link_6: y {cy_*1000:.1f} mm z {cz*1000:.1f} mm pitch {np.degrees(cpitch):.1f} roll {np.degrees(croll):.1f} yaw {np.degrees(cyaw):.1f}")
Rb = base_T(bxyz, br, bp, by); TL = L @ link6_T(qL); TR = Rb @ link6_T(qR); Tc = cam_T(cy_, cz, cpitch, croll, cyaw)
for arm, Town, Tfar in (("right", TR, TL), ("left", TL, TR)):
    for key, Tsrc, feat in (("own_pads", Town, "pads"), ("far_pads", Tfar, "pads"), ("far_plate", Tfar, "plate"), ("far_cam", Tfar, "cam")):
        pw = Tsrc[:3, :3] @ FEAT[feat] + Tsrc[:3, 3]; print(f"  {arm} {key:9s} pick {PICKS[arm][key]} -> {project(Town @ Tc, pw, f).round(0)}")
json.dump({"fovy": fovy, "right_base_xyz": list(map(float, bxyz)), "right_base_rpy_deg": list(map(float, np.degrees([br, bp, by]))),
           "cam_y": float(cy_), "cam_z": float(cz), "cam_pitch_deg": float(np.degrees(cpitch)), "cam_roll_deg": float(np.degrees(croll)),
           "cam_yaw_deg": float(np.degrees(cyaw)), "T_cam_in_link6": Tc.tolist(), "picks": PICKS}, open("out/register_cell.json", "w"), indent=1)
print("wrote out/register_cell.json")
