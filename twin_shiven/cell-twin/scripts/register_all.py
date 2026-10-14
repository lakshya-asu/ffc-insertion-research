"""Joint registration of the right base, the wrist D405 mount (shared) and the scene camera from hand-picked points in
the real frames at TWO poses of one demo (frame 0 = preinsertion2, last = seated), plus the 16 seated demos' tool-axis
collinearity. Picks are 640x480 pixels. Features: pad midpoint, carriage block (gripper_housing) centre, D405 centre.
writes out/register_cell.json, out/register_scene.json, out/register_all_report.txt"""
import sys, json, glob, numpy as np, mujoco
from scipy.optimize import least_squares
sys.path.insert(0, "."); import cell_twin.layout as LAY
EPS = "inputs/nuc/episodes/objects"; PRE = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"; EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"
# picks: (pose, camera) -> {(arm, feature): (u, v)}
PICKS = {   # tips: the two pad tip ends of one gripper, "tipA"/"tipB" ordered in the image (scene: A = upper; wrist: A = left)
 ("f0", "scene"):  {("right", "tipA"): (336, 250), ("right", "tipB"): (345, 282), ("left", "tipA"): (422, 252), ("left", "tipB"): (427, 280), ("right", "plate"): (60, 262), ("left", "plate"): (615, 255)},
 ("last", "scene"):{("right", "tipA"): (393, 252), ("right", "tipB"): (400, 287), ("left", "tipA"): (405, 250), ("left", "tipB"): (415, 277), ("right", "plate"): (75, 258), ("left", "plate"): (615, 255)},
 ("f0", "right"):  {("right", "tipA"): (300, 395), ("right", "tipB"): (365, 395), ("left", "tipA"): (325, 325), ("left", "tipB"): (385, 320), ("left", "plate"): (370, 200), ("left", "cam"): (310, 35)},
 ("last", "right"):{("right", "tipA"): (297, 387), ("right", "tipB"): (362, 387), ("left", "tipA"): (300, 370), ("left", "tipB"): (367, 367), ("left", "plate"): (330, 235), ("left", "cam"): (280, 60)},
 ("f0", "left"):   {("left", "tipA"): (295, 385), ("left", "tipB"): (360, 385), ("right", "tipA"): (325, 362), ("right", "tipB"): (385, 357), ("right", "plate"): (322, 230), ("right", "cam"): (280, 62)},
 ("last", "left"): {("left", "tipA"): (292, 390), ("left", "tipB"): (360, 390), ("right", "tipA"): (300, 372), ("right", "tipB"): (367, 370), ("right", "plate"): (320, 240), ("right", "cam"): (275, 60)},
}
W = {"tipA": 1.0, "tipB": 1.0, "plate": 0.3, "cam": 0.3}
spec = mujoco.MjSpec.from_file("assets/yam/yam_linear_4310.xml"); m = spec.compile(); d = mujoco.MjData(m)
def link6_T(q, arm=None):
    d.qpos[:] = 0; d.qpos[:6] = q
    if arm is not None: d.qpos[5] += LAY.j6_offset(arm)
    mujoco.mj_forward(m, d)
    T = np.eye(4); T[:3, :3] = d.xmat[m.body("link_6").id].reshape(3, 3); T[:3, 3] = d.xpos[m.body("link_6").id]; return T
from cell_twin.bench import GRIP_SLIDE
T0i = np.linalg.inv(link6_T(np.zeros(6)))
d.qpos[6] = d.qpos[7] = GRIP_SLIDE; mujoco.mj_forward(m, d)          # pads open on the housing end faces
def in_link6(g): p = d.geom_xpos[m.geom(g).id]; return T0i[:3, :3] @ p + T0i[:3, 3]
FEAT = {"tip_l": in_link6("left_pad") + np.array([0, 0, 0.0072]), "tip_r": in_link6("right_pad") + np.array([0, 0, 0.0072]), "plate": in_link6("gripper_housing"), "cam": in_link6("d405")}
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
def T_of(xyz, R): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = xyz; return T
L = T_of(LAY.ARM_BASE["left"][0], rpy(0, 0, np.radians(LAY.ARM_BASE["left"][1])))
def cam_T(y, z, pitch, roll, yaw):
    R0 = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], float); return T_of([0.0, y, z], R0 @ rpy(pitch, yaw, roll))
S = np.load(f"{EPS}/{EP}/frames.npz")["observation.state"]; POSES = {"f0": S[0], "last": S[-1]}
seated = []
for src in sorted(glob.glob(f"{EPS}/*/source.json")):
    meta = json.load(open(src))
    if meta.get("starting_pose") == PRE and not meta["title"].startswith("Correction"):
        s = np.load(src.replace("source.json", "frames.npz"))["observation.state"][-1]; seated.append((link6_T(s[:6], 'left'), link6_T(s[7:13], 'right')))
def project(Tcam, p, f):
    v = Tcam[:3, :3].T @ (p - Tcam[:3, 3])
    return np.array([1e3, 1e3]) if v[2] > -0.01 else np.array([320 + f * v[0] / (-v[2]), 240 - f * v[1] / (-v[2])])
def unpack(x):
    return dict(base=T_of(x[0:3], rpy(*x[3:6])), wrist=cam_T(*x[6:11]), scene=T_of(x[11:14], rpy(*x[14:17])))
def residuals(x, fw, fs, detail=False):
    P = unpack(x); res = []; rows = []
    for (pose, cam), picks in PICKS.items():
        st = POSES[pose]; Tarm = {"left": L @ link6_T(st[:6], "left"), "right": P["base"] @ link6_T(st[7:13], "right")}
        Tcam = P["scene"] if cam == "scene" else Tarm[cam] @ P["wrist"]; f = fs if cam == "scene" else fw
        tips = {}
        for arm in ("left", "right"):
            prs = [project(Tcam, Tarm[arm][:3, :3] @ FEAT[k] + Tarm[arm][:3, 3], f) for k in ("tip_l", "tip_r")]
            prs.sort(key=lambda p: p[1] if cam == "scene" else p[0])        # A = upper (scene) or left (wrist)
            tips[(arm, "tipA")], tips[(arm, "tipB")] = prs
        for (arm, feat), uv in picks.items():
            pr = tips[(arm, feat)] if feat in ("tipA", "tipB") else project(Tcam, Tarm[arm][:3, :3] @ FEAT[feat] + Tarm[arm][:3, 3], f)
            res.extend(W[feat] * (pr - np.array(uv, float)) / 10.0); rows.append((pose, cam, arm, feat, uv, pr))
    for TLl, TRl in seated:
        TLw, TRw = L @ TLl, P["base"] @ TRl; zL, zR = TLw[:3, 2], TRw[:3, 2]; v = TRw[:3, 3] - TLw[:3, 3]
        res.extend((v - (v @ zL) * zL) * 1000 / 2.0); res.append(np.degrees(np.arccos(np.clip(-zL @ zR, -1, 1))))
    res.extend(np.array([x[14], x[15]]) / np.radians(20))          # weak prior: scene camera near straight down
    return (np.array(res), rows) if detail else np.array(res)
x0 = np.array([0.6273, -0.0536, 0.082, *np.radians([17.11, 3.88, 178.51]), 0.0735, 0.0628, np.radians(-23.2), np.radians(7.2), np.radians(6.9), 0.0847, 0.0886, 0.4838, *np.radians([1.0, -9.3, -178.4])])   # previous two-pose solution
best = None
import os
FOVW = tuple(float(v) for v in os.environ.get('FOVW', '58,50').split(','))
FOVS = tuple(float(v) for v in os.environ.get('FOVS', '58,42').split(','))
for fovw in FOVW:
    for fovs in FOVS:
        fw, fs = 240 / np.tan(np.radians(fovw) / 2), 240 / np.tan(np.radians(fovs) / 2)
        for yaw0 in (x0[16],):
            for broll in (x0[3],):
                x = x0.copy()
                r = least_squares(residuals, x, args=(fw, fs), max_nfev=6000)
                if best is None or r.cost < best[0]: best = (r.cost, r.x, fovw, fovs)
    print(f"wrist fovy {fovw}: best cost so far {best[0]:.2f} (fovs {best[3]})")
cost, x, fovw, fovs = best; fw, fs = 240 / np.tan(np.radians(fovw) / 2), 240 / np.tan(np.radians(fovs) / 2)
res, rows = residuals(x, fw, fs, True); npx = 2 * sum(len(p) for p in PICKS.values())
lines = [f"fovy wrist {fovw} scene {fovs}; pixel rms {np.sqrt(np.mean((res[:npx]*10)**2)):.1f} px (weighted)"]
seat = res[npx:npx + 4 * len(seated)].reshape(-1, 4); lines.append(f"seated: lateral rms {np.sqrt(np.mean((seat[:, :3]*2)**2)):.2f} mm, axis mean {np.mean(np.abs(seat[:, 3])):.2f} deg")
lines.append(f"right base xyz {np.round(x[0:3],4)} rpy {np.degrees(x[3:6]).round(2)}")
lines.append(f"wrist mount y {x[6]*1000:.1f} z {x[7]*1000:.1f} mm pitch {np.degrees(x[8]):.1f} roll {np.degrees(x[9]):.1f} yaw {np.degrees(x[10]):.1f}")
lines.append(f"scene cam xyz {np.round(x[11:14],4)} rpy {np.degrees(x[14:17]).round(1)}")
for pose, cam, arm, feat, uv, pr in rows: lines.append(f"  {pose:4s} {cam:5s} {arm:5s} {feat:5s} pick {uv} -> ({pr[0]:.0f},{pr[1]:.0f})  err {np.hypot(pr[0]-uv[0], pr[1]-uv[1]):.0f} px")
print("\n".join(lines)); open("out/register_all_report.txt", "w").write("\n".join(lines))
P = unpack(x); q = np.empty(4); mujoco.mju_mat2Quat(q, P["scene"][:3, :3].reshape(-1))
json.dump({"fovy": fovw, "right_base_xyz": x[0:3].tolist(), "right_base_rpy_deg": np.degrees(x[3:6]).tolist(), "cam_y": x[6], "cam_z": x[7],
           "cam_pitch_deg": np.degrees(x[8]), "cam_roll_deg": np.degrees(x[9]), "cam_yaw_deg": np.degrees(x[10]), "T_cam_in_link6": P["wrist"].tolist(),
           "source": "register_all.py, two poses of " + EP}, open("out/register_cell.json", "w"), indent=1)
json.dump({"fovy": fovs, "pos": x[11:14].tolist(), "quat": q.tolist(), "rpy_deg": np.degrees(x[14:17]).tolist(), "source": "register_all.py"}, open("out/register_scene.json", "w"), indent=1)
print("wrote out/register_cell.json, out/register_scene.json")
