"""Shared look for the FFC cell's 3D pages: material presets (metal with reflections, matte printed parts with layer lines, steel, PEEK, silicone,
polyimide, FANUC yellow and Meca500 grey arms), an environment map and shadows, and the fasteners at every joint (socket head screws, flat heads,
dowels, nuts, heat-set inserts) placed from the stack's own bolt patterns so every part visibly attaches to its neighbour.
Used by scripts/ffc_zif_export3d.py (the run viewers), scripts/ffc_zif_blueprint.py and scripts/ffc_servo_tool_3d.py (the mechanism schematic).
Frames: the twin's tool frame (x along the body axis, y east, z down from the flange face) and jaw frame (x down, y east, z forward); the schematic's
assembly frame (X forward, Y east, Z up). All positions here are metres unless a function says mm."""
import math
import numpy as np

# ---------------------------------------------------------------- materials
MATERIALS = {   # key: three.js MeshPhysicalMaterial fields
    "alu":       dict(color="#c9ccd1", metalness=0.85, roughness=0.38),
    "alu_dark":  dict(color="#5a5d63", metalness=0.8, roughness=0.45),      # anodised housings (MEGP, LEHZ16 body)
    "steel":     dict(color="#9aa0a8", metalness=1.0, roughness=0.28),
    "stainless": dict(color="#b5b9bf", metalness=1.0, roughness=0.22),
    "ati":       dict(color="#b8bcc2", metalness=0.9, roughness=0.4),       # ATI sensor housing
    "cell":      dict(color="#c0c4ca", metalness=0.8, roughness=0.42),      # TAL220B bar, bare aluminium
    "printed":   dict(color="#6e6e74", metalness=0.05, roughness=0.92, bump=True),   # PAHT-CF prints black; drawn graphite grey so the printed parts read against the black servo and camera
    "printed_lt":dict(color="#a0a0a4", metalness=0.05, roughness=0.9, bump=True),    # a lighter printed part for contrast (standoff)
    "peek":      dict(color="#d9c48a", metalness=0.0, roughness=0.55),
    "silicone":  dict(color="#b03060", metalness=0.0, roughness=0.95),
    "polyimide": dict(color="#c58a1c", metalness=0.0, roughness=0.5, opacity=0.85),
    "plastic_blk": dict(color="#1c1c1e", metalness=0.1, roughness=0.5),     # servo, D405, cameras
    "lens":      dict(color="#1a2a3a", metalness=0.0, roughness=0.08, clearcoat=1.0),
    "connector": dict(color="#222225", metalness=0.1, roughness=0.6),
    "pcb":       dict(color="#1f2a1f", metalness=0.2, roughness=0.6),
    "laptop":    dict(color="#2a2a2d", metalness=0.6, roughness=0.5),
    "bench":     dict(color="#3b3b40", metalness=0.7, roughness=0.6),
    "pallet":    dict(color="#e09a3c", metalness=0.6, roughness=0.5),
    "arm_meca":  dict(color="#4a4a4f", metalness=0.3, roughness=0.6),
    "arm_fanuc": dict(color="#f2c230", metalness=0.2, roughness=0.55),
    "light":     dict(color="#2a6fd6", metalness=0.2, roughness=0.4, emissive="#1e5bd0", emissiveIntensity=0.6),
    "brass":     dict(color="#b5a642", metalness=0.9, roughness=0.4),
    "rubber":    dict(color="#202020", metalness=0.0, roughness=1.0),
    "glass":     dict(color="#dfe8f0", metalness=0.0, roughness=0.05, opacity=0.35),
}


def material_key(g, layer, arm, G=None, cells=False):
    """material preset for a twin geom by name and layer"""
    if g.startswith(("stack_nano", "stack_mini")): return "ati"
    if g in ("bracket_h", "cell_b", "cell_p") and cells: return "cell"
    if g.startswith(("stack_adapter", "bracket_", "cell_corner", "cell_tab", "arm_d405", "arm_plate")): return "alu"
    if g == "stack_flange": return "stainless"
    if g == "stack_standoff": return "printed_lt"
    if g == "stack_zaber": return "alu_dark"
    if g.startswith("gripper_") or g.startswith("jaw_"):
        return "printed" if cells else "alu_dark"
    if g in ("plate_lo", "leg_e", "plate_up", "pad_leg_e", "pad_arm", "plate_up_tongue", "plate_up_web"): return "printed"
    if g in ("shoe_g", "shoe_bar", "blade"): return "steel"
    if g == "pad_g": return "silicone"
    if g.startswith("ffc"): return "polyimide"
    if g.startswith("zif"): return "connector"
    if g == "d405_g" or g.endswith("_g") and g.startswith("fixcam_"): return "plastic_blk"
    if g.endswith("_lens"): return "lens"
    if g.startswith("light_g"): return "light"
    if g.startswith(("fixcam_", "light_post", "cam_post")): return "alu"
    if g in ("mainboard", "dimm_slot", "dimm_module"): return "pcb"
    if layer == "laptop": return "laptop"
    if layer == "pallet": return "pallet"
    if layer == "bench": return "bench"
    if layer == "arm": return "arm_fanuc" if arm == "lrmate200id" else "arm_meca"
    return "alu"


# ---------------------------------------------------------------- fasteners
def _f(kind, d, pos, axis, body, title, length=None, head=None):
    axis = np.asarray(axis, float); axis = axis / np.linalg.norm(axis)
    return dict(kind=kind, d=float(d), pos=[float(x) for x in pos], axis=[float(x) for x in axis], body=body, title=title,
                len=float(length if length is not None else {"shcs": 4 * d, "csk": 4 * d, "dowel": 3 * d, "nut": 0.8 * d, "insert": 1.6 * d}[kind]) * 1e-3,
                head=float(head if head is not None else {"shcs": d, "csk": 0.3 * d, "dowel": 0, "nut": 0.8 * d, "insert": 0}[kind]) * 1e-3)


def pcd(n, dia, a0_deg):
    return [(dia / 2 * math.cos(math.radians(a0_deg + 360 * k / n)), dia / 2 * math.sin(math.radians(a0_deg + 360 * k / n))) for k in range(n)]


def fasteners_tool_frame(F, arm, tool, model):
    """fasteners for a stack, each in the frame of the body it rides on ('tool', 'jaws', 'pad', 'shoe'); positions in metres, d in mm.
    Patterns: the Meca500 flange (4 x M3 on 15.5 mm at 45 deg, 2 x 3 mm dowels at 0/180), ISO 9409-1-31.5 (4 x M5 at 45 deg, one 5 mm dowel),
    the Nano25 (3 x M3 on 20.32 at 90 deg, 2 dowels at 30 deg), the Mini40 (3 x M3 on 33 at 90 deg, 2 dowels at 30 deg) as drawn in
    scripts/ffc_custom_parts.py; the MEGP finger pattern (M4 + 2 x 1.5 mm pins); the servo tool's joints as drawn in scripts/ffc_servo_tool_cad.py."""
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]; cells = bool(G.get("cells")); out = []
    gz = {}
    for i in range(model.ngeom):
        n = model.geom(i).name
        if n.startswith("stack_"):
            half = model.geom_size[i][1] if model.geom_type[i] == 5 else model.geom_size[i][2]     # cylinder half-height is size[1]
            gz[n[6:]] = (float(model.geom_pos[i][2] - half), float(model.geom_pos[i][2] + half), [float(x) for x in model.geom_pos[i]])
    DN = np.array([0, 0, 1.0]); UP = -DN
    # ---- flange -> adapter plate
    if arm == "meca500":
        zb = gz["adapter_plate"][1]
        for x, y in pcd(4, 15.5e-3, 45):
            out.append(_f("csk", 3, (x, y, zb), UP, "tool", "M3 flat head, up into the Meca500 flange (4 mm max engagement, 1.5 N m)", length=8))
        for x, y in pcd(2, 15.5e-3, 0):
            out.append(_f("dowel", 3, (x, y, 0.0), DN, "tool", "3 mm dowel, flange to adapter plate", length=6))
    else:
        zb = gz["adapter_plate"][1]
        for x, y in pcd(4, 31.5e-3, 45):
            out.append(_f("csk", 5, (x, y, zb), UP, "tool", "M5 flat head, up into the LR Mate ISO 31.5 flange (8 mm max engagement)", length=12))
        out.append(_f("dowel", 5, (0, 31.5e-3 / 2, 0.0), DN, "tool", "5 mm dowel, ISO 9409-1 position", length=8))
    if cells:
        # ---- adapter -> standoff: 3 x M3 flat head from above, heads flush in the adapter's top face (under the flange)
        z0 = gz["adapter_plate"][0]
        for x, y in ((11e-3, 0), (-11e-3, 0), (0, 11e-3)):
            out.append(_f("csk", 3, (x, y, z0), DN, "tool", "M3 flat head, down through the adapter into a heat-set insert in the standoff", length=10))
            out.append(_f("insert", 3, (x, y, gz["standoff"][0] + 0.5e-3), DN, "tool", "M3 heat-set insert in the standoff's top face"))
        # ---- standoff -> cell A fixed end (and the D405 bar lapped under it): 2 x M4 from below
        Ls = gz["standoff"][1]
        bh = model.geom("bracket_h").id; zcA = float(model.geom_pos[bh][2]); hA = float(model.geom_size[bh][2]); za_bot = zcA + hA
        bar = model.geom("arm_d405_y"); zbar_bot = float(model.geom_pos[bar.id][2] + model.geom_size[bar.id][2])
        for y in (-8e-3, 2e-3):
            out.append(_f("shcs", 4, (0, y, zbar_bot), UP, "tool", "M4 socket head, up through the camera bar and cell A's fixed end into a heat-set insert in the standoff", length=28))
            out.append(_f("insert", 4, (0, y, Ls - 0.5e-3), UP, "tool", "M4 heat-set insert in the standoff's underside"))
        # ---- cell A free end -> corner plate (on top): 2 x M5 from above into the cell's threaded free end
        cc = model.geom("cell_corner"); zc_top = float(model.geom_pos[cc.id][2] - model.geom_size[cc.id][2]); w_leg = float(model.geom_pos[bh][1] + model.geom_size[bh][1]) - 0.014
        for y in (w_leg - 11e-3, w_leg - 4e-3):
            out.append(_f("shcs", 5, (0, y, zc_top), DN, "tool", "M5 socket head, down through the corner plate into cell A's free end (M5 threaded)", length=12))
        # ---- corner plate -> cell B fixed end: 2 x M4 from above
        cb = model.geom("cell_b"); yb = float(model.geom_pos[cb.id][1]); xb1 = float(model.geom_pos[cb.id][0] + model.geom_size[cb.id][0])
        for x in (xb1 - 4e-3, xb1 - 12e-3):
            out.append(_f("shcs", 4, (x, yb, zc_top), DN, "tool", "M4 socket head, down through the corner plate into cell B's fixed end (M4 threaded)", length=12))
        # ---- cell B free end -> tab: 2 x M5 from above
        ct = model.geom("cell_tab"); zt_top = float(model.geom_pos[ct.id][2] - model.geom_size[ct.id][2]); xt = float(model.geom_pos[ct.id][0])
        for x in (xt - 4e-3, xt + 4e-3):
            out.append(_f("shcs", 5, (x, yb, zt_top), DN, "tool", "M5 socket head, down through the tab into cell B's free end (M5 threaded)", length=12))
        # ---- side plate -> body: 4 x M3 from the east into heat-set inserts (behind the carriers' slot)
        bv = model.geom("bracket_v"); y_out = float(model.geom_pos[bv.id][1] + model.geom_size[bv.id][1]); xv = float(model.geom_pos[bv.id][0])
        zc_plate = float(model.geom_pos[bv.id][2]); hv = float(model.geom_size[bv.id][2])
        for x in (xv - 5.15e-3, xv + 4.85e-3):
            for z in (zc_plate - hv + 6e-3, zc_plate + hv - 4e-3):
                out.append(_f("shcs", 3, (x, y_out, z), (0, -1, 0), "tool", "M3 socket head, through the side plate into a heat-set insert in the body's east face", length=10))
                out.append(_f("insert", 3, (x, y_out - 4.5e-3, z), (0, -1, 0), "tool", "M3 heat-set insert in the printed body"))
        # ---- jaw frame items (x down, y east, z forward): carriers to fingers, cell P, cover, pulley
        C = F.JAW_LO_FACE + G["jaw_sep_open"] / 2 + G["jaw_t"]; ZJ = 0.002; FO = F.FINGER_OFF; bw = G["body_w"]; bl = G["body_l"] + G["end_l"]
        x_up = -(C + G["jaw_sep_open"] / 2 + G["jaw_t"]); x_lo = -F.JAW_LO_FACE
        # upper carrier -> cell P fixed end: 2 x M4 through bolts, nuts under the carrier (heads on top of the cell)
        for y in (FO - 5e-3, FO + 5e-3):
            out.append(_f("shcs", 4, (x_up - 0.0127, y, ZJ - 0.001), (1, 0, 0), "pad", "M4 through bolt, down through cell P's fixed end and the upper carrier", length=20))
            out.append(_f("nut", 4, (x_up + G["jaw_t"] + 0.0004, y, ZJ - 0.001), (1, 0, 0), "pad", "M4 nut under the upper carrier (in the open gap)"))
        # cell P free end -> upper plate tongue: 2 x M5 from above
        for y in (-8e-3, -18e-3):
            out.append(_f("shcs", 5, (x_up - 0.0127 - 0.004, y, ZJ + 0.002), (1, 0, 0), "pad", "M5 socket head, down through the printed tongue into cell P's free end (M5 threaded)", length=12))
        # lower carrier -> lower finger plate: M4 through bolt from below (head under the plate), nut on top of the carrier; 2 x 1.5 mm pins
        out.append(_f("shcs", 4, (x_lo + 0.006, FO, ZJ - 0.001), (-1, 0, 0), "shoe", "M4 through bolt, up through the lower finger plate and the lower carrier", length=14))
        out.append(_f("nut", 4, (x_lo - G["jaw_t"] - 0.0004, FO, ZJ - 0.001), (-1, 0, 0), "shoe", "M4 nut on the lower carrier (in the open gap)"))
        for y in (FO - 3e-3, FO + 3e-3):
            out.append(_f("dowel", 1.5, (x_lo + 0.003, y, ZJ - 0.001), (-1, 0, 0), "shoe", "1.5 mm pin, lower finger plate to carrier (MEGP pattern)", length=6))
        # belt cover -> body: 4 x M2
        for z in (ZJ - 0.006 - 0.004, ZJ - bl + 0.004):
            for y in (FO - bw * 0 - G["body_d"] / 2 + 0.003, FO + G["body_d"] / 2 - 0.003):
                out.append(_f("shcs", 2, (-C - bw / 2 - 0.006, y, z), (1, 0, 0), "jaws", "M2 socket head, cover to body", length=6))
        # rocker pivot pin
        out.append(_f("dowel", 2, (-C, FO - 0.011, ZJ - 0.004), (0, 1, 0), "jaws", "2 mm rocker pivot pin through the end block", length=22))
        return out
    # ---- bought stacks
    sensor = next((k for k in gz if k in ("nano17", "nano25", "mini40")), None)
    if sensor:
        z0s, z1s, _ = gz[sensor]; zb_bot = gz["adapter_plate"][0]
        n_bc = 20.32e-3 if sensor != "mini40" else 33e-3
        for x, y in pcd(3, n_bc, 90):
            out.append(_f("shcs", 3, (x, y, zb_bot), DN, "tool", f"M3 socket head, counterbored in the adapter plate, down into the {sensor} mounting side", length=8))
        for x, y in pcd(2, n_bc, 30):
            out.append(_f("dowel", 3, (x, y, z0s), DN, "tool", "3 mm dowel, adapter plate to sensor", length=6))
        bh = model.geom("bracket_h"); zbh_bot = float(model.geom_pos[bh.id][2] + model.geom_size[bh.id][2])
        for x, y in pcd(3, n_bc, 90):
            out.append(_f("shcs", 3, (x, y, zbh_bot), UP, "tool", f"M3 socket head, counterbored in the bracket from below, up into the {sensor} tool side", length=8))
        for x, y in pcd(2, n_bc, 30):
            out.append(_f("dowel", 3, (x, y, z1s), UP, "tool", "3 mm dowel, sensor tool side to bracket", length=6))
    # bracket vertical leg -> gripper (its side plate or back face): 4 screws on the gripper's own pattern, heads on the leg's outer face
    bv = model.geom("bracket_v")
    if G["mount"] == "side":
        y_out = float(model.geom_pos[bv.id][1] + model.geom_size[bv.id][1]); zc = float(model.geom_pos[bv.id][2]) + G["plate_wh"] / 2 - 0.002
        for x, z in pcd(4, 15.5e-3, 45):
            out.append(_f("shcs", 3, (x, y_out, zc + z), (0, -1, 0), "tool", "M3 socket head, through the bracket's vertical leg into the gripper's side plate (Meca500 pattern)", length=10))
        for x, z in pcd(2, 15.5e-3, 0):
            out.append(_f("dowel", 3, (x, y_out - 0.006, zc + z), (0, -1, 0), "tool", "3 mm dowel, bracket to the gripper's side plate", length=6))
    else:
        x_out = float(model.geom_pos[bv.id][0] + model.geom_size[bv.id][0]); zc = float(model.geom_pos[bv.id][2]) + float(model.geom_size[bv.id][2]) - G["body_w"] / 2
        for y, z in ((-7e-3, -7e-3), (7e-3, -7e-3), (-7e-3, 7e-3), (7e-3, 7e-3)):
            out.append(_f("shcs", 4, (x_out, F.FINGER_OFF + y, zc + z), (-1, 0, 0), "tool", "M4 socket head, through the bracket's vertical leg into the gripper's back end face (ESTIMATE positions)", length=12))
    # jaws -> fingers (MEGP pattern on both bought grippers' jaws): M4 + 2 x 1.5 mm pins, on the jaw's outer face
    C = F.JAW_LO_FACE + G["jaw_sep_open"] / 2 + G["jaw_t"]; ZJ = 0.002; FO = F.FINGER_OFF
    x_up = -(C + G["jaw_sep_open"] / 2 + G["jaw_t"]); x_lo = -F.JAW_LO_FACE; FPL = 0.006
    out.append(_f("shcs", 4, (x_up - FPL, FO, ZJ - 0.003), (1, 0, 0), "pad", "M4 socket head, down through the upper finger plate into the upper jaw (MEGP pattern)", length=10))
    out.append(_f("shcs", 4, (x_lo + FPL, FO, ZJ - 0.003), (-1, 0, 0), "shoe", "M4 socket head, up through the lower finger plate into the lower jaw (MEGP pattern)", length=10))
    for y in (FO - 3e-3, FO + 3e-3):
        out.append(_f("dowel", 1.5, (x_up - 0.001, y, ZJ - 0.003), (1, 0, 0), "pad", "1.5 mm pin, upper finger to jaw", length=5))
        out.append(_f("dowel", 1.5, (x_lo + 0.001, y, ZJ - 0.003), (-1, 0, 0), "shoe", "1.5 mm pin, lower finger to jaw", length=5))
    return out


def to_body_frames(F, items, model, data=None):
    """convert 'pad' and 'shoe' items (given in the jaw frame) into their own body frames so a viewer can attach them to those bodies"""
    G = None
    px0 = -(F.PAD_OPEN + F.PAD_T_UP / 2); pz = F.PAD_D / 2 + 0.010
    for it in items:
        if it["body"] == "pad":
            it["pos"] = [it["pos"][0] - px0, it["pos"][1], it["pos"][2] - pz]
    return items


# ---------------------------------------------------------------- JS snippet shared by the pages
MATERIAL_JS = r"""
const MATS = __MATS__;
let _bump = null;
function layerBump(){ if(_bump) return _bump; const c = document.createElement('canvas'); c.width = 8; c.height = 64; const x = c.getContext('2d'); x.fillStyle = '#808080'; x.fillRect(0,0,8,64);
 for(let i=0;i<64;i+=4){ x.fillStyle = '#a0a0a0'; x.fillRect(0,i,8,1); x.fillStyle = '#606060'; x.fillRect(0,i+2,8,1); } const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(1, 40); _bump = t; return t; }
function makeMaterial(key, extra){ const m = MATS[key] || MATS.alu; const o = Object.assign({}, m, extra || {});
 const mat = new THREE.MeshPhysicalMaterial({color: new THREE.Color(o.color), metalness: o.metalness, roughness: o.roughness, clearcoat: o.clearcoat || 0, transparent: (o.opacity !== undefined && o.opacity < 1), opacity: o.opacity === undefined ? 1 : o.opacity, side: THREE.DoubleSide, envMapIntensity: 1.0});
 if(o.emissive){ mat.emissive = new THREE.Color(o.emissive); mat.emissiveIntensity = o.emissiveIntensity || 0.5; }
 if(o.bump){ mat.bumpMap = layerBump(); mat.bumpScale = o.bumpScale || 0.0008; }
 return mat; }
async function setupLook(renderer, scene, extent, center){ const {RoomEnvironment} = await import('three/addons/environments/RoomEnvironment.js');
 const pm = new THREE.PMREMGenerator(renderer); scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture; renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.05; renderer.outputColorSpace = THREE.SRGBColorSpace;
 renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
 const sun = new THREE.DirectionalLight(0xfff4e0, 1.6); sun.position.set(center[0] + extent*0.6, center[1] - extent*0.9, center[2] + extent*1.4); sun.target.position.set(center[0], center[1], center[2]); scene.add(sun); scene.add(sun.target);
 sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048); const sc = sun.shadow.camera; sc.left = -extent; sc.right = extent; sc.top = extent; sc.bottom = -extent; sc.near = extent*0.1; sc.far = extent*5; sun.shadow.bias = -extent*1e-5; sun.shadow.normalBias = extent*2e-4;
 scene.add(new THREE.HemisphereLight(0xffffff, 0x8a7f70, 0.35)); return sun; }
function fastenerMesh(f, scale){ // f: {kind, d (mm), len (m), head (m), pos (m), axis}; scale = page units per metre
 const g = new THREE.Group(); const d = f.d*1e-3*scale, L = f.len*scale, H = f.head*scale; const ax = new THREE.Vector3(...f.axis).normalize();
 const steel = makeMaterial(f.kind === 'insert' ? 'brass' : f.kind === 'dowel' ? 'stainless' : 'steel');
 const add = (geo, off) => { const m = new THREE.Mesh(geo, steel); m.position.copy(ax.clone().multiplyScalar(off)); m.castShadow = true; g.add(m); };
 if(f.kind === 'shcs'){ add(new THREE.CylinderGeometry(0.85*d, 0.85*d, H, 24), -H/2); add(new THREE.CylinderGeometry(0.5*d, 0.5*d, L, 16), L/2);
  const sock = new THREE.Mesh(new THREE.CylinderGeometry(0.3*d, 0.3*d, H*0.5, 6), makeMaterial('rubber')); sock.position.copy(ax.clone().multiplyScalar(-H + H*0.25)); g.add(sock); }
 else if(f.kind === 'csk'){ add(new THREE.CylinderGeometry(0.5*d, d, 0.6*d, 24), 0.3*d); add(new THREE.CylinderGeometry(0.5*d, 0.5*d, L, 16), L/2); }
 else if(f.kind === 'dowel'){ add(new THREE.CylinderGeometry(0.5*d, 0.5*d, L, 16), L/2); }
 else if(f.kind === 'nut'){ add(new THREE.CylinderGeometry(0.95*d, 0.95*d, H, 6), H/2); }
 else if(f.kind === 'insert'){ add(new THREE.CylinderGeometry(0.75*d, 0.75*d, L, 16), L/2); }
 for(const m of g.children){ m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), ax); m.position.add(ax.clone().multiplyScalar(0)); }
 // CylinderGeometry is along local y: rotate each child so local y = axis, keeping the offsets along the axis
 g.position.set(f.pos[0]*scale, f.pos[1]*scale, f.pos[2]*scale); g.userData = {info: {title: f.title, what: 'fastener: ' + f.kind + ' M' + f.d, detail: ''}, fastener: true}; return g; }
"""


def material_js():
    import json
    return MATERIAL_JS.replace("__MATS__", json.dumps(MATERIALS))
