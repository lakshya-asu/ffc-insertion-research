"""Dump the design numbers of every custom part (plates, brackets, fingers, camera arms) for each tool stack, straight from the twin's
geometry, so the CAD generator (scripts/ffc_custom_parts.py, cadquery) and the drawings use the same numbers the simulation ran with.
usage: .venv/bin/python scripts/ffc_zif_design_numbers.py   -> out/ffc_zif/design.json"""
import json, sys
from pathlib import Path
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import ffc_zif as F

mm = lambda v: round(float(v) * 1e3, 3)
out = {"units": "mm", "stacks": {}}
for arm, tool in (("meca500", "megp_nano25"), ("lrmate200id", "lehz16_mini40"), ("meca500", "megp_nano25_zaber")):
    spec = F.build(arm, "laptop", tool); model = spec.compile()
    g = lambda n: (model.geom_pos[model.geom(n).id].tolist(), model.geom_size[model.geom(n).id].tolist())
    ti = F.TOOL_INFO[tool]; G = F.GRIPPERS[ti["gripper"]]
    stack = []
    for pid, part, shape, dims, mass, col, xoff in F.STACKS[ti["stack"]]:
        stack.append({"id": pid, "part": part, "shape": shape, "dims_mm": [mm(d) for d in dims], "x_offset_mm": mm(xoff)})
    bh_p, bh_s = g("bracket_h"); bv_p, bv_s = g("bracket_v")
    jaws_pos = model.body_pos[model.body("jaws").id].tolist()
    d = {
        "arm": arm, "tool": tool, "gripper": ti["gripper"], "gripper_label": G["label"], "mount": G["mount"],
        "stack_flange_down": stack,
        "bracket": {"thickness_mm": mm(F.BRACKET_T), "width_mm": mm(F.BRACKET_W),
                    "horizontal_leg": {"centre_tool_xyz_mm": [mm(v) for v in bh_p], "half_size_mm": [mm(v) for v in bh_s]},
                    "vertical_leg": {"centre_tool_xyz_mm": [mm(v) for v in bv_p], "half_size_mm": [mm(v) for v in bv_s]},
                    "flange_axis_to_jaw_end_mm": mm(-jaws_pos[0] + 0.002) if G["mount"] == "side" else mm(-jaws_pos[0] + 0.002),
                    "gripper_mount_pattern": ("Meca500 flange pattern via the stock STAP01 plate: 4 x M3 clearance on 15.5 mm PCD + 2 x 3 mm dowels" if ti["gripper"] == "megp25e"
                                              else "SMC LEHZ16 side pattern: 2 x M3 clearance + 2 x 2.5 H9 pins (spacing and position 30 mm behind the jaw end ESTIMATE)"),
                    "stroke_centre_tool_z_mm": mm(jaws_pos[2] - (G.get("jaw_lo_face", F.JAW_LO_FACE) + G["jaw_sep_open"] / 2 + G["jaw_t"]))},
        "jaws_body_pos_tool_mm": [mm(v) for v in jaws_pos],
        "finger_off_mm": mm(F.FINGER_OFF), "jaw_lo_face_mm": mm(G.get("jaw_lo_face", F.JAW_LO_FACE)), "jaw_sep_open_mm": mm(G["jaw_sep_open"]), "jaw_t_mm": mm(G["jaw_t"]), "jaw_depth_mm": mm(G["jaw_depth"]),
        "finger_plate_t_mm": mm(F.FPLATE_T), "leg_w_mm": mm(F.LEG_W), "leg_y_mm": mm(F.CABLE_W / 2 + 0.001 + F.LEG_W / 2),
        "shoe": {"t_mm": mm(ti["shoe_t"]), "w_mm": mm(F.SHOE_W), "d_mm": mm(F.SHOE_D), "lip_l_mm": mm(F.BLADE_L), "lip_t_mm": mm(F.BLADE_T), "lip_w_mm": mm(F.BLADE_W), "wing_to_mm": mm(F.CABLE_W / 2 + 0.001 + F.LEG_W)},
        "pad": {"t_mm": mm(F.PAD_T_UP), "w_mm": mm(F.PAD_W), "d_mm": mm(F.PAD_D), "open_gap_mm": mm(F.PAD_OPEN)},
        "geoms_jaw_frame": {n: {"pos_mm": [mm(v) for v in g(n)[0]], "half_mm": [mm(v) for v in g(n)[1]]} for n in ("plate_lo", "leg_e", "shoe_g", "shoe_bar", "blade", "jaw_lo", "jaw_up", "plate_up", "pad_leg_e", "pad_arm", "pad_g", "d405_g", "macro_e_g", "macro_w_g") if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0},
        "arms_tool_frame": {n: {"pos_mm": [mm(v) for v in g(n)[0]], "half_mm": [mm(v) for v in g(n)[1]]} for n in ("arm_d405_y", "arm_d405", "arm_d405_drop", "arm_d405_plate", "arm_macro_e_post", "arm_macro_e_link", "arm_macro_e", "arm_macro_e_block", "arm_macro_w_post", "arm_macro_w_link", "arm_macro_w", "arm_macro_w_block") if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0},
        "cameras": ({"fixed_world_mm": {nm: [mm(v) for v in model.body_pos[model.body(f"fixcam_{nm}_body").id]] for nm in ("macro_e", "macro_w")}, "az_deg": list(F.CAMERAS[arm]["az_deg"]), "el_deg": F.CAMERAS[arm]["el_deg"], "wd_mm": mm(F.CAMERAS[arm]["wd"])} if F.FIXED_CAMS else {"macro_jaw_frame_mm": {"e": [mm(v) for v in model.body_pos[model.body("macro_e_body").id]], "w": [mm(v) for v in model.body_pos[model.body("macro_w_body").id]]}}) | {"d405_jaw_frame_mm": [mm(v) for v in g("d405_g")[0]]},
    }
    out["stacks"][f"{arm}/{tool}"] = d
(ROOT / "out" / "ffc_zif" / "design.json").write_text(json.dumps(out, indent=1)); print("wrote out/ffc_zif/design.json")
