"""Write ASSUMPTIONS.md: every number and structural choice in the twin, with its source class.

Classes: D datasheet or manufacturer document · L published measurement · I arithmetic from D or L · P placeholder
awaiting the bench · M modelling choice (a simplification we chose, with its known consequence).
"""
import sys
from dataclasses import fields
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.spec import BOARDS, DEFAULT, LEVELS  # noqa: E402
from ffc_twin.sensor_models import SensorSuite  # noqa: E402
from ffc_twin import detector, expert, env  # noqa: E402

STRUCTURAL = [
    ("M", "cable", "The stiffened end is one rigid plate; the exposed-contact region is backed by the support tape and does not bend separately.", "Fine for insertion loads; wrong if the contact region delaminates or curls."),
    ("M", "cable", "The flexible tail is 12 hinged links of 6 mm with bend and twist springs; in-plane bending is locked (13 000 times stiffer than out-of-plane).", "Cannot show local buckling of the film or crease damage."),
    ("M", "cable", "Only 72 mm of tail is modelled; the remaining 120 mm hangs free and is dropped.", "Tail weight and drape on the tool are underestimated by that fraction."),
    ("M", "grip", "The grip is a dry-friction slide joint: locked below 2 mu N, free above, with 1 g of numerical armature so the friction constraint stays well conditioned against oblique contacts (verified: holds to 2.4 N, then slides at 2.4 N). Pads do not deform; no roll or yaw slip in the jaws.", "Loads above the limit produce real slips; pad compliance and twist slip are absent. A cable that slides more than the jaw length counts as dropped."),
    ("M", "connector", "Housing walls, funnels and backstop are rigid boxes; the latch is not modelled; no contact springs act on the cable with the latch open.", "Right for a true zero-insertion-force socket with the actuator lifted; wrong if the real part preloads the contacts."),
    ("M", "connector", "Side funnels default to none (straight walls); top and bottom funnels are placeholders.", "Lateral capture is the drawing's 0.2 mm until the housing is measured."),
    ("M", "contact", "MuJoCo soft contacts with solref 0.0005, solimp 0.95/0.99 and a 20 um margin; box primitives, no mesh; 1 ms step.", "Penetration was checked against the clearance at 1 ms; no measured contact stiffness."),
    ("M", "tool", "The arm is a six-axis position-controlled tool: a first-order lag on the commanded pose plus an observation-to-action delay. No arm dynamics, joint limits, or Cartesian-impedance dead zone.", "Sub-millimetre steady-state error from joint friction is not reproduced; absorbed into the proprioception bias."),
    ("M", "tool", "The jaws are visual only (no collision) and 1.5 mm long on the rear of the tape.", "Whether 1.5 mm jaws clear the lifted actuator is an open geometric question on the TE drawing."),
    ("M", "physics", "Gravity points into the slot (Pi 4 upright socket). The Pi Zero side-entry profile sets gravity along -z.", ""),
    ("M", "connector", "Zero board only: 22 spring-supported contact noses on the slot floor, copied from Lakshya's 045 model (100 N/m, 0.1 mm bounded travel, tapered front). Numerically settled with 0.1 g of joint armature, critical damping and an upper stop 0.02 mm above rest.", "Contact preload and friction on the cable are hypotheses; the settling terms are numerical, not material."),
    ("M", "tool", "The jaws carry no weight (0.5 g): a real arm compensates its own tool gravity, and a 50 g tool on the 2000 N/m servo stand-in sagged 0.25 mm once gravity pointed along the cable normal.", ""),
    ("M", "tool", "Arm class switch: dof=4 (SCARA) zeroes the roll and pitch actions; start errors in those axes persist for the whole episode.", "A SCARA gets no credit for correcting tilt; the fixture and lead-in must absorb it."),
    ("M", "tool", "Body grip option (Lakshya's): 6 mm pads centred 12 mm behind the leading edge, so 3 mm of thin film is free between the pads and the stiffener; modelled as two hinges with the film's bending and twist stiffness.", "Free film pitches under sub-newton loads."),
    ("M", "camera", "Estimator preset 'lakshya': per-episode bias std of 0.03 mm in depth, 0.03 mm sideways and 0.088 mm in height, his 046 stereo result (95 um, 88 of it in height).", "Copies the size of his error per axis, not its cause (edge thickness); the sign of his height bias is not fixed here."),
    ("M", "reset", "Start distributions L0 and L1 are uniform boxes chosen by us; the tip starts 1.5 mm before the mouth, at rest.", "Real starts come from the grasp and approach skills and will not be uniform."),
    ("M", "randomization", "Friction 0.2 to 0.6, stiffness x0.5 to x2, tool gain x0.7 to x1.3, drawn per episode.", "Bands chosen to bracket the placeholders, not measured spreads."),
    ("M", "seated rule", f"Travel since the estimated mouth crossing >= seat - {detector.TRAVEL_TOL*1e3:.1f} mm, force > {detector.STALL_FORCE} N, estimated depth > seat - {detector.EST_DEPTH_TOL*1e3:.1f} mm and pose in band.", "Thresholds hand-tuned on the twin; the stall signature should come from bench force-depth curves."),
    ("M", "expert", "Line up at 40 percent gain per tick on a smoothed estimate; contact at 0.3 N; stall at 1 N with no progress for 2 ticks; back off 0.4 mm and shift 0.12 mm away from the wall; 8 retries; 1.5 N ceiling.", "Hand-tuned; the learned rungs are meant to replace these numbers."),
    ("M", "success", "Seated = all four tip corners inside the slot band, deeper than seat - 0.3 mm and no deeper than seat + 0.05 mm (Lakshya's rule), peak force < 3 N.", "No electrical, latch or retention criterion in the twin."),
    ("M", "actions", f"At most {env.ACTION_POS*1e3:.1f} mm and {env.ACTION_ROT*57.3:.1f} deg per 50 ms tick; {env.MAX_TICKS} ticks; abort above {env.HARD_ABORT_FORCE} N.", ""),
    ("M", "camera", "The vision model is not run; its output is generated directly from truth with the researched error model. Rendering and a real estimator are the Isaac stage.", "Appearance effects (glare, texture, lighting) enter only through the noise and outlier rates."),
    ("M", "force sensor", "The board sensor reading is the contact wrench at the tip (the reaction the board carries), through the fixture and electronics model. Fixture resonance is clamped to 200 Hz because the twin samples at 1 kHz.", "Ringing above 200 Hz is not represented; it is below the 35 Hz low-pass anyway."),
]


def main():
    out = Path(__file__).resolve().parents[1] / "ASSUMPTIONS.md"
    lines = ["# Assumptions register", "", "Generated by `scripts/assumptions.py`. Classes: D datasheet · L published measurement · I arithmetic from D or L · P placeholder awaiting the bench · M modelling choice.", ""]
    for board, spec in BOARDS.items():
        title = "Pi 4 top-entry, TE 1-1734248-5 reference" if board == "pi4" else "Pi Zero 2 W side-entry, Molex 54548 reference matched to Lakshya's experiment 045"
        lines += [f"## Task numbers, {title} (spec.py)", "", "| Class | Item | Value | Source |", "|---|---|---|---|"]
        for group_name in ("cable", "connector", "tool", "physics", "success"):
            group = getattr(spec, group_name)
            for f in fields(group):
                q = getattr(group, f.name)
                if hasattr(q, "source") and not q.source.startswith("not modelled"):
                    cls = "P" if q.placeholder else ("D" if q.source.startswith(("TE", "Raspberry", "Molex", "D:")) else ("I" if q.source.startswith(("computed", "I:", "derived")) else "M"))
                    lines.append(f"| {cls} | {group_name}.{f.name} | {q.value:g} {q.unit} | {q.source} |")
            for f in fields(group):
                q = getattr(group, f.name)
                if isinstance(q, str) and f.name in ("grasp_on", "grasp_model", "gravity_source"):
                    lines.append(f"| M | {group_name}.{f.name} | {q} | |")
        lines.append("")
    lines += ["", "## Start distributions", "", "| Level | lateral | height | yaw | pitch | roll |", "|---|---|---|---|---|---|"]
    for k, lv in LEVELS.items():
        lines.append(f"| {k} | ±{lv.lateral*1e3:.1f} mm | ±{lv.height*1e3:.1f} mm | ±{lv.yaw*57.3:.0f}° | ±{lv.pitch*57.3:.0f}° | ±{lv.roll*57.3:.0f}° |")
    lines += ["", "## Sensor models (sensor_models.py)", "", "| Class | Item | Value | Source |", "|---|---|---|---|"]
    for r in SensorSuite().register():
        lines.append(f"| {r['cls']} | {r['group']}.{r['name']} | {r['value']:.6g} | {r['source']} |")
    lines += ["", "## Structural modelling choices", "", "| Class | Area | Choice | Known consequence |", "|---|---|---|---|"]
    for cls, area, choice, consequence in STRUCTURAL:
        lines.append(f"| {cls} | {area} | {choice} | {consequence} |")
    counts = {}
    for line in lines:
        if line.startswith("| ") and line[2] in "DLIPM" and line[3] == " ":
            counts[line[2]] = counts.get(line[2], 0) + 1
    lines += ["", "## Counts", "", ", ".join(f"{k}: {v}" for k, v in sorted(counts.items())), ""]
    out.write_text("\n".join(lines))
    print("wrote", out, counts)


if __name__ == "__main__":
    main()
