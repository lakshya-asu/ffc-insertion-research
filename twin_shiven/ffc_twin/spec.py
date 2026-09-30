"""Single source of truth for the ribbon-insertion twin.

Every number carries a source. Anything whose source starts with "PLACEHOLDER" is a guess that a
measurement must replace; `placeholders()` lists them so they can be printed on every plot.

Frames follow Lakshya's task contract: connector frame C at the slot mouth, +x into the connector,
+y across the cable width, +z the cable surface normal. Units: metres, radians, seconds, newtons.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, fields

TE_DRAWING = "TE customer drawing C-1734248 rev E1, 15 positions (likely Pi 4 CSI/DSI part 1-1734248-5)"
PI_CABLE_DRAWING = "Raspberry Pi RP-008146-DS-1, standard-standard 200 mm camera cable"


@dataclass(frozen=True)
class Q:
    """A quantity with provenance."""

    value: float
    source: str
    unit: str = "m"

    @property
    def placeholder(self) -> bool:
        return self.source.startswith("PLACEHOLDER")


@dataclass(frozen=True)
class Cable:
    width: Q = Q(16.0e-3, PI_CABLE_DRAWING + ": width 16.000 +-0.10")
    header_thickness: Q = Q(0.309e-3, PI_CABLE_DRAWING + ": contact header thickness 0.309 +-0.05")
    body_thickness: Q = Q(0.137e-3, PI_CABLE_DRAWING + ": conductor 0.027 + two insulation layers 0.055 (derived)")
    exposed_contact_length: Q = Q(5.0e-3, PI_CABLE_DRAWING + ": strip length 5.000 +-0.50")
    support_tape_length: Q = Q(6.3e-3, PI_CABLE_DRAWING + ": support length 6.300 +-0.50")
    mass_per_length: Q = Q(4.97e-3, "computed: 15 x 0.7 x 0.027 mm copper + 16 x 0.11 mm PET film", "kg/m")
    modulus: Q = Q(3.0e9, "PLACEHOLDER: homogeneous laminate modulus; fit from cantilever tests", "Pa")
    tail_links: int = 12
    tail_link_length: Q = Q(6.0e-3, "modelling choice: 12 x 6 mm of tail; the rest hangs free and is dropped")
    contacts: int = 15
    contact_pitch: Q = Q(1.0e-3, "Raspberry Pi documentation: standard camera interface, 15 positions at 1 mm")

    @property
    def header_length(self) -> float:
        """Rigid end = the support tape. The exposed contacts sit on the opposite face over the first 5 mm and are
        backed by the tape; the two do NOT add end to end (corrected 27 Sep after re-reading the drawing)."""
        return self.support_tape_length.value

    def bend_stiffness_per_hinge(self) -> float:
        """EI / link length for out-of-plane bending of the body, N m / rad."""
        ei = self.modulus.value * self.width.value * self.body_thickness.value ** 3 / 12
        return ei / self.tail_link_length.value

    def twist_stiffness_per_hinge(self) -> float:
        gj = (self.modulus.value / 2.6) * self.width.value * self.body_thickness.value ** 3 / 3
        return gj / self.tail_link_length.value


@dataclass(frozen=True)
class Connector:
    opening_width: Q = Q(16.40e-3, TE_DRAWING + ": DIM C = 16.40 +-0.1 (read as the FPC opening)")
    seat_depth: Q = Q(3.45e-3, TE_DRAWING + ": section X-X, 3.45 from the top face to the stop")
    slot_height: Q = Q(0.509e-3, "PLACEHOLDER: header 0.309 + 0.1 above and below; open-slot height is not on the drawing")
    top_leadin_half_length: Q = Q(0.6e-3, "PLACEHOLDER: funnel above and below the slot, 25 deg; typical ZIF housing")
    top_leadin_angle: Q = Q(math.radians(25.0), "PLACEHOLDER: lead-in angle", "rad")
    side_leadin_half_length: Q = Q(0.0, "PLACEHOLDER: drawing shows corner chamfers at the pocket ends, undimensioned; 0 = straight walls")
    side_leadin_angle: Q = Q(math.radians(45.0), "PLACEHOLDER: corner chamfer angle", "rad")
    wall_thickness: Q = Q(1.5e-3, "modelling choice: rigid housing thickness")
    cable_thickness_spec: Q = Q(0.3e-3, TE_DRAWING + ": applicable FPC thickness 0.3 +-0.05")
    housing_friction: Q = Q(0.4, "PLACEHOLDER: housing-on-cable friction; the per-episode randomization overrides it", "")
    # Spring-supported contact noses on the floor of the slot (Lakshya's experiment 045 model). 0 = not modelled.
    contact_count: int = 0
    contact_pitch: Q = Q(1.0e-3, "not modelled for this board")
    contact_depth_from_mouth: Q = Q(0.0, "not modelled for this board")
    contact_nose_length: Q = Q(0.4e-3, "not modelled for this board")
    contact_nose_width: Q = Q(0.2e-3, "not modelled for this board")
    contact_nose_height: Q = Q(0.1e-3, "not modelled for this board")
    contact_bevel_length: Q = Q(0.2e-3, "not modelled for this board")
    contact_rest_top_rel_center: Q = Q(-0.17e-3, "not modelled for this board")
    contact_spring: Q = Q(100.0, "not modelled for this board", "N/m")
    contact_damping: Q = Q(0.01, "not modelled for this board", "N s/m")
    contact_mass: Q = Q(1.0e-5, "not modelled for this board", "kg")
    contact_max_deflection: Q = Q(0.1e-3, "not modelled for this board")


@dataclass(frozen=True)
class Tool:
    # With 3.45 mm of the 6.3 mm tape inside the slot, only 2.85 mm of tape is outside; jaws cannot sit there without
    # hitting the housing, so the grip is on the thin film a short distance behind the tape.
    # Two grip options, both modelled. "tape": short jaws on the 2.85 mm of stiffener that stays outside the slot when
    # seated (needs clearance to the lifted actuator, unverified on the drawing). "film": jaws behind the tape on the
    # thin body; a 3 mm free film pitches the tip by tenths of a millimetre under sub-newton loads, more than the slot
    # height clearance, so it fails in the twin. Default is "tape".
    grasp_on: str = "tape"
    # 6 = a six-axis arm (FR3, Stäubli TX2). 4 = a SCARA (Stäubli TS2): pitch and roll cannot be commanded, so start
    # errors in them persist and must be absorbed by the lead-in and the cable.
    dof: int = 6
    jaw_length: Q = Q(1.5e-3, "design choice: thin jaws so they fit on the 2.85 mm of tape outside the seated slot")
    free_film_length: Q = Q(0.0, "design choice: 0 for the tape grip; 3e-3 for the film grip (fails: tip pitches)")
    # Lakshya's simulated preload of 0.36 N holds only 2 x 0.4 x 0.36 = 0.29 N of axial load, below the 1 to 2 N seen in
    # stalls, so the cable slides in the jaws. A clamp of about 3 N is REQUIRED for the expert's 1.5 N ceiling with margin.
    clamp_force: Q = Q(3.0, "I: derived requirement: >= force ceiling x 1.5 margin / (2 x mu); replaces Lakshya's 0.36 N simulation preload, which slips", "N")
    pad_friction: Q = Q(0.4, "PLACEHOLDER: pad-on-film friction until the pull test", "")
    grasp_model: str = "friction joint: the header may slide along the cable axis in the jaws once tangential load exceeds 2 x mu x clamp"
    position_kp: Q = Q(2000.0, "PLACEHOLDER: stand-in for a Cartesian impedance controller", "N/m")
    position_kv: Q = Q(40.0, "PLACEHOLDER", "N s/m")
    rotation_kp: Q = Q(2.0, "PLACEHOLDER", "N m/rad")
    rotation_kv: Q = Q(0.05, "PLACEHOLDER", "N m s/rad")


@dataclass(frozen=True)
class Physics:
    gravity_in_C: tuple = (9.81, 0.0, 0.0)  # Pi 4: the socket is upright, insertion is downward, so gravity points +x INTO the slot in frame C
    gravity_source: str = "D: Pi 4 CSI socket is vertical (Lakshya's hardware review); for the Pi Zero side-entry case gravity is along -z"
    timestep: Q = Q(1.0e-3, "measured on this Mac: <= 1 ms keeps a 0.3 mm tip from penetrating a 0.5 mm slot", "s")
    control_hz: Q = Q(20.0, "design target from Lakshya's architecture guide", "Hz")
    contact_margin: Q = Q(2.0e-5, "rule: margin below the clearance")
    friction: Q = Q(0.4, "PLACEHOLDER: randomize 0.2 to 0.6 until the pull test", "")
    solref: str = "0.0005 1"
    solimp: str = "0.95 0.99 0.0005 0.5 2"


@dataclass(frozen=True)
class Level:
    """Reset distribution: uniform +- these bounds, tip starting 1.5 mm before the mouth."""

    name: str
    lateral: float
    height: float
    yaw: float
    pitch: float
    roll: float
    grasp_offset: float
    grasp_yaw: float


LEVELS = {
    "L0": Level("L0", 0.3e-3, 0.3e-3, math.radians(2), math.radians(2), math.radians(2), 0.3e-3, math.radians(2)),
    "L1": Level("L1", 1.5e-3, 1.5e-3, math.radians(6), math.radians(6), math.radians(6), 0.5e-3, math.radians(3)),
}


@dataclass(frozen=True)
class Success:
    corner_depth_tolerance: Q = Q(0.3e-3, "Lakshya seating.py: all four tip corners deeper than seat depth minus this")
    over_insertion_tolerance: Q = Q(0.25e-3, "M: Lakshya used 0.05 mm; the twin's soft backstop contact penetrates about 0.1 mm per newton, so 0.25 mm covers the 1.5 N ceiling. A hard stop cannot be over-inserted physically")
    peak_force_limit: Q = Q(3.0, "PLACEHOLDER: until the insertion-force curves exist", "N")
    max_jaw_slip: Q = Q(0.3e-3, "PLACEHOLDER")
    start_gap: Q = Q(1.5e-3, "modelling choice: tip starts this far before the mouth")


@dataclass(frozen=True)
class Spec:
    cable: Cable = field(default_factory=Cable)
    connector: Connector = field(default_factory=Connector)
    tool: Tool = field(default_factory=Tool)
    physics: Physics = field(default_factory=Physics)
    success: Success = field(default_factory=Success)
    version: str = "0.2.0"
    board: str = "pi4"

    def placeholders(self) -> list[str]:
        out = []
        for group_name in ("cable", "connector", "tool", "physics", "success"):
            group = getattr(self, group_name)
            for f in fields(group):
                q = getattr(group, f.name)
                if isinstance(q, Q) and q.placeholder:
                    out.append(f"{group_name}.{f.name} = {q.value} {q.unit}: {q.source[len('PLACEHOLDER: '):] if q.source.startswith('PLACEHOLDER: ') else q.source}")
        return out

    def to_json(self) -> str:
        d = asdict(self)
        d["placeholders"] = self.placeholders()
        d["levels"] = {k: asdict(v) for k, v in LEVELS.items()}
        return json.dumps(d, indent=2)


DEFAULT = Spec()

# ---------------------------------------------------------------------------------------------------------------------
# Pi Zero 2 W side-entry case, matched to Lakshya's experiment 045 reference socket (Molex 54548 family) and his cable
# profile. His assumed_* values are hypotheses, so they stay placeholders here, with his config named as the source.
MOLEX_DRAWING = "Molex PSD 54548 drawing, 22 positions (Lakshya's pi-socket-evidence-v1: family reference for the Zero 2 W)"
LAKSHYA_045 = "PLACEHOLDER: Lakshya config/connectors/zero-reference-contact-v4 assumption"
PCN36 = "Raspberry Pi PCN 36 (RP-009201-PC-1), camera cable Standard-Mini 200 mm revision two"

_rho_lakshya = 1800.0  # kg/m^3, his homogeneous laminate assumption
_lead, _exp = 0.30e-3, 0.15e-3

ZERO_CABLE = Cable(
    width=Q(11.5e-3, PCN36 + ": mini end width 11.5 (Molex mating FFC 11.5 +-0.07)"),
    header_thickness=Q(0.30e-3, MOLEX_DRAWING + ": mating FFC thickness 0.3 +-0.03"),
    body_thickness=Q(0.14e-3, "PLACEHOLDER: Lakshya's cable profile assumption (bounds 0.10 to 0.25)"),
    exposed_contact_length=Q(4.0e-3, "PLACEHOLDER: Lakshya's pi-zero-task assumption; Molex minimum 3.0"),
    support_tape_length=Q(6.0e-3, "PLACEHOLDER: Lakshya's pi-zero-task assumption; Molex minimum stiffener 5.0"),
    mass_per_length=Q(_rho_lakshya * 11.5e-3 * 0.14e-3, "I: Lakshya's 1800 kg/m^3 x 11.5 mm x 0.14 mm", "kg/m"),
    contacts=22,
    contact_pitch=Q(0.5e-3, PCN36 + ": mini end 22 positions at 0.5 mm"),
)

ZERO_CONNECTOR = Connector(
    opening_width=Q(11.65e-3, MOLEX_DRAWING + ": DIM D 11.65, used by Lakshya as the throat width hypothesis"),
    seat_depth=Q(2.5e-3, MOLEX_DRAWING + ": FFC insertion depth 2.5"),
    slot_height=Q(0.40e-3, LAKSHYA_045 + ": open throat height 0.40"),
    top_leadin_half_length=Q(math.hypot(_lead, _exp) / 2, LAKSHYA_045 + ": guide 0.30 long, 0.15 expansion per side"),
    top_leadin_angle=Q(math.atan2(_exp, _lead), LAKSHYA_045 + ": guide angle from 0.15 over 0.30", "rad"),
    side_leadin_half_length=Q(math.hypot(_lead, _exp) / 2, LAKSHYA_045 + ": same guide on the side walls"),
    side_leadin_angle=Q(math.atan2(_exp, _lead), LAKSHYA_045 + ": guide angle", "rad"),
    cable_thickness_spec=Q(0.3e-3, MOLEX_DRAWING + ": mating FFC thickness 0.3 +-0.03"),
    housing_friction=Q(0.3, LAKSHYA_045 + ": socket static friction 0.3 (dynamic 0.2)", ""),
    contact_count=22,
    contact_pitch=Q(0.5e-3, MOLEX_DRAWING + ": 0.5 mm pitch"),
    contact_depth_from_mouth=Q(2.5e-3 - 1.35e-3, "I: Molex insertion depth 2.5 minus length-to-contact-point 1.35 (Lakshya's registration hypothesis)"),
    contact_nose_length=Q(0.4e-3, LAKSHYA_045 + ": nose length"),
    contact_nose_width=Q(0.2e-3, LAKSHYA_045 + ": nose width"),
    contact_nose_height=Q(0.1e-3, LAKSHYA_045 + ": nose height"),
    contact_bevel_length=Q(0.2e-3, LAKSHYA_045 + ": tapered nose, bevel 0.2 (revision 4 analytic ramp)"),
    contact_rest_top_rel_center=Q(-0.17e-3, LAKSHYA_045 + ": nose top 0.17 below the cable centre line at rest"),
    contact_spring=Q(100.0, LAKSHYA_045 + ": spring 100 N/m per contact", "N/m"),
    contact_damping=Q(0.01, LAKSHYA_045 + ": damping", "N s/m"),
    contact_mass=Q(1.0e-5, LAKSHYA_045 + ": contact mass", "kg"),
    contact_max_deflection=Q(0.1e-3, LAKSHYA_045 + ": bounded deflection 0.1"),
)

ZERO_PHYSICS = Physics(
    gravity_in_C=(0.0, 0.0, -9.81),
    gravity_source="D: Zero 2 W socket is parallel to the board; the cable lies flat and gravity is along -z (the cable normal)",
)

ZERO = Spec(cable=ZERO_CABLE, connector=ZERO_CONNECTOR, physics=ZERO_PHYSICS, board="zero")

BOARDS = {"pi4": DEFAULT, "zero": ZERO}


def body_grip_tool(spec: Spec, setback_m: float = 12.0e-3, pad_length_m: float = 6.0e-3, dof: int = 6) -> Tool:
    """Lakshya's grip: pads of pad_length centred setback_m behind the leading edge, on the thin body. The free film
    between the jaws and the stiffener is what the twin bends."""
    from dataclasses import replace

    free = setback_m - spec.cable.header_length - pad_length_m / 2
    if free <= 0:
        raise ValueError("setback puts the jaws on the stiffener; use the tape grip")
    return replace(spec.tool, grasp_on="film", jaw_length=Q(pad_length_m, f"Lakshya's pad, {pad_length_m*1e3:.0f} mm along the cable"),
                   free_film_length=Q(free, f"I: {setback_m*1e3:.0f} mm setback minus the {spec.cable.header_length*1e3:.1f} mm stiffener and half the pad"), dof=dof)


def make_spec(board: str = "pi4", grip: str = "tape", dof: int = 6, setback_mm: float = 12.0) -> Spec:
    """One entry point for the scripts: board profile, grip option and arm class."""
    from dataclasses import replace

    base = BOARDS[board]
    if grip == "tape":
        tool = replace(base.tool, dof=dof)
    elif grip == "body":
        tool = body_grip_tool(base, setback_mm * 1e-3, dof=dof)
    else:
        raise ValueError(grip)
    return replace(base, tool=tool)
