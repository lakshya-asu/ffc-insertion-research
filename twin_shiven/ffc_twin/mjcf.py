"""Emit the MuJoCo scene from the spec. Geometry is identical to the USD emitter, by construction."""
from __future__ import annotations

import math

from .spec import DEFAULT, Spec


def build_mjcf(spec: Spec = DEFAULT) -> str:
    c, k, t, p = spec.cable, spec.connector, spec.tool, spec.physics
    W, T_HDR, T_BODY = c.width.value, c.header_thickness.value, c.body_thickness.value
    hdr = c.header_length
    seg, n_tail = c.tail_link_length.value, c.tail_links
    k_bend, k_twist = c.bend_stiffness_per_hinge(), c.twist_stiffness_per_hinge()
    slot_w, slot_h, slot_d, wall = k.opening_width.value, k.slot_height.value, k.seat_depth.value, k.wall_thickness.value
    ch, a, ch_t = k.top_leadin_half_length.value, k.top_leadin_angle.value, 0.05e-3
    ch_cx = ch * math.cos(a) - ch_t * math.sin(a)
    ch_cz = slot_h / 2 + ch * math.sin(a) + ch_t * math.cos(a)
    side = k.side_leadin_half_length.value
    side_geoms = ""
    if side > 0:
        s_a = k.side_leadin_angle.value
        cx = side * math.cos(s_a) - ch_t * math.sin(s_a)
        cy = slot_w / 2 + side * math.sin(s_a) + ch_t * math.cos(s_a)
        side_geoms = (
            f'<geom type="box" pos="{-cx:.8f} {cy:.8f} 0" euler="0 0 {math.degrees(s_a):.2f}" size="{side:.8f} {ch_t:.8f} {slot_h/2+wall:.8f}" rgba="0.86 0.84 0.78 1"/>'
            f'<geom type="box" pos="{-cx:.8f} {-cy:.8f} 0" euler="0 0 {-math.degrees(s_a):.2f}" size="{side:.8f} {ch_t:.8f} {slot_h/2+wall:.8f}" rgba="0.86 0.84 0.78 1"/>'
        )
    # Spring-supported contact noses (Lakshya's 045 model): a flat land plus a ramp facing the mouth, on a z slide with a
    # spring, bounded deflection. Each nose is its own body so the cable rides over it and presses it down.
    contacts = ""
    if k.contact_count:
        n_l, n_w, n_h, bev = k.contact_nose_length.value, k.contact_nose_width.value, k.contact_nose_height.value, k.contact_bevel_length.value
        top = k.contact_rest_top_rel_center.value
        slope = math.atan2(n_h, bev)
        ramp_len = math.hypot(bev, n_h)
        r_t = n_h / 2
        # ramp centre: from the mouth-side end of the nose (x = -n_l/2 at floor level) rising to (x = -n_l/2 + bev, z = top)
        r_cx = -n_l / 2 + bev / 2 - (r_t / 2) * math.sin(slope)
        r_cz = top - n_h / 2 - (r_t / 2) * math.cos(slope)
        # Numerical settling of a 10 mg nose on a 100 N/m spring at a 1 ms step: add 0.1 g of joint armature (inertia
        # without weight) and critical damping on the joint, and put the upper stop just above rest so the spring, not the
        # stop, holds the nose at rest. Lakshya's Isaac run used the same trick class (artificial drag) for settling.
        arm = 1.0e-4
        c_crit = 2.0 * math.sqrt(k.contact_spring.value * (k.contact_mass.value + arm))
        for i in range(k.contact_count):
            y = (i - (k.contact_count - 1) / 2) * k.contact_pitch.value
            contacts += (
                f'<body name="contact{i}" pos="{k.contact_depth_from_mouth.value:.8f} {y:.8f} 0">'
                f'<joint name="contact{i}_z" type="slide" axis="0 0 1" stiffness="{k.contact_spring.value}" damping="{max(k.contact_damping.value, c_crit):.5f}" '
                f'limited="true" range="{-k.contact_max_deflection.value:.8f} {0.02e-3:.8f}" armature="{arm}" solreflimit="0.002 1"/>'
                # collision bit 4: the noses touch only the header (contype 5), never the floor they sit in (Lakshya filters that pair too)
                f'<geom name="contact{i}_land" type="box" pos="{bev/2:.8f} 0 {top - n_h/2:.8f}" size="{(n_l-bev)/2:.8f} {n_w/2:.8f} {n_h/2:.8f}" mass="{k.contact_mass.value:.2e}" rgba="0.75 0.57 0.16 1" friction="{k.housing_friction.value} 0.005 0.0001" contype="0" conaffinity="4"/>'
                f'<geom name="contact{i}_ramp" type="box" pos="{r_cx:.8f} 0 {r_cz:.8f}" euler="0 {-math.degrees(slope):.3f} 0" size="{ramp_len/2:.8f} {n_w/2:.8f} {r_t/2:.8f}" mass="1e-9" rgba="0.75 0.57 0.16 1" friction="{k.housing_friction.value} 0.005 0.0001" contype="0" conaffinity="4"/>'
                f'</body>'
            )
    tail, close = "", ""
    for i in range(n_tail):
        m = c.mass_per_length.value * seg
        pos = -seg if i else -seg / 2
        tail += (
            f'<body name="tail{i}" pos="{pos:.8f} 0 0">'
            f'<joint name="bend{i}" type="hinge" axis="0 1 0" stiffness="{k_bend:.4e}" damping="{k_bend*0.02:.4e}" armature="1e-9"/>'
            f'<joint name="twist{i}" type="hinge" axis="1 0 0" stiffness="{k_twist:.4e}" damping="{k_twist*0.02:.4e}" armature="1e-9"/>'
            f'<geom type="box" size="{seg/2:.8f} {W/2:.8f} {T_BODY/2:.8f}" mass="{m:.4e}" rgba="0.16 0.17 0.19 1" contype="2" conaffinity="0"/>'
        )
        close += "</body>"
    n_c, pitch = c.contacts, c.contact_pitch.value
    strip_z = -(T_HDR / 2 + 0.00002) if k.contact_count else T_HDR / 2 + 0.00002   # contacts face the sprung noses when modelled
    strips = "".join(
        f'<geom type="box" pos="{hdr/2 - c.exposed_contact_length.value/2:.8f} {(i-(n_c-1)/2)*pitch:.8f} {strip_z:.8f}" '
        f'size="{c.exposed_contact_length.value/2:.8f} {0.35*pitch:.6f} 0.00002" rgba="0.85 0.66 0.25 1" contype="0" conaffinity="0"/>'
        for i in range(n_c)
    )
    film = t.free_film_length.value if t.grasp_on == "film" else 0.0
    grip_limit = 2 * t.pad_friction.value * t.clamp_force.value
    jl = t.jaw_length.value
    hdr_col = 'contype="5" conaffinity="1"'   # bit 1: housing; bit 4: the sprung contact noses
    if t.grasp_on == "tape":
        # jaws sit on the rear jl of the tape: the header body carries the jaws; tip is hdr - jl/2 ahead of the jaw centre
        tool_x = -(hdr - jl / 2) - spec.success.start_gap.value
        film_and_header = f'''<body name="header" pos="{hdr/2 - jl/2:.8f} 0 0">
            <geom name="hdr" type="box" size="{hdr/2:.8f} {W/2:.8f} {T_HDR/2:.8f}" mass="{c.mass_per_length.value*hdr*2:.4e}" rgba="0.18 0.42 0.85 1" {hdr_col}/>
            {strips}
            <site name="tip" pos="{hdr/2:.8f} 0 0" size="0.0003"/>
          </body>'''
    else:
        tool_x = -hdr - film - jl / 2 - spec.success.start_gap.value
        kb = c.bend_stiffness_per_hinge() * seg / film; kt = c.twist_stiffness_per_hinge() * seg / film
        film_and_header = f'''<body name="free_film" pos="{jl/2 + film/2:.8f} 0 0">
          <joint name="film_bend" type="hinge" axis="0 1 0" stiffness="{kb:.4e}" damping="{kb*0.02:.4e}" armature="1e-9"/>
          <joint name="film_twist" type="hinge" axis="1 0 0" stiffness="{kt:.4e}" damping="{kt*0.02:.4e}" armature="1e-9"/>
          <geom name="film_free" type="box" size="{film/2:.8f} {W/2:.8f} {T_BODY/2:.8f}" mass="{c.mass_per_length.value*film:.4e}" rgba="0.16 0.17 0.19 1" contype="2" conaffinity="0"/>
          <body name="header" pos="{film/2 + hdr/2:.8f} 0 0">
            <geom name="hdr" type="box" size="{hdr/2:.8f} {W/2:.8f} {T_HDR/2:.8f}" mass="{c.mass_per_length.value*hdr*2:.4e}" rgba="0.18 0.42 0.85 1" {hdr_col}/>
            {strips}
            <site name="tip" pos="{hdr/2:.8f} 0 0" size="0.0003"/>
          </body>
        </body>'''
    return f"""<mujoco model="ffc_twin_v{spec.version}">
  <option timestep="{p.timestep.value}" gravity="{p.gravity_in_C[0]} {p.gravity_in_C[1]} {p.gravity_in_C[2]}" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="2"/>
  <default>
    <geom condim="4" friction="{p.friction.value} 0.005 0.0001" solref="{p.solref}" solimp="{p.solimp}" margin="{p.contact_margin.value}"/>
  </default>
  <worldbody>
    <!-- Connector frame C: mouth at the origin, +x into the slot, +y across the cable, +z cable normal -->
    <body name="connector" pos="0 0 0">
      <geom name="wall_l" type="box" pos="{slot_d/2:.8f} {(slot_w/2+wall/2):.8f} 0" size="{slot_d/2:.8f} {wall/2:.8f} {slot_h/2+wall:.8f}" rgba="0.86 0.84 0.78 1"/>
      <geom name="wall_r" type="box" pos="{slot_d/2:.8f} {-(slot_w/2+wall/2):.8f} 0" size="{slot_d/2:.8f} {wall/2:.8f} {slot_h/2+wall:.8f}" rgba="0.86 0.84 0.78 1"/>
      <geom name="roof" type="box" pos="{slot_d/2:.8f} 0 {(slot_h/2+wall/2):.8f}" size="{slot_d/2:.8f} {slot_w/2+wall:.8f} {wall/2:.8f}" rgba="0.86 0.84 0.78 0.35"/>
      <geom name="floor" type="box" pos="{slot_d/2:.8f} 0 {-(slot_h/2+wall/2):.8f}" size="{slot_d/2:.8f} {slot_w/2+wall:.8f} {wall/2:.8f}" rgba="0.86 0.84 0.78 1"/>
      <geom name="backstop" type="box" pos="{slot_d+wall/2:.8f} 0 0" size="{wall/2:.8f} {slot_w/2+wall:.8f} {slot_h/2+wall:.8f}" rgba="0.45 0.43 0.38 1"/>
      <geom name="leadin_top" type="box" pos="{-ch_cx:.8f} 0 {ch_cz:.8f}" euler="0 {math.degrees(a):.2f} 0" size="{ch:.8f} {slot_w/2+wall:.8f} {ch_t:.8f}" rgba="0.86 0.84 0.78 1"/>
      <geom name="leadin_bottom" type="box" pos="{-ch_cx:.8f} 0 {-ch_cz:.8f}" euler="0 {-math.degrees(a):.2f} 0" size="{ch:.8f} {slot_w/2+wall:.8f} {ch_t:.8f}" rgba="0.86 0.84 0.78 1"/>
      {side_geoms}
    </body>
    <!-- Tool: six joints standing in for a Cartesian impedance controller; jaws clamp the support tape -->
    <body name="tool" pos="{tool_x:.8f} 0 0">
      <joint name="tx" type="slide" axis="1 0 0"/><joint name="ty" type="slide" axis="0 1 0"/><joint name="tz" type="slide" axis="0 0 1"/>
      <joint name="rx" type="hinge" axis="1 0 0"/><joint name="ry" type="hinge" axis="0 1 0"/><joint name="rz" type="hinge" axis="0 0 1"/>
      <!-- jaws are visual and nearly massless: a real arm carries its own tool weight (gravity feedforward), so the
           position servo stand-in must not sag under it (50 g on a 2000 N/m stand-in sagged 0.25 mm with gravity along z) -->
      <geom name="jaw_top" type="box" pos="0 0 {T_HDR/2+0.0006:.8f}" size="{jl/2:.8f} 0.006 0.0005" mass="0.0005" rgba="0.30 0.32 0.36 1" contype="0" conaffinity="0"/>
      <geom name="jaw_bottom" type="box" pos="0 0 {-(T_HDR/2+0.0006):.8f}" size="{jl/2:.8f} 0.006 0.0005" mass="0.0005" rgba="0.30 0.32 0.36 1" contype="0" conaffinity="0"/>
      <!-- gripped film segment: may slide along the cable axis against dry friction = 2 mu N (the grasp model) -->
      <body name="gripped" pos="0 0 0">
        <joint name="grip_slip" type="slide" axis="1 0 0" frictionloss="{grip_limit:.6f}" damping="0.5" armature="0.001" solreffriction="0.002 1" solimpfriction="0.99 0.999 0.0001 0.5 2" limited="true" range="{-jl:.6f} {jl:.6f}"/>
        <geom name="film_in_jaws" type="box" size="{jl/2:.8f} {W/2:.8f} {T_BODY/2:.8f}" mass="{c.mass_per_length.value*jl:.4e}" rgba="0.16 0.17 0.19 1" contype="0" conaffinity="0"/>
        {film_and_header}
        <!-- tail behind the jaws -->
        <body name="tailroot" pos="{-jl/2:.8f} 0 0">
          {tail}{close}
        </body>
      </body>
    </body>
    <!-- sprung contact noses come AFTER the tool so the tool's six joints stay the first six degrees of freedom -->
    {contacts}
  </worldbody>
  <actuator>
    <position joint="tx" kp="{t.position_kp.value}" kv="{t.position_kv.value}"/><position joint="ty" kp="{t.position_kp.value}" kv="{t.position_kv.value}"/><position joint="tz" kp="{t.position_kp.value}" kv="{t.position_kv.value}"/>
    <position joint="rx" kp="{t.rotation_kp.value}" kv="{t.rotation_kv.value}"/><position joint="ry" kp="{t.rotation_kp.value}" kv="{t.rotation_kv.value}"/><position joint="rz" kp="{t.rotation_kp.value}" kv="{t.rotation_kv.value}"/>
  </actuator>
  <sensor><force name="tip_force" site="tip"/><torque name="tip_torque" site="tip"/></sensor>
</mujoco>"""
