"""Native PhysX surface FEM ribbon with explicit finite-thickness contact.

The solver is isotropic and supports one material per surface. This is an
effective homogeneous laminate model, not resolved copper/PET microstructure.
"""

from __future__ import annotations

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade, Vt


def make_shell(stage, cfg: dict) -> str:
    """Create a uniformly triangulated ribbon with explicit material and contact thickness."""
    import numpy as np
    from omni.physx.scripts import deformableUtils

    c = cfg["cable"]
    shell = cfg["shell"]
    length, width, thickness = c["length_m"], c["width_m"], c["thickness_m"]
    nx, ny = shell["longitudinal_elements"], shell["width_elements"]
    xs = np.linspace(0, length, nx + 1)
    ys = np.linspace(-width / 2, width / 2, ny + 1)
    origin = np.asarray(c["start_xyz_m"])
    points = [(origin + [x, y, 0]).tolist() for x in xs for y in ys]
    triangles = []
    for i in range(nx):
        for j in range(ny):
            a, b = i * (ny + 1) + j, (i + 1) * (ny + 1) + j
            triangles.extend([(a, b, b + 1), (a, b + 1, a + 1)])
    path = "/World/Cable/Surface"
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr([v for tri in triangles for v in tri])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.88, 0.8, 0.6)])
    prim = mesh.GetPrim()
    if not deformableUtils.set_physics_surface_deformable_body(stage, prim.GetPath()):
        raise RuntimeError("Native surface deformable creation failed")
    prim.GetAttribute("omniphysics:mass").Set(c["mass_kg"])
    prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
    prim.GetAttribute("physxDeformableBody:selfCollision").Set(True)
    iterations = shell.get("solver_iterations", 64)
    if not 1 <= iterations <= 255:
        raise ValueError("Installed PhysX schema permits 1–255 deformable solver iterations")
    prim.GetAttribute("physxDeformableBody:solverPositionIterationCount").Set(iterations)
    contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    contact.CreateRestOffsetAttr(thickness / 2)
    contact.CreateContactOffsetAttr(thickness / 2 + c["contact_offset_m"])
    material = UsdShade.Material.Define(stage, "/World/Materials/Shell")
    m = material.GetPrim()
    for api in (
        "OmniPhysicsBaseMaterialAPI",
        "OmniPhysicsDeformableMaterialAPI",
        "OmniPhysicsSurfaceDeformableMaterialAPI",
        "PhysxSurfaceDeformableMaterialAPI",
    ):
        if not m.ApplyAPI(api):
            raise RuntimeError(f"Missing deformable material API: {api}")
    E, nu = shell["youngs_modulus_pa"], shell["poisson_ratio"]
    for name, value in {
        "omniphysics:dynamicFriction": shell["dynamic_friction"],
        "omniphysics:density": c["mass_kg"] / (length * width * thickness),
        "omniphysics:youngsModulus": E,
        "omniphysics:poissonsRatio": nu,
        "omniphysics:surfaceThickness": thickness,
        "omniphysics:surfaceBendStiffness": E / (12 * (1 - nu * nu)),
        "physxDeformableMaterial:elasticityDamping": shell["elasticity_damping"],
        "physxDeformableMaterial:bendDamping": shell["bend_damping"],
    }.items():
        attr = m.GetAttribute(name)
        if not attr:
            raise RuntimeError(f"Missing material attribute: {name}")
        attr.Set(value)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
    return path


def shell_points(stage, path="/World/Cable/Surface") -> list:
    """Read actual simulation vertices, not a prescribed animation."""
    return [list(p) for p in UsdGeom.Mesh.Get(stage, path).GetPointsAttr().Get()]


def validate_shell(stage, cfg: dict) -> dict:
    """Check rest shape, native solver schemas, mass and dimensional consistency."""
    import numpy as np

    c, s = cfg["cable"], cfg["shell"]
    p = stage.GetPrimAtPath("/World/Cable/Surface")
    assert p.HasAPI("OmniPhysicsSurfaceDeformableSimAPI")
    assert p.HasAPI(UsdPhysics.CollisionAPI)
    vertices = np.asarray(shell_points(stage))
    assert vertices.shape == ((s["longitudinal_elements"] + 1) * (s["width_elements"] + 1), 3)
    spans = np.ptp(vertices, axis=0)
    assert np.allclose(spans[:2], [c["length_m"], c["width_m"]], atol=1e-7)
    mass = p.GetAttribute("omniphysics:mass").Get()
    assert abs(mass - c["mass_kg"]) < 1e-8
    return {
        "model": "native_surface_deformable",
        "vertices": len(vertices),
        "triangles": len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()),
        "mass_kg": mass,
        "length_m": float(spans[0]),
        "width_m": float(spans[1]),
        "material_calibrated": False,
        "laminate_anisotropy_resolved": False,
    }
