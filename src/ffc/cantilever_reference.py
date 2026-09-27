"""Static references independent of the time-stepping physics engine."""

import numpy as np
from scipy.optimize import minimize


def hinge_chain_reference(cable: dict) -> dict:
    """Homogeneous chain, first whole segment clamped, remaining hinges elastic.

    Angles are positive downwards. Translational joints are ideal, contacts are
    absent, and each free segment carries its mass at its center. This benchmark
    excludes end stiffeners, rather than treating them as homogeneous material.
    """
    if cable["stiffener_segments"]:
        raise ValueError("Static homogeneous benchmark requires stiffeners disabled")
    count = cable["segments"] - 1
    spacing = cable["length_m"] / cable["segments"]
    weight = cable["mass_kg"] * 9.81 / cable["segments"]
    stiffness = cable["bend_stiffness_nm_per_degree"] * 180 / np.pi * cable.get("angular_drive_gain_scale", 1)
    weights = np.arange(count - 0.5, 0, -1)
    factor = weight * spacing / stiffness

    def energy(theta):
        orientations = np.cumsum(theta)
        value = 0.5 * (theta @ theta) - factor * (weights @ np.sin(orientations))
        gradient = theta - factor * np.cumsum((weights * np.cos(orientations))[::-1])[::-1]
        return value, gradient

    guess = factor * np.arange(count, 0, -1) ** 2 / 2
    solution = minimize(
        energy,
        guess,
        jac=True,
        method="L-BFGS-B",
        bounds=[(-np.deg2rad(35), np.deg2rad(35))] * count,
        options={"gtol": 1e-12, "ftol": 1e-15, "maxiter": 5000},
    )
    residual = float(np.max(np.abs(energy(solution.x)[1])))
    if not solution.success or residual > 1e-6:
        raise RuntimeError(f"Independent static-chain solve failed: {solution.message}; residual={residual}")
    return {
        "linear_hinge_chain_prediction_m": float(
            weight * spacing**2 / (2 * stiffness) * sum(i**3 for i in range(1, count + 1))
        ),
        "nonlinear_hinge_chain_prediction_m": float(spacing * np.sin(np.cumsum(solution.x)).sum()),
        "reference_maximum_gradient_residual_rad": residual,
        "reference_joint_angles_rad": solution.x.tolist(),
    }
