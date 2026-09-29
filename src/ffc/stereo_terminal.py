"""RGB-only stereo terminal measurement for a controlled blue-stiffener trial.

No USD, object pose, simulator mask, depth buffer or actuator access. A resolved
quad is an appearance measurement; independent geometric evaluation must qualify
it before it can authorize a robot action. Only the designated mini-end ROI is
supported, with the cable body extending roughly in +X of the calibrated cell.
"""

import itertools

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def terminal_quad(rgb):
    values = np.asarray(rgb, float) / 255
    r, g, b = np.moveaxis(values, -1, 0)
    mask = (b > 0.16) & (b > 1.7 * r) & (b > 1.18 * g)
    labels, n = ndimage.label(mask)
    if not n:
        raise ValueError("Blue terminal not visible")
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    mask = labels == sizes.argmax()
    if mask.sum() < 200 or mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
        raise ValueError("Terminal absent, too small or clipped")
    boundary = mask & ~ndimage.binary_erosion(mask)
    y, x = np.nonzero(boundary)
    points = np.c_[x, y].astype(float)
    hull = points[ConvexHull(points).vertices]
    if len(hull) > 40:
        raise ValueError("Irregular terminal outline")
    best = None
    for indices in itertools.combinations(range(len(hull)), 4):
        quad = hull[list(indices)]
        area = abs(np.sum(quad[:, 0] * np.roll(quad[:, 1], -1) - quad[:, 1] * np.roll(quad[:, 0], -1)))
        if best is None or area > best[0]:
            best = area, quad
    if best is None:
        raise ValueError("Insufficient corners")
    quad = best[1]
    smooth = ndimage.gaussian_filter(values, sigma=(1, 1, 0))
    lines = []
    offsets = np.linspace(-10, 10, 81)
    for a, b in zip(quad, np.roll(quad, -1, axis=0), strict=True):
        tangent = (b - a) / np.linalg.norm(b - a)
        normal = np.array([-tangent[1], tangent[0]])
        samples = a + np.linspace(0.15, 0.85, 80)[:, None] * (b - a)
        locations = samples[:, None, :] + offsets[None, :, None] * normal
        colours = np.stack(
            [
                ndimage.map_coordinates(smooth[:, :, c], [locations[:, :, 1], locations[:, :, 0]], order=1)
                for c in range(3)
            ],
            axis=-1,
        )
        gradient = np.linalg.norm(np.gradient(colours, offsets, axis=1), axis=2)
        chosen = offsets[np.argmax(gradient, axis=1)]
        points_on_edge = samples + chosen[:, None] * normal
        for _ in range(3):
            center = points_on_edge.mean(axis=0)
            n = np.linalg.svd(points_on_edge - center)[2][-1]
            residual = abs((points_on_edge - center) @ n)
            good = residual <= max(1.0, 3 * np.median(residual))
            points_on_edge = points_on_edge[good]
        lines.append(np.r_[n, -n @ center])
    refined = []
    for i, line in enumerate(lines):
        other = lines[(i - 1) % 4]
        corner = np.linalg.solve(np.stack([other[:2], line[:2]]), -np.array([other[2], line[2]]))
        if np.linalg.norm(corner - quad[i]) > 15:
            raise ValueError("Unstable edge refinement")
        refined.append(corner)
    return np.array(refined), mask


def triangulate(a, b, p, q):
    matrix = np.stack([a[0] * p[2] - p[0], a[1] * p[2] - p[1], b[0] * q[2] - q[0], b[1] * q[2] - q[1]])
    _, _, vt = np.linalg.svd(matrix)
    homogeneous = vt[-1]
    if abs(homogeneous[3]) < 1e-10:
        raise ValueError("Degenerate stereo geometry")
    return homogeneous[:3] / homogeneous[3]


def estimate(left_rgb, right_rgb, left_projection, right_projection):
    left, _ = terminal_quad(left_rgb)
    right, _ = terminal_quad(right_rgb)
    p, q = np.asarray(left_projection), np.asarray(right_projection)
    candidates = []
    for permutation in itertools.permutations(range(4)):
        matched = right[list(permutation)]
        xyz = np.array([triangulate(a, b, p, q) for a, b in zip(left, matched, strict=True)])
        errors = []
        positive = True
        for projection, pixels in [(p, left), (q, matched)]:
            projected = np.c_[xyz, np.ones(4)] @ projection.T
            positive &= bool(np.all(projected[:, 2] > 0))
            errors.extend(np.linalg.norm(projected[:, :2] / projected[:, 2:] - pixels, axis=1))
        lengths = np.linalg.norm(np.roll(xyz, -1, axis=0) - xyz, axis=1)
        expected = np.array([0.006, 0.006, 0.0115, 0.0115])
        shape_error = np.max(abs(np.sort(lengths) - expected))
        if positive and shape_error < 0.001 and max(errors) < 2:
            candidates.append((max(errors), xyz, lengths, permutation))
    if not candidates:
        raise ValueError("No consistent stereo terminal correspondence")
    candidates.sort(key=lambda c: c[0])
    error, xyz, lengths, permutation = candidates[0]
    wide_edges = np.argsort(lengths)[-2:]
    index = min(wide_edges, key=lambda i: (xyz[i, 0] + xyz[(i + 1) % 4, 0]) / 2)
    a, b = xyz[index], xyz[(index + 1) % 4]
    if a[1] > b[1]:
        a, b = b, a
    width = (b - a) / np.linalg.norm(b - a)
    normal = np.linalg.svd(xyz - xyz.mean(axis=0))[2][-1]
    normal *= 1 if normal[2] > 0 else -1
    body_axis = np.cross(width, normal)
    if body_axis[0] < 0.8:
        raise ValueError("Outside controlled presentation heading")
    return {
        "tip_center_m": ((a + b) / 2).tolist(),
        "tip_endpoints_m": [a.tolist(), b.tolist()],
        "body_axis": body_axis.tolist(),
        "normal": normal.tolist(),
        "yaw_deg": float(np.degrees(np.arctan2(body_axis[1], body_axis[0]))),
        "reprojection_max_px": float(error),
        "corners_m": xyz.tolist(),
        "left_corners_px": left.tolist(),
        "right_corners_px": right[list(permutation)].tolist(),
        "method": "RGB blue component, convex quadrilateral, calibrated stereo",
        "motion_permitted": False,
        "face_polarity_verified": False,
    }


def estimate_top_surface(left_rgb, right_rgb, left_projection, right_projection):
    """Fit known terminal top-face dimensions using each camera's far edge.

    A near silhouette can include the vertical cut face, so it is unsuitable as
    a shared stereo landmark. Each far top edge is used in its own camera, with
    a rigid 6 x 11.5 mm rectangle prior. Controlled, top-facing presentation only.
    """
    initial = estimate(left_rgb, right_rgb, left_projection, right_projection)
    xyz = np.array(initial["corners_m"])
    center = xyz.mean(axis=0)
    normal, x_axis = np.array(initial["normal"]), np.array(initial["body_axis"])
    y_axis = np.cross(normal, x_axis)
    rotation = np.stack([x_axis, y_axis, normal], axis=1)
    centered = (xyz - center) @ rotation
    object_points = np.c_[np.sign(centered[:, 0]) * 0.003, np.sign(centered[:, 1]) * 0.00575, np.zeros(4)]
    observations = []
    for projection, key in [(left_projection, "left_corners_px"), (right_projection, "right_corners_px")]:
        projection = np.asarray(projection)
        camera_h = np.linalg.svd(projection)[2][-1]
        eye = camera_h[:3] / camera_h[3]
        if (eye - center) @ normal <= 0:
            raise ValueError("Top-face fit requires cameras above the terminal")
        far_sign = -np.sign((eye - center) @ x_axis)
        indices = np.flatnonzero(np.sign(object_points[:, 0]) == far_sign)
        if len(indices) != 2:
            raise ValueError("Unresolved far top edge")
        observations.append((projection, object_points[indices], np.array(initial[key])[indices]))
    if np.sign(observations[0][1][0, 0]) == np.sign(observations[1][1][0, 0]):
        raise ValueError("Need views from opposite ends of the terminal")

    def residual(parameters):
        rotation = Rotation.from_rotvec(parameters[3:]).as_matrix()
        errors = []
        for projection, points, pixels in observations:
            world = points @ rotation.T + parameters[:3]
            homogeneous = np.c_[world, np.ones(len(world))] @ projection.T
            errors.extend((homogeneous[:, :2] / homogeneous[:, 2:] - pixels).ravel())
        return np.array(errors)

    seed = np.r_[center, Rotation.from_matrix(rotation).as_rotvec()]
    bounds = np.r_[np.full(3, 0.001), np.full(3, 0.2)]
    fit = least_squares(
        residual, seed, bounds=(seed - bounds, seed + bounds), x_scale=[0.001] * 3 + [0.1] * 3
    )
    error = float(np.max(np.linalg.norm(residual(fit.x).reshape(-1, 2), axis=1)))
    if not fit.success or error > 1:
        raise ValueError("Inconsistent far-edge surface fit")
    rotation = Rotation.from_rotvec(fit.x[3:]).as_matrix()
    edge = np.array([[-0.003, -0.00575, 0], [-0.003, 0.00575, 0]]) @ rotation.T + fit.x[:3]
    result = dict(initial)
    result.update(
        tip_center_m=edge.mean(axis=0).tolist(),
        tip_endpoints_m=edge.tolist(),
        body_axis=rotation[:, 0].tolist(),
        normal=rotation[:, 2].tolist(),
        yaw_deg=float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))),
        surface_fit_reprojection_max_px=error,
        method="RGB far top edges plus a known 6 x 11.5 mm planar terminal prior",
        limitation="Top face must be visible, planar and dimensionally consistent; near silhouette is unused",
    )
    return result
