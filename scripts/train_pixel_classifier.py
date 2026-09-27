"""Offline supervised perception calibration using DEVELOPMENT images only."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.optimize import minimize
from scipy.special import expit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ffc.cable_perception import color_features


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    a = parser.parse_args()
    output = a.run / "pixel-model.json"
    if output.exists():
        parser.error("Model already exists; retain frozen model or choose another run")
    labels = json.loads((a.run / "offline/labels.json").read_text())
    generator = np.random.default_rng(927)
    features = []
    targets = []
    sources = []
    for row in labels:
        if row["case"]["split"] != "development":
            continue
        img = np.array(Image.open(a.run / "sensor" / row["file"]))
        mask = np.array(Image.open(a.run / "offline" / row["mask"])) > 0
        x = color_features(img).reshape(-1, 10)
        y = mask.ravel()
        for value in [False, True]:
            indices = np.flatnonzero(y == value)
            chosen = generator.choice(indices, min(2000, len(indices)), replace=False)
            features.append(x[chosen])
            targets.append(y[chosen].astype(float))
        sources.append(
            {
                "file": row["file"],
                "image_sha256": hashlib.sha256((a.run / "sensor" / row["file"]).read_bytes()).hexdigest(),
                "label_sha256": hashlib.sha256((a.run / "offline" / row["mask"]).read_bytes()).hexdigest(),
            }
        )
    x = np.concatenate(features)
    y = np.concatenate(targets)
    # Each class has equal aggregate weight despite differing available pixel counts.
    weight = np.where(y == 1, 0.5 / (y == 1).sum(), 0.5 / (y == 0).sum())
    regularization = 1e-5

    def objective(w):
        z = x @ w
        loss = float(
            np.sum(weight * (np.logaddexp(0, z) - y * z)) + 0.5 * regularization * np.sum(w[1:] ** 2)
        )
        gradient = x.T @ (weight * (expit(z) - y))
        gradient[1:] += regularization * w[1:]
        return loss, gradient

    fit = minimize(
        objective,
        np.zeros(10),
        jac=True,
        method="L-BFGS-B",
        options={"maxiter": 1000, "ftol": 1e-12, "gtol": 1e-8},
    )
    if not fit.success:
        raise RuntimeError(fit.message)
    model = {
        "schema": "quadratic-rgb-v1",
        "coefficients": fit.x.tolist(),
        "features": ["1", "r", "g", "b", "r2", "g2", "b2", "rg", "rb", "gb"],
        "rgb_scale": 255,
        "threshold_logit": 0.0,
        "training_scope": "development split only; offline semantic supervision",
        "images": sources,
        "sampled_pixels": len(y),
        "loss": float(fit.fun),
        "regularization": regularization,
        "limitations": [
            "synthetic appearance only",
            "no face recognition",
            "no depth",
            "not an action policy",
        ],
    }
    output.write_text(json.dumps(model, indent=2))
    print(
        json.dumps(
            {
                "images": len(sources),
                "pixels": len(y),
                "loss": fit.fun,
                "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            }
        )
    )


if __name__ == "__main__":
    main()
