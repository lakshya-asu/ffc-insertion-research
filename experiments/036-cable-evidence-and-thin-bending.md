# 036 — Cable evidence and thin-body bending

The cable is now identified by product, outline revision and a hash-pinned parameter profile. The revised full-cable runner reads that profile. The numerical bending checks fail, so this work does not release a pickup or insertion training dataset.

## Source review

The target is the Raspberry Pi Camera Cable Standard–Mini 200 mm, revision two under PCN 36. The [dossier](../research/cables/standard-mini-200.md) links the primary sources and separates published interface dimensions from assumptions. PCN 48 concerns different 38/150 mm cables. No reviewed source supplies the exact shielded laminate's directional stiffness, friction or damage limits.

`config/cables/rpi-camera-standard-mini-200-rev2.json` has SHA-256 `09e4a5bacf516b39b0c545489d34d050a2c7407df6c3cf7629db69d67aba42c9`. The profile includes the 200 mm length, 11.5/16 mm end widths and 22/15 contacts. Thicknesses, terminal lengths, transition position and material/contact parameters remain explicit assumptions. Exploratory bounds are neither manufacturing tolerances nor measured distributions. PCN PDFs and their hashes are retained in `outputs/cable-source-review-001`; the profile carries their source URLs and hashes.

The revised fixture runner represents both end stiffeners and the wider standard end. Contact strips are visual only; the opposite-face convention remains unverified. Existing static scene parameters remain for compatibility, with an explicit pointer to this profile. Old datasets and scenes have not silently changed.

## Independent numerical check

`scripts/isaac_thin_cantilever.py` creates a 32 mm × 11.5 mm × 0.14 mm equivalent homogeneous strip with 16 rigid segments. E = 3 GPa and density = 1,800 kg/m³ are held fixed. The first whole segment is clamped. The independent nonlinear hinge-chain energy calculation predicts 0.415083 mm sag. This compares the same discrete boundary condition, rather than comparing against an unmatched continuum clamp.

Every case advances one simulated second. The benchmark uses 255 position iterations, 8 velocity iterations, 20/s artificial body drag for settling and the same authored drive gains. The drag is not identified material damping. There is no cable/fixture contact, pickup, suction, perception or controller. Videos show actual Isaac frames. Output geometry and positions are offline measurements only.

| Run | Solver | Timestep | Observed sag | Relative reference error | 5% agreement |
|---|---|---|---|---|---|
| thin-bend-001 | PGS | 0.125 ms | 4.694251 mm | 1,030.9% | Fail |
| thin-bend-002 | PGS | 0.0625 ms | 2.475015 mm | 496.3% | Fail |
| thin-bend-003 | TGS | 0.0625 ms | 2.864144 mm | 590.0% | Fail |

The final 50 ms is almost stationary in all three runs. That does not establish correct equilibrium. Both PGS timesteps fail, and changing to TGS does not repair the error. Isaac warns that TGS behavior with more than four velocity iterations has changed; these results describe the installed 6.1.0 settings, not a universal solver comparison. Material stiffness was not fitted to reduce the discrepancy.

These results isolate the numerical concern from desk or fixture contact. They do not yet identify its cause. A diagnostic of the saved final USD finds maximum adjacent joint-endpoint separation below 0.00051 mm in all three runs, much smaller than the excess sag. The dominant discrepancy therefore appears rotational rather than simple separation at the link endpoints; this is a diagnostic inference, not a solver diagnosis.

The full profile smoke test, `outputs/fixture-profile-001`, also completed: 100 segments, 16,000 physics steps over two seconds at 0.125 ms. Leading-segment center drop was 2.492 mm and grip-region center drop 0.716 mm. This is an unanchored gravity/contact run without a gripper or controller. It verifies that the revised profile executes; it does not qualify those deflections. Its actual video is published alongside all three thin-body failures.

## Dataset preparation

The implemented evidence gate rejects missing, failed, altered and wrong-profile review records. Running the checked-in pickup manifest exits with code 2: bending convergence, contact, pickup and sensor-boundary reviews are missing. This gate checks evidence provenance, not scientific truth or episode integrity. It is not wired into earlier static segmentation trainers.

The dossier specifies the next dataset's sensor/label boundary, episode-level splits, held-out parameter families, bounded demonstrations, imitation baseline and later constrained RL. No new insertion policy has been trained. The next full skill remains flexible-cable pickup, with all commercial gripper options retained.

## Reproduction

Use a fresh output directory for every run:

```bash
docker compose run --rm experiment scripts/isaac_thin_cantilever.py \
  --output /workspace/outputs/thin-bend-NEW --dt 0.0000625 --solver PGS
docker compose run --rm experiment scripts/isaac_fixture_settle.py \
  --output /workspace/outputs/fixture-profile-NEW --dt 0.000125
.venv/bin/python scripts/review_skill_dataset.py \
  --manifest config/cables/pickup-release-review.json \
  --evidence-root outputs --output outputs/NEW-release-review.json
```

Next compare a revised constraint formulation or rod/shell representation against this same frozen reference and add mesh refinement. Return to fixture contact qualification only once the numerical error is within the declared budget.

Validation: 155 CPU tests and 10 ROS contract tests passed (one middleware-dependent test deselected); core and new-script lint passed. Tests cover profile provenance, invalid ranges, out-of-bounds geometry, missing/stale/altered/failed dataset review evidence and the existing independent reference.
