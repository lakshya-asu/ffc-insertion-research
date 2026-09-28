# Connected project overview

`docs/index.html` is the current entry page. It connects the published evidence rather than regenerating or reclassifying historical experiments. `overview.css` and `overview.js` own presentation only; they have no actuator endpoint.

- Existing root videos remain at `docs/experiments.html`. Known old experiment hashes forward there; non-JavaScript visitors retain an archive link.
- `scripts/build_public_site.py` now writes the historical archive and cannot overwrite the new overview.
- `scripts/build_overview_assets.py` produces smaller copies of three existing renders. The image manifest records original and derived hashes. Original evidence images remain unchanged.
- Mechanics plots are copies of `outputs/benchmarks/segment-convergence.png` and `shell-convergence.png`; the CSV and joint-chain JSON are retained beside them. Their interpretation comes from experiments 005–007.
- The perception inspector reads the existing frozen-test manifest and references all 180 original published views. It does not run inference or use annotation as model input.
- The learning section is a proposed curriculum. No trained insertion policy is implied.

Validation: desktop/mobile (1440/390), light/dark, native video metadata, actual motion/cancel playback, all 180 review images, missing-manifest and no-JavaScript fallbacks, archive redirects, local references, axe and mobile Lighthouse. Curated results are in `validation.json`. The initial missing favicon and brand accessible-name mismatch were corrected before the final Lighthouse run. Its 3.0 s simulated-mobile LCP exceeds the 2.5 s target; hosted caching/compression may differ.
