# Working on this research

Read [AGENTS.md](AGENTS.md), [LAB_LIVE.md](LAB_LIVE.md) and the relevant experiment record before changing a pipeline.

- Keep runtime observations deployable: RGB/calibration, measured robot/tool state and realistic contact/tactile signals. Simulator truth belongs only in offline supervision and evaluation.
- Use fresh output directories and preserve failed runs. Record configuration, seeds, sources, checkpoint hashes and the meaning of each acceptance metric.
- Select models on development validation. Freeze weights, preprocessing and decision rules before capturing or evaluating an independent final test.
- Label static renderings, assumed dimensions and numerical proxies clearly. Do not promote mask overlap into insertion accuracy or a single deterministic trial into reliability.
- Fetch the pinned geometry-test assets once with `uv run python scripts/fetch_fr3.py`. Run `uv sync --locked`, `uv run pytest -q` and `uv run ruff check src tests` for core changes. ROS, GPU and mechanics experiments require their own documented checks.
- Do not commit credentials, gated checkpoints, unreviewed external assets or large raw datasets. Keep curated public evidence and its attribution in `docs/`.

A useful issue or pull request states the concrete problem, proposed behavior, observed evidence and remaining limitations. Cite the affected experiment and include reproduction commands where practical.
