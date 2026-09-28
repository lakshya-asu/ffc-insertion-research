# Research notebook · edition 01

[Download the 23-page PDF](https://lakshya-asu.github.io/ffc-insertion-research/notes/ffc-research-notebook.pdf)

A reading edition covering the task, hardware, CAD, optics, mount, ROS2, perception results, mechanics, learning roadmap and evaluation plan. Evidence snapshot: commit `30b6a78`, 28 September 2026 UTC. Rendered figures are identified as simulation; historical privileged-state trials are separated from the current observation-only Pi work.

Rebuild from the repository root:

```bash
python3 scripts/build_research_notebook.py
node scripts/render_research_notebook.cjs
```

The renderer requires Playwright with Chromium. If installed outside the project, set `PLAYWRIGHT_MODULE` to its module directory. Fonts and scene images are local repository assets. The export checks page bounds, waits for fonts and images, and includes PDF tags, outline and clickable links.

Review evidence is in the ignored `outputs/research-notebook-review-001`: per-page layout measurements, rasterized PDF contact sheets and text extraction. All 23 pages were visually reviewed; text extraction yielded approximately 6,262 words with no replacement characters.

See the final PDF page for source links and image attribution. Derived Zero PCB scene images retain CC BY-SA 4.0 attribution.
