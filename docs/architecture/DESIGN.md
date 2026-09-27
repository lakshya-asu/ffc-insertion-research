# Architecture guide design

Audience: the researcher building and discussing an FR3 cable-assembly cell. The page is an engineering field guide, with source-linked claims, actual experiment imagery, readable explanations and progressive disclosure for component detail.

Design dials: variance 5 (diagrams, stage explorer, tables and roadmap give long material different reading rhythms); motion 3 (interaction feedback only, no scroll choreography); density 7 (the user explicitly needs a complete technical discussion, with generous line spacing and a persistent index).

The existing site is static GitHub Pages. This page therefore uses native HTML, CSS and small JavaScript modules without adding a framework or runtime CDN dependency. Self-hosted IBM Plex fonts include their license. Paper/green colors distinguish controls and evidence; dark mode uses explicit contrast tokens. Real workcell imagery and semantic SVG diagrams replace decorative stock imagery. Diagrams have explanatory captions and accessible names.

Mobile layouts collapse the index and module grid, keep tables/diagrams inside local scroll regions, and use native controls. Reduced motion is respected. Implementation status appears before proposed architecture. The historical state-driven run has a different inspector view from the target sensor-driven system.

Review: Playwright exercised all 39 stage selections, all three architecture modes, tool states, camera calculator and page anchors at 1440px and 390px in light/dark themes; no page overflow, browser exceptions or failing HTTP responses. Lighthouse results and measured checks are recorded in validation.json after review. These are website checks, not physical robot validation.
