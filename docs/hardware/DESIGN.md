# Hardware review design

Reading this as an engineering field guide for a robotics research discussion, continuing the existing restrained editorial site. Native static HTML/CSS, self-hosted IBM Plex, and the existing green accent/token system; this is not an implementation of Carbon or another packaged design system.

Design variance 4, motion intensity 2, visual density 4. Accurate technical content and actual Isaac renders take priority over marketing-page patterns. Images must remain evidence from the model: generated product photography would misrepresent its fidelity. Tables and the conceptual mating cross-section explain engineering relationships rather than decorate a landing page.

The top image introduces the imported board. The view selector supports seven actual renders with captions that explain what is and is not represented. The orbit video is user-controlled and does not autoplay. Later sections separate dimensions, physical mating, fidelity limits, the second task and validation order. No perception or insertion success is inferred from asset rendering.

Mobile layout collapses content and sequence diagrams deliberately. The single diagram allows contained horizontal scrolling when text would otherwise become too small. Page-wide light/dark themes share one token system. All image buttons are native keyboard-operable controls with pressed states; failures have an inline status message.

Verification covers all seven views at 1440 px and 390 px, both themes, missing assets, page overflow and browser errors. Lighthouse reports are summarized in validation.json. Visible text and captions were reviewed for unsupported precision and for confusing imported geometry with calibrated behavior.
