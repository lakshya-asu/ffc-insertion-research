'use strict';
const $ = id => document.getElementById(id);
const choose = (attribute, value) => document.querySelectorAll(`[${attribute}]`).forEach(button => button.setAttribute('aria-pressed', String(button.getAttribute(attribute) === value)));
let theme = 'system';
try { theme = localStorage.getItem('ffc-theme') || 'system'; } catch (_) { /* Storage is optional. */ }
function setTheme(value) {
  theme = ['system', 'light', 'dark'].includes(value) ? value : 'system';
  if (theme === 'system') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.dataset.theme = theme;
  $('theme-toggle').textContent = `Theme: ${theme}`;
  try { localStorage.setItem('ffc-theme', theme); } catch (_) { /* Keep controls usable. */ }
}
setTheme(theme);
$('theme-toggle').addEventListener('click', () => setTheme({system: 'light', light: 'dark', dark: 'system'}[theme]));
$('chapter-select').addEventListener('change', event => { location.hash = event.target.value; });
const modules = {
  hardware: [
    ['Sensors', 'Raw images, tactile readings, robot joints, jaw load, vacuum pressure and fixture wrench.', 'Calibrated devices with timestamps, validity flags and physically realizable mounts.', 'Recorded sensor packets', 'No source may supply hidden simulated cable geometry.'],
    ['Observation gate', 'Validate and align observations before a controller consumes them.', 'A vector-only prototype is implemented and tested. Camera transport and isolation are next.', 'Immutable measured samples', 'Reject stale, missing, malformed or unsynchronized data.'],
    ['Estimator + skill', 'Estimate cable/connector alignment and contact belief from recent observations.', 'Proposed perception, tactile fusion and sensor-driven skill controller.', 'Bounded tool and gripper requests', 'Uncertainty triggers reobservation or a supported recovery.'],
    ['Local control', 'Execute bounded requests through the robot and tool interfaces.', 'Proposed real-time Franka process, tool I/O and independent watchdog.', 'Measured motion and sensor feedback', 'No network inference or file writing in the real-time callback.']
  ],
  simulation: [
    ['Experiment runner', 'Configure the historical Isaac scene, physics clock and output artifacts.', 'scripts/isaac_workcell.py and scripts/run_pipeline.sh', 'Versioned runs and source snapshots', 'Historical privileged baseline; explicit flag now required for the suite.'],
    ['Scene + mechanics', 'Simulate FR3, tool, segmented cable, suction and connector envelope.', 'isaac_scene.py, isaac_suction.py and isaac_contacts.py', 'Body dynamics and contact records', 'Nominal material and clearance slot are not physically calibrated.'],
    ['Scripted control', 'Execute historical stage trajectories and tip alignment.', 'isaac_episode.py and isaac_kinematics.py', 'Robot and tool drive commands', 'Reads exact simulator state. Retained only as historical geometry evidence.'],
    ['Offline evidence', 'Record stage clips and independently inspect grip and seating.', 'isaac_recording.py, audit_grip.py and audit_seating.py', 'Videos, reports and geometric audits', 'Audit success does not certify latch closure or electrical continuity.']
  ],
  learning: [
    ['Sensor dataset', 'Synchronized clean observations, executed actions and observed outcomes.', 'Proposed episode-level dataset with calibration and specimen identity.', 'Training and held-out episode groups', 'No overlays, magic tracking cameras or adjacent-frame evaluation leakage.'],
    ['Policy + critic', 'Predict short action sequences or bounded corrections from sensor history.', 'Imitation baseline first; sensor-conditioned RL only after validation.', 'Small tool-frame actions', 'Actor and critic never receive privileged cable state.'],
    ['Supervisor', 'Apply the same limits and sensor-health checks in simulation and hardware.', 'Proposed bounded controller with explicit recovery states.', 'Permitted local commands', 'The neural policy cannot bypass stale-data or load checks.'],
    ['Evaluation', 'Compare policies on untouched instances, faults and physical checks.', 'Independent reports with all attempts and interventions counted.', 'Success, damage, recovery and timing evidence', 'Simulator truth stays offline and cannot decide live stage transitions.']
  ]
};
function renderSystem(view) {
  choose('data-system', view);
  $('system-description').textContent = {hardware: 'Target architecture: every decision uses deployable sensor observations. Only the vector ingress prototype is implemented so far.', simulation: 'Historical architecture: exact simulator state drives the recorded geometric baseline.', learning: 'Proposed learning architecture: policies and critics use sensor history, with independent offline evaluation.'}[view];
  $('module-map').replaceChildren();
  modules[view].forEach((module, i) => {
    const button = document.createElement('button'); button.type = 'button';
    button.textContent = `${i + 1}. ${module[0]}`; button.setAttribute('aria-controls', 'module-inspector');
    button.addEventListener('click', () => inspect(i)); $('module-map').append(button);
  });
  function inspect(index) {
    [...$('module-map').children].forEach((button, i) => button.setAttribute('aria-pressed', String(i === index)));
    const m = modules[view][index];
    $('module-inspector').innerHTML = `<h3>${m[0]}</h3><p>${m[1]}</p><dl><dt>Implementation</dt><dd>${m[2]}</dd><dt>Output</dt><dd>${m[3]}</dd><dt>Boundary</dt><dd>${m[4]}</dd></dl>`;
  }
  inspect(0);
}
document.querySelectorAll('[data-system]').forEach(button => button.addEventListener('click', () => renderSystem(button.dataset.system)));
renderSystem('hardware');
const toolStates = {
 pickup: ['translate(0 135)', 'translate(-180 -130)', true, 'The suction face meets the supported cable. The lower jaw stays clear of the desk. Schematic positions are illustrative.'],
 lift: ['translate(0 0)', 'translate(-180 -130)', true, 'Vacuum holds the lifted cable while the lower jaw remains parked. Real pickup must be confirmed with pressure and vision.'],
 deploy: ['translate(0 0)', 'translate(0 0)', true, 'The lower jaw moves beneath the cable while suction continues to support it. Deployment and clamping are separate axes.'],
 pinch: ['translate(0 0)', 'translate(0 -25)', true, 'The jaw applies mechanical preload. A real handoff requires jaw-load and tactile evidence before venting.'],
 vent: ['translate(0 0)', 'translate(0 -25)', false, 'Vacuum releases while mechanical preload remains. Observe retention; jaw position alone does not prove a grasp.']
};
function renderTool(state) {
  choose('data-tool', state); const s = toolStates[state];
  $('tool-assembly').setAttribute('transform', s[0]); $('lower-jaw').setAttribute('transform', s[1]);
  $('vacuum-lines').style.opacity = s[2] ? '1' : '0'; $('vacuum-label').textContent = s[2] ? 'Vacuum active' : 'Vacuum vented';
  $('tool-description').textContent = s[3];
}
document.querySelectorAll('[data-tool]').forEach(button => button.addEventListener('click', () => renderTool(button.dataset.tool)));
renderTool('pickup');
function sampling() {
 const fov = Number($('fov').value), pixels = Number($('image-width').value), scale = 1000 * fov / pixels;
 $('fov-value').textContent = `${fov} mm`; $('image-width-value').textContent = `${pixels} px`;
 $('pixel-scale').textContent = `${scale.toFixed(1)} µm / pixel`; $('clearance-pixels').textContent = `${(100 / scale).toFixed(1)} pixels across 0.10 mm`;
}
['fov','image-width'].forEach(id => $(id).addEventListener('input', sampling)); sampling();
fetch('assets/stages.json').then(response => { if (!response.ok) throw new Error('Stage manifest unavailable'); return response.json(); }).then(stages => {
 function strategy(mode) {
  choose('data-strategy', mode); $('stage-list').replaceChildren();
  stages[mode].forEach((stage, i) => {
   const button = document.createElement('button'); button.type = 'button';
   button.textContent = `${String(i + 1).padStart(2, '0')}  ${stage.title}`;
   button.setAttribute('aria-controls', 'stage-detail'); button.addEventListener('click', () => select(i)); $('stage-list').append(button);
  });
  function select(index) {
   const stage = stages[mode][index];
   [...$('stage-list').children].forEach((button, i) => button.setAttribute('aria-pressed', String(index === i)));
   $('stage-video').pause(); $('stage-video').src = stage.clip; $('stage-video').load();
   $('stage-title').textContent = stage.title;
   $('stage-facts').textContent = `${mode.toUpperCase()} / STAGE ${index + 1} OF ${stages[mode].length} / ${(stage.frames / 30).toFixed(2)} s clip / historical simulation`;
   $('stage-explanation').innerHTML = `<dt>Recorded action</dt><dd>${stage.explanation}</dd><dt>Future sensing</dt><dd>${stage.sensing}</dd><dt>Failure to test</dt><dd>${stage.risk}</dd>`;
   $('stage-download').href = stage.clip; $('stage-error').hidden = true;
  }
  select(0);
 }
 document.querySelectorAll('[data-strategy]').forEach(button => button.addEventListener('click', () => strategy(button.dataset.strategy)));
 $('stage-video').addEventListener('error', () => { $('stage-error').hidden = false; $('stage-error').textContent = 'This clip could not load. Use the clip link below or return to the experiment recordings.'; });
 strategy('direct');
}).catch(() => { $('stage-error').hidden = false; $('stage-error').textContent = 'The stage manifest could not load. The historical recordings remain available on the Experiments page.'; });
const chapters = [...document.querySelectorAll('.chapter')];
let scrollPending = false;
function updateChapter() {
  scrollPending = false;
  const passed = chapters.filter(section => section.getBoundingClientRect().top <= 180);
  const id = (passed[passed.length - 1] || chapters[0]).id;
  document.querySelectorAll('.toc a').forEach(link => {
    if (link.hash === `#${id}`) link.setAttribute('aria-current', 'location');
    else link.removeAttribute('aria-current');
  });
  $('chapter-select').value = id;
}
window.addEventListener('scroll', () => {
  if (!scrollPending) { scrollPending = true; requestAnimationFrame(updateChapter); }
}, {passive: true});
updateChapter();
