/* Presentation-only controls. No simulator, model execution or actuator endpoint. */
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const legacy = new Set(['e040-direct-pgs', 'e040-fixture-pgs', 'e039-pgs-transfer', 'e043-selected-static', 'e044-pgs-tool-smoke', 'e023-native-workcell', 'comparison']);
  const forwardArchive = () => {
    if (legacy.has(location.hash.slice(1))) location.replace('experiments.html' + location.hash);
  };
  forwardArchive();
  window.addEventListener('hashchange', forwardArchive);

  let theme = 'system';
  try { theme = localStorage.getItem('ffc-theme') || 'system'; } catch (_) { /* Storage is optional. */ }
  if (!['system', 'light', 'dark'].includes(theme)) theme = 'system';
  const applyTheme = () => {
    if (theme === 'system') delete document.documentElement.dataset.theme;
    else document.documentElement.dataset.theme = theme;
    $('theme').textContent = `Theme: ${theme}`;
    $('theme').setAttribute('aria-label', `Color theme: ${theme}. Activate to change.`);
  };
  applyTheme(); $('theme').hidden = false;
  $('theme').addEventListener('click', () => {
    theme = ['system', 'light', 'dark'][(['system', 'light', 'dark'].indexOf(theme) + 1) % 3];
    applyTheme();
    try { localStorage.setItem('ffc-theme', theme); } catch (_) { /* Keep in-memory setting. */ }
  });

  const videos = {
    perception: {src:'demo/live-perception.mp4', poster:'demo/poster.jpg', kind:'Implemented · stationary scene', title:'Camera RGB meets learned entrance features.', description:'47 matched camera/prediction pairs over 38.8 seconds of acquisition time. The frozen model receives ROS observations, preserving their timestamps. This recording shows perception, not robot movement.', link:'demo/', linkText:'Open the live demo →'},
    motion: {src:'hardware/mounted-tool/cycle.mp4', poster:'hardware/mounted-tool/poster.jpg', kind:'Actual physics · empty-tool commissioning', title:'The mounted tool moves with the FR3.', description:'44.009 simulated seconds, 44,009 physics steps and 1,100 recorded frames. The shoulder moves out and back, then the tool exercises its two prismatic axes. This is the larger PQ12-based mechanism, without cable or PCB; the compact PGEA is not integrated.', link:'hardware/custom-gripper.html#mounted-motion', linkText:'Motion evidence and source hashes →'},
    cancel: {src:'hardware/mounted-tool/tool-cancel.mp4', poster:'hardware/mounted-tool/poster.jpg', kind:'Actual physics · cancellation test', title:'Cancel the deployment. Measure the hold.', description:'The tool is interrupted during deployment. Measured post-cancel drift was about 0.0066 mm in this one simulated test. This is evidence for the bounded controller under the declared model, not a hardware protective-stop guarantee.', link:'hardware/mounted-tool/tool-cancel-results.json', linkText:'Inspect the recorded result →'},
    mechanics: {src:'outputs/e043-selected-static/experiment.mp4', poster:'outputs/e043-selected-static/latest-frame.jpg', kind:'Historical physics · numerical benchmark', title:'Check the cable against a mechanics reference.', description:'The selected articulated cable reached 46.081 mm sag against a 46.132 mm nominal independent reference. This tests numerical agreement for assumed material properties; it does not calibrate the Pi cable or qualify insertion contact.', link:'experiments.html#e043-selected-static', linkText:'Benchmark, plot and assumptions →'}
  };
  const video = $('evidence-video');
  $('video-controls').hidden = false;
  $('video-controls').addEventListener('click', (event) => {
    const button = event.target.closest('button[data-video]');
    if (!button) return;
    const selected = videos[button.dataset.video];
    video.pause();
    video.poster = selected.poster;
    video.src = selected.src;
    video.setAttribute('aria-label', selected.title);
    video.load();
    $('video-kind').textContent = selected.kind;
    $('video-title').textContent = selected.title;
    $('video-description').textContent = selected.description;
    $('video-detail').href = selected.link;
    $('video-detail').textContent = selected.linkText;
    $('video-file').href = selected.src;
    $('video-status').textContent = 'Recording selected. Press play; no audio.';
    document.querySelectorAll('[data-video]').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
  });
  video.addEventListener('error', () => { $('video-status').textContent = 'The recording could not load. Use the video-file link, or open its detailed evidence page.'; });
  video.addEventListener('playing', () => { $('video-status').textContent = 'Playing a saved simulation recording. This player does not control the robot.'; });
  video.addEventListener('ended', () => { $('video-status').textContent = 'Recording complete. Select another experiment or follow its evidence link.'; });

  const cellViews = {
    placement: {src:'live/mount/03-placement.webp', caption:'The macro camera sits on a braced stand across the task, looking down at 45°. This is a static clearance and visibility study.', alt:'Pi Zero work area and opposite-side camera support'},
    cell: {src:'live/mount/03-cell.webp', caption:'The whole FR3 workcell. This scene retains the earlier offset tool; new gripper geometry needs its own visibility and clearance checks.', alt:'Whole FR3 cell with Pi board and camera stand'},
    macro: {src:'live/mount/03-macro.webp', caption:'The modeled view at the connector mouth. The cable is 0.3 mm away in this static authored pose; this is not a measured insertion or a motion target.', alt:'Static macro view of the Pi Zero connector mouth and cable end'}
  };
  $('cell-controls').hidden = false;
  $('cell-controls').addEventListener('click', event => {
    const button = event.target.closest('button[data-cell]');
    if (!button) return;
    const view = cellViews[button.dataset.cell];
    $('cell-image').srcset = '';
    $('cell-image').src = view.src;
    $('cell-image').alt = view.alt;
    $('cell-caption').textContent = view.caption;
    document.querySelectorAll('[data-cell]').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
  });
  $('cell-image').addEventListener('error', () => { $('cell-caption').textContent = 'This render could not load. The full mounting study is linked beside the viewer.'; });

  const benchmarks = {
    shape: {src:'outputs/e043-selected-static/static-shape-comparison.png', caption:'Selected PGS static benchmark: 46.081 mm sag versus 46.132 mm independent nonlinear reference, approximately 0.11% difference. The historical ribbon is 150 × 8 × 0.3 mm; these are not the current Pi Standard–Mini cable dimensions.', description:'The independent reference minimizes gravitational and elastic energy. Benchmark-only body damping helps reach equilibrium and is disallowed in handling runs. This static agreement does not establish dynamic forces, laminate properties or insertion contact.', source:'007-contact-and-transport.md'},
    segments: {src:'overview/mechanics/segment-convergence.png', caption:'Short-span articulated cable: timestep and solver-iteration sweep against the independently derived discrete-chain prediction. These are early numerical profiles, not the selected final full-span benchmark.', description:'The coarsest configuration sagged about 25 times its static prediction. With the same material stiffness, finer timesteps reduced the error to 4.6% at 0.0625 ms. This isolated numerical compliance instead of hiding it by changing material constants.', source:'005-numerical-validation.md'},
    shell: {src:'overview/mechanics/shell-convergence.png', caption:'Native surface FEM: mesh, timestep and iteration sensitivity against a homogeneous continuum cantilever reference. It is a mechanics comparison, not a successful native-FEM handling rollout.', description:'Measured sag/reference ratios ranged from about 1.32 to 1.91 across the reported valid configurations. Mesh refinement alone did not produce convergence. A request for 512 iterations exceeded the 255-iteration schema limit and was excluded.', source:'005-numerical-validation.md'}
  };
  $('benchmark-controls').hidden = false;
  $('benchmark-controls').addEventListener('click', event => {
    const button = event.target.closest('button[data-benchmark]');
    if (!button) return;
    const row = benchmarks[button.dataset.benchmark];
    $('benchmark-image').src = row.src;
    $('benchmark-image').alt = row.caption;
    $('benchmark-caption').textContent = row.caption;
    $('benchmark-explanation').textContent = row.description;
    $('benchmark-source').href = 'https://github.com/lakshya-asu/ffc-insertion-research/blob/main/experiments/' + row.source;
    document.querySelectorAll('[data-benchmark]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));
  });
  $('benchmark-image').addEventListener('error', () => { $('benchmark-caption').textContent='This plot could not load. The numerical audit below contains the measurements and interpretation.'; });

  const layerCaptions = {
    rgb: 'Raw camera RGB. Neither semantic colors nor object transforms are input to the learned estimator.',
    prediction: 'Learned labels over camera RGB. Colors mark predicted classes, not validated targets or calibrated confidence.',
    offline: 'Simulator annotation for offline supervision and scoring only. This image is not available to the runtime estimator.'
  };
  const descriptions = {
    ordinary:'An ordinary held-out pose. Check whether the two entrance rims and the cable leading band are separately visible.',
    near_entry:'The cable approaches the mouth. Small projected gaps and occlusion challenge rim separation.',
    short_grasp:'The grasp is close to the end. Fingers can hide the leading band; lack of visible tip evidence must not become a motion target.',
    focus_drift:'A focus-drift stress case. Blur can bias a rim even when much of the predicted region overlaps correctly.',
    board_rotation:'A rotated board tests changes in projected feature orientation.',
    bright:'A bright-light stress case. Line-fit residuals can stay small even when localization is biased.'
  };
  let frames = [], selectedIndex = 0, layer = 'rgb';
  const names = {1:'Upper rim', 2:'Lower rim', 3:'Leading band', 4:'Open slider', 5:'Closed slider'};
  function updateReview() {
    const frame = frames[selectedIndex];
    $('review-image').src = 'live/macro-model/' + frame[layer];
    $('review-image').alt = `Scene ${String(frame.index).padStart(3,'0')}, ${frame.condition.replaceAll('_',' ')}, ${layer === 'offline' ? 'offline scorer annotation' : layer}`;
    $('review-caption').textContent = layerCaptions[layer];
    $('review-kind').textContent = layer === 'offline' ? 'Offline scorer only · not a model input' : 'Fresh test after model freeze';
    $('review-title').textContent = `${String(frame.index).padStart(3,'0')} · ${frame.condition.replaceAll('_',' ')}`;
    $('review-description').textContent = descriptions[frame.condition] || 'A held-out synthetic stress scene. Compare the RGB, prediction and offline labels to see both successful regions and misses.';
    const facts = $('review-facts'); facts.replaceChildren();
    const label = document.createElement('p');
    label.textContent = 'Offline review: visible-class IoU'; facts.append(label);
    const dl = document.createElement('dl');
    for (const score of frame.metrics.scores.filter(s => s.visible)) {
      const row = document.createElement('div'), dt = document.createElement('dt'), dd = document.createElement('dd');
      dt.textContent = names[score.class_id]; dd.textContent = score.iou == null ? 'N/A' : (100*score.iou).toFixed(1)+'%';
      row.append(dt,dd); dl.append(row);
    }
    if (!dl.children.length) { const p=document.createElement('p');p.textContent='No foreground class passes the offline visibility threshold.';facts.append(p); }
    else facts.append(dl);
    const p = document.createElement('p');
    const m = frame.metrics;
    p.textContent = m.latch_visible ? `Slider: predicted ${m.predicted_slider_state}; authored ${m.authored_slider_state}${m.predicted_slider_state !== m.authored_slider_state ? '. This is a recorded failure.' : '.'}` : `Slider unobservable; prediction: ${m.predicted_slider_state}.`;
    facts.append(p);
    $('review-status').textContent = `Scene ${selectedIndex+1} of ${frames.length}. ${layer === 'offline' ? 'Offline annotation selected.' : layer === 'prediction' ? 'Prediction selected.' : 'RGB selected.'}`;
    document.querySelectorAll('[data-layer]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.layer===layer)));
  }
  $('review-image').addEventListener('error', () => { $('review-status').textContent = 'The selected image could not load. Try another scene or use the full evaluation link.'; });
  async function loadReview() {
    try {
      const response = await fetch('live/macro-model/manifest.json');
      if (!response.ok) throw new Error('Manifest unavailable');
      const manifest = await response.json();
      if (!Array.isArray(manifest.frames) || !manifest.frames.length) throw new Error('Empty review');
      frames = manifest.frames;
      $('review-scene').replaceChildren(...frames.map((frame,i) => {
        const option = document.createElement('option'); option.value=String(i);
        option.textContent=`${String(frame.index).padStart(3,'0')} · ${frame.condition.replaceAll('_',' ')}${frame.index===43?' · slider miss':''}`;return option;
      }));
      $('review-scene').disabled=false;
      $('review-controls').hidden=false;
      $('review-scene').addEventListener('change',()=>{selectedIndex=Number($('review-scene').value);updateReview();});
      $('review-controls').addEventListener('click',event=>{const button=event.target.closest('button[data-layer]');if(button){layer=button.dataset.layer;updateReview();}});
      updateReview();
    } catch (_) {
      $('review-status').textContent='The review manifest could not load. The first RGB image remains available; use the full evaluation link to inspect the published results.';
    }
  }
  loadReview();
})();
