'use strict';
const themeButton = document.querySelector('#theme-toggle');
let theme = 'system';
try { theme = localStorage.getItem('ffc-theme') || 'system'; } catch (_) {}
function applyTheme() {
  if (theme === 'system') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  themeButton.textContent = `Theme: ${theme}`;
}
themeButton.addEventListener('click', () => {
  theme = {system: 'dark', dark: 'light', light: 'system'}[theme] || 'system';
  try { localStorage.setItem('ffc-theme', theme); } catch (_) {}
  applyTheme();
});
applyTheme();
const pretty = value => value.replaceAll('_', ' ');
const percent = value => value === null ? '—' : `${(value * 100).toFixed(1)}%`;
const addRow = (parent, values) => {
  const row = document.createElement('tr');
  for (const value of values) { const cell = document.createElement('td'); cell.textContent = value; row.append(cell); }
  parent.append(row);
};
async function loadExperiment() {
  try {
    const ready = await fetch('data/ready.json', {cache:'no-store'});
    if (!ready.ok) { setTimeout(loadExperiment, 5000); return; }
    const response = await fetch('data/evaluation.json', {cache:'no-store'});
    if (!response.ok) { setTimeout(loadExperiment, 5000); return; }
    const report = await response.json();
    document.querySelector('#pending-evaluation').hidden = true;
    for(const id of ['inspect','results','video']) document.getElementById(id).hidden = false;
    document.querySelector('#inspect-action').href = '#inspect';
    document.querySelector('#inspect-action').textContent = 'Inspect the predictions';
    const groups = report.groups;
    document.querySelector('#score-iou').textContent = percent(groups.split.test.foreground_mean_iou);
    for (const name of ['cable', 'contacts', 'stiffener', 'csi', 'dsi']) {
      addRow(document.querySelector('#class-results'), [name === 'csi' ? 'CSI camera socket' : name === 'dsi' ? 'DSI display socket' : pretty(name), ...[groups.split.test, groups.split.challenge, groups.camera_id.desk, groups.camera_id.board].map(g => percent(g.iou[name]))]);
    }
    for (const [name, g] of Object.entries(groups.condition)) {
      if (name === 'ordinary') continue;
      const c = g.objects.csi;
      addRow(document.querySelector('#challenge-results'), [pretty(name), g.images, percent(g.foreground_mean_iou), `${c.false_positive_frames} / ${c.absent_frames}`]);
    }
    const csi = groups.all.objects.csi;
    document.querySelector('#object-summary').textContent = `Across all 112 images, CSI was detected in ${csi.detected_frames} of ${csi.visible_frames} eligible views; ${csi.localized_within_12px} had the largest-region centroid within 12 pixels of the labelled centroid. There were ${csi.false_positive_frames} CSI detections among ${csi.absent_frames} views without an eligible labelled CSI region.`;
    document.querySelector('#latency-summary').textContent = `Isolated CPU model inference: ${report.cpu_latency_ms.median.toFixed(0)} ms median and ${report.cpu_latency_ms.p95.toFixed(0)} ms at the 95th percentile. Timing includes tensor inference and mask extraction, but excludes image decoding, connected-component review and file output.`;
    const caseSelect = document.querySelector('#case-select');
    const cameraSelect = document.querySelector('#camera-select');
    const frames = new Map(report.frames.map(f => [f.file.replace('.png',''), f]));
    for (const frame of report.frames.filter(f => f.camera_id === 'desk')) {
      const id = frame.file.split('-')[0];
      const option = document.createElement('option'); option.value = id; option.textContent = `${id} · ${pretty(frame.condition)}`; caseSelect.append(option);
    }
    let mode = 'rgb';
    const img = document.querySelector('#view-image');
    function update() {
      const id = `${caseSelect.value}-${cameraSelect.value}`;
      const frame = frames.get(id);
      const label = {rgb:'Camera RGB', prediction:'Learned prediction', truth:'Offline simulator annotation'}[mode];
      document.querySelector('#view-error').hidden = true;
      img.src = `assets/${id}-${mode}.webp`;
      img.alt = `${label}: ${pretty(frame.condition)}, scene ${caseSelect.value}, ${cameraSelect.value} camera.`;
      document.querySelector('#view-caption').textContent = `${label} / ${pretty(frame.condition)} / ${frame.split} scene ${caseSelect.value} / ${cameraSelect.value} camera. ${mode === 'truth' ? 'This annotation is available only to the offline scorer, never to the inference worker.' : mode === 'prediction' ? 'Mask from the frozen RGB model. No simulator labels were accessible during inference.' : 'Rendered RGB received by the model; no annotation overlay.'}`;
    }
    img.addEventListener('error', () => { document.querySelector('#view-error').hidden = false; });
    caseSelect.addEventListener('change', update); cameraSelect.addEventListener('change', update);
    for (const button of document.querySelectorAll('[data-mode]')) button.addEventListener('click', () => {
      mode = button.dataset.mode;
      for (const b of document.querySelectorAll('[data-mode]')) b.setAttribute('aria-pressed', String(b === button));
      update();
    });
    update();
  } catch (error) { document.querySelector('#load-error').hidden = false; console.error(error); setTimeout(loadExperiment, 5000); }
}
loadExperiment();
