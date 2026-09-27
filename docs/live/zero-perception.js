/* Saved evaluation is independent of the temporary live renderer. */
(async () => {
  const status = document.getElementById('zp-status');
  if (!status) return;
  const base = 'zero-perception/';
  const percent = x => x == null ? '—' : `${(100 * x).toFixed(1)}%`;
  const row = (target, values) => {
    const tr = document.createElement('tr');
    for (const value of values) { const td = document.createElement('td'); td.textContent = value; tr.append(td); }
    document.getElementById(target).append(tr);
  };
  try {
    const [manifest, report] = await Promise.all(['manifest.json', 'evaluation.json'].map(async name => {
      const r = await fetch(base + name); if (!r.ok) throw new Error(`${name}: HTTP ${r.status}`); return r.json();
    }));
    const frames = manifest.frames, select = document.getElementById('zp-frame'), layer = document.getElementById('zp-layer');
    frames.forEach((f, i) => { const o = document.createElement('option'); o.value = i; o.textContent = `${i + 1}/${frames.length} · Scene ${f.scene} · ${f.camera_id} · ${f.condition.replaceAll('_', ' ')}`; select.append(o); });
    const update = () => {
      const f = frames[Number(select.value)], url = base + f[layer.value];
      const img = document.getElementById('zp-image'); img.src = url; img.alt = `${layer.selectedOptions[0].textContent}: scene ${f.scene}, ${f.camera_id}, ${f.condition}`;
      document.getElementById('zp-link').href = url;
      status.textContent = `${f.split === 'challenge' ? 'Challenge' : 'Test'} · ${select.selectedOptions[0].textContent} · ${layer.selectedOptions[0].textContent}`;
    };
    select.addEventListener('change', update); layer.addEventListener('change', update);
    document.getElementById('zp-prev').onclick = () => { select.value = (Number(select.value) + frames.length - 1) % frames.length; update(); };
    document.getElementById('zp-next').onclick = () => { select.value = (Number(select.value) + 1) % frames.length; update(); };
    document.getElementById('zp-image').addEventListener('error', () => { status.textContent = 'This review image failed to load. Use the manifest link below to inspect the saved asset.'; });
    update();
    for (const [key, label] of [['dino', 'DINOv2 + RGB detail'], ['rgb_ablation', 'RGB-only ablation']]) {
      const m = report.models[key]; row('zp-models', [label, ...['test', 'challenge', 'all'].map(g => percent(m.groups[g].foreground_mean_iou)), `${m.latency_ms_excluding_first.median.toFixed(1)} ms`]);
    }
    for (const c of report.models.dino.groups.all.classes) row('zp-classes', [c.class.replaceAll('_', ' '), percent(c.iou), `${c.visible_iou_at_least_half} / ${c.visible_images}`, `${c.false_positive_images} / ${c.absent_images}`]);
    for (const [name, g] of Object.entries(report.models.dino.groups)) {
      if (['all', 'test', 'challenge', 'entrance', 'offset'].includes(name)) continue;
      row('zp-conditions', [name.replaceAll('_', ' '), g.images, percent(g.foreground_mean_iou), percent(report.models.rgb_ablation.groups[name].foreground_mean_iou)]);
    }
  } catch (e) { status.textContent = `The saved review could not load (${e.message}). Please reload or open the evaluation links below.`; }
})();
