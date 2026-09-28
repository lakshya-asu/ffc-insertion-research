(() => {
  const select = document.querySelector('#alignment-scene');
  const image = document.querySelector('#alignment-image');
  const caption = document.querySelector('#alignment-caption');
  fetch('alignment/manifest.json').then(r => { if (!r.ok) throw Error('Manifest unavailable'); return r.json(); }).then(data => {
    data.frames.forEach((frame, i) => {
      const option = document.createElement('option'); option.value = i;
      option.textContent = `${i + 1} · ${frame.condition.replaceAll('_', ' ')} · ${frame.status === 'abstain' ? 'abstain' : 'image measurement'}`;
      select.append(option);
    });
    function show() {
      const frame = data.frames[Number(select.value)]; image.src = 'alignment/' + frame.image;
      image.alt = `Predicted edge fits and offline reference projections: ${frame.condition}`;
      const m = frame.measurement;
      caption.textContent = m ? `Projected normal separation: ${m.projected_normal_separation_px.toFixed(1)} px. Relative image angle: ${m.relative_image_angle_deg.toFixed(2)}°. No depth, calibrated uncertainty or motion authorization.` : `Abstained: ${frame.reasons.join('; ')}.`;
    }
    select.addEventListener('change', show); show();
  }).catch(error => { caption.textContent = `Geometry review unavailable: ${error.message}`; });
})();
