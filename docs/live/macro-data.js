(() => {
  const root = document.querySelector('#macro-data-review');
  if (!root) return;
  const select = root.querySelector('select');
  const image = root.querySelector('img');
  const caption = root.querySelector('[data-caption]');
  const mode = root.querySelector('input');
  fetch('macro-data/manifest.json').then(r => { if (!r.ok) throw Error('Manifest unavailable'); return r.json(); }).then(data => {
    data.frames.forEach((f, i) => {
      const option = document.createElement('option');
      option.value = i;
      option.textContent = `${f.group} ${f.scene + 1} · ${f.condition.replaceAll('_', ' ')} · ${f.setback_mm} mm setback`;
      select.append(option);
    });
    const show = () => {
      const frame = data.frames[Number(select.value)];
      image.src = `macro-data/${mode.checked ? frame.offline : frame.rgb}`;
      image.alt = `${mode.checked ? 'Offline label overlay' : 'Rendered camera RGB'}: ${frame.condition}, ${frame.setback_mm} mm setback`;
      caption.textContent = `${mode.checked ? 'Offline geometry labels; no learned prediction.' : 'Raw rendered camera view.'} Leading-band visible pixels: ${frame.visible_pixels[3].toLocaleString()}. Static scene; no insertion motion.`;
    };
    select.addEventListener('change', show); mode.addEventListener('change', show); show();
  }).catch(error => { caption.textContent = `Review could not load: ${error.message}. Please reload.`; });
})();
