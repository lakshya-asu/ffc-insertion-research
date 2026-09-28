(() => {
  const root = document.querySelector('#macro-model-review');
  if (!root) return;
  const scene = root.querySelector('[data-scene]');
  const mode = root.querySelector('[data-mode]');
  const image = root.querySelector('img');
  const caption = root.querySelector('figcaption');
  fetch('macro-model/manifest.json').then(r => { if (!r.ok) throw Error('Manifest unavailable'); return r.json(); }).then(data => {
    data.frames.forEach((f, i) => {
      const option = document.createElement('option'); option.value = i;
      option.textContent = `${i + 1} / ${data.frames.length} · ${f.condition.replaceAll('_', ' ')}`;
      scene.append(option);
    });
    function show() {
      const frame = data.frames[Number(scene.value)];
      image.src = `macro-model/${frame[mode.value]}`;
      image.alt = `${mode.options[mode.selectedIndex].text}: ${frame.condition}`;
      const m = frame.metrics;
      caption.textContent = `Scene ${frame.index + 1}. Slider prediction: ${m.predicted_slider_state}; offline label: ${m.latch_visible ? m.authored_slider_state : 'unobservable'}. Static simulated image; no verified pose or motion permission.`;
    }
    scene.addEventListener('change', show); mode.addEventListener('change', show); show();
  }).catch(error => { caption.textContent = `Review could not load: ${error.message}`; });
})();
