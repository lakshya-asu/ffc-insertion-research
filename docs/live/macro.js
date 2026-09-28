(async () => {
  const status = document.querySelector('#macro-status');
  if (!status) return;
  try {
    const response = await fetch('macro/manifest.json');
    if (!response.ok) throw new Error(`Manifest HTTP ${response.status}`);
    const {frames} = await response.json();
    const select = document.querySelector('#macro-case');
    const mode = document.querySelector('#macro-mode');
    frames.forEach((f, i) => select.add(new Option(`${f.placement} · ${f.gap_mm} mm gap`, i)));
    select.value = String(frames.length - 1);
    function update() {
      const frame = frames[Number(select.value)];
      const src = `macro/${frame[mode.value]}`;
      const im = document.querySelector('#macro-image');
      im.src = src;
      im.alt = `${frame.placement}, ${frame.gap_mm} mm cable gap, ${mode.selectedOptions[0].text}`;
      im.width = mode.value === 'overview' ? 1280 : 2448;
      im.height = mode.value === 'overview' ? 960 : 2048;
      document.querySelector('#macro-full').href = src;
      status.textContent = `${frame.placement}. Gap ${frame.gap_mm} mm. Static rendered RGB; no inference or robot motion.`;
    }
    select.addEventListener('change', update);
    mode.addEventListener('change', update);
    update();
  } catch (error) {
    status.textContent = `Could not load the macro study: ${error.message}. The saved image is still available below.`;
  }
})();
