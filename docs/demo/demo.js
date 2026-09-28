(() => {
  const image = document.querySelector('#view'), video = document.querySelector('#recording');
  const status = document.querySelector('#status'), detail = document.querySelector('#detail');
  let base = '', mode = 'live', latest = null, connected = false, busy = false;
  document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => {
    mode = button.dataset.mode; latest = null;
    document.querySelectorAll('[data-mode]').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
    video.hidden = mode !== 'recording'; image.hidden = mode === 'recording'; video.pause();
    image.src = mode === 'cell' ? '../architecture/assets/workcell.webp' : 'poster.jpg';
    image.alt = mode === 'cell' ? 'Whole simulated robot cell' : 'Camera RGB beside learned feature predictions';
    if (mode === 'recording') {
      status.textContent = 'Recorded live ROS inference · static simulated scene · not a current camera feed';
      detail.textContent = 'Playback preserves the recorded acquisition cadence. Press play to begin.';
    } else { status.textContent = 'Connecting; a recorded image is shown.'; poll(); }
  }));
  async function poll() {
    if (!base || mode === 'recording' || busy) return;
    busy = true;
    const requestedMode = mode;
    try {
      const path = mode === 'live' ? '/api/perception' : '/api/status';
      const response = await fetch(base + path, {cache: 'no-store', signal: AbortSignal.timeout(4000)});
      if (!response.ok) throw Error('Feed unavailable');
      const data = await response.json();
      if (mode !== requestedMode) return;
      if (mode === 'live') { image.src = data.image; latest = data; }
      else {
        const frame = data.feeds.workcell;
        if (!frame) throw Error('Cell frame unavailable');
        image.src = base + '/api/frame/workcell?t=' + frame.sequence;
        latest = {acquired_at: frame.captured_at};
      }
      connected = true; age();
    } catch (_) {
      connected = false;
      if (mode === requestedMode) { status.textContent = 'Live connection unavailable. Use Recorded fallback; any displayed live image is now saved.'; }
    } finally { busy = false; }
  }
  function age() {
    if (!latest || mode === 'recording') return;
    const seconds = Math.max(0, Date.now()/1000 - latest.acquired_at);
    if (connected) status.textContent = mode === 'live'
      ? `${seconds <= .5 ? 'Fresh feature frame' : 'Saved / expired feature frame'} · acquired ${seconds.toFixed(1)} s ago · motion disabled`
      : `Live simulated cell · last render ${seconds.toFixed(1)} s ago`;
    detail.textContent = mode === 'live'
      ? `Model ${latest.model_sha256.slice(0,12)} · inference ${latest.processing_ms.toFixed(1)} ms · client-computed age; browser clock must be accurate. Pixel classes are not verified poses.`
      : 'A stationary rendered cell. The robot is not executing an insertion trajectory.';
  }
  fetch('../live/connection.json', {cache:'no-store'}).then(r => r.json()).then(c => {
    base = location.hostname === '127.0.0.1' || location.hostname === 'localhost' ? location.origin : c.base_url;
    poll();
  }).catch(() => { status.textContent = 'Live connection settings unavailable. Recorded fallback is ready.'; });
  // One request at a time, including on slow tunnel connections.
  setInterval(poll, 1000);
  setInterval(age, 200);
})();
