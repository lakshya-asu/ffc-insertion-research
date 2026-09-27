'use strict';
const views = {
  hardware: ['Pi 4, Camera Module 3 and loose 200 mm ribbon laid out separately on the desk.', 'The cable starts loose on the desk. The camera module is shown separately; neither end is connected in this review scene.'],
  pi4: ['Oblique view of the imported Pi 4 Model B.', 'Pi 4 community STEP: the major connectors and chips are present. Small passives, board markings and measured finishes remain incomplete.'],
  'pi4-top': ['Top view of the Pi 4 board and its two ribbon sockets.', 'The camera socket lies between micro HDMI and the audio jack. The socket on the opposite short edge is the display connector.'],
  'csi-macro': ['Close view into the upright CSI socket from the imported Pi 4 CAD.', 'The upright camera socket replaces the old horizontal slot. Latch travel, contact compliance and open-state geometry have not been qualified.'],
  camera3: ['Official Camera Module 3 simplified CAD with the lens facing up.', 'Official normal-FoV Camera Module 3 STEP, rotated lens-up. The review applies a black lens-assembly material; optical response is not simulated.'],
  'cable-tip': ['Fifteen separate exposed conductors at the camera cable end.', '15 contacts at 1 mm pitch, 0.7 mm conductor width and 5 mm exposed length. The opposite cable end exposes contacts on the other face.'],
  workcell: ['Imported Raspberry Pi hardware and cable on the FR3 workcell desk.', 'The assemblies are loaded in the FR3 workcell. Robot and cable remain stationary; this is a visual asset inspection.']
};
const img = document.querySelector('#asset-image');
for (const button of document.querySelectorAll('[data-view]')) button.addEventListener('click', () => {
  const [alt, caption] = views[button.dataset.view];
  document.querySelector('#image-error').hidden = true;
  img.src = `assets/${button.dataset.view}.webp`;
  img.alt = alt;
  document.querySelector('#asset-caption').textContent = caption;
  for (const b of document.querySelectorAll('[data-view]')) b.setAttribute('aria-pressed', String(b === button));
});
img.addEventListener('error', () => { document.querySelector('#image-error').hidden = false; });
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
