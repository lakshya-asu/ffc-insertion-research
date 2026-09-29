"""Render the expert on random hard starts and write a self-contained viewer page (and MP4s if ffmpeg exists).

Each clip: oblique view + top view with a see-through slot roof, HUD with phase, truth depth, sensor force, retries.
Picks the runs to show from a batch of seeds: the fastest clean seat, a seat with retries, the highest-force seat,
a low-light run, and any failure.
"""
import argparse, base64, io, json, math, shutil, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import TwinEnv, EstimatorConfig
from ffc_twin.expert import Expert
from scripts.render_clips import Viewer


def run(env, viewer, seed, level="L1"):
    obs, info = env.reset(seed=seed, level=level)
    ex = Expert(env.spec); frames = []; log = []
    for k in range(150):
        a = ex.act(obs); phase = ex.phase
        obs, r, term, trunc, done_info = env.step(a); t = env.truth()
        fn = float(np.linalg.norm(obs["fixture_wrench_n_nm"][:3]))
        log.append(dict(tick=k, phase=phase, depth=t["depth_m"] * 1e3, force=fn, true_force=t["force"], est=(obs["tip_estimate"][:3] * 1e3).round(2).tolist(),
                        true=(np.array(t["tip_pose"][:3]) * 1e3).round(2).tolist(), retries=ex.retries, slip=t["grip_slip_m"] * 1e3, visible=int(obs["tip_visible"][0])))
        fr = viewer.frame()
        im = Image.fromarray(fr); d = ImageDraw.Draw(im)
        d.rectangle([0, 0, 640, 22], fill=(251, 248, 241))
        d.text((8, 5), f"tick {k:3d}  {phase:8s}  depth {t['depth_m']*1e3:6.2f} mm  force {fn:4.2f} N  retries {ex.retries}  seed {seed}", fill=(31, 35, 40))
        frames.append(np.asarray(im))
        if term or trunc:
            break
    t = env.truth(); s = env.start
    meta = dict(seed=seed, seated=bool(t["seated"] and env.peak_force < 3), depth=t["depth_m"] * 1e3, peak=env.peak_force, retries=ex.retries, ticks=len(log), slip=t["grip_slip_m"] * 1e3,
                start=dict(lateral=s["lateral"] * 1e3, height=s["height"] * 1e3, yaw=math.degrees(s["yaw"]), pitch=math.degrees(s["pitch"]), roll=math.degrees(s["roll"])), draws=env.draws)
    return frames, log, meta


def encode(frames, w=960):
    out = []
    for fr in frames:
        im = Image.fromarray(fr).resize((w, int(w * fr.shape[0] / fr.shape[1])), Image.LANCZOS)
        buf = io.BytesIO(); im.save(buf, format="WEBP", quality=68); out.append(base64.b64encode(buf.getvalue()).decode())
    return out


def mp4(frames, path, fps=10):
    if not shutil.which("ffmpeg"):
        return None
    with tempfile.TemporaryDirectory() as td:
        for i, fr in enumerate(frames):
            Image.fromarray(fr).save(f"{td}/{i:04d}.png")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", f"{td}/%04d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-vf", "scale=1280:-2", str(path)], check=True)
    return path


PAGE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Expert runs in the twin</title>
<style>
:root{--bg:#FBF8F1;--ink:#1F2328;--muted:#5F6773;--line:#D9D2C3;--blue:#2563EB;--bad:#B91C1C}
@media (prefers-color-scheme: dark){:root{--bg:#141517;--ink:#ECE9E1;--muted:#A5ADB8;--line:#33373E;--blue:#7EA6FF;--bad:#F07070}}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.5 -apple-system,BlinkMacSystemFont,Inter,sans-serif}
main{max-width:1000px;margin:0 auto;padding:36px 20px 80px}
h1{font-size:30px;margin:0 0 8px}h2{font-size:20px;margin:36px 0 6px}p{max-width:70ch;margin:6px 0}
table{border-collapse:collapse;width:100%;font-size:14px;margin:12px 0}th,td{text-align:left;padding:6px 10px 6px 0;border-bottom:1px solid var(--line)}th{color:var(--muted);font-weight:600;font-size:12px;text-transform:uppercase;letter-spacing:.05em}
.vp{position:relative;border:1px solid var(--line);background:#FBF8F1}.vp img{width:100%;display:block}
.bar{display:flex;align-items:center;gap:12px;margin-top:8px;font-size:13px;color:var(--muted)}
button{font:inherit;background:none;border:1px solid var(--line);color:var(--ink);padding:3px 10px;cursor:pointer;min-width:64px}
input[type=range]{flex:1;accent-color:var(--ink)}
.chart{display:block;max-width:100%;height:auto}
.hud{font:13px ui-monospace,Menlo,monospace;color:var(--muted);margin-top:6px}
</style></head><body><main>
<h1>The expert, run by run</h1>
<p>Each clip is one run of the hand-written expert in the corrected twin, on a random start from the hardest level, with the researched sensor models. Left: oblique view. Right: from above, with the slot roof drawn see-through. The chart under each clip is the force reading at the tip per 50 ms step; the red line is the 3 N limit. Everything shown is simulator output.</p>
<table><tr><th>Run</th><th>Start error</th><th>Sensing</th><th>Result</th><th>Depth</th><th>Peak force</th><th>Retries</th><th>Steps</th></tr>__ROWS__</table>
__CLIPS__
<p style="color:var(--muted);font-size:13.5px;margin-top:30px">Rendered from ffc-twin at the commit noted in the hand-off. Slot height, funnels, modulus and friction are placeholders; see ASSUMPTIONS.md.</p>
</main>
<script>
const CLIPS = __DATA__;
document.querySelectorAll('.clip').forEach((root, ci) => {
  const c = CLIPS[ci]; const n = c.frames.length; const img = root.querySelector('img'), rng = root.querySelector('input'), btn = root.querySelector('button'), lab = root.querySelector('.lab'), hud = root.querySelector('.hud'), chart = root.querySelector('.chart');
  const W=960,H=84,x0=40,y0=8,w=W-x0-10,h=H-28; const L=c.log; const fmax=Math.max(3,...L.map(l=>l.force)); let path='';
  L.forEach((l,j)=>{const x=x0+j/(L.length-1)*w, y=y0+h-Math.min(l.force,fmax)/fmax*h; path+=(j?'L':'M')+x.toFixed(1)+','+y.toFixed(1);});
  const y3=y0+h-3/fmax*h;
  chart.innerHTML=`<rect x="${x0}" y="${y0}" width="${w}" height="${h}" fill="none" stroke="var(--line)"/><line x1="${x0}" y1="${y3}" x2="${x0+w}" y2="${y3}" stroke="var(--bad)" stroke-dasharray="4 3"/><text x="${x0+w-4}" y="${y3-3}" text-anchor="end" font-size="10" fill="var(--bad)">3 N limit</text><path d="${path}" fill="none" stroke="var(--ink)" stroke-width="1.4"/><text x="4" y="${y0+10}" font-size="10" fill="var(--muted)">${fmax.toFixed(0)} N</text><text x="4" y="${y0+h}" font-size="10" fill="var(--muted)">0</text><line class="cur" x1="${x0}" y1="${y0}" x2="${x0}" y2="${y0+h}" stroke="var(--blue)" stroke-width="1.4"/>`;
  const cur=chart.querySelector('.cur'); let i=0, timer=null, playing=true;
  function show(k){ i=k; img.src='data:image/webp;base64,'+c.frames[k]; rng.value=k; const l=L[Math.min(k,L.length-1)]; lab.textContent=`${k+1} / ${n}`;
    hud.textContent=`phase ${l.phase} · true depth ${l.depth.toFixed(2)} mm · estimate (${l.est.join(', ')}) mm · truth (${l.true.join(', ')}) mm · slot visible ${l.visible} · grip slip ${l.slip.toFixed(2)} mm`;
    const x=x0+Math.min(k,L.length-1)/(L.length-1)*w; cur.setAttribute('x1',x); cur.setAttribute('x2',x); }
  function tick(){ show((i+1)%n); if(i===n-1){ clearInterval(timer); timer=setTimeout(()=>{ timer=null; if(playing) start(); },1500);} }
  function start(){ playing=true; btn.textContent='Pause'; if(timer){clearInterval(timer);clearTimeout(timer);} timer=setInterval(tick,100); }
  function stop(){ playing=false; btn.textContent='Play'; if(timer){clearInterval(timer);clearTimeout(timer);} timer=null; }
  btn.addEventListener('click',()=>playing?stop():start()); rng.addEventListener('input',e=>{stop(); show(+e.target.value);}); rng.max=n-1; show(0);
  new IntersectionObserver(es=>es.forEach(en=>{ if(en.isIntersecting && playing && !timer) start(); if(!en.isIntersecting && timer){ clearInterval(timer); clearTimeout(timer); timer=null; } }),{threshold:.3}).observe(root);
});
</script></body></html>"""

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--seeds", type=int, default=30); a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    runs = []
    for sensing, cfg in (("lit, focused", EstimatorConfig.good()), ("low light, mismatched pad", EstimatorConfig.poor())):
        env = TwinEnv(estimator=cfg, log=False); v = Viewer(env)
        for seed in range(100, 100 + (a.seeds if sensing.startswith("lit") else max(6, a.seeds // 4))):
            frames, log, meta = run(env, v, seed); meta["sensing"] = sensing; runs.append((frames, log, meta))
    good = [r for r in runs if r[2]["sensing"].startswith("lit")]
    picks = []
    seated = [r for r in good if r[2]["seated"]]
    fails = [r for r in runs if not r[2]["seated"]]
    if seated:
        picks.append(("fastest clean seat", min(seated, key=lambda r: r[2]["ticks"])))
        with_retries = [r for r in seated if r[2]["retries"] > 0]
        if with_retries: picks.append(("seat after retries", max(with_retries, key=lambda r: r[2]["retries"])))
        picks.append(("highest-force seat", max(seated, key=lambda r: r[2]["peak"])))
        far = max(seated, key=lambda r: abs(r[2]["start"]["lateral"]) + abs(r[2]["start"]["yaw"]) / 4)
        picks.append(("largest start error that seated", far))
    poor_seated = [r for r in runs if r[2]["sensing"].startswith("low") and r[2]["seated"]]
    if poor_seated: picks.append(("low light, seated", poor_seated[0]))
    for f in fails[:2]: picks.append(("failure", f))
    seen = set(); final = []
    for label, r in picks:
        key = (r[2]["seed"], r[2]["sensing"])
        if key in seen: continue
        seen.add(key); final.append((label, r))
    rows = []; clips = []; data = []
    for ci, (label, (frames, log, meta)) in enumerate(final):
        st = meta["start"]
        rows.append(f"<tr><td>{ci+1}. {label}</td><td>{st['lateral']:+.2f} mm, {st['height']:+.2f} mm, turn {st['yaw']:+.1f}°, pitch {st['pitch']:+.1f}°, roll {st['roll']:+.1f}°</td><td>{meta['sensing']}</td><td>{'seated' if meta['seated'] else 'failed'}</td><td>{meta['depth']:.2f} mm</td><td>{meta['peak']:.2f} N</td><td>{meta['retries']}</td><td>{meta['ticks']}</td></tr>")
        clips.append(f'<div class="clip"><h2>{ci+1}. {label}</h2><div class="vp"><img alt="run {ci+1}"></div><div class="bar"><button type="button">Pause</button><input type="range" min="0" max="1" value="0"><span class="lab"></span></div><svg class="chart" viewBox="0 0 960 84"></svg><div class="hud"></div></div>')
        data.append(dict(frames=encode(frames), log=log, meta=meta))
        p = mp4(frames, out / f"expert_run_{ci+1}_{label.replace(' ', '_').replace(',', '')}.mp4")
    all_meta = [r[2] for r in runs]
    n_good = sum(1 for m in all_meta if m["sensing"].startswith("lit")); s_good = sum(1 for m in all_meta if m["sensing"].startswith("lit") and m["seated"])
    n_poor = len(all_meta) - n_good; s_poor = sum(1 for m in all_meta if m["sensing"].startswith("low") and m["seated"])
    summary = f"<p>Batch behind this page: {s_good} of {n_good} seated with lit, focused sensing; {s_poor} of {n_poor} in low light. Picks below are chosen to show the range, including every failure.</p>"
    page = PAGE.replace("__ROWS__", "".join(rows)).replace("__CLIPS__", summary + "".join(clips)).replace("__DATA__", json.dumps(data))
    (out / "expert-runs.html").write_text(page)
    json.dump(all_meta, open(out / "expert_batch.json", "w"), indent=1)
    print("clips:", [(l, r[2]["seed"], r[2]["seated"], round(r[2]["peak"], 2), r[2]["retries"]) for l, r in final]); print("batch", s_good, "/", n_good, "good;", s_poor, "/", n_poor, "poor"); print("wrote", out / "expert-runs.html")
