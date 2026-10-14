"""Research register page: every outside method the cell uses, with the concept in plain words, how and where we use it, the sources
(papers, project pages with the authors' own videos, code) and the evidence we produced. Built from docs/tracker/research.json.
usage: python3 scripts/ffc_research_register.py [out.html]"""
import json, sys, html, datetime
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Desktop" / "Synphony-BOM" / "FFC-research-register.html"
D = json.loads((ROOT / "docs/tracker/research.json").read_text()); e = html.escape
COL = {"in use": "#2f8f5b", "planned": "#1f6fb2", "watching": "#b7791f", "rejected": "#888"}
cards = []
for r in D["entries"]:
    media = "".join(f"<figure><img src='{e(m['src'])}' alt=''><figcaption>{e(m.get('text',''))}</figcaption></figure>" if m.get("kind") == "image" else f"<p class='n'><a href='{e(m['url'])}'>{e(m['text'])}</a></p>" for m in r.get("media", []))
    cards.append(f"""<section class='card' data-status='{e(r['status'])}'><div class='head'><h2>{e(r['title'])}</h2><span class='st' style='background:{COL.get(r['status'], '#888')}'>{e(r['status'])}</span></div>
<div class='cols'><div><div class='eyebrow'>The concept</div><p>{e(r['concept'])}</p></div>
<div><div class='eyebrow'>How and where we use it</div><p><b>{e(r['where'])}.</b> {e(r['use'])}</p></div>
<div><div class='eyebrow'>Sources</div><ul>{''.join(f"<li><a href='{e(s['url'])}'>{e(s['text'])}</a></li>" for s in r['sources'])}</ul>{media}</div>
<div><div class='eyebrow'>Evidence we checked or produced</div><p>{e(r['evidence']) if r.get('evidence') else '<span class=n>none yet</span>'}</p></div></div></section>""")
counts = {}
for r in D["entries"]:
    counts[r["status"]] = counts.get(r["status"], 0) + 1
page = f"""<!doctype html><html><head><meta charset='utf-8'><title>FFC cell research register</title>
<style>body{{font:15px/1.55 -apple-system,Inter,Helvetica,sans-serif;margin:0;background:#FDF9F2;color:#222}} main{{max-width:1240px;margin:0 auto;padding:24px 20px 60px}}
h1{{font-size:26px;margin:0 0 4px}} h2{{font-size:17px;margin:0;flex:1}} .n{{color:#666;font-size:13px}} .eyebrow{{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:#7a5520;margin-bottom:4px}}
.card{{background:#F6EFE2;border-radius:12px;padding:14px 18px;margin:12px 0}} .head{{display:flex;align-items:center;gap:10px;margin-bottom:8px}} .st{{color:#fff;font-size:11px;padding:2px 8px;border-radius:6px}}
.cols{{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}} @media(max-width:900px){{.cols{{grid-template-columns:1fr 1fr}}}} .cols p{{margin:4px 0;font-size:13.5px}} ul{{margin:4px 0 4px 18px;padding:0;font-size:13.5px}} li{{margin:2px 0}}
img{{max-width:100%;border-radius:8px}} figcaption{{font-size:12px;color:#666}} .filters{{display:flex;gap:8px;margin:8px 0}} .filters button{{border:0;border-radius:8px;padding:5px 10px;background:#EFE6D6;cursor:pointer;font:13px inherit}} .filters button.on{{background:#1E1B16;color:#fff}}
</style></head><body><main><h1>Research the cell uses, and where</h1>
<p class='n'>Built {datetime.date.today().isoformat()} from cell-twin/docs/tracker/research.json. {e(D['note'])}</p>
<div class='filters'><button class='on' data-f='all'>all ({len(D['entries'])})</button>{''.join(f"<button data-f='{s}'>{s} ({n})</button>" for s, n in counts.items())}</div>
{''.join(cards)}
<script>for(const b of document.querySelectorAll('.filters button')) b.onclick = () => {{ for(const x of document.querySelectorAll('.filters button')) x.classList.remove('on'); b.classList.add('on'); for(const c of document.querySelectorAll('.card')) c.style.display = (b.dataset.f === 'all' || c.dataset.status === b.dataset.f) ? '' : 'none'; }};</script>
</main></body></html>"""
OUT.write_text(page); print("wrote", OUT, len(D["entries"]), "entries")
