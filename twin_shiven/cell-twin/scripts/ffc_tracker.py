"""Fix tracker page for the FFC cell review items: one card per item with the issue, the evidence, what research found, the fix and how it
was verified (numbers, renders, links). Built from docs/tracker/items.json; images referenced relative to the Desktop page.
usage: python3 scripts/ffc_tracker.py [out.html]"""
import json, sys, html, datetime
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Desktop" / "Synphony-BOM" / "FFC-fix-tracker.html"
D = json.loads((ROOT / "docs/tracker/items.json").read_text())
e = html.escape
STATUS = {"open": ("#b23", "open"), "researching": ("#b7791f", "researching"), "fixing": ("#1f6fb2", "fixing"), "verified": ("#2f8f5b", "verified"), "closed": ("#2f8f5b", "closed by decision"), "deferred": ("#888", "deferred")}


def evidence(ev):
    out = []
    for x in ev:
        if x["kind"] == "image":
            out.append(f"<figure><img src='{e(x['src'])}' alt=''><figcaption>{e(x.get('text', ''))}</figcaption></figure>")
        elif x["kind"] == "video":
            out.append(f"<figure><video src='{e(x['src'])}' controls muted loop playsinline></video><figcaption>{e(x.get('text', ''))}</figcaption></figure>")
        elif x["kind"] == "iframe":
            out.append(f"<figure><iframe src='{e(x['src'])}' style='width:100%;height:{int(x.get('height', 520))}px;border:0;border-radius:8px;background:#fff'></iframe><figcaption>{e(x.get('text', ''))} <a href='{e(x['src'])}'>open full page</a></figcaption></figure>")
        elif x["kind"] == "number":
            out.append(f"<div class='big'><span>{e(str(x['value']))}</span><small>{e(x.get('text', ''))}</small></div>")
        elif x["kind"] == "link":
            out.append(f"<p class='n'><a href='{e(x['url'])}'>{e(x['text'])}</a></p>")
        elif x["kind"] == "table":
            out.append("<table>" + "".join("<tr>" + "".join(f"<td>{e(str(c))}</td>" for c in row) + "</tr>" for row in x["rows"]) + "</table>")
        else:
            out.append(f"<p class='n'>{e(x['text'])}</p>")
    return "".join(out)


items = D["items"]; counts = {}
for it in items:
    counts[it["status"]] = counts.get(it["status"], 0) + 1
done = counts.get("verified", 0) + counts.get("closed", 0)
cards = []
for it in items:
    col, lab = STATUS.get(it["status"], ("#888", it["status"]))
    cards.append(f"""<section class='card' id='item{it['id']}' data-status='{e(it['status'])}' data-group='{e(it['group'])}'>
<div class='head'><span class='num'>{it['id']}</span><h2>{e(it['title'])}</h2><span class='st' style='background:{col}'>{lab}</span><span class='grp'>{e(it['group'])}</span></div>
{('<div class=hero>' + evidence(it['hero']) + '</div>') if it.get('hero') else ''}
<div class='cols'>
<div><div class='eyebrow'>What was wrong</div><p>{e(it['issue'])}</p>{evidence(it.get('evidence', []))}</div>
<div><div class='eyebrow'>What research found</div>{('<p>' + e(it['research']) + '</p>') if it.get('research') else "<p class='n'>not started</p>"}{evidence(it.get('research_evidence', []))}</div>
<div><div class='eyebrow'>The fix</div><p>{e(it['fix'])}</p>{evidence(it.get('fix_evidence', []))}</div>
<div><div class='eyebrow'>How it was verified</div>{('<p>' + e(it['verification']) + '</p>') if it.get('verification') else "<p class='n'>not yet</p>"}{evidence(it.get('verification_evidence', []))}</div>
</div></section>""")
groups = sorted({it["group"] for it in items}, key=lambda g: [it["group"] for it in items].index(g))
page = f"""<!doctype html><html><head><meta charset='utf-8'><title>FFC cell fix tracker</title>
<style>
body{{font:15px/1.55 -apple-system,Inter,Helvetica,sans-serif;margin:0;background:#FDF9F2;color:#222}} main{{max-width:1240px;margin:0 auto;padding:24px 20px 60px}}
h1{{font-size:26px;margin:0 0 4px}} h2{{font-size:17px;margin:0;flex:1}} .n{{color:#666;font-size:13px}} .eyebrow{{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:#7a5520;margin-bottom:4px}}
.bar{{height:10px;background:#EFE6D6;border-radius:5px;overflow:hidden;margin:10px 0 18px}} .bar div{{height:100%;background:#2f8f5b}}
.card{{background:#F6EFE2;border-radius:12px;padding:14px 18px;margin:12px 0}} .head{{display:flex;align-items:center;gap:10px;margin-bottom:8px}} .num{{font-weight:700;color:#7a5520;width:28px}}
.st{{color:#fff;font-size:11px;padding:2px 8px;border-radius:6px;white-space:nowrap}} .grp{{font-size:12px;color:#666}}
.cols{{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}} @media(max-width:900px){{.cols{{grid-template-columns:1fr 1fr}}}} .cols p{{margin:4px 0;font-size:13.5px}}
figure{{margin:8px 0}} img,video{{max-width:100%;border-radius:8px}} .hero{{margin:6px 0 12px}} .hero figure{{margin:0}} .hero img,.hero video{{max-height:420px}} .big{{display:flex;align-items:baseline;gap:10px;margin:6px 0}} .big span{{font-size:26px;font-weight:700;color:#7a5520}} .big small{{color:#666}} figcaption{{font-size:12px;color:#666}} table{{border-collapse:collapse;font-size:12.5px;margin:6px 0}} td{{padding:3px 8px;border-bottom:1px solid #EFE6D6}}
.filters{{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 4px}} .filters button{{border:0;border-radius:8px;padding:5px 10px;background:#EFE6D6;cursor:pointer;font:13px inherit}} .filters button.on{{background:#1E1B16;color:#fff}}
.dec{{background:#fff;border-radius:10px;padding:10px 14px;margin:10px 0;font-size:13.5px}} .dec li{{margin:3px 0}}
</style></head><body><main>
<h1>FFC cell: the 34 review items, and what happened to each</h1>
<p class='n'>Built {datetime.date.today().isoformat()} from cell-twin/docs/tracker/items.json. Each card says what was wrong, what the research found (with sources), what was changed, and how the change was checked. Nothing is marked verified without a number, a render or a link you can open.</p>
<div class='bar'><div style='width:{100*done/len(items):.0f}%'></div></div>
<p class='n'>{done} of {len(items)} verified or closed · {counts.get('fixing',0)} being fixed · {counts.get('researching',0)} in research · {counts.get('open',0)} open · {counts.get('deferred',0)} deferred</p>
<div class='dec'><div class='eyebrow'>Decisions taken</div><ul>{''.join(f"<li>{e(d['date'])}: {e(d['text'])}</li>" for d in D['decisions'])}</ul></div>
<div class='filters'><button class='on' data-f='all'>all</button>{''.join(f"<button data-f='s:{s}'>{STATUS[s][1]}</button>" for s in ('open','researching','fixing','verified','closed','deferred') if counts.get(s))}{''.join(f"<button data-f='g:{e(g)}'>{e(g)}</button>" for g in groups)}</div>
{''.join(cards)}
<script>
for(const b of document.querySelectorAll('.filters button')) b.onclick = () => {{ for(const x of document.querySelectorAll('.filters button')) x.classList.remove('on'); b.classList.add('on'); const f = b.dataset.f;
 for(const c of document.querySelectorAll('.card')) c.style.display = (f === 'all' || (f.startsWith('s:') && c.dataset.status === f.slice(2)) || (f.startsWith('g:') && c.dataset.group === f.slice(2))) ? '' : 'none'; }};
</script></main></body></html>"""
OUT.write_text(page); print("wrote", OUT, f"{len(items)} items, {done} done")
