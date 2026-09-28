(async () => {
 const status = document.getElementById('zr-outcome'); if (!status) return;
 const base = 'zero-reliability/';
 const pct = x => x == null ? 'Unknown' : `${(100*x).toFixed(1)}%`;
 function row(id, values) {const tr = document.createElement('tr');for(const v of values){const td=document.createElement('td');td.textContent=v;tr.append(td);}document.getElementById(id).append(tr);}
 try {
  const [manifest,report]=await Promise.all(['manifest.json','evaluation.json'].map(async f=>{const r=await fetch(base+f);if(!r.ok)throw Error(`${f}: ${r.status}`);return r.json();}));
  const all=manifest.frames,filter=document.getElementById('zr-filter'),select=document.getElementById('zr-frame'),layer=document.getElementById('zr-layer');
  let frames=[];
  function update(){const f=frames[Number(select.value)],img=document.getElementById('zr-image');if(!f){img.hidden=true;document.getElementById('zr-link').removeAttribute('href');document.getElementById('zr-decision').textContent='No views in this category.';return;}img.hidden=false;img.src=base+f[layer.value];img.alt=`${layer.selectedOptions[0].textContent}: ${f.file}, ${f.condition}`;document.getElementById('zr-link').href=img.src;const d=f.decision;document.getElementById('zr-decision').textContent=`${d.accepted?'Provisional component review':'Need another view'} · ${d.reasons.length?d.reasons.join(', ').replaceAll('_',' '):'Passed the frozen review gate'}${d.accepted&&!f.good_component_pair?' · OFFLINE FINDING: this acceptance was wrong.':''} · Motion disabled.`;}
  function populate(){frames=all.filter(f=>filter.value==='all'||(filter.value==='accepted'&&f.decision.accepted)||(filter.value==='abstained'&&!f.decision.accepted)||(filter.value==='bad'&&f.decision.accepted&&!f.good_component_pair));select.replaceChildren();frames.forEach((f,i)=>{const o=document.createElement('option');o.value=i;o.textContent=`${i+1}/${frames.length} · ${f.file.replace('.png','')} · ${f.condition.replaceAll('_',' ')}`;select.append(o);});update();}
  select.onchange=update;layer.onchange=update;filter.onchange=populate;
  for(const [id,delta] of [['zr-prev',-1],['zr-next',1]])document.getElementById(id).onclick=()=>{if(frames.length){select.value=(Number(select.value)+frames.length+delta)%frames.length;update();}};
  document.getElementById('zr-image').onerror=()=>{document.getElementById('zr-decision').textContent='Image could not load. The manifest below lists every saved view.';};
  populate();
  for(const [name,title] of [['ensemble','New two-head model'],['previous','Earlier DINOv2']]){const m=report.models[name],end=m.classes.find(c=>c.class==='mini_end');row('zr-models',[title,pct(m.foreground_mean_iou),pct(end.pooled_iou),pct(end.mean_visible_image_iou),`${end.visible_iou_at_least_half} / ${end.visible_images}`]);}
  for(const [camera,g] of Object.entries(report.cameras))row('zr-gate',[camera,report.thresholds[camera]??'Always abstain',`${g.accepted} / ${g.images}`,`${g.bad_accepts} / ${g.accepted}`,pct(g.nominal_bad_accept_upper_97_5pct),`${g.good_pairs_accepted} / ${g.good_pairs_available}`]);
  for(const [condition,g] of Object.entries(report.conditions))row('zr-conditions',[condition.replaceAll('_',' '),g.images,g.accepted,g.bad_accepts]);
  const a=report.all;status.textContent=`On ${all.length} fresh test views, the frozen gate accepted ${a.accepted} for component review and asked for another view ${a.abstained} times. ${a.bad_accepts} accepted reviews failed the offline mask criterion. Motion remains disabled.`;
  const rejected=report.stress.filter(s=>!s.decision.accepted).length;document.getElementById('zr-faults').textContent=`Camera-fault replay: ${rejected} of ${report.stress.length} black, white and severe-blur probes were rejected. The frozen policy always abstains, so these rejections do not demonstrate selective fault detection.`;
 } catch(e){status.textContent=`The reliability review could not load (${e.message}). Reload or use the report links below.`;}
})();
