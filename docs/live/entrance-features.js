(async()=>{
 const status=document.getElementById('ef-status');if(!status)return;
 const base='entrance-features/';
 try{
  const [manifest,report]=await Promise.all(['manifest.json','evaluation.json'].map(async f=>{const r=await fetch(base+f);if(!r.ok)throw Error(`${f}: ${r.status}`);return r.json();}));
  const select=document.getElementById('ef-frame'),layer=document.getElementById('ef-layer');
  manifest.frames.forEach((f,i)=>{const o=document.createElement('option');o.value=i;o.textContent=`${i+1}/${manifest.frames.length} · ${f.file} · ${f.condition}`;select.append(o);});
  function update(){const f=manifest.frames[Number(select.value)],image=document.getElementById('ef-image');image.src=base+f[layer.value];image.alt=`${layer.selectedOptions[0].textContent}, ${f.file}, ${f.condition}`;document.getElementById('ef-caption').textContent=`${f.camera} camera · ${f.condition} · provisional slider classification: ${f.predicted_slider_state}. This does not verify clamping or seating.`;}
  select.onchange=update;layer.onchange=update;document.getElementById('ef-next').onclick=()=>{select.value=(Number(select.value)+1)%manifest.frames.length;update();};update();
  const pct=v=>v==null?'Unknown':`${(v*100).toFixed(1)}%`;
  for(const c of report.classes){const tr=document.createElement('tr');for(const v of [c.name.replaceAll('_',' '),pct(c.pooled_iou),pct(c.mean_visible_iou),pct(c.mean_boundary_f1_2px),`${c.absent_false_positives_8px} / ${c.absent_images}`]){const td=document.createElement('td');td.textContent=v;tr.append(td);}document.getElementById('ef-metrics').append(tr);}
  const s=report.slider;status.textContent=`${manifest.frames.length} fresh test views. Slider classification correct in ${s.correct} of ${s.visible_views} visibly labelled views; ${s.unknown} were unknown. The model also made ${s.predictions_on_unobservable_views} slider predictions in ${s.unobservable_views} views with no visible slider label. Motion remains disabled.`;
 }catch(e){status.textContent=`Feature review could not load (${e.message}). Use the report links below.`;}
})();
