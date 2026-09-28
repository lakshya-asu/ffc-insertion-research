(async()=>{
 const root=document.getElementById('gv-status');if(!root)return;
 try{
  const response=await fetch('grasp-visibility/review.json');if(!response.ok)throw Error(response.status);
  const report=await response.json(),select=document.getElementById('gv-case'),layer=document.getElementById('gv-layer');
  report.cases.forEach((c,i)=>{const o=document.createElement('option');o.value=i;o.textContent=`${c.gap_mm} mm approach · ${c.jaws?`setback ${c.setback_mm}, width ${c.width_mm}, thickness ${c.thickness_mm} mm`:'no fingers / baseline'}`;select.append(o);});
  function update(){const index=Number(select.value),parts=[];for(const camera of ['entrance','offset']){const row=report.frames.find(r=>r.file===`${String(index).padStart(3,'0')}-${camera}.png`),image=document.getElementById(`gv-${camera}`);image.src=`grasp-visibility/${row.file.replace('.png',layer.value==='rgb'?'.webp':'-offline.webp')}`;image.alt=`${camera} camera, ${select.selectedOptions[0].textContent}, ${layer.value==='rgb'?'raw RGB':'offline component annotation'}`;parts.push(`${camera}: connector ${(100*row.retention_vs_no_jaws.connector).toFixed(1)}%, mini-end ${(100*row.retention_vs_no_jaws.mini_end).toFixed(1)}%`);}document.getElementById('gv-counts').textContent=`Visible component pixels retained relative to matching no-finger image — ${parts.join(' · ')}. These are full-frame counts; images show the fixed native-pixel review window.`;}
  select.onchange=update;layer.onchange=update;
  document.getElementById('gv-next').onclick=()=>{select.value=(Number(select.value)+1)%report.cases.length;update();};
  update();root.textContent=`${report.cases.length} static configurations, ${report.frames.length} camera views. No physics advancement or motion permission.`;
 }catch(e){root.textContent=`Visibility review unavailable (${e.message}). Please reload or use the report link.`;}
})();
