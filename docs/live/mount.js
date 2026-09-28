(async()=>{
 const status=document.querySelector('#mount-status');if(!status)return;
 try{
  const response=await fetch('mount/manifest.json');if(!response.ok)throw Error(`HTTP ${response.status}`);
  const {frames}=await response.json(),select=document.querySelector('#mount-case'),view=document.querySelector('#mount-view');
  frames.forEach((f,i)=>select.add(new Option(`${f.segment.replaceAll('_',' ')} · offset ${f.offset_m.map(v=>(v*1000).toFixed(1)).join(', ')} mm`,i)));
  select.value=String(frames.findIndex(f=>f.segment==='near_entry'));
  function update(){const f=frames[Number(select.value)],src=`mount/${f[view.value]}`,im=document.querySelector('#mount-image');im.src=src;im.width=view.value==='macro'?2448:1280;im.height=view.value==='macro'?2048:960;im.alt=`${f.segment.replaceAll('_',' ')}: ${view.selectedOptions[0].text}`;document.querySelector('#mount-full').href=src;status.textContent=`Offline ${f.group==='cable'?'cable-route':'latch-access'} pose: ${f.segment.replaceAll('_',' ')}. Static rendering; motion disabled.`;}
  select.addEventListener('change',update);view.addEventListener('change',update);update();
 }catch(e){status.textContent=`Could not load the mount review: ${e.message}. The saved image remains available.`;}
})();
