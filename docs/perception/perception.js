'use strict';
const $ = id => document.getElementById(id);
let theme='system';
try { theme=localStorage.getItem('ffc-theme')||'system'; } catch (_) {}
function applyTheme(value){theme=['light','dark','system'].includes(value)?value:'system';if(theme==='system')document.documentElement.removeAttribute('data-theme');else document.documentElement.dataset.theme=theme;$('theme-toggle').textContent=`Theme: ${theme}`;try{localStorage.setItem('ffc-theme',theme);}catch(_){}}
applyTheme(theme);$('theme-toggle').addEventListener('click',()=>applyTheme({system:'light',light:'dark',dark:'system'}[theme]));
fetch('frames.json').then(r=>{if(!r.ok)throw Error('manifest');return r.json();}).then(frames=>{
 let view='rgb';
 frames.forEach((f,i)=>{const option=document.createElement('option');option.value=i;option.textContent=`${f.case} / ${f.camera}`;$('frame-select').append(option);});
 function render(){const f=frames[Number($('frame-select').value)];$('frame-image').src=f[view];$('frame-image').alt=`${view==='rgb'?'Clean RGB input':view==='prediction'?'Predicted cable mask':'Offline endpoint review'}: ${f.case}, ${f.camera} camera`;$('frame-caption').textContent=`${f.case} / ${f.camera} · mask IoU ${f.iou.toFixed(3)} · endpoint error ${f.maximum_endpoint_error_px===null?'not available':f.maximum_endpoint_error_px.toFixed(2)+' px'} · motion not qualified`;document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.view===view)));}
 $('frame-select').addEventListener('change',render);document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{view=b.dataset.view;render();}));render();
}).catch(()=>{$('frame-caption').textContent='The frame manifest could not load. Raw images and full measurements are linked below.';});
