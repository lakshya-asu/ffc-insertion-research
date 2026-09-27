'use strict';
const themeButton=document.querySelector('#theme-toggle');
let theme='system';try{theme=localStorage.getItem('ffc-theme')||'system';}catch(_){}
function applyTheme(){if(theme==='system')delete document.documentElement.dataset.theme;else document.documentElement.dataset.theme=theme;themeButton.textContent=`Theme: ${theme}`;}
themeButton.addEventListener('click',()=>{theme={system:'dark',dark:'light',light:'system'}[theme]||'system';try{localStorage.setItem('ffc-theme',theme);}catch(_){}applyTheme();});applyTheme();
const notes=document.querySelector('#lab-notes'),noteStatus=document.querySelector('#notes-status');
try{notes.value=localStorage.getItem('ffc-lab-notes')||'';}catch(_){}
notes.addEventListener('input',()=>{try{localStorage.setItem('ffc-lab-notes',notes.value);noteStatus.textContent='Saved in this browser only.';}catch(_){noteStatus.textContent='Browser storage unavailable; copy your notes before leaving.';}});
document.querySelector('#copy-notes').addEventListener('click',async()=>{try{await navigator.clipboard.writeText(notes.value);noteStatus.textContent='Copied. Paste into our chat to discuss.';}catch(_){notes.select();noteStatus.textContent='Select and copy the notes manually.';}});
let base='',camera='workcell',current=null,frameKey='',eventKey='',connected=false,clockOffset=0;
const image=document.querySelector('#cell-image');
const timeFormat=new Intl.DateTimeFormat(undefined,{month:'short',day:'numeric',hour:'2-digit',minute:'2-digit',second:'2-digit'});
const ageText=seconds=>seconds<60?`${Math.floor(seconds)}s ago`:seconds<3600?`${Math.floor(seconds/60)}m ago`:`${Math.floor(seconds/3600)}h ago`;
function drawFrame(){
 const feed=current?.feeds[camera];
 if(!feed){document.querySelector('#frame-status').textContent='No frame for this camera yet';return;}
 const age=Math.max(0,Date.now()/1000+clockOffset-feed.captured_at);
 document.querySelector('#frame-status').textContent=age<6&&connected?`Fresh render · ${ageText(age)}`:`Last saved frame · ${ageText(age)}`;
 document.querySelector('#frame-time').textContent=`${timeFormat.format(new Date(feed.captured_at*1000))} · frame ${feed.sequence} · ${feed.source}`;
 document.querySelector('#frame-description').textContent=feed.description;
 const key=`${camera}-${feed.captured_at}`;
 if(key!==frameKey){image.src=`${base}/api/frame/${camera}?v=${feed.captured_at}`;image.alt=`${camera} view of the simulated Pi workcell. ${feed.description}`;frameKey=key;}
}
image.addEventListener('error',()=>{document.querySelector('#frame-status').textContent='Frame unavailable; waiting to reconnect';frameKey='';});
for(const button of document.querySelectorAll('[data-camera]'))button.addEventListener('click',()=>{camera=button.dataset.camera;for(const b of document.querySelectorAll('[data-camera]'))b.setAttribute('aria-pressed',String(b===button));drawFrame();});
function render(data){
 current=data;clockOffset=data.server_time-Date.now()/1000;
 const s=data.status;
 document.querySelector('#work-title').textContent=s.title||'Waiting for a work update';
 document.querySelector('#work-detail').textContent=s.detail||'No current update is available.';
 document.querySelector('#phase-label').textContent=s.phase||'Idle';
 document.querySelector('#next-step').textContent=s.next||'Awaiting the next experiment.';
 const t=data.training;
 document.querySelector('#training-progress').hidden=!t.epochs_completed;
 document.querySelector('#epochs').value=t.epochs_completed;
 document.querySelector('#epoch-text').textContent=`${t.epochs_completed} / ${t.epochs_planned} epochs${t.invalid?' · invalid run':t.complete?' · finished':''}`;
 if(t.epochs_completed){document.querySelector('#training-score').textContent=`Best validation foreground IoU: ${(100*t.best_validation_foreground_mean_iou).toFixed(1)}%. Validation selects the checkpoint; this is not the held-out test score.`;document.querySelector('#training-line').setAttribute('points',t.history.map((r,i)=>`${i/Math.max(t.epochs_planned-1,1)*300},${85-r.foreground_mean_iou*80}`).join(' '));}
 const key=JSON.stringify(s.events);
 if(key!==eventKey){const events=document.querySelector('#events');events.replaceChildren();for(const event of [...(s.events||[])].reverse()){const li=document.createElement('li'),when=document.createElement('time'),body=document.createElement('div'),title=document.createElement('h3'),p=document.createElement('p');when.dateTime=new Date(event.time*1000).toISOString();when.textContent=timeFormat.format(new Date(event.time*1000));title.textContent=event.title;p.textContent=event.detail;body.append(title,p);li.append(when,body);events.append(li);}eventKey=key;}
 drawFrame();
}
async function poll(){
 try{const r=await fetch(`${base}/api/status`,{cache:'no-store',signal:AbortSignal.timeout(7000)});if(!r.ok)throw new Error('Lab unavailable');const data=await r.json();connected=true;document.querySelector('.connection').dataset.connected='true';document.querySelector('#connection-label').textContent='Connected · updates every 2s';render(data);}catch(_){connected=false;document.querySelector('.connection').dataset.connected='false';document.querySelector('#connection-label').textContent='Disconnected · showing last available view';drawFrame();}
 setTimeout(poll,2000);
}
(async()=>{try{if(location.hostname.endsWith('.github.io')){const r=await fetch('connection.json',{cache:'no-store'});const c=await r.json();base=c.base_url.replace(/\/$/,'');}}catch(_){document.querySelector('#connection-label').textContent='Live connection configuration unavailable';}poll();})();
setInterval(drawFrame,1000);
