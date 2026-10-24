// Control page served at http://<robot>/ by BalancingRobotTunerESP32S3. It speaks the same
// binary frames as the phone app over the WebSocket at /ws: control at 20 Hz while open,
// parameter set/get, commands; it shows the telemetry. Plain HTML/JS, no external files
// (the robot's own Wi-Fi has no internet).
#pragma once

const char WEB_PAGE[] PROGMEM = R"rawpage(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Balance Tuner</title>
<style>
:root{--bg:#eef1ee;--card:#fff;--sunk:#e3e8e4;--fg:#17201c;--muted:#5a6861;--line:#cfd7d1;--accent:#1f5fd1;--soft:#dfe8fa;--ok:#2f7d4f;--oks:#dcefe2;--bad:#c2410c;--bads:#fbe7dc}
@media (prefers-color-scheme:dark){:root{--bg:#111613;--card:#1a201d;--sunk:#252c28;--fg:#e4ebe6;--muted:#9aa7a0;--line:#333c37;--accent:#7aa2ff;--soft:#22304d;--ok:#6fcf97;--oks:#1d3326;--bad:#fb8a4c;--bads:#3a2419}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:12px max(12px,env(safe-area-inset-left)) 24px max(12px,env(safe-area-inset-right))}
main{max-width:520px;margin:0 auto;display:grid;gap:12px}
header{display:flex;align-items:center;gap:8px}h1{font-size:19px;margin:0;flex:1}
.pill{font:12px ui-monospace,Menlo,monospace;padding:3px 10px;border-radius:99px;background:var(--sunk);color:var(--muted)}
.pill.ok{background:var(--oks);color:var(--ok)}.pill.bad{background:var(--bads);color:var(--bad)}
.card{background:var(--card);border-radius:10px;padding:12px;display:grid;gap:10px}
.row{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
h2{font-size:12px;letter-spacing:.08em;color:var(--muted);margin:0}
button{font:inherit;color:var(--fg);background:var(--sunk);border:0;border-radius:8px;padding:9px 14px;touch-action:manipulation}
button.on{background:var(--soft);color:var(--accent)}button:active{filter:brightness(.92)}
button:focus-visible,input:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;font:13px ui-monospace,Menlo,monospace}
.stats div{background:var(--sunk);border-radius:6px;padding:6px 8px}.stats b{display:block;font:600 15px ui-monospace,Menlo,monospace}
canvas{display:block;width:100%;touch-action:none}
#stick{width:220px;height:220px;margin:0 auto}
.hold{flex:1;padding:14px 0}
label.sw{display:flex;align-items:center;gap:8px;margin-left:auto}
.param{display:grid;grid-template-columns:1fr 110px auto;gap:6px;align-items:center}
.param span small{display:block;color:var(--muted);font:11px ui-monospace,Menlo,monospace}
input[type=number]{font:15px ui-monospace,Menlo,monospace;width:100%;padding:7px;border:1px solid var(--line);border-radius:6px;background:var(--card);color:var(--fg)}
#msg{color:var(--muted);font-size:13px;min-height:1.4em}
</style></head><body><main>
<header><h1>Balance Tuner</h1><span id="link" class="pill">connecting…</span></header>
<div class="stats">
<div>tilt<b id="tilt">–</b></div><div>dist<b id="dist">–</b></div><div>mode<b id="mode">–</b></div>
<div>PWM L<b id="pwml">–</b></div><div>PWM R<b id="pwmr">–</b></div><div>rate<b id="hz">–</b></div>
</div>
<canvas id="chart" height="90"></canvas>
<section class="card"><div class="row"><h2>DRIVE</h2>
<label class="sw"><input type="checkbox" id="motors" checked> Motors</label></div>
<div class="row"><button data-mode="1" class="on">Remote</button><button data-mode="2">Avoid</button><button data-mode="3">Follow</button></div>
<canvas id="stick" width="220" height="220" aria-label="Drive stick"></canvas>
<div class="row"><button class="hold" data-hold="ccw">⟲ Spin</button><button class="hold" data-hold="horn">Horn</button><button class="hold" data-hold="cw">Spin ⟳</button></div>
</section>
<section class="card"><h2>PID PARAMETERS</h2><div id="params" style="display:grid;gap:8px"></div>
<div class="row"><button id="get">Read</button><button id="save">Save to robot</button><button id="load">Reload saved</button><button id="defaults">Kit defaults</button></div>
<div id="msg"></div></section>
</main><script>
const PARAMS=[["Angle Kp",2],["Angle Kd",3],["Speed Kp",2],["Speed Ki",4],["Turn Kp",2],["Turn Kd",3],["Balance angle",2],["Drive speed",0]];
const CMD={get:1,save:2,defaults:3,motors:4,rate:5,load:6};
const $=id=>document.getElementById(id);
let ws=null,stick={x:0,y:0},hold={ccw:0,cw:0,horn:0},modeReq=0,modeFrames=0,robot=[],frames=0,tilts=[];
const sum=(b,a,e)=>{let s=0;for(let i=a;i<e;i++)s+=b[i];return s&255};
function send(b){if(ws&&ws.readyState===1)ws.send(b)}
function command(c,a=0){send(new Uint8Array([0xA7,c,a,(c+a)&255,0x5A]))}
function setParam(id,v){const b=new Uint8Array(8),d=new DataView(b.buffer);b[0]=0xA6;b[1]=id;d.setFloat32(2,v,true);b[6]=sum(b,1,6);b[7]=0x5A;send(b)}
function control(){const b=new Uint8Array(11);b[0]=0xA5;b[1]=Math.round(stick.x*127)&255;b[2]=Math.round(stick.y*127)&255;
 b[3]=hold.horn;b[4]=hold.ccw;b[5]=hold.cw;if(modeFrames>0){b[5+modeReq]=1;modeFrames--}b[9]=sum(b,1,9);b[10]=0x5A;send(b)}
// ---- incoming frames (a byte stream; a frame may arrive split)
let rx=[];
function onBytes(u){for(const c of u)rx.push(c);
 while(rx.length){const h=rx[0];let n=h===0xB6?8:h===0xB8?5:h===0xA5?4:h===0xB7?(rx.length>1?rx[1]+4:0):-1;
  if(n<0){rx.shift();continue}if(n===0||rx.length<n)break;const f=rx.slice(0,n);
  if(f[n-1]!==0x5A){rx.shift();continue}
  if(h===0xB6&&sum(f,1,6)===f[6])onParam(f[1],new DataView(new Uint8Array(f).buffer).getFloat32(2,true));
  else if(h===0xB7&&sum(f,2,n-2)===f[n-2])onTelem(new DataView(new Uint8Array(f).buffer,2,f[1]));
  else if(h===0xB8&&((f[1]+f[2])&255)===f[3])onAck(f[1],f[2]);
  else if(h===0xA5&&f[1]===f[2])$("dist").textContent=f[1]+" cm";
  else{rx.shift();continue}
  rx.splice(0,n)}}
function onParam(id,v){robot[id]=v;const r=document.querySelector(`[data-id="${id}"]`);if(!r)return;
 r.querySelector("small").textContent="robot: "+v.toFixed(PARAMS[id][1]);const i=r.querySelector("input");if(document.activeElement!==i)i.value=v.toFixed(PARAMS[id][1])}
function onTelem(d){frames++;const g=k=>d.getInt16(k*2,true);const angle=g(1)/100,tilt=angle+(robot[6]||0);
 $("tilt").textContent=(tilt>=0?"+":"")+tilt.toFixed(2)+"°";$("pwml").textContent=g(6);$("pwmr").textContent=g(7);
 const fl=g(11),m=(fl>>2)&3;$("mode").textContent=["–","remote","avoid","follow"][m];
 document.querySelectorAll("[data-mode]").forEach(b=>b.classList.toggle("on",+b.dataset.mode===m));
 if(fl&2)$("mode").textContent="FALLEN";if(document.activeElement!==$("motors"))$("motors").checked=!!(fl&1);
 tilts.push(tilt);if(tilts.length>200)tilts.shift();drawChart()}
function onAck(c,ok){$("msg").textContent=c===CMD.save?(ok?"Saved to the robot's flash.":"Save failed."):c===CMD.load?(ok?"Reloaded saved values.":"Nothing saved on the robot yet."):c===CMD.defaults?"Kit defaults applied (not saved).":""}
// ---- connection
function connect(){ws=new WebSocket(`ws://${location.host}/ws`);ws.binaryType="arraybuffer";
 ws.onopen=()=>{$("link").textContent="● Wi-Fi";$("link").className="pill ok";command(CMD.rate,20);command(CMD.get)};
 ws.onmessage=e=>onBytes(new Uint8Array(e.data));
 ws.onclose=()=>{$("link").textContent="offline";$("link").className="pill bad";setTimeout(connect,2000)}}
setInterval(()=>{if(!document.hidden)control()},50);
setInterval(()=>{$("hz").textContent=frames+" Hz";frames=0},1000);
// ---- chart: tilt, last 10 s, symmetric around 0
function drawChart(){const c=$("chart"),w=c.width=c.clientWidth*devicePixelRatio,h=c.height=90*devicePixelRatio,x=c.getContext("2d"),
 cs=getComputedStyle(document.body);x.fillStyle=cs.getPropertyValue("--card");x.fillRect(0,0,w,h);
 const r=Math.max(1,...tilts.map(Math.abs));x.strokeStyle=cs.getPropertyValue("--line");x.beginPath();x.moveTo(0,h/2);x.lineTo(w,h/2);x.stroke();
 x.strokeStyle=cs.getPropertyValue("--accent");x.lineWidth=1.6*devicePixelRatio;x.beginPath();
 tilts.forEach((v,i)=>{const px=w-(tilts.length-1-i)*w/199,py=h/2-v/r*(h/2-6);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
 x.fillStyle=cs.getPropertyValue("--muted");x.font=`${11*devicePixelRatio}px ui-monospace,monospace`;x.fillText(`tilt ±${r.toFixed(1)}°`,6,14*devicePixelRatio)}
// ---- stick
const sc=$("stick"),sx=sc.getContext("2d");let pid=null;
function drawStick(){const cs=getComputedStyle(document.body),r=109;sx.clearRect(0,0,220,220);
 sx.fillStyle=cs.getPropertyValue("--sunk");sx.strokeStyle=cs.getPropertyValue("--line");sx.beginPath();sx.arc(110,110,r,0,7);sx.fill();sx.stroke();
 sx.beginPath();sx.moveTo(110,110-r*.76);sx.lineTo(110,110+r*.76);sx.moveTo(110-r*.76,110);sx.lineTo(110+r*.76,110);sx.stroke();
 sx.fillStyle=cs.getPropertyValue("--accent");sx.beginPath();sx.arc(110+stick.x*r*.7,110-stick.y*r*.7,r*.3,0,7);sx.fill()}
function move(e){const b=sc.getBoundingClientRect();let nx=(e.clientX-b.left-b.width/2)/(b.width/2),ny=-(e.clientY-b.top-b.height/2)/(b.height/2);
 const m=Math.hypot(nx,ny);if(m>1){nx/=m;ny/=m}stick=Math.hypot(nx,ny)<.12?{x:0,y:0}:{x:nx,y:ny};drawStick()}
sc.addEventListener("pointerdown",e=>{pid=e.pointerId;try{sc.setPointerCapture(pid)}catch(_){}move(e)});
sc.addEventListener("pointermove",e=>{if(e.pointerId===pid)move(e)});
["pointerup","pointercancel"].forEach(t=>sc.addEventListener(t,()=>{pid=null;stick={x:0,y:0};drawStick()}));
// ---- buttons
document.querySelectorAll("[data-hold]").forEach(b=>{const k=b.dataset.hold;
 b.addEventListener("pointerdown",()=>hold[k]=1);["pointerup","pointerleave","pointercancel"].forEach(t=>b.addEventListener(t,()=>hold[k]=0))});
document.querySelectorAll("[data-mode]").forEach(b=>b.onclick=()=>{modeReq=+b.dataset.mode;modeFrames=3});
$("motors").onchange=e=>command(CMD.motors,e.target.checked?1:0);
$("get").onclick=()=>command(CMD.get);$("load").onclick=()=>command(CMD.load);$("defaults").onclick=()=>command(CMD.defaults);
$("save").onclick=()=>{if(confirm("Save the current values to the robot? They will be used after every reset."))command(CMD.save)};
PARAMS.forEach(([name,dec],id)=>{const r=document.createElement("div");r.className="param";r.dataset.id=id;
 r.innerHTML=`<span>${name}<small>robot: –</small></span><input type="number" step="any" inputmode="decimal" id="p${id}"><button>Set</button>`;
 const i=r.querySelector("input"),go=()=>{const v=parseFloat(i.value.replace(",","."));if(isFinite(v))setParam(id,v)};
 r.querySelector("button").onclick=go;i.onkeydown=e=>{if(e.key==="Enter")go()};$("params").appendChild(r)});
drawStick();drawChart();connect();
</script></body></html>)rawpage";
