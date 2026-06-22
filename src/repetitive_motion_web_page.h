#pragma once

#include <Arduino.h>

static const char REPETITIVE_MOTION_WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b1114">
<title>Motion Unit 01</title>
<style>
:root{--bg:#0b1114;--panel:#11191d;--line:#344047;--muted:#8e999f;--text:#e9edf0;--cyan:#29b9e8;--green:#39bf51;--red:#e43131;--field:#0c1215}
*{box-sizing:border-box}html{background:var(--bg);color:var(--text);font-family:"Arial Narrow","Roboto Condensed",Arial,sans-serif}body{margin:0;min-height:100vh;background:var(--bg)}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:16px 18px;border-bottom:1px solid var(--line);font:700 13px/1.2 monospace;letter-spacing:1.5px;text-transform:uppercase}.online{color:var(--green)}.online:before{content:"";display:inline-block;width:10px;height:10px;margin-right:8px;border-radius:50%;background:currentColor}.ota{color:var(--cyan)}
main{max-width:760px;margin:auto}.instrument{padding:24px 20px 18px;text-align:center;border-bottom:1px solid var(--line)}canvas{display:block;width:100%;height:auto}.phase{margin-top:-190px;color:var(--muted);font:700 13px monospace;letter-spacing:1.5px;text-transform:uppercase}.phase strong{display:block;margin-top:7px;color:var(--cyan);font-size:19px}.angle{margin:10px 0 0;color:var(--cyan);font:700 48px monospace}.angle-label{color:var(--muted);font:12px monospace;letter-spacing:1px}
.controls{display:grid;grid-template-columns:1fr 1fr;padding:18px 20px 24px;border-bottom:1px solid var(--line)}button{min-height:72px;border:0;border-radius:4px;font:900 25px/1 monospace;letter-spacing:1px;cursor:pointer}.run{background:var(--green);color:#07130a}.stop{background:var(--red);color:#170505}.run:disabled,.stop:disabled{filter:saturate(.2);opacity:.45;cursor:default}
section{padding:18px 20px;border-bottom:1px solid var(--line)}h2{margin:0 0 15px;padding-bottom:10px;border-bottom:1px solid var(--line);color:#aab2b6;font:700 14px monospace;letter-spacing:2px;text-transform:uppercase}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px 28px}.field label{display:block;margin:0 0 8px;color:#abb4b8;font:700 13px monospace;letter-spacing:1px;text-transform:uppercase}.input{display:grid;grid-template-columns:1fr auto;border:1px solid #7b858a;border-radius:4px;background:var(--field)}input{min-width:0;width:100%;height:55px;padding:0 13px;border:0;outline:0;background:transparent;color:var(--cyan);font:700 25px monospace}input:focus{box-shadow:inset 0 0 0 2px var(--cyan)}.unit{min-width:58px;padding:18px 10px;border-left:1px solid #59636a;color:#d4d9db;font:700 14px monospace;text-align:center}.hint{margin-top:7px;color:#727e84;font:11px monospace}
.adjust{width:100%;min-height:42px;margin-top:10px;border:1px solid var(--cyan);background:transparent;color:var(--cyan);font-size:13px}.adjust:disabled{border-color:#586268;color:#586268;opacity:.6}.save-wrap{padding:18px 20px 28px}.save{width:100%;min-height:62px;background:var(--cyan);color:#061116}.settings-link{display:block;margin-top:17px;color:var(--cyan);font:700 13px monospace;letter-spacing:1px;text-align:center;text-decoration:none}.message{min-height:24px;margin-top:13px;color:var(--green);font:13px monospace;text-align:center}.message.error{color:#ff6565}
@media(max-width:520px){.top{font-size:11px;padding:14px 12px}.instrument{padding-inline:10px}.phase{margin-top:-108px}.controls,section,.save-wrap{padding-left:14px;padding-right:14px}.grid{gap:18px 12px}.angle{font-size:42px}button{font-size:22px}.unit{min-width:48px;padding-inline:7px}input{font-size:22px;padding-inline:9px}}
</style>
</head>
<body>
<header class="top"><span>MOTION UNIT 01</span><span class="online">Wi-Fi conectado</span><span class="ota" id="ota">OTA/AP: IDLE</span></header>
<main>
  <div class="instrument">
    <canvas id="gauge" width="720" height="420" aria-label="Indicador angular"></canvas>
    <div class="phase">Fase atual<strong id="phase">CARREGANDO</strong></div>
    <div class="angle" id="angle">---.-°</div><div class="angle-label">POSIÇÃO ATUAL</div>
  </div>
  <div class="controls"><button class="run" id="run">RUN</button><button class="stop" id="stop">STOP</button></div>
  <form id="config">
    <section><h2>Limites</h2><div class="grid">
      <div class="field"><label for="start">Start deg</label><div class="input"><input id="start" name="start" type="number" min="0" max="359.99" step="0.1" required><span class="unit">°</span></div><div class="hint">0.0 – 359.9°</div><button class="adjust" id="goStart" type="button">IR AO INÍCIO</button></div>
      <div class="field"><label for="end">End deg</label><div class="input"><input id="end" name="end" type="number" min="0" max="359.99" step="0.1" required><span class="unit">°</span></div><div class="hint">0.0 – 359.9°</div><button class="adjust" id="goEnd" type="button">IR AO FIM</button></div>
    </div></section>
    <section><h2>RPM</h2><div class="grid">
      <div class="field"><label for="rpmOut">Ida</label><div class="input"><input id="rpmOut" name="rpmOut" type="number" min="0.1" step="0.01" required><span class="unit">RPM</span></div></div>
      <div class="field"><label for="rpmBack">Volta</label><div class="input"><input id="rpmBack" name="rpmBack" type="number" min="0.1" step="0.01" required><span class="unit">RPM</span></div></div>
    </div></section>
    <section><h2>Pausas</h2><div class="grid">
      <div class="field"><label for="waitStart">Início ms</label><div class="input"><input id="waitStart" name="waitStart" type="number" min="0" max="3600000" step="1" required><span class="unit">ms</span></div></div>
      <div class="field"><label for="waitEnd">Fim ms</label><div class="input"><input id="waitEnd" name="waitEnd" type="number" min="0" max="3600000" step="1" required><span class="unit">ms</span></div></div>
    </div></section>
    <div class="save-wrap"><button class="save" type="submit">APLICAR PARÂMETROS</button><div class="message" id="message"></div><a class="settings-link" href="/settings">AJUSTES AVANÇADOS</a></div>
  </form>
</main>
<script>
const $=id=>document.getElementById(id), fields=['start','end','rpmOut','rpmBack','waitStart','waitEnd'];let editing=false,last={start:0,end:180,angle:0};
fields.forEach(id=>{$(id).addEventListener('focus',()=>editing=true);$(id).addEventListener('blur',()=>editing=false)});
function drawGauge(){const c=$('gauge'),x=c.getContext('2d'),w=c.width,h=c.height,cx=w/2,cy=h-18,r=Math.min(w*.43,h*.92);x.clearRect(0,0,w,h);x.lineCap='butt';for(let i=0;i<=24;i++){let a=Math.PI+(Math.PI*i/24),major=i%4===0,r0=r-(major?18:8);x.beginPath();x.moveTo(cx+Math.cos(a)*r0,cy+Math.sin(a)*r0);x.lineTo(cx+Math.cos(a)*r,cy+Math.sin(a)*r);x.strokeStyle=major?'#e9edf0':'#849096';x.lineWidth=major?3:2;x.stroke()}x.beginPath();x.arc(cx,cy,r-25,Math.PI,2*Math.PI);x.strokeStyle='#59656b';x.lineWidth=2;x.stroke();x.fillStyle='#cfd5d8';x.font='20px monospace';x.textAlign='center';for(let d=0;d<=360;d+=60){let a=Math.PI+Math.PI*d/360,rr=r+28;x.fillText(d,cx+Math.cos(a)*rr,cy+Math.sin(a)*rr+7)}function mark(deg,color){let a=Math.PI+Math.PI*(deg/360),rr=r-25;x.beginPath();x.moveTo(cx+Math.cos(a)*(rr-13),cy+Math.sin(a)*(rr-13));x.lineTo(cx+Math.cos(a)*(rr+8),cy+Math.sin(a)*(rr+8));x.strokeStyle=color;x.lineWidth=6;x.stroke()}mark(last.start,'#29b9e8');mark(last.end,'#29b9e8');mark(last.angle,'#e9edf0')}
async function api(path,body){let o={method:body?'POST':'GET'};if(body){o.headers={'Content-Type':'application/x-www-form-urlencoded'};o.body=new URLSearchParams(body)}let r=await fetch(path,o),j=await r.json();if(!r.ok)throw Error(j.error||'Falha na operação');return j}
let messageTimer;
function setMessage(text,error=false){clearTimeout(messageTimer);const el=$('message');el.textContent=text;el.className='message'+(error?' error':'');if(text)messageTimer=setTimeout(()=>{el.textContent='';el.className='message'},5000)}
function render(s){const phases={STOPPED:'EM REPOUSO',TO_END:'INDO AO FIM',DWELL_END:'PAUSA NO FIM',TO_START:'INDO AO INÍCIO',DWELL_START:'PAUSA NO INÍCIO'};last={start:s.start,end:s.end,angle:s.angle||0};$('phase').textContent=s.stall?'FALHA: MOTOR TRAVADO':(s.moveActive&&!s.running?'AJUSTANDO POSIÇÃO':(phases[s.phase]||s.phase));$('angle').textContent=s.sensor?s.angle.toFixed(1)+'°':'SEM SENSOR';$('run').disabled=s.running||s.moveActive||s.otaBusy;$('stop').disabled=!s.running&&!s.moveActive;const canAdjust=!s.running&&!s.moveActive&&!s.otaBusy&&s.sensor;$('goStart').disabled=!canAdjust;$('goEnd').disabled=!canAdjust;$('ota').textContent=s.otaBusy?'OTA/AP: UPDATE':'OTA/AP: IDLE';if(!editing){$('start').value=s.start.toFixed(1);$('end').value=s.end.toFixed(1);$('rpmOut').value=s.rpmOut.toFixed(2);$('rpmBack').value=s.rpmBack.toFixed(2);$('waitStart').value=s.waitStart;$('waitEnd').value=s.waitEnd;$('rpmOut').max=s.maxRpm;$('rpmBack').max=s.maxRpm}drawGauge()}
async function refresh(){try{render(await api('/api/status'))}catch(e){setMessage('Sem comunicação com o controlador',true)}}
$('run').onclick=async()=>{try{render(await api('/api/run',{running:'1'}));setMessage('Homing iniciado')}catch(e){setMessage(e.message,true)}};
$('stop').onclick=async()=>{try{render(await api('/api/run',{running:'0'}));setMessage('Movimento parado')}catch(e){setMessage(e.message,true)}};
$('goStart').onclick=async()=>{try{render(await api('/api/adjust',{target:'start'}));setMessage('Movendo para a posição inicial')}catch(e){setMessage(e.message,true)}};
$('goEnd').onclick=async()=>{try{render(await api('/api/adjust',{target:'end'}));setMessage('Movendo para a posição final')}catch(e){setMessage(e.message,true)}};
$('config').onsubmit=async e=>{e.preventDefault();let body={};fields.forEach(id=>body[id]=$(id).value);try{render(await api('/api/config',body));setMessage('Configuração salva com sucesso')}catch(err){setMessage(err.message,true)}};
refresh();setInterval(refresh,1000);addEventListener('resize',drawGauge);
</script>
</body></html>
)HTML";
