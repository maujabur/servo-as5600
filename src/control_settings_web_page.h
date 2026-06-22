#pragma once

#include <Arduino.h>

static const char CONTROL_SETTINGS_WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b1114"><title>Ajustes · Motion Unit</title>
<style>
:root{--bg:#0b1114;--line:#344047;--muted:#8e999f;--text:#e9edf0;--cyan:#29b9e8;--green:#39bf51;--red:#ff6565;--field:#0c1215}*{box-sizing:border-box}html{background:var(--bg);color:var(--text);font-family:"Arial Narrow","Roboto Condensed",Arial,sans-serif}body{margin:0;min-height:100vh}.top{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:16px 18px;border-bottom:1px solid var(--line);font:700 13px monospace;letter-spacing:1.5px;text-transform:uppercase}.online{color:var(--green)}.back{color:var(--cyan);text-decoration:none}main{max-width:760px;margin:auto}.intro{padding:25px 20px;border-bottom:1px solid var(--line)}h1{margin:0 0 8px;font:800 27px monospace;letter-spacing:1px}.intro p{margin:0;color:var(--muted);font:13px/1.5 monospace}section{padding:19px 20px;border-bottom:1px solid var(--line)}h2{margin:0 0 15px;padding-bottom:10px;border-bottom:1px solid var(--line);color:#aab2b6;font:700 14px monospace;letter-spacing:2px;text-transform:uppercase}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px 28px}.field label{display:block;margin-bottom:8px;color:#abb4b8;font:700 12px monospace;letter-spacing:1px;text-transform:uppercase}.input{display:grid;grid-template-columns:1fr auto;border:1px solid #7b858a;border-radius:4px;background:var(--field)}input{min-width:0;width:100%;height:55px;padding:0 13px;border:0;outline:0;background:transparent;color:var(--cyan);font:700 23px monospace}input:focus{box-shadow:inset 0 0 0 2px var(--cyan)}.unit{min-width:62px;padding:19px 9px;border-left:1px solid #59636a;color:#d4d9db;font:700 12px monospace;text-align:center}.hint{margin-top:6px;color:#727e84;font:11px monospace}.actions{padding:20px}.save{width:100%;min-height:64px;border:0;border-radius:4px;background:var(--cyan);color:#061116;font:900 22px monospace;cursor:pointer}.save:disabled{opacity:.45}.message{min-height:22px;margin-top:13px;color:var(--green);font:13px monospace;text-align:center}.message.error{color:var(--red)}@media(max-width:520px){.top{font-size:11px;padding:14px 12px}.intro,section,.actions{padding-left:14px;padding-right:14px}.grid{gap:17px 12px}.unit{min-width:48px;padding-inline:6px}input{font-size:20px;padding-inline:8px}}
</style></head><body>
<header class="top"><span>AJUSTES DO CONTROLADOR</span><span class="online">MOTOR PARADO</span><a class="back" href="/">VOLTAR</a></header>
<main><div class="intro"><h1>PARÂMETROS AVANÇADOS</h1><p>Alterações disponíveis somente com o movimento parado. Todos os valores são persistentes.</p></div>
<form id="form">
<section><h2>ADRC</h2><div class="grid">
<div class="field"><label for="wc">Banda do controlador · wc</label><div class="input"><input id="wc" type="number" min="1" max="100" step="0.1" required><span class="unit">rad/s</span></div></div>
<div class="field"><label for="wo">Banda do observador · wo</label><div class="input"><input id="wo" type="number" min="1" max="300" step="0.1" required><span class="unit">rad/s</span></div></div>
<div class="field"><label for="b0">Ganho estimado · b0</label><div class="input"><input id="b0" type="number" min="1" max="2000" step="0.1" required><span class="unit">GAIN</span></div></div>
</div></section>
<section><h2>Motor e saída</h2><div class="grid">
<div class="field"><label for="maxRpm">RPM máxima de comando</label><div class="input"><input id="maxRpm" type="number" min="0.1" max="10" step="0.1" required><span class="unit">RPM</span></div></div>
<div class="field"><label for="physRpm">RPM física estimada</label><div class="input"><input id="physRpm" type="number" min="0.1" max="10" step="0.1" required><span class="unit">RPM</span></div></div>
<div class="field"><label for="powerLimit">Limite de potência</label><div class="input"><input id="powerLimit" type="number" min="0" max="100" step="1" required><span class="unit">%</span></div></div>
<div class="field"><label for="pwmHz">Frequência PWM</label><div class="input"><input id="pwmHz" type="number" min="500" max="20000" step="100" required><span class="unit">Hz</span></div></div>
<div class="field"><label for="minPwm">PWM mínimo em movimento</label><div class="input"><input id="minPwm" type="number" min="0" max="45" step="1" required><span class="unit">%</span></div><div class="hint">Piso do anti-stall; máximo 45%</div></div>
</div></section>
<section><h2>Perfil e chegada</h2><div class="grid">
<div class="field"><label for="stopWindow">Janela de chegada</label><div class="input"><input id="stopWindow" type="number" min="0.2" max="20" step="0.1" required><span class="unit">°</span></div></div>
<div class="field"><label for="stopSamples">Amostras para parar</label><div class="input"><input id="stopSamples" type="number" min="1" max="20" step="1" required><span class="unit">N</span></div></div>
<div class="field"><label for="accelMs">Rampa de aceleração</label><div class="input"><input id="accelMs" type="number" min="50" max="2000" step="10" required><span class="unit">ms</span></div></div>
<div class="field"><label for="decelMs">Rampa de desaceleração</label><div class="input"><input id="decelMs" type="number" min="50" max="2000" step="10" required><span class="unit">ms</span></div></div>
</div></section>
<section><h2>Kick de partida</h2><div class="grid">
<div class="field"><label for="kickPct">Potência do kick</label><div class="input"><input id="kickPct" type="number" min="0" max="100" step="1" required><span class="unit">%</span></div></div>
<div class="field"><label for="kickMs">Duração do kick</label><div class="input"><input id="kickMs" type="number" min="0" max="1000" step="10" required><span class="unit">ms</span></div></div>
</div></section>
<section><h2>Proteção de stall</h2><div class="grid">
<div class="field"><label for="stallMs">Tempo para detectar</label><div class="input"><input id="stallMs" type="number" min="100" max="10000" step="100" required><span class="unit">ms</span></div></div>
<div class="field"><label for="stallVel">Velocidade considerada parada</label><div class="input"><input id="stallVel" type="number" min="0.1" max="20" step="0.1" required><span class="unit">°/s</span></div></div>
</div></section>
<section><h2>Estimador de velocidade</h2><div class="grid">
<div class="field"><label for="velWindow">Janela de medição</label><div class="input"><input id="velWindow" type="number" min="20" max="1000" step="10" required><span class="unit">ms</span></div></div>
<div class="field"><label for="velSamples">Número de amostras</label><div class="input"><input id="velSamples" type="number" min="2" max="20" step="1" required><span class="unit">N</span></div></div>
</div></section>
<div class="actions"><button class="save" id="save" type="submit">SALVAR AJUSTES</button><div class="message" id="message">Carregando configuração…</div></div>
</form></main>
<script>
const ids=['wc','wo','b0','maxRpm','physRpm','powerLimit','pwmHz','minPwm','stopWindow','stopSamples','accelMs','decelMs','kickPct','kickMs','stallMs','stallVel','velWindow','velSamples'];const $=id=>document.getElementById(id);
async function api(body){let o={method:body?'POST':'GET'};if(body){o.headers={'Content-Type':'application/x-www-form-urlencoded'};o.body=new URLSearchParams(body)}let r=await fetch('/api/settings',o),j=await r.json();if(!r.ok)throw Error(j.error||'Falha na operação');return j}
function msg(t,e=false){$('message').textContent=t;$('message').className='message'+(e?' error':'')}
function render(s){ids.forEach(id=>$(id).value=s[id]);$('save').disabled=!s.canEdit;document.querySelector('.online').textContent=s.canEdit?'MOTOR PARADO':'AJUSTES BLOQUEADOS'}
async function load(){try{render(await api());msg('Configuração persistente carregada')}catch(e){msg(e.message,true)}}
$('form').onsubmit=async e=>{e.preventDefault();let b={};ids.forEach(id=>b[id]=$(id).value);try{render(await api(b));msg('Ajustes salvos e aplicados')}catch(x){msg(x.message,true)}};load();
</script></body></html>
)HTML";
