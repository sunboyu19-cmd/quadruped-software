const $=id=>document.getElementById(id),client=crypto.randomUUID();let seq=0,held=null,pointer=null,timer=null,polling=false,connected=false,mine=false,ready=false;
const names=['左前','右前','左后','右后']; const directionRecords={},directionDrafts={};
async function api(path,data={}){const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Key':window.CONSOLE_KEY},body:JSON.stringify({client,...data})});const d=await r.json();if(!r.ok)throw Error(d.error);return d}
function note(t){$('notice').textContent=t}
async function sendCommand(command){try{await api('command',{seq:++seq,command,fixed:$('fixed').checked});note('已发送：'+command)}catch(e){note(e.message)}}
const postureQueue=new LatestTarget(sendCommand);
function cmd(command){command=command.trim().toUpperCase();if(command==='SAVEWHEELS'){if(Object.keys(directionRecords).length!==4){sendCommand('DIRSHOW');note('请先读取四轮方向，再保存。');return}if([1,2,3,4].some(i=>$('dir'+i).value!==directionRecords[i].value)){note('方向修改尚未应用，请逐行应用确认后保存。');return}}if(command==='LOADWHEELS'){for(const i of [1,2,3,4])directionDrafts[i]=false;}if(/^(BLEND|POSTURE) /.test(command)){postureQueue.push(command);note('准备执行最新姿态目标：'+command);return}if(['STOP','DISARM','RELAX','POSTURESTOP'].includes(command))postureQueue.cancel();return sendCommand(command)}
function stop(){endPostureKey(false);held=null;pointer=null;clearInterval(timer);timer=null;document.querySelectorAll('.pressed').forEach(b=>b.classList.remove('pressed'));if(connected&&mine)cmd('STOP')}
function renew(){if(!held)return;if(held.wheel){api('wheeltest',{seq:++seq,wheel:held.wheel,speed:held.speed}).catch(e=>{stop();note(e.message)});return;}let f,t;try{({f,t}=mixDrive(held[0],held[1],Number($('speed').value),Number($('turn').value),Number($('curve').value)))}catch(e){stop();note(e.message);return;}api('drive',{seq:++seq,f,t}).catch(e=>{stop();note(e.message)})}
function start(v){if(!connected||!mine||!ready){note('请先连接并确认A板，完成舵机使能与轮驱动启用。');return}held=v;if(!timer)timer=setInterval(renew,100);renew()}
for(let i=1;i<=4;i++){$('jointcards').insertAdjacentHTML('beforeend',`<div class="joint"><strong>0${i} / ${names[i-1]}</strong><p id="feedback${i}">等待反馈<br>— V · — °C</p><button data-cmd="J ${i} -1">−1°</button> <button data-cmd="J ${i} 1">+1°</button></div>`);$('calrows').insertAdjacentHTML('beforeend',`<tr><td>${i} ${names[i-1]}</td><td><select id="dir${i}"><option>1</option><option>-1</option></select> <button data-dir="${i}">应用确认</button></td><td id="saved${i}">待读取</td><td id="points${i}">—</td><td id="range${i}">—</td><td id="teach${i}">待读取</td><td><button data-cmd="TEACH STAND ${i}">记录站</button> <button data-cmd="TEACH LIE ${i}">记录趴</button> <button data-cmd="SETMID ${i} FIXED">原位回中</button></td></tr>`)}
document.querySelectorAll('[data-cmd]').forEach(b=>b.onclick=()=>cmd(b.dataset.cmd));document.querySelectorAll('[data-dir]').forEach(b=>b.onclick=()=>cmd(`WDIR ${b.dataset.dir} ${$('dir'+b.dataset.dir).value}`));
document.querySelectorAll('[data-drive]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();pointer=e.pointerId;b.setPointerCapture(e.pointerId);start(b.dataset.drive.split(',').map(Number));if(held)b.classList.add('pressed')};b.onpointerup=b.onpointercancel=b.onlostpointercapture=e=>{if(pointer===e.pointerId)stop()}});
$('stop').onclick=$('padstop').onclick=stop;window.addEventListener('blur',stop);document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});window.addEventListener('pagehide',()=>{endPostureKey(false);postureQueue.cancel();held=null;clearInterval(timer);timer=null;if(connected&&mine)fetch('/api/disconnect',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Console-Key':window.CONSOLE_KEY},body:JSON.stringify({client})}).catch(()=>{})});
const keys={w:[1,0],s:[-1,0],a:[0,1],d:[0,-1]};let key=null;
window.addEventListener('keydown',e=>{if(e.key==='Escape'){stop();return}if(['INPUT','SELECT','TEXTAREA','BUTTON'].includes(e.target.tagName))return;let k=e.key.toLowerCase();if(currentView==='operate'&&keys[k]&&!e.repeat){e.preventDefault();key=k;start(keys[k])}});window.addEventListener('keyup',e=>{if(e.key.toLowerCase()===key){key=null;stop()}});
$('connect').onclick=async()=>{try{await api('connect',{port:$('port').value,mode:$('transport').value,host:$('wifihost').value.trim()});note('已打开串口，等待A板状态。')}catch(e){note(e.message)}};$('disconnect').onclick=async()=>{stop();try{const d=await api('disconnect');connected=false;mine=false;ready=false;note(d.released?'已断开，四舵机卸力已确认。':'已断开，卸力未获全部确认，请检查实物。')}catch(e){note(e.message)}};
for(const id of ['speed','turn','curve'])$(id).oninput=()=>$(id+'out').textContent=$(id).value+(id==='curve'?'':' rad/s');
$('commandform').onsubmit=e=>{e.preventDefault();cmd($('command').value);$('command').value=''};
function fields(s){return Object.fromEntries([...s.matchAll(/([A-Za-z_]+)=([^ ;]+)/g)].map(m=>[m[1],m[2]]))}
function render(lines){const latestDirs={};for(const s of lines){const d=fields(s);if(s.startsWith('role=FRONT_MASTER ')){$('link').textContent=d.link==='1'?'在线':'离线';$('wheelstate').textContent=d.state+' / '+d.peer_state}if(s.startsWith('servos_active='))$('servostate').textContent=`${d.servos_active==='1'?'已使能':'未使能'} / ${d.servo_fault==='0'?'正常':'故障'}`;if(s.startsWith('servo=')&&d.servo>=1&&d.servo<=4)$('feedback'+d.servo).textContent=`编码器 ${d.pos} · ${d.valid==='1'?'反馈有效':'反馈异常'}\n${d.V} V · ${d.C} °C`;if(s.startsWith('DIRDATA ')&&d.id>=1&&d.id<=4)latestDirs[d.id]=d;if(s.startsWith('DIRDATA ')&&d.id>=1&&d.id<=4)$('saved'+d.id).textContent=`当前 ${d.value} / 已存 ${d.saved_present==='1'?d.saved_value:'无'} · ${d.verified==='1'?'已确认':'未确认'} · ${d.dirty==='1'?'未保存':'已保存'}`;if(s.startsWith('TEACHDATA ')&&d.id>=1&&d.id<=4){$('points'+d.id).textContent=d.stand+' / '+d.lie;$('range'+d.id).textContent=d.sw_min+' ～ '+d.sw_max;$('teach'+d.id).textContent=teachingLabel(d.state)}}for(const [id,d] of Object.entries(latestDirs)){const merged=mergeDirection($('dir'+id).value,!!directionDrafts[id],d.value);$('dir'+id).value=merged.value;directionDrafts[id]=merged.dirty;directionRecords[id]=d;}}
async function poll(){if(polling)return;polling=true;try{const d=await api('state',{after:logCursor.after});connected=d.connected;mine=d.mine;ready=d.master;$('connection').textContent=connected?(mine?(d.transport==='wifi'?'Wi-Fi 已连接':'USB 已连接'):'其他窗口控制中'):'未连接设备';if(!connected||!mine||!ready){endPostureKey(false);held=null;clearInterval(timer);timer=null}const options=d.ports.join('|');if($('port').dataset.ports!==options){const old=$('port').value;$('port').replaceChildren(...d.ports.map(p=>new Option(p,p)));if(d.ports.includes(old))$('port').value=old;$('port').dataset.ports=options}render(d.lines);if(d.log){logHistory.push(...logCursor.accept(d.log))}else{logHistory.push(...legacyLogCursor.accept(d.lines));if(!warnedOldLog){warnedOldLog=true;note('后台为旧版：已兼容显示现有日志。先托住车身，再重启网页服务启用完整日志保存。')}}flushLog()}catch(e){endPostureKey(false);held=null;clearInterval(timer);timer=null;note('本机服务未响应，已停止续发运动指令。')}finally{polling=false}}


function setPercent(v){v=Math.max(0,Math.min(100,Number(v)));if(!Number.isFinite(v))return; $('percent').value=v;$('percentnumber').value=v;$('percentout').textContent=v+'%'}
$('percent').oninput=()=>setPercent($('percent').value);$('percentnumber').oninput=()=>setPercent($('percentnumber').value);
$('applypercent').onclick=()=>{const v=Number($('percentnumber').value);if(!Number.isFinite(v)||v<0||v>100){note('百分比须为0～100');return}cmd('BLEND '+(v/100).toFixed(3))};

function groupPercent(group){const v=Number($('percentnumber').value);if(!Number.isFinite(v)||v<0||v>100){note('百分比须为0～100');return}cmd('BLEND '+group+' '+(v/100).toFixed(3))}
$('applyfront').onclick=()=>groupPercent('FRONT');$('applyrear').onclick=()=>groupPercent('REAR');

const postureKeys={'1':['ALL',1],'2':['ALL',-1],'3':['FRONT',1],'4':['FRONT',-1],'5':['REAR',1],'6':['REAR',-1]};
let postureKey=null,postureTimer=null;
function endPostureKey(send=true){const active=postureKey!==null;postureKey=null;clearInterval(postureTimer);postureTimer=null;postureQueue.cancel();if(active&&send&&connected&&mine)sendCommand('POSTURESTOP')}
function beginPostureKey(k){if(!connected||!mine||!ready){note('请先连接A板并使能舵机、读取有效站趴点位');return}endPostureKey(false);postureKey=k;const [group,sign]=postureKeys[k];const shift=()=>sendCommand('SHIFT '+group+' '+(sign*0.0075).toFixed(4));shift();postureTimer=setInterval(shift,250)}
window.addEventListener('keydown',e=>{if(e.ctrlKey||e.altKey||e.metaKey||['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(currentView==='operate'&&postureKeys[e.key]){e.preventDefault();if(!e.repeat&&postureKey!==e.key)beginPostureKey(e.key)}});
window.addEventListener('keyup',e=>{if(postureKey===e.key){e.preventDefault();endPostureKey()}});



let currentView='operate';
function view(name){stop();currentView=name;document.querySelectorAll('[data-view]').forEach(e=>e.hidden=e.dataset.view!==name)}
$('viewoperate').onclick=()=>view('operate');$('viewdebug').onclick=()=>view('debug');
$('transport').onchange=()=>{stop();$('port').hidden=$('transport').value==='wifi';$('wifihost').hidden=$('transport').value!=='wifi';note('先断开现有连接，再连接所选模式。')};
document.querySelectorAll('[data-testwheel]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();if(!ready||!mine)return;const speed=Number($('testspeed').value)*Number(b.dataset.testwheel);if(!Number.isFinite(speed)||Math.abs(speed)>1||Math.abs(speed)<.1){note('测试速度须为0.1～1');return}pointer=e.pointerId;b.setPointerCapture(e.pointerId);held={wheel:Number($('testwheel').value),speed};if(!timer)timer=setInterval(renew,100);renew()};b.onpointerup=b.onpointercancel=b.onlostpointercapture=e=>{if(pointer===e.pointerId)stop()}});
$('legplus').onclick=()=>cmd('J '+$('testleg').value+' 1');$('legminus').onclick=()=>cmd('J '+$('testleg').value+' -1');$('leginfo').onclick=()=>cmd('SERVOINFO '+$('testleg').value);

document.querySelectorAll('nav a').forEach(a=>a.addEventListener('click',()=>{if(a.hash==='#calibration')view('debug');else if(a.hash!=='#console')view('operate')}));

for(const i of [1,2,3,4])$('dir'+i).addEventListener('change',()=>{directionDrafts[i]=true});

const logCursor=new LogCursor(),legacyLogCursor=new LegacyLogCursor();let warnedOldLog=false;let logHistory=[],logRendered=0;
function flushLog(){if($('pauselog').checked)return;const added=logHistory.slice(logRendered);logRendered=logHistory.length;for(const [id,periodic,auto] of [['logs',false,'autoscroll'],['statuslogs',true,'statusautoscroll']]){const box=$(id),top=box.scrollTop,lines=added.filter(line=>isPeriodicLog(line)===periodic);if(lines.length)box.appendChild(document.createTextNode(lines.join('\n')+'\n'));box.scrollTop=$(auto).checked?box.scrollHeight:top}}
$('logbottom').onclick=()=>{$('pauselog').checked=false;flushLog();$('logs').scrollTop=$('logs').scrollHeight;$('statuslogs').scrollTop=$('statuslogs').scrollHeight};
$('autoscroll').onchange=()=>{if($('autoscroll').checked)$('logs').scrollTop=$('logs').scrollHeight};
$('pauselog').onchange=flushLog;
$('telemetry').onchange=()=>{$('statuslogpanel').hidden=!$('telemetry').checked};
$('statusautoscroll').onchange=()=>{if($('statusautoscroll').checked)$('statuslogs').scrollTop=$('statuslogs').scrollHeight};
$('clearlog').onclick=()=>{logHistory=[];logRendered=0;$('logs').replaceChildren();$('statuslogs').replaceChildren();note('仅清空网页显示，完整日志文件仍保留。')};
$('exportlog').onclick=async()=>{try{const r=await fetch('/api/logexport',{method:'POST',headers:{'Content-Type':'application/json','X-Console-Key':window.CONSOLE_KEY},body:JSON.stringify({client})});if(!r.ok)throw Error('导出失败');const url=URL.createObjectURL(await r.blob());const a=document.createElement('a');a.href=url;a.download='quad-runtime-'+new Date().toISOString().replace(/[:.]/g,'-')+'.log';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}catch(e){note(e.message)}};

setInterval(poll,250);poll();
