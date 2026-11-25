const $=id=>document.getElementById(id),client=crypto.randomUUID();let seq=0,held=null,pointer=null,timer=null,polling=false,connected=false,mine=false,ready=false;
let driveLimits={speed:12,turn:4};
let turnRevision=null,yawMode=false;
const names=['左前','右前','左后','右后']; const directionRecords={},directionDrafts={};
async function api(path,data={}){const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Key':window.CONSOLE_KEY},body:JSON.stringify({client,...data})});const d=await r.json();if(!r.ok)throw Error(d.error);return d}
function note(t){$('notice').textContent=t}
async function sendCommand(command){try{await api('command',{seq:++seq,command,fixed:command.startsWith('IMU ')?$('imufixed').checked:$('fixed').checked});note('已发送：'+command)}catch(e){note(e.message)}}
const postureQueue=new LatestTarget(sendCommand);
function cmd(command){command=command.trim().toUpperCase();if(command==='SAVEWHEELS'){if(Object.keys(directionRecords).length!==4){sendCommand('DIRSHOW');note('请先读取四轮方向，再保存。');return}if([1,2,3,4].some(i=>$('dir'+i).value!==directionRecords[i].value)){note('方向修改尚未应用，请逐行应用确认后保存。');return}}if(command==='LOADWHEELS'){for(const i of [1,2,3,4])directionDrafts[i]=false;}if(/^(BLEND|POSTURE) /.test(command)){postureQueue.push(command);note('准备执行最新姿态目标：'+command);return}if(['STOP','DISARM','RELAX','RELAX LIE','POSTURESTOP'].includes(command))postureQueue.cancel();return sendCommand(command)}
function clearWheelInput(){
  key=null;held=null;pointer=null;clearInterval(timer);timer=null;
  centerJoystick();
  document.querySelectorAll('[data-testwheel]').forEach(b=>b.classList.remove('pressed'));
}
function stopWheels(){clearWheelInput();if(connected&&mine)cmd('WSTOP')}
function stop(){endPostureKey(false);clearWheelInput();if(connected&&mine)cmd('STOP')}
function renew(){
  if(!held)return;
  if(held.wheel){
    api('wheeltest',{seq:++seq,wheel:held.wheel,speed:held.speed}).catch(e=>{stop();note(e.message)});
    return;
  }
  const [forward,turn]=held;
  const speed=Number($('speed').value),cap=Number($('turn').value),curve=Number($('curve').value);
  const turnForward=$('tightturn').checked?Number($('turnfwd').value):0;
  let data;
  try{
    if(yawMode&&(held.joystick||forward===0)&&turn!==0&&cap>0){
      const f=held.joystick?forward*speed:turnForward;
      const yaw=turn*Number($('yawrate').value);
      data={f,t:cap,yaw};
    }else{
      const {f,t}=mixDrive(forward,turn,speed,cap,curve,turnForward,driveLimits,!!held.joystick);
      data={f,t};
    }
  }catch(e){stop();note(e.message);return;}
  api('drive',{seq:++seq,...data}).catch(e=>{stop();note(e.message)});
}
function start(v){if(!connected||!mine||!ready){note('请先连接并确认主控，完成舵机使能与轮驱动启用。');return}held=v;if(!timer)timer=setInterval(renew,100);renew()}
for(let i=1;i<=4;i++){$('calrows').insertAdjacentHTML('beforeend',`<tr><td>${i} ${names[i-1]}</td><td><select id="dir${i}"><option>1</option><option>-1</option></select> <button data-dir="${i}">应用确认</button></td><td id="saved${i}">待读取</td><td id="points${i}">—</td><td id="range${i}">—</td><td id="teach${i}">待读取</td><td><button data-cmd="TEACH STAND ${i}">记录站</button> <button data-cmd="TEACH LIE ${i}">记录趴</button> <button data-cmd="SETMID ${i} FIXED">原位回中</button></td></tr>`)}
document.querySelectorAll('[data-cmd]').forEach(b=>b.onclick=()=>cmd(b.dataset.cmd));document.querySelectorAll('[data-dir]').forEach(b=>b.onclick=()=>cmd(`WDIR ${b.dataset.dir} ${$('dir'+b.dataset.dir).value}`));
const joystick=$('joystick'),joystickThumb=$('joystick-thumb');
function centerJoystick(){
  joystickThumb.style.transform='translate(-50%,-50%)';
  joystick.classList.remove('pressed');
}
function aimJoystick(e){
  const box=joystick.getBoundingClientRect(),radius=Math.min(box.width,box.height)*0.35;
  const x=(e.clientX-box.left-box.width/2)/radius,y=(e.clientY-box.top-box.height/2)/radius;
  const length=Math.hypot(x,y);
  const scale=length>0.12?Math.min(1,(length-0.12)/0.88)/length:0;
  const forward=-y*scale,turn=-x*scale;
  joystickThumb.style.transform=`translate(-50%,-50%) translate(${-turn*radius}px,${-forward*radius}px)`;
  const direction=Object.assign([forward,Math.abs(turn)<0.01?0:turn],{joystick:true});
  if(!held)start(direction);
  else held=direction;
}
joystick.onpointerdown=e=>{
  if(pointer!==null || e.button!==0)return;
  e.preventDefault();
  if(held)stopWheels();
  pointer=e.pointerId;
  joystick.setPointerCapture(pointer);
  joystick.classList.add('pressed');
  aimJoystick(e);
};
joystick.onpointermove=e=>{if(e.pointerId===pointer)aimJoystick(e)};
joystick.onpointerup=joystick.onpointercancel=joystick.onlostpointercapture=e=>{
  if(e.pointerId===pointer)stopWheels();
};
$('stop').onclick=$('padstop').onclick=stop;window.addEventListener('blur',stop);document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});window.addEventListener('pagehide',()=>{endPostureKey(false);postureQueue.cancel();held=null;clearInterval(timer);timer=null;if(connected&&mine)fetch('/api/disconnect',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Console-Key':window.CONSOLE_KEY},body:JSON.stringify({client})}).catch(()=>{})});
const keys={q:[1,1],w:[1,0],e:[1,-1],a:[0,1],d:[0,-1],z:[-1,1],s:[-1,0],c:[-1,-1]};let key=null;
window.addEventListener('keydown',e=>{if(e.key==='Escape'){stop();return}if(e.ctrlKey||e.altKey||e.metaKey||e.target.isContentEditable||['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;const k=e.key.toLowerCase();
if(currentView!=='operate'||e.repeat)return;
if(k==='r'||k==='f'){e.preventDefault();jogSelectedLeg(k==='r'?-1:1);return}
if(keys[k]&&pointer===null){e.preventDefault();key=k;start(keys[k])}});window.addEventListener('keyup',e=>{if(e.key.toLowerCase()===key)stopWheels()});
$('connect').onclick=async()=>{try{await api('connect',{port:$('port').value,mode:$('transport').value,host:$('wifihost').value.trim()});note('已打开'+transportLabel($('transport').value)+'连接，等待主控状态。')}catch(e){note(e.message)}};$('disconnect').onclick=async()=>{stop();try{const d=await api('disconnect');connected=false;mine=false;ready=false;renderTurnSettings(null);renderServoTemperatures([]);note(d.released?'已断开，四舵机卸力已确认。':'已断开，卸力未获全部确认，请检查实物。')}catch(e){note(e.message)}};
for(const id of ['speed','turn','curve','turnfwd'])$(id).oninput=()=>$(id+'out').textContent=$(id).value+(id==='curve'?'':' rad/s');
$('tightturn').onchange=()=>{if(held)stopWheels();$('turnfwd').disabled=!$('tightturn').checked};
$('yawrate').oninput=()=>{$('yawrateout').textContent=$('yawrate').value+' °/s'};
$('saveturn').onclick=()=>{
  const args=`${Number($('turnfwd').value).toFixed(1)} ${Number($('turn').value).toFixed(1)} ${$('tightturn').checked?1:0}`;
  sendCommand('TURNCONFIG '+args+(yawMode?' '+$('yawrate').value:''));
};
function renderYawDrive(enabled,lines){
  yawMode=enabled;
  $('yawsetting').hidden=$('yawfeedback').hidden=!enabled;
  $('turnlabel').textContent=enabled?'转向差速上限':'A/D 转向差速';
  if(!enabled)return;
  const line=lines.findLast(s=>s.startsWith('YAWDRIVE ')||s.startsWith('YAW_BLOCKED '))||'';
  const status=fields(line);
  $('yawfeedback').textContent=line.startsWith('YAW_BLOCKED ')?'车身转速控制已停止，请检查 IMU':
    status.active==='1'?`车身目标 ${status.target} °/s · 实际 ${status.measured} °/s`:'车身转速闭环待命';
}
function renderTurnSettings(config){
  $('saveturn').disabled=!config;
  if(!config){
    turnRevision=null;
    $('turnsaved').textContent='';
    return;
  }
  if(turnRevision===config.revision)return;
  if(turnRevision===null){
    $('turnfwd').value=String(config.forward);
    $('turn').value=String(config.difference);
    if(config.yaw!=null) {
      $('yawrate').value=String(config.yaw);
      $('yawrate').oninput();
    }
    $('turnfwd').oninput();
    $('turn').oninput();
    $('tightturn').checked=config.enabled;
    $('turnfwd').disabled=!config.enabled;
  }
  turnRevision=config.revision;
  $('turnsaved').textContent=({
    read:config.stored?'已读取主控设置':'使用默认设置',
    saved:'已保存到主控',busy:'关闭轮驱动后再保存',invalid:'转向设置超限',failed:'保存失败，请查看日志'
  })[config.result]||'';
}
$('commandform').onsubmit=e=>{e.preventDefault();cmd($('command').value);$('command').value=''};
function fields(s){return Object.fromEntries([...s.matchAll(/([A-Za-z_]+)=([^ ;]+)/g)].map(m=>[m[1],m[2]]))}
function render(lines){const latestDirs={};for(const s of lines){const d=fields(s);if(s.startsWith('role=SINGLE_CONTROLLER ')){$('link').textContent='单主控';$('wheelstate').textContent=d.state+' · '+d.loop_hz+' Hz'}if(s.startsWith('IMULEVEL '))$('levelstatus').textContent=`${({OFF:'已暂停',FAULT:'联动停止：请查看调试日志',CENTERING:'正在到达中位',ADJUSTING:'正在调平',HOLDING:'已达到目标，可手动操作舵机',PAUSED_DRIVE:'轮驱动校准中，暂停修正',WAIT_DRIVE_STABLE:'行驶波动较大，暂时保持腿姿',WAIT_STILL:'等待车身静止',WAIT_SERVO:'等待舵机跟上，目标保持'})[d.state]||d.state} · 参考腿 ${d.reference} · 目标 X ${d.target_x}° / Y ${d.target_y}°`;if(s.startsWith('servos_active='))$('servostate').textContent=`${d.servos_active==='1'?'已使能':'未使能'} / ${d.servo_fault==='0'?'正常':'故障'}`;if(s.startsWith('servo=')&&d.servo>=1&&d.servo<=4)$('feedback'+d.servo).textContent=`编码器 ${d.pos} · ${d.valid==='1'?'反馈有效':'反馈异常'}\n${d.V} V · ${d.C} °C`;if(s.startsWith('DIRDATA ')&&d.id>=1&&d.id<=4)latestDirs[d.id]=d;if(s.startsWith('DIRDATA ')&&d.id>=1&&d.id<=4)$('saved'+d.id).textContent=`当前 ${d.value} / 已存 ${d.saved_present==='1'?d.saved_value:'无'} · ${d.verified==='1'?'已确认':'未确认'} · ${d.dirty==='1'?'未保存':'已保存'}`;if(s.startsWith('TEACHDATA ')&&d.id>=1&&d.id<=4){$('points'+d.id).textContent=d.stand+' / '+d.lie;$('range'+d.id).textContent=d.sw_min+' ～ '+d.sw_max;$('teach'+d.id).textContent=teachingLabel(d.state)}}for(const [id,d] of Object.entries(latestDirs)){const merged=mergeDirection($('dir'+id).value,!!directionDrafts[id],d.value);$('dir'+id).value=merged.value;directionDrafts[id]=merged.dirty;directionRecords[id]=d;}}
function renderServoTemperatures(temperatures){
  const online=connected&&ready;
  const readings=online&&Array.isArray(temperatures)?temperatures:[];
  let count=0,hot=false;
  for(let i=0;i<4;i++){
    const value=readings[i],visible=Number.isFinite(value);
    const item=$('servotemp'+(i+1));
    item.textContent=visible?value+' °C':'—';
    const high=visible&&value>=60;
    item.dataset.hot=String(high);
    if(visible)count++;
    hot=hot||high;
  }
  const state=$('servotempstate');
  state.textContent=!online?'未连接':hot?'温度达到 60 °C 保护线':count===4?'每 2 秒更新':count?'部分舵机无反馈':'等待舵机反馈';
  state.dataset.hot=String(hot);
}
function updateDriveLimits(advertised){
  const speed=advertised?.speed===16?16:12;
  const turn=speed===16&&[6,16].includes(advertised?.turn)?advertised.turn:4;
  if(driveLimits.speed===speed&&driveLimits.turn===turn)return;
  driveLimits={speed,turn};
  for(const [id,limit] of [['speed',speed],['turnfwd',speed],['turn',turn]]){
    const slider=$(id);
    slider.max=String(limit);
    if(Number(slider.value)>limit){slider.value=String(limit);slider.oninput()}
  }
  $('maxwheelspeed').textContent=String(speed);
  $('maxturnspeed').textContent=String(turn);
}
async function poll(){if(polling)return;polling=true;try{const d=await api('state',{after:logCursor.after});connected=d.connected;mine=d.mine;ready=d.master;updateDriveLimits(connected&&ready?d.drive_limits:null);renderTurnSettings(connected&&mine&&ready?d.turn_config:null);$('connection').textContent=connected?(mine?(transportLabel(d.transport)+' 已连接'):'其他窗口控制中'):'未连接设备';if(!connected||!mine||!ready){endPostureKey(false);clearWheelInput()}const options=JSON.stringify([d.ports,d.port_details||[]]);if($('port').dataset.ports!==options){const old=$('port').value;$('port').replaceChildren(...d.ports.map(p=>new Option(portLabel(p,d.port_details),p)));if(d.ports.includes(old))$('port').value=old;$('port').dataset.ports=options}render(d.lines);renderYawDrive(!!(connected&&mine&&ready&&d.yaw_drive),d.lines);renderImu(d.lines);renderServoTemperatures(d.servo_temperatures);
if(!connected||!ready)for(const id of ['link','wheelstate','servostate'])$(id).textContent='—';if(d.log){logHistory.push(...logCursor.accept(d.log))}else{logHistory.push(...legacyLogCursor.accept(d.lines));if(!warnedOldLog){warnedOldLog=true;note('后台为旧版：已兼容显示现有日志。先托住车身，再重启网页服务启用完整日志保存。')}}flushLog()}catch(e){endPostureKey(false);clearWheelInput();connected=false;ready=false;renderServoTemperatures([]);note('本机服务未响应，已停止续发运动指令。')}finally{polling=false}}


const postureKeys={'1':['ALL',1],'2':['ALL',-1],'3':['FRONT',1],'4':['FRONT',-1],'5':['REAR',1],'6':['REAR',-1]};
let postureKey=null,postureTimer=null;
function endPostureKey(send=true){const active=postureKey!==null;postureKey=null;clearInterval(postureTimer);postureTimer=null;postureQueue.cancel();document.querySelectorAll('[data-posture-key]').forEach(b=>b.classList.remove('pressed'));if(active&&send&&connected&&mine)sendCommand('POSTURESTOP')}
function beginPostureKey(k){if(!connected||!mine||!ready){note('请先连接主控并使能舵机、读取有效站趴点位');return}endPostureKey(false);postureKey=k;const [group,sign]=postureKeys[k];const shift=()=>sendCommand('SHIFT '+group+' '+(sign*0.04).toFixed(4));shift();postureTimer=setInterval(shift,100)}
document.querySelectorAll('[data-posture-key]').forEach(b=>{let activePointer=null;b.onpointerdown=e=>{e.preventDefault();activePointer=e.pointerId;b.setPointerCapture(e.pointerId);beginPostureKey(b.dataset.postureKey);if(postureKey===b.dataset.postureKey)b.classList.add('pressed')};b.onpointerup=b.onpointercancel=b.onlostpointercapture=e=>{if(activePointer!==e.pointerId)return;activePointer=null;if(postureKey===b.dataset.postureKey)endPostureKey()}});
window.addEventListener('keydown',e=>{if(e.ctrlKey||e.altKey||e.metaKey||e.target.isContentEditable||['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(currentView==='operate'&&postureKeys[e.key]){e.preventDefault();if(!e.repeat&&postureKey!==e.key)beginPostureKey(e.key)}});
window.addEventListener('keyup',e=>{if(postureKey===e.key){e.preventDefault();endPostureKey()}});



let currentView='operate';
function view(name){stop();currentView=name;document.querySelectorAll('[data-view]').forEach(e=>e.hidden=e.dataset.view!==name);$('viewoperate').setAttribute('aria-pressed',String(name==='operate'));$('viewdebug').setAttribute('aria-pressed',String(name==='debug'))}
$('viewoperate').onclick=()=>view('operate');$('viewdebug').onclick=()=>view('debug');
$('transport').onchange=()=>{stop();$('transporthint').textContent=transportHint($('transport').value);$('port').hidden=$('transport').value==='wifi';$('wifihost').hidden=$('transport').value!=='wifi';note('先断开现有连接，再连接所选模式。')};
document.querySelectorAll('[data-testwheel]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();if(!ready||!mine)return;const speed=Number($('testspeed').value)*Number(b.dataset.testwheel);if(!Number.isFinite(speed)||Math.abs(speed)>1||Math.abs(speed)<.1){note('测试速度须为0.1～1');return}pointer=e.pointerId;b.setPointerCapture(e.pointerId);held={wheel:Number($('testwheel').value),speed};if(!timer)timer=setInterval(renew,100);renew()};b.onpointerup=b.onpointercancel=b.onlostpointercapture=e=>{if(pointer===e.pointerId)stop()}});
function levelRequest(enable=false){const ref=Number($('levelref').value),x=Number($('levelx').value),y=Number($('levely').value);if(![ref,x,y].every(Number.isFinite)||![1,2,3,4].includes(ref)||Math.abs(x)>5||Math.abs(y)>5){note('参考腿1～4，X/Y目标角度限±5°');return}cmd((enable?'SERVOS ON ':'LEVEL ')+ref+' '+x.toFixed(2)+' '+y.toFixed(2))}
$('servoenable').onclick=()=>levelRequest(true);$('levelapply').onclick=()=>levelRequest();
function jogSelectedLeg(step){
  if(!connected||!mine||!ready){note('请先连接主控并使能舵机。');return}
  return cmd('J '+$('testleg').value+' '+step);
}
$('legplus').onclick=()=>jogSelectedLeg(1);$('legminus').onclick=()=>jogSelectedLeg(-1);$('leginfo').onclick=()=>cmd('SERVOINFO '+$('testleg').value);

document.querySelectorAll('nav a').forEach(a=>a.addEventListener('click',()=>{if(a.hash!=='#console')view(a.hash==='#calibration'?'debug':'operate')}));

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

let lastImuLine='',lastImuAt=0;
function renderImu(lines){
  const line=[...lines].reverse().find(s=>s.startsWith('IMU '));
  if(line&&line!==lastImuLine){lastImuLine=line;lastImuAt=performance.now();}
  const v=imuView(line||'');
  const fresh=connected&&mine&&ready&&v.valid&&performance.now()-lastImuAt<1800;
  $('imustate').textContent=!connected||!mine||!ready?'未连接':v.busy?'静止校准 '+v.samples+'/200':!fresh?'数据无效 / 未更新':v.calibrated?(v.saved?'已校准 · 已保存':'已校准 · 未保存'):'在线 · 未校准';
  for(const k of ['roll','pitch','yaw','temp'])$('imu'+k).textContent=fresh?v[k].toFixed(k==='temp'?1:2)+(k==='temp'?' °C':' °'):'—';
}
