// Debounce posture clicks, keep only the latest target; STOP cancels pending work.
class LatestTarget {
  constructor(send,schedule=setTimeout,cancel=clearTimeout){this.send=send;this.schedule=schedule;this.clear=cancel;this.pending=null;this.timer=null;this.busy=false}
  push(value){this.pending=value;if(this.timer!==null)this.clear(this.timer);if(!this.busy)this.timer=this.schedule(()=>this.flush(),200)}
  cancel(){if(this.timer!==null)this.clear(this.timer);this.timer=null;this.pending=null}
  async flush(){this.timer=null;if(this.pending===null||this.busy)return;const value=this.pending;this.pending=null;this.busy=true;try{await this.send(value)}finally{this.busy=false;if(this.pending!==null)this.timer=this.schedule(()=>this.flush(),200)}}
}
if(typeof module!=='undefined')module.exports=LatestTarget;
function mergeDirection(current,dirty,reported){return dirty&&current!==String(reported)?{value:current,dirty:true}:{value:String(reported),dirty:false}}
function teachingLabel(state){return ({VALID:'已核验，可用',UNVERIFIED:'站趴点已载入，舵机配置尚未核验',CONFIG_READ_FAILED:'舵机配置读取失败，请查看诊断日志',EMPTY:'未记录站趴点',NEED_STAND:'缺少站立点',NEED_LIE:'缺少趴下点',WRAP:'跨编码器零点，需回中重录',NEAR_ZERO:'点位靠近零点，需回中',TOO_CLOSE:'站趴两点过近',STALE_ZERO:'零偏或相位已改变，需重录',HW_LIMIT:'点位超出舵机硬件限位'})[state]||state}
if(typeof module!=='undefined'){module.exports.mergeDirection=mergeDirection;module.exports.teachingLabel=teachingLabel}
class LogCursor {
  constructor(){this.session=null;this.after=0}
  accept(page){
    if(!page)return [];
    if(this.session!==null&&this.session!==page.session){this.session=page.session;this.after=0;return ['——网页服务重新启动，保留先前显示日志——']}
    this.session=page.session;const result=[];
    if(page.gap)result.push('——部分历史日志未同步到网页，可导出本次服务的完整日志——');
    for(const record of page.records||[])if(record.id>this.after){this.after=record.id;result.push(record.text)}
    return result;
  }
}
class LegacyLogCursor {
  constructor(){this.previous=[]}
  accept(lines){
    if(!Array.isArray(lines))return [];
    let overlap=Math.min(this.previous.length,lines.length);
    for(;overlap>0;--overlap){
      if(this.previous.slice(-overlap).every((v,i)=>v===lines[i]))break;
    }
    this.previous=lines.slice();return lines.slice(overlap);
  }
}
if(typeof module!=='undefined')module.exports.LegacyLogCursor=LegacyLogCursor;
if(typeof module!=='undefined')module.exports.LogCursor=LogCursor;
function isPeriodicLog(line){return /^(IMU |IMULEVEL |YAWDRIVE |HOST_ALIVE |role=|wheel=|WDIR=|TEACHDATA |DIRDATA |servo=|servos_active=|unloaded_setup=|>>> HB$)/.test(line)}
if(typeof module!=='undefined')module.exports.isPeriodicLog=isPeriodicLog;
function mixDrive(forward,turn,speed,spin,curve=.7,turnForward=0,limits={speed:12,turn:4},analog=false){
  if(![forward,turn,speed,spin,curve,turnForward].every(Number.isFinite)||speed<.1||speed>limits.speed||spin<0||spin>limits.turn||curve<.1||curve>1||turnForward<0||turnForward>limits.speed)throw Error('驾驶参数超限');
  let f=forward*speed,t=turn*(analog||forward===0?spin:Math.min(limits.turn,speed*curve));
  if(!analog&&forward===0&&turn!==0) f=turnForward;
  const peak=Math.max(Math.abs(f-t),Math.abs(f+t));
  if(peak>limits.speed){const scale=(limits.speed-.0002)/peak;f*=scale;t*=scale}
  return {f,t,left:f-t,right:f+t};
}
if(typeof module!=='undefined')module.exports.mixDrive=mixDrive;

function imuView(line){
  const d=Object.fromEntries([...line.matchAll(/([A-Za-z_]+)=([^ ;]+)/g)].map(m=>[m[1],m[2]]));
  const keys=['roll','pitch','yaw_rel','temp','age_ms','sample_ms'];
  const valid=line.startsWith('IMU ')&&d.valid==='1'&&keys.every(k=>d[k]!==undefined&&Number.isFinite(Number(d[k])))&&Number(d.age_ms)>=0&&Number(d.age_ms)<=200;
  return {valid,calibrated:d.calibrated==='1',saved:d.saved==='1',busy:d.cal_busy==='1',samples:Number(d.samples)||0,
    roll:valid?Number(d.roll):null,pitch:valid?Number(d.pitch):null,yaw:valid?Number(d.yaw_rel):null,temp:valid?Number(d.temp):null};
}
if(typeof module!=='undefined')module.exports.imuView=imuView;

function transportLabel(mode){return {serial:'USB',wifi:'Wi-Fi',bluetooth:'蓝牙'}[mode]||'未知连接'}
function transportHint(mode){
  if(mode==='bluetooth')return '蓝牙：先在Windows配对 QUAD-Single-BT，再选该设备的传出COM口；如提示PIN，填246810。切换前先托住车身并断开原连接。';
  if(mode==='wifi')return 'Wi-Fi：电脑连接 QUAD-Single 热点，目标IP为192.168.4.1。切换前先托住车身并断开原连接。';
  return 'USB：选择ESP32的数据线COM口。';
}
function portLabel(port,details){const found=(details||[]).find(p=>p.device===port);return found&&found.description?port+' · '+found.description:port;}
if(typeof module!=='undefined')Object.assign(module.exports,{transportLabel,transportHint,portLabel});
