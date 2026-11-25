// Exercise the actual browser script with mocked DOM/network: never contacts hardware.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=process.argv[2]||path.resolve(__dirname,'web');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8'),nodes=[],byId=new Map();
class Element {
  constructor(tag,attrs={}){this.tagName=tag.toUpperCase();this.attrs=attrs;this.id=attrs.id;this.dataset={};this.listeners={};this.value=attrs.value||'';this.hidden='hidden' in attrs;this.checked='checked' in attrs;this.scrollTop=0;this.scrollHeight=0;this.textContent='';this.style={};this.isContentEditable=false;const classes=new Set((attrs.class||'').split(' '));this.classList={add:c=>classes.add(c),remove:c=>classes.delete(c),contains:c=>classes.has(c)};for(const [k,v]of Object.entries(attrs))if(k.startsWith('data-'))this.dataset[k.slice(5).replace(/-([a-z])/g,(_,c)=>c.toUpperCase())]=v;this.hash=attrs.href||'';}
  addEventListener(type,fn){(this.listeners[type]??=[]).push(fn)}
  setAttribute(k,v){this.attrs[k]=v}
  setPointerCapture(){}
  insertAdjacentHTML(_,text){parse(text)}
  replaceChildren(){}
  appendChild(){}
}
function parse(text){for(const m of text.matchAll(/<([a-z][a-z0-9]*)\b([^>]*)>/gi)){const attrs={};for(const a of m[2].matchAll(/([\w-]+)(?:="([^"]*)")?/g))attrs[a[1]]=a[2]??'';const e=new Element(m[1],attrs);if(e.tagName==='SELECT'){const first=text.slice(m.index+m[0].length).match(/<option(?: value="([^"]*)")?[^>]*>([^<]*)/);if(first)e.value=first[1]??first[2]};nodes.push(e);if(e.id){assert.ok(!byId.has(e.id),'duplicate id: '+e.id);byId.set(e.id,e)}}}
parse(html);
const listeners={},requests=[],intervals=new Map();let timerId=0;
function queryAll(selector){if(selector==='nav a')return nodes.filter(e=>e.tagName==='A');if(selector==='.pressed')return nodes.filter(e=>e.classList.contains('pressed'));const attr=selector.match(/^\[([^\]]+)\]$/)?.[1];return attr?nodes.filter(e=>attr in e.attrs):[]}
const document={getElementById:id=>byId.get(id)||null,querySelectorAll:queryAll,addEventListener:(t,f)=>(listeners['doc:'+t]??=[]).push(f),createTextNode:t=>({textContent:t}),createElement:t=>new Element(t)};
const context=vm.createContext({document,window:{CONSOLE_KEY:'test',addEventListener:(t,f)=>(listeners[t]??=[]).push(f)},crypto:{randomUUID:()=> 'unit-test'},performance:{now:()=>100},setInterval:(fn,ms)=>{intervals.set(++timerId,{fn,ms});return timerId},clearInterval:id=>intervals.delete(id),setTimeout:()=>++timerId,clearTimeout:()=>{},Option:class{},console,fetch:async(url,options)=>{requests.push({url,...JSON.parse(options.body)});return {ok:true,json:async()=>({connected:true,mine:true,master:true,ports:[],lines:[],log:{session:'test',records:[]}})}}});
vm.runInContext(fs.readFileSync(path.join(root,'latest_target.js'),'utf8'),context);
vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8'),context);
function emit(type,key,opts={}){const e={key,target:{tagName:'BODY'},preventDefault(){},repeat:false,...opts};for(const fn of listeners[type]||[])fn(e)}
const last=()=>requests.at(-1);
(async()=>{
  await new Promise(setImmediate);
  for(const id of ['percent','percentnumber','applypercent','alljoints','jointcards'])assert.equal(byId.has(id),false);
  assert.equal(queryAll('[data-posture-key]').length,6);
  for(const [key,f,t]of [['q',2,1.4],['e',2,-1.4],['z',-2,1.4],['c',-2,-1.4],['w',2,0],['s',-2,0],['a',2,1.6],['d',2,-1.6]]){
    emit('keydown',key);assert.equal(last().url,'/api/drive');assert.equal(last().f,f);assert.equal(last().t,t);
    emit('keyup',key);assert.equal(last().command,'WSTOP');
    assert.equal([...intervals.values()].filter(v=>v.ms===100).length,0);
  }
  for(const [key,group,delta]of [['1','ALL',.04],['2','ALL',-.04],['3','FRONT',.04],['4','FRONT',-.04],['5','REAR',.04],['6','REAR',-.04]]){
    emit('keydown',key);assert.equal(last().command,`SHIFT ${group} ${delta.toFixed(4)}`);
    emit('keyup',key);assert.equal(last().command,'POSTURESTOP');
    assert.equal([...intervals.values()].filter(v=>v.ms===100).length,0);
  }
  for(const key of ['q','1'])for(const opts of [{ctrlKey:true},{altKey:true},{metaKey:true},{target:{tagName:'INPUT'}},{target:{tagName:'SELECT'}},{target:{tagName:'DIV',isContentEditable:true}}]){
    const before=requests.length;emit('keydown',key,opts);assert.equal(requests.length,before);
  }
  emit('keydown','q',{target:{tagName:'BUTTON'}});assert.equal(last().url,'/api/drive');emit('keyup','q');
  byId.get('tightturn').checked=false;
  emit('keydown','a');assert.equal(last().f,0);assert.equal(last().t,1.6);
  emit('keyup','a');assert.equal(last().command,'WSTOP');
  byId.get('tightturn').checked=true;
  byId.get('turnfwd').value='0.9';byId.get('turn').value='1.2';
  emit('keydown','d');assert.equal(last().f,0.9);assert.equal(last().t,-1.2);
  emit('keyup','d');
  byId.get('turnfwd').value='0';byId.get('turn').value='0.7';
  emit('keydown','a');assert.equal(last().f,0);assert.equal(last().t,0.7);
  emit('keyup','a');
  emit('keydown','w');emit('keydown','1');emit('keydown','Escape');assert.equal(last().command,'STOP');
  assert.equal([...intervals.values()].filter(v=>v.ms===100).length,0);
  byId.get('viewdebug').onclick();const before=requests.length;emit('keydown','q');emit('keydown','1');assert.equal(requests.length,before);
  assert.equal(byId.get('console').hidden,false);assert.equal(byId.get('drive').hidden,false); // parent operation container is hidden
  byId.get('viewoperate').onclick();assert.equal(byId.get('console').hidden,false);
  byId.get('testleg').value='3';byId.get('legminus').onclick();assert.equal(last().command,'J 3 -1');
  byId.get('legplus').onclick();assert.equal(last().command,'J 3 1');
  byId.get('leginfo').onclick();assert.equal(last().command,'SERVOINFO 3');
  emit('keydown','w');assert.equal(last().url,'/api/drive');
  emit('keydown','r');assert.equal(last().command,'J 3 -1');
  const afterLift=requests.length;
  emit('keydown','r',{repeat:true});
  emit('keyup','r');
  assert.equal(requests.length,afterLift);
  [...intervals.values()].find(v=>v.ms===100).fn();
  assert.equal(last().url,'/api/drive');
  emit('keydown','f');assert.equal(last().command,'J 3 1');
  emit('keyup','w');assert.equal(last().command,'WSTOP');
  byId.get('levelref').value='2';byId.get('levelx').value='1.5';byId.get('levely').value='-2';
  byId.get('servoenable').onclick();assert.equal(last().command,'SERVOS ON 2 1.50 -2.00');
  byId.get('levelapply').onclick();assert.equal(last().command,'LEVEL 2 1.50 -2.00');
  byId.get('levelx').value='6';const beforeInvalid=requests.length;byId.get('levelapply').onclick();assert.equal(requests.length,beforeInvalid);
  vm.runInContext("render(['IMULEVEL active=1 state=HOLDING reference=2 target_x=1.50 target_y=-2.00 settled=1'])",context);
  assert.match(byId.get('levelstatus').textContent,/已达到目标/);
  const button=queryAll('[data-posture-key]')[0];button.onpointerdown({pointerId:7,preventDefault(){}});assert.equal(last().command,'SHIFT ALL 0.0400');button.onpointerup({pointerId:7});assert.equal(last().command,'POSTURESTOP');
  emit('keydown','w');emit('blur','');assert.equal(last().command,'STOP');
  assert.equal([...intervals.values()].filter(v=>v.ms===100).length,0);
  vm.runInContext("updateDriveLimits({speed:16,turn:16});renderYawDrive(true,[])",context);
  byId.get('turn').value='16';
  byId.get('turnfwd').value='2';
  byId.get('yawrate').value='45';
  emit('keydown','a');
  assert.deepEqual([last().f,last().t,last().yaw],[2,16,45]);
  emit('keyup','a');assert.equal(last().command,'WSTOP');

  const joystick=byId.get('joystick');
  joystick.getBoundingClientRect=()=>({left:0,top:0,width:200,height:200});
  const pointer=(clientX,clientY)=>({pointerId:3,button:0,clientX,clientY,preventDefault(){}});
  const nextDrive=()=>[...intervals.values()].find(v=>v.ms===100).fn();
  vm.runInContext("renderYawDrive(false,[])",context);
  joystick.onpointerdown(pointer(30,99));
  assert.ok(last().f>0&&last().t>15.9&&last().yaw===undefined);
  joystick.onpointerup(pointer(30,99));assert.equal(last().command,'WSTOP');
  vm.runInContext("renderYawDrive(true,[])",context);
  joystick.onpointerdown(pointer(100,100));
  assert.deepEqual([last().f,last().t,last().yaw],[0,0,undefined]);
  joystick.onpointermove(pointer(30,100));nextDrive();
  assert.deepEqual([Math.abs(last().f),last().t,last().yaw],[0,16,45]);
  joystick.onpointermove(pointer(65,65));nextDrive();
  assert.ok(last().f>0&&last().f<2&&last().yaw>0&&last().yaw<45);
  joystick.onpointermove(pointer(100,170));nextDrive();
  assert.deepEqual([last().f,last().t,last().yaw],[-2,0,undefined]);
  joystick.onpointercancel(pointer(100,170));assert.equal(last().command,'WSTOP');
  assert.equal([...intervals.values()].filter(v=>v.ms===100).length,0);
  joystick.onpointerdown(pointer(170,100));
  assert.equal(last().yaw,-45);
  joystick.onlostpointercapture(pointer(170,100));assert.equal(last().command,'WSTOP');
  assert.equal(byId.get('joystick-thumb').style.transform,'translate(-50%,-50%)');
  console.log('Joystick and IMU yaw tests passed.');
  console.log('Console controls passed: eight directions, release/escape/blur, 1-6, touch holds, input guards, single leg, debug view. No hardware used.');
})().catch(e=>{console.error(e);process.exitCode=1});
