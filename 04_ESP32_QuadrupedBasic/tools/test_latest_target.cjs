const assert=require('node:assert/strict'),LatestTarget=require('./web/latest_target.js');
(async()=>{
 const mix=LatestTarget.mixDrive;
 assert.deepEqual(mix(0,1,2,2),{f:0,t:2,left:-2,right:2});
 const arc=mix(1,1,2,2,.7);assert.ok(arc.right>arc.left*3);
 for(const speed of [0.1,2,7,12])for(const turn of [-1,0,1]){
  const r=mix(1,turn,speed,4,1);assert.ok(Math.max(Math.abs(r.left),Math.abs(r.right))<=12);
  assert.ok(Math.abs(r.t)<=4);
 }
 const legacy=new LatestTarget.LegacyLogCursor();
 assert.deepEqual(legacy.accept(['command','status']),['command','status']);
 assert.deepEqual(legacy.accept(['command','status']),[]);
 assert.deepEqual(legacy.accept(['status','new']),['new']);
 assert.equal(LatestTarget.isPeriodicLog('>>> BLEND 0.58'),false);
 assert.equal(LatestTarget.isPeriodicLog('SERVO_FEEDBACK_FAIL id=2'),false);
 assert.equal(LatestTarget.isPeriodicLog('role=FRONT_MASTER state=0'),true);
 assert.equal(LatestTarget.isPeriodicLog('servo=1 valid=1'),true);
 assert.equal(LatestTarget.isPeriodicLog('>>> HB'),true);
 const cursor=new LatestTarget.LogCursor();
 assert.deepEqual(cursor.accept({session:'a',records:[{id:1,text:'first'}]}),['first']);
 assert.deepEqual(cursor.accept({session:'a',records:[{id:1,text:'first'},{id:2,text:'second'}]}),['second']);
 cursor.accept({session:'b',records:[]});assert.equal(cursor.after,0);
 assert.deepEqual(cursor.accept({session:'b',records:[{id:1,text:'new'}]}),['new']);
 const merge=LatestTarget.mergeDirection;
 assert.deepEqual(merge('1',false,'-1'),{value:'-1',dirty:false});
 assert.deepEqual(merge('1',true,'-1'),{value:'1',dirty:true});
 assert.deepEqual(merge('-1',true,'-1'),{value:'-1',dirty:false});
 assert.match(LatestTarget.teachingLabel('UNVERIFIED'),/尚未核验/);
 assert.match(LatestTarget.teachingLabel('CONFIG_READ_FAILED'),/读取失败/);
 assert.equal(LatestTarget.teachingLabel('VALID'),'已核验，可用');
 let next=0,jobs=new Map(),sent=[];
 const q=new LatestTarget(async v=>sent.push(v),fn=>{jobs.set(++next,fn);return next},id=>jobs.delete(id));
 q.push('.2');q.push('.4');q.push('.58');assert.equal(jobs.size,1);
 await [...jobs.values()][0]();assert.deepEqual(sent,['.58']);jobs.clear();
 q.push('.7');q.cancel();assert.equal(jobs.size,0);assert.equal(q.pending,null);
 let release;const busy=new LatestTarget(()=>new Promise(r=>release=r),fn=>{jobs.set(++next,fn);return next},id=>jobs.delete(id));
 busy.push('.1');const run=[...jobs.values()][0]();jobs.clear();busy.push('.9');busy.cancel();release();await run;assert.equal(jobs.size,0);
 console.log('Latest-target tests passed: collapse clicks, stop cancels queued target, cancel during in-flight request.');
})().catch(e=>{console.error(e);process.exitCode=1});
