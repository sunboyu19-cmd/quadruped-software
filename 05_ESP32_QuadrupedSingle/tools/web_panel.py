"""Loopback-only USB/WiFi/Bluetooth-SPP console. Browser drive requests expire after 300ms."""
import json, re, secrets, threading, time, webbrowser
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import serial
from collections import deque
from datetime import datetime
from serial.tools import list_ports
from wifi_transport import WifiTransport

ROOT=Path(__file__).resolve().parent/'web'
class Bridge:
    def __init__(self, clock=time.monotonic,archive=False):
        self.clock=clock; self.lock=threading.RLock(); self.port=None
        self.owner=None; self.master=False; self.features=set(); self.lines=[]; self.buffer=''
        self.transport='serial'; self.transport_mismatch_reported=False
        self.last_rx=0; self.browser_seen=0; self.last_hb=0; self.last_drive=0
        self.pose_until=0; self.motion=None; self.motion_at=0; self.seq=-1; self.closed=False
        self.timeout_active=False; self.last_probe=0
        self.needs_limit_check=True; self.wheel_off=False
        self.log_records=deque(maxlen=5000); self.log_seq=0; self.log_id=secrets.token_hex(8)
        self.archive=archive; self.log_file=None; self.log_path=None; self.log_flushed=0
    def log(self,line):
        self.lines.append(line); self.lines=self.lines[-300:]
        self.log_seq+=1; self.log_records.append({'id':self.log_seq,'text':line})
        if self.archive:
            if self.log_file is None:
                folder=ROOT.parent.parent/'reports'; folder.mkdir(exist_ok=True)
                self.log_path=folder/('runtime_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+self.log_id+'.log')
                self.log_file=self.log_path.open('a',encoding='utf-8')
            self.log_file.write(line+'\n')
            if time.monotonic()-self.log_flushed>=1:
                self.log_file.flush(); self.log_flushed=time.monotonic()
    def log_page(self,after=0):
        records=[r for r in self.log_records if r['id']>after][:500]
        first=self.log_records[0]['id'] if self.log_records else 1
        return {'session':self.log_id,'records':records,'gap':after<first-1,'last':self.log_seq}
    def export_log(self):
        if self.log_file:
            self.log_file.flush(); return self.log_path.read_bytes()
        return ('\n'.join(r['text'] for r in self.log_records)+'\n').encode('utf-8')
    def send(self,line):
        if self.port:
            self.port.write((line+'\n').encode('ascii'))
            self.log('>>> '+line)
    def stop(self):
        self.motion=None; self.send('STOP')
    def disconnect(self,release=False):
        self.motion=None; self.pose_until=0
        verified=False
        try:
            if self.port:
                if release: self.port.reset_input_buffer()
                self.send('STOP'); self.send('RELAX' if release else 'DISARM')
                if release:
                    self.port.flush(); deadline=time.monotonic()+1.2; received=b''
                    while time.monotonic()<deadline:
                        received+=self.port.read(min(self.port.in_waiting,4096))
                        if b'all_servo_torque_off_verified=1' in received: verified=True; break
                        if b'all_servo_torque_off_verified=0' in received: break
                        time.sleep(.01)
                    self.log('四舵机卸力已确认。' if verified else '已请求卸力，但未确认四舵机全部卸力；请检查实物。')
        finally:
            if self.port: self.port.close(); self.port=None
            self.owner=None; self.master=False; self.features.clear()
            self.transport='serial'; self.transport_mismatch_reported=False
        return verified
    def connect(self,owner,port,mode='serial',host='192.168.4.1'):
        if mode not in ('serial','wifi','bluetooth'): raise ValueError('请选择USB串口、WiFi或蓝牙模式。')
        target=f'wifi:{host}:9000' if mode=='wifi' else port
        if self.port and self.owner!=owner and self.clock()-self.browser_seen<=3: raise ValueError('已有控制窗口连接，请先在原窗口断开。')
        if self.port and self.port.port==target and self.transport==mode:
            self.stop(); self.send('DISARM'); self.pose_until=0
            self.owner=owner; self.seq=-1; self.browser_seen=self.clock()
            self.needs_limit_check=True; self.wheel_off=False
            self.send('H'); self.send('S'); return
        self.disconnect()
        if mode=='wifi': p=WifiTransport(host)
        else:
            p=serial.Serial(port=None,baudrate=115200,timeout=0,write_timeout=.15)
            p.dtr=False; p.rts=False; p.port=port; p.open()
        self.port=p; self.owner=owner; self.seq=-1; self.buffer=''
        self.transport=mode; self.transport_mismatch_reported=False
        self.last_rx=self.browser_seen=self.clock(); self.last_hb=0
        self.timeout_active=False
        self.needs_limit_check=True; self.wheel_off=False
        for c in ('H','REPORT ON','S'): self.send(c)
    def require(self,owner):
        if not self.port or self.owner!=owner: raise ValueError('请先在本窗口连接主控。')
    def drive(self,owner,seq,f,t):
        self.require(owner)
        if not self.master: raise ValueError('尚未确认主控。')
        if seq<=self.seq: return
        if not (-12<=f<=12 and -4<=t<=4) or max(abs(f-t),abs(f+t))>12: raise ValueError('轮速或旋转速度超限。')
        self.seq=seq; self.motion=f'MOVE {f:.4f} {t:.4f}'; self.motion_at=self.clock()
        self.send(self.motion); self.last_drive=self.clock()
    def command(self,owner,seq,c,fixed=False):
        self.require(owner)
        if seq<=self.seq: return
        c=c.strip().upper()
        if not re.fullmatch(r'[A-Z0-9 .+\-]{1,100}',c): raise ValueError('命令格式不正确。')
        op=c.split()[0]
        if op in {'ARM','ENABLE'}: self.wheel_off=False
        allowed={'H','S','ARM','ENABLE','DISARM','STOP','RELAX','CHECK','SERVOS','J','POSE','WDIR','SDIR','SAVE','SAVEWHEELS','LOADWHEELS','DIRSHOW','LIMITS','TEACH','TEACHSAVE','TEACHSHOW','TEACHCLEAR','SERVOINFO','SETMID','RESTOREMID','POSTURE','BLEND','REPORT','SHIFT','POSTURESTOP','IMU'}
        if op=='ENABLE' and 'FOC_CACHE_V1' not in self.features: raise ValueError('请先更新单主控固件，支持已保存FOC校准启用。')
        if op not in allowed: raise ValueError('驾驶请使用按住方向按钮。')
        if not self.master and op not in {'H','S','STOP','DISARM'}: raise ValueError('尚未确认主控。')
        if op in {'TEACH','SETMID','RESTOREMID'} and not fixed: raise ValueError('请先勾选车身已支撑固定。')
        if op in {'TEACH','TEACHSAVE','TEACHSHOW','TEACHCLEAR','LIMITS','POSTURE','BLEND','SETMID','RESTOREMID','SHIFT'} and 'TEACHING_V1' not in self.features: raise ValueError('当前固件尚未确认支持标定。')
        if op=='IMU':
            if c not in {'IMU READ','IMU CAL','IMU SAVE','IMU CANCEL'}: raise ValueError('IMU命令格式错误。')
            if 'IMU_V1' not in self.features: raise ValueError('请先烧录支持IMU的新单主控固件。')
            if c in {'IMU CAL','IMU SAVE'} and not fixed: raise ValueError('请先托住并固定车身，关闭轮驱动和舵机扭矩，再勾选IMU固定确认。')
        if op=='SHIFT':
            parts=c.split()
            if len(parts)!=3 or parts[1] not in {'ALL','FRONT','REAR'} or not -0.02<=float(parts[2])<=0.02: raise ValueError('SHIFT范围错误')
            self.pose_until=self.clock()+.35
        elif op in {'STOP','DISARM','RELAX','POSTURESTOP','BLEND','POSTURE'}: self.pose_until=0
        self.seq=seq
        if (op!='IMU' or c not in {'IMU READ','IMU CANCEL'}) and op not in {'J','POSE','BLEND','POSTURE','SHIFT','POSTURESTOP','S','H','SERVOINFO','TEACHSHOW','DIRSHOW'}: self.stop()
        self.send(c)
    def tick(self):
        if not self.port: return
        now=self.clock()
        data=self.port.read(min(self.port.in_waiting,8192))
        if data:
            self.last_rx=now; self.buffer+=data.decode('utf-8',errors='replace')
            while '\n' in self.buffer:
                line,self.buffer=self.buffer.split('\n',1); line=line.strip()
                if line:
                    self.log(line)
                    if line.startswith('role=SINGLE_CONTROLLER '):
                        fields=dict(re.findall(r'(\w+)=([^ ;]+)',line))
                        reported=fields.get('transport')
                        mismatch=(reported is not None and reported!=self.transport) or (self.transport=='bluetooth' and reported is None)
                        self.master=not mismatch
                        self.wheel_off=not mismatch and fields.get('state')=='0'
                        if mismatch and not self.transport_mismatch_reported:
                            self.log('连接模式与主控反馈不一致；蓝牙请选择QUAD-Single-BT配对生成的传出COM口，并烧录支持蓝牙的新固件。')
                            self.transport_mismatch_reported=True
                    if line.startswith('FEATURES '): self.features.add(line.split()[1])
            self.buffer=self.buffer[-16384:]
        if self.needs_limit_check and self.master and self.wheel_off and 'TEACHING_V1' in self.features:
            self.needs_limit_check=False
            self.send('DIRSHOW'); self.send('LIMITS'); self.send('TEACHSHOW')
            self.log('轮驱动已关闭，自动只读核验已存站趴点位与硬件限位。')
        if self.pose_until and now>self.pose_until:
            self.pose_until=0; self.send('POSTURESTOP')
        if self.motion and now-self.motion_at>.3: self.stop()
        if now-self.browser_seen>3 or now-self.last_rx>3:
            if not self.timeout_active:
                self.stop(); self.send('DISARM'); self.pose_until=0
                self.log('页面或反馈超时：已停轮并关闭轮驱动，串口连接保留。')
                self.timeout_active=True
            if now-self.last_rx>3:
                self.master=False
                if now-self.last_probe>1: self.send('S'); self.last_probe=now
        else: self.timeout_active=False
        if self.master and now-self.last_hb>.5:
            self.send('HB'); self.last_hb=now
        if self.motion and now-self.last_drive>.1:
            self.send(self.motion); self.last_drive=now
    def test_wheel(self,owner,seq,wheel,speed):
        self.require(owner)
        if not self.master: raise ValueError('尚未确认主控。')
        if seq<=self.seq: return
        if wheel not in (1,2,3,4) or not -1<=speed<=1: raise ValueError('单轮测试限±1 rad/s，轮号1～4。')
        self.seq=seq; self.motion=f'W {wheel} {speed:.3f}'; self.motion_at=self.clock()
        self.send(self.motion); self.last_drive=self.clock()
    def run(self):
        while not self.closed:
            with self.lock:
                try: self.tick()
                except Exception as e:
                    self.log('串口错误：'+str(e))
                    try: self.disconnect()
                    except Exception: self.port=None; self.master=False; self.owner=None; self.motion=None
            time.sleep(.02)

def make_handler(bridge,token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def reply(self,code,data,kind='application/json'):
            raw=json.dumps(data,ensure_ascii=False).encode() if kind=='application/json' else data
            self.send_response(code); self.send_header('Content-Type',kind+'; charset=utf-8'); self.send_header('Cache-Control','no-store'); self.send_header('X-Content-Type-Options','nosniff'); self.end_headers(); self.wfile.write(raw)
        def valid_host(self): return self.headers.get('Host')==f'127.0.0.1:{self.server.server_port}'
        def do_GET(self):
            if not self.valid_host(): self.reply(403,{'error':'仅允许本机访问'}); return
            if self.path=='/': self.reply(200,(ROOT/'index.html').read_bytes().replace(b'__TOKEN__',token.encode()),'text/html'); return
            if self.path in ('/app.js','/latest_target.js','/style.css'):
                self.reply(200,(ROOT/self.path[1:]).read_bytes(),'application/javascript' if self.path.endswith('.js') else 'text/css'); return
            self.reply(404,{'error':'不存在'})
        def do_POST(self):
            origin=self.headers.get('Origin')
            if not self.valid_host() or (origin and origin!=f'http://127.0.0.1:{self.server.server_port}') or self.headers.get('X-Console-Key')!=token:
                self.reply(403,{'error':'请求来源不匹配'}); return
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=4096: raise ValueError('请求大小错误')
                d=json.loads(self.rfile.read(n)); owner=d['client']
                with bridge.lock:
                    if self.path=='/api/state':
                        if owner==bridge.owner: bridge.browser_seen=bridge.clock()
                        result={'connected':bool(bridge.port),'mine':owner==bridge.owner,'master':bridge.master,'transport':bridge.transport,'lines':bridge.lines,'ports':[p.device for p in list_ports.comports()]}
                        result['port_details']=[{'device':p.device,'description':getattr(p,'description','') or ''} for p in list_ports.comports()]
                        result['log']=bridge.log_page(max(0,int(d.get('after',0))))
                    elif self.path=='/api/logexport':
                        self.reply(200,bridge.export_log(),'text/plain'); return
                    elif self.path=='/api/connect': bridge.connect(owner,d.get('port',''),d.get('mode','serial'),d.get('host','192.168.4.1')); result={}
                    elif self.path=='/api/disconnect': bridge.require(owner); result={'released':bridge.disconnect(release=True)}
                    elif self.path=='/api/drive': bridge.drive(owner,int(d['seq']),float(d['f']),float(d['t'])); result={}
                    elif self.path=='/api/wheeltest': bridge.test_wheel(owner,int(d['seq']),int(d['wheel']),float(d['speed'])); result={}
                    elif self.path=='/api/command': bridge.command(owner,int(d['seq']),d['command'],bool(d.get('fixed'))); result={}
                    else: raise ValueError('未知请求')
                self.reply(200,result)
            except Exception as e: self.reply(400,{'error':str(e)})
    return Handler

if __name__=='__main__':
    b=Bridge(archive=True); server=ThreadingHTTPServer(('127.0.0.1',8766),make_handler(b,secrets.token_urlsafe(32)))
    threading.Thread(target=b.run,daemon=True).start()
    print('四轮足网页控制台：http://127.0.0.1:8766 （先关闭旧串口面板）',flush=True)
    webbrowser.open('http://127.0.0.1:8766')
    try: server.serve_forever()
    finally:
        b.closed=True
        with b.lock: b.disconnect()
        if b.log_file: b.log_file.close()
        server.server_close()
