"""Local USB panel for front_master; no firmware upload or automatic enable."""
import time
import json
from pathlib import Path
from datetime import datetime
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports

TEACH_STATES = {
    'EMPTY':'尚未记录','NEED_STAND':'缺站立点','NEED_LIE':'缺趴下点',
    'WRAP':'跨零点，需回中重录','TOO_CLOSE':'两点过近，不能判方向',
    'NEAR_ZERO':'端点近零点，需回中','UNVERIFIED':'舵机配置尚未核验','CONFIG_READ_FAILED':'舵机配置读取失败',
    'STALE_ZERO':'零位/相位已变，需重录','HW_LIMIT':'超出内部限位','VALID':'可用',
}

def parse_teaching(line):
    if not line.startswith('TEACHDATA '): return None
    try:
        data=dict(token.split('=',1) for token in line.split()[1:] if '=' in token)
        required={'id','pos','feedback','hw_min','hw_max','hw_known','stand','lie','sw_min','sw_max','dir','state','offset','taught_offset'}
        if not required.issubset(data): return None
        for key in ('id','pos','feedback','hw_min','hw_max','hw_known','stand','lie','sw_min','sw_max','dir','offset','taught_offset','near_zero'):
            data[key]=int(data.get(key,0 if key=='near_zero' else -1))
        data['age_ms']=int(data.get('age_ms',-1))
        if data['id'] not in range(1,5) or data.get('state') not in TEACH_STATES: return None
        if data['dir'] not in (-1,0,1) or data['feedback'] not in (0,1) or data['hw_known'] not in (0,1): return None
        return data
    except (ValueError,TypeError): return None


def parse_wheel_profile(line):
    if not line.startswith('DIRDATA '): return None
    try:
        data=dict(token.split('=',1) for token in line.split()[1:] if '=' in token)
        for key in ('id','value','verified','saved_value','saved_verified','saved_present','dirty'):
            data[key]=int(data[key])
        if data['id'] not in range(1,5) or data['value'] not in (-1,1) or data['saved_value'] not in (-1,1): return None
        if any(data[k] not in (0,1) for k in ('verified','saved_verified','saved_present','dirty')): return None
        if data['source'] not in ('NONE','PROFILE','LEGACY'): return None
        return data
    except (KeyError,ValueError,TypeError): return None


def scrollable_tab(tabs,title):
    container=ttk.Frame(tabs); tabs.add(container,text=title)
    scroll=ttk.Scrollbar(container,orient='vertical'); scroll.pack(side='right',fill='y')
    canvas=tk.Canvas(container,highlightthickness=0,height=330,background='#0e1014',yscrollcommand=scroll.set)
    canvas.pack(side='left',fill='both',expand=True); scroll.configure(command=canvas.yview)
    body=ttk.Frame(canvas); window=canvas.create_window((0,0),window=body,anchor='nw')
    body.bind('<Configure>',lambda event:canvas.configure(scrollregion=canvas.bbox('all')))
    canvas.bind('<Configure>',lambda event:canvas.itemconfigure(window,width=event.width))
    return body


class HoldDrive:
    """Only keep a motion command alive while its button is held."""
    def __init__(self, send, clock=time.monotonic):
        self.send, self.clock = send, clock
        self.command = None
        self.last = float('-inf')

    def press(self, command):
        self.command = command
        self.last = float('-inf')
        self.tick()

    def tick(self):
        now = self.clock()
        if self.command and now - self.last >= 0.1:
            self.last = now
            self.send(self.command)

    def release(self):
        was_active = self.command is not None
        self.command = None
        if was_active:
            self.send('STOP')


class Panel:
    def __init__(self, root):
        style=ttk.Style(root); style.theme_use('clam')
        root.configure(background='#0e1014')
        style.configure('.',background='#161920',foreground='#e5e8ee',font=('Microsoft YaHei UI',10),bordercolor='#2b313c')
        style.configure('TButton',background='#252a34',padding=(12,7))
        style.map('TButton',background=[('active','#1f3354'),('pressed','#5b9cf6')])
        style.configure('TEntry',fieldbackground='#1d2129',insertcolor='#e5e8ee')
        style.configure('TCombobox',fieldbackground='#1d2129',arrowcolor='#5b9cf6')
        style.configure('TSpinbox',fieldbackground='#1d2129',arrowcolor='#5b9cf6')
        style.configure('Treeview',fieldbackground='#161920',rowheight=28)
        style.configure('Drive.TButton',font=('Microsoft YaHei UI',12,'bold'),padding=(20,16),background='#1f3354')
        style.configure('Stop.TButton',background='#d7263d',foreground='white',font=('Microsoft YaHei UI',12,'bold'))
        self.root = root
        self.port = None
        self.master = False
        self.received = 0
        self.buffer = ''
        self.snapshot_pending = False
        self.snapshot_logging = False
        self.last_log_line = None
        self.teaching_supported=False
        self.teach_records={}
        self.wheel_save_supported=False
        self.wheel_profiles={}
        self.wheel_loading=False
        self.last_hb=0
        self.hold = HoldDrive(self.send)
        root.title('QUADRUPED / 四轮足遥操控制台')
        root.geometry(f'1000x{min(820,max(620,root.winfo_screenheight()-100))}')
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<Escape>', lambda event: self.stop())
        root.bind('<ButtonRelease-1>', lambda event: self.hold.release(), add='+')
        root.bind('<FocusOut>', self.focus_out, add='+')
        top = ttk.Frame(root, padding=10); top.pack(fill='x')
        ttk.Label(top, text='A板串口').pack(side='left')
        self.port_name = tk.StringVar()
        self.ports = ttk.Combobox(top, textvariable=self.port_name, width=18)
        self.ports.pack(side='left', padx=6)
        ttk.Button(top, text='刷新', command=self.refresh).pack(side='left')
        ttk.Button(top, text='连接', command=self.connect).pack(side='left', padx=6)
        ttk.Button(top, text='断开', command=self.disconnect).pack(side='left')
        # Pack command input before expandable content so scaling cannot hide it.
        command_bar=ttk.LabelFrame(root,text='命令输入（下方大框是只读日志）',padding=8)
        command_bar.pack(fill='x',padx=8,pady=4)
        self.entry=ttk.Entry(command_bar,font=('Microsoft YaHei UI',11),state='normal')
        self.entry.pack(side='left',fill='x',expand=True)
        self.entry.bind('<Return>',lambda event:self.manual())
        ttk.Button(command_bar,text='发送命令 / 回车',command=self.manual).pack(side='left',padx=8)
        self.state = tk.StringVar(value='未连接；先按README接线和分别烧录A/B板')
        ttk.Label(root, textvariable=self.state, padding=8, wraplength=920).pack(fill='x')
        ttk.Label(root, text='1左前  2右前  3左后  4右后。先悬空校准与验方向；方向按钮按住才运动，松开停轮。', padding=8).pack(fill='x')
        actions = ttk.Frame(root, padding=8); actions.pack(fill='x')
        for index,(label, command) in enumerate([('读取状态','S'), ('四轮悬空校准','ARM'), ('停轮 STOP','STOP'),
                               ('关闭轮驱动','DISARM'), ('舵机当前位置使能','SERVOS ON'),
                               ('检查四舵机','CHECK'), ('卸全部扭矩（先支撑）','RELAX')]):
            ttk.Button(actions, text=label, command=lambda c=command:self.action(c)).grid(row=index//4,column=index%4,padx=3,pady=2,sticky='ew')
        content=tk.PanedWindow(root,orient='vertical',sashwidth=6,showhandle=True)
        content.pack(fill='both',expand=True,padx=8,pady=4)
        tabs=ttk.Notebook(content); content.add(tabs,minsize=180,stretch='always')
        cockpit=scrollable_tab(tabs,'驾驶控制台')
        ttk.Label(cockpit,text='QUADRUPED  /  DRIVE CONSOLE',font=('Segoe UI',18,'bold'),foreground='#5b9cf6',padding=12).pack(anchor='w')
        ttk.Label(cockpit,text='先使能舵机、悬空校准四轮，确认方向后驾驶。按住运动，松开或窗口失焦停轮。',padding=8).pack(anchor='w')
        settings=ttk.LabelFrame(cockpit,text='速度设置 · rad/s',padding=12); settings.pack(fill='x',padx=10,pady=6)
        self.cruise=tk.StringVar(value='0.5'); self.rotation=tk.StringVar(value='0.5')
        ttk.Label(settings,text='行驶速度 0.1～12').grid(row=0,column=0,padx=8)
        ttk.Spinbox(settings,from_=0.1,to=12,increment=0.1,textvariable=self.cruise,width=7).grid(row=0,column=1)
        ttk.Label(settings,text='旋转/差速 0.1～4').grid(row=0,column=2,padx=8)
        ttk.Spinbox(settings,from_=0.1,to=4,increment=0.1,textvariable=self.rotation,width=7).grid(row=0,column=3)
        pad=ttk.LabelFrame(cockpit,text='四轮差速驾驶',padding=12); pad.pack(fill='x',padx=10,pady=8)
        for label,f,t,r,c in [('↖ 左前弧线',1,1,0,0),('↑ 前进',1,0,0,1),('↗ 右前弧线',1,-1,0,2),
                              ('↺ 原地左旋',0,1,1,0),('↻ 原地右旋',0,-1,1,2),
                              ('↙ 左后弧线',-1,1,2,0),('↓ 后退',-1,0,2,1),('↘ 右后弧线',-1,-1,2,2)]:
            b=ttk.Button(pad,text=label,style='Drive.TButton'); b.grid(row=r,column=c,sticky='ew',padx=5,pady=5)
            b.bind('<ButtonPress-1>',lambda e,f=f,t=t:self.cockpit_drive(f,t))
        for c in range(3): pad.columnconfigure(c,weight=1)
        ttk.Button(pad,text='■ 停轮',style='Stop.TButton',command=self.stop).grid(row=1,column=1,sticky='nsew',padx=5,pady=5)
        ttk.Label(cockpit,text='原地旋转：同侧前后轮同向、左右两侧反向。弧线自动等比例限制轮速，保持转向比例。',padding=8).pack(anchor='w')
        drive_tab=scrollable_tab(tabs,'轮电机与方向')
        joint_tab=scrollable_tab(tabs,'舵机点动与姿态')
        teach_tab=scrollable_tab(tabs,'限位／站趴标定／回中')
        drive = ttk.LabelFrame(drive_tab, text='轮电机：需两板都为state=3；整车运动前设置四个WDIR', padding=8)
        drive.pack(fill='x', padx=10, pady=6)
        self.speed = tk.StringVar(value='0.5')
        ttk.Label(drive,text='测试速度 rad/s（0.1～1）').grid(row=0,column=0)
        ttk.Spinbox(drive,from_=0.1,to=1,increment=0.1,textvariable=self.speed,width=6).grid(row=0,column=1)
        for col,(label,command) in enumerate([('按住前进','F'),('按住后退','BACK'),('按住左转','LEFT'),('按住右转','RIGHT')]):
            b=ttk.Button(drive,text=label); b.grid(row=1,column=col,padx=6,pady=8)
            b.bind('<ButtonPress-1>',lambda e,c=command:self.drive(c))
        self.wheel = tk.StringVar(value='1')
        ttk.Combobox(drive,values=['1','2','3','4'],textvariable=self.wheel,state='readonly',width=5).grid(row=2,column=0)
        for col,sign in [(1,1),(2,-1)]:
            b=ttk.Button(drive,text='按住单轮'+('正转' if sign==1 else '反转'))
            b.grid(row=2,column=col,pady=4); b.bind('<ButtonPress-1>',lambda e,s=sign:self.drive('W',s))
        wheel_setup=ttk.LabelFrame(drive_tab,text='四轮方向标定：悬空逐轮试转 → 关闭轮驱动 → 应用四行 → 保存',padding=8)
        wheel_setup.pack(fill='x',padx=10,pady=5)
        self.wheel_values={}; self.wheel_labels={}
        for sid,name in enumerate(['左前','右前','左后','右后'],1):
            ttk.Label(wheel_setup,text=f'{sid} {name}').grid(row=sid-1,column=0,padx=5)
            self.wheel_values[sid]=tk.StringVar(value='1')
            ttk.Combobox(wheel_setup,values=['1','-1'],textvariable=self.wheel_values[sid],state='readonly',width=5).grid(row=sid-1,column=1,padx=5)
            ttk.Button(wheel_setup,text='应用并确认',command=lambda k=sid:self.apply_wheel(k)).grid(row=sid-1,column=2,padx=5,pady=3)
            self.wheel_labels[sid]=tk.StringVar(value='待读取A板保存状态')
            ttk.Label(wheel_setup,textvariable=self.wheel_labels[sid]).grid(row=sid-1,column=3,sticky='w',padx=5)
        for col,(label,cmd) in enumerate([('保存四轮方向','SAVEWHEELS'),('读取保存状态','DIRSHOW'),('恢复已保存方向','LOADWHEELS')]):
            ttk.Button(wheel_setup,text=label,command=lambda c=cmd:self.wheel_action(c)).grid(row=4,column=col,pady=7,padx=3)
        ttk.Label(wheel_setup,text='正向应使该轮推动车身前进；保存后断电保留。舵机可保持支撑；未确认四轮前不能整车运动。').grid(row=5,column=0,columnspan=4,sticky='w')
        setup=ttk.LabelFrame(drive_tab,text='手动舵机方向：支撑后RELAX；站趴示教也可自动判定舵机方向',padding=8)
        setup.pack(fill='x',padx=10,pady=5)
        self.identity=tk.StringVar(value='1'); self.direction=tk.StringVar(value='1')
        ttk.Combobox(setup,values=['1','2','3','4'],textvariable=self.identity,state='readonly',width=5).pack(side='left')
        ttk.Combobox(setup,values=['1','-1'],textvariable=self.direction,state='readonly',width=5).pack(side='left',padx=6)
        for label,cmd in [('设置舵机方向','SDIR')]:
            ttk.Button(setup,text=label,command=lambda c=cmd:self.action(f'{c} {self.identity.get()} {self.direction.get()}')).pack(side='left',padx=4)
        ttk.Button(setup,text='SAVE 保存两组方向',command=lambda:self.action('SAVE')).pack(side='left',padx=6)
        joints=ttk.LabelFrame(joint_tab,text='舵机：先SERVOS ON；每次±1°；完整示教后按站趴区间限位，否则按原±15°窗口',padding=8)
        joints.pack(fill='x',padx=10,pady=5)
        for i,name in enumerate(['左前1','右前2','左后3','右后4']):
            ttk.Label(joints,text=name).grid(row=0,column=i,padx=16)
            ttk.Button(joints,text='＋1°',command=lambda k=i+1:self.action(f'J {k} 1')).grid(row=1,column=i)
            ttk.Button(joints,text='－1°',command=lambda k=i+1:self.action(f'J {k} -1')).grid(row=2,column=i)
        self.pose=[tk.StringVar(value='0') for _ in range(4)]
        for i in range(4): ttk.Spinbox(joints,from_=-15,to=15,increment=1,textvariable=self.pose[i],width=7).grid(row=3,column=i,pady=8)
        ttk.Button(joints,text='发送四关节POSE（每次变化≤5°）',command=self.pose_command).grid(row=4,column=0,columnspan=4)
        transition=ttk.LabelFrame(joint_tab,text='已示教的站立／趴下：需四组点位有效、舵机已使能、轮子停止；首次有支撑验证',padding=8)
        transition.pack(fill='x',padx=10,pady=8)
        ttk.Button(transition,text='缓慢到站立',command=lambda:self.cal_action('POSTURE STAND')).pack(side='left',padx=4)
        ttk.Button(transition,text='缓慢到趴下',command=lambda:self.cal_action('POSTURE LIE')).pack(side='left',padx=4)
        self.blend=tk.DoubleVar(value=0)
        ttk.Scale(transition,from_=0,to=100,variable=self.blend,length=220).pack(side='left',padx=8)
        ttk.Button(transition,text='应用比例（0趴／100站）',command=lambda:self.cal_action(f'BLEND {self.blend.get()/100:.3f}')).pack(side='left')
        ttk.Button(transition,text='停止姿态运动',command=self.stop).pack(side='left',padx=5)
        ttk.Label(teach_tab,text='读取内部限位；手动摆出舒适的站立/趴下再记录，不顶机械硬限位。单位：编码器计数，一圈4096。',padding=6).pack(fill='x')
        columns=('joint','pos','hardware','stand','lie','software','direction','state')
        table_frame=ttk.Frame(teach_tab); table_frame.pack(fill='x',padx=6)
        self.teach_table=ttk.Treeview(table_frame,columns=columns,show='headings',height=4)
        for key,title,width in zip(columns,['关节','当前位置','内部限位','站立点','趴下点','软件区间','向站立方向','状态'],[90,65,105,65,65,105,100,175]):
            self.teach_table.heading(key,text=title); self.teach_table.column(key,width=width,minwidth=55,anchor='center')
        self.teach_table.pack(fill='x')
        self.teach_table.tag_configure('warning',foreground='#B54900'); self.teach_table.tag_configure('ready',foreground='#087F56')
        for i,name in enumerate(['左前','右前','左后','右后'],1): self.teach_table.insert('', 'end',iid=str(i),values=(f'{i} {name}','—','—','—','—','—','—','待读取'))
        row=ttk.Frame(teach_tab,padding=6); row.pack(fill='x')
        for label,cmd in [('读取限位/当前位置','LIMITS'),('查看已存点位','TEACHSHOW'),('保存点位及方向','TEACHSAVE')]:
            ttk.Button(row,text=label,command=lambda c=cmd:self.cal_action(c)).pack(side='left',padx=3)
        ttk.Button(row,text='导出标定记录',command=self.export_teaching).pack(side='left',padx=3)
        self.fixed=tk.BooleanVar(value=False)
        ttk.Checkbutton(teach_tab,text='已支撑固定车身，当前要记录/校准的腿姿已摆好并固定；保持不动',variable=self.fixed).pack(anchor='w',padx=10)
        capture=ttk.Frame(teach_tab,padding=6); capture.pack(fill='x')
        ttk.Button(capture,text='记录四腿：站立',command=lambda:self.cal_action('TEACH STAND',True)).pack(side='left',padx=3)
        ttk.Button(capture,text='记录四腿：趴下',command=lambda:self.cal_action('TEACH LIE',True)).pack(side='left',padx=3)
        ttk.Label(capture,text='所选关节').pack(side='left',padx=6)
        self.cal_servo=tk.StringVar(value='1')
        selector=ttk.Combobox(capture,values=['1','2','3','4'],textvariable=self.cal_servo,state='readonly',width=4)
        selector.pack(side='left')
        selector.bind('<<ComboboxSelected>>',lambda event:self.teach_table.selection_set(self.cal_servo.get()))
        self.teach_table.bind('<<TreeviewSelect>>',lambda event:self.cal_servo.set(self.teach_table.selection()[0]) if self.teach_table.selection() else None)
        ttk.Button(capture,text='记录所选站立',command=lambda:self.cal_action(f'TEACH STAND {self.cal_servo.get()}',True)).pack(side='left',padx=3)
        ttk.Button(capture,text='记录所选趴下',command=lambda:self.cal_action(f'TEACH LIE {self.cal_servo.get()}',True)).pack(side='left',padx=3)
        midpoint=ttk.Frame(teach_tab,padding=6); midpoint.pack(fill='x')
        ttk.Button(midpoint,text='所选原位设为2048（写零偏/清所选点位）',command=lambda:self.cal_action(f'SETMID {self.cal_servo.get()} FIXED',True)).pack(side='left',padx=3)
        ttk.Button(midpoint,text='所选只读诊断',command=lambda:self.cal_action(f'SERVOINFO {self.cal_servo.get()}')).pack(side='left',padx=3)
        ttk.Button(midpoint,text='清除所选记录',command=lambda:self.cal_action(f'TEACHCLEAR {self.cal_servo.get()}')).pack(side='left',padx=3)
        ttk.Label(teach_tab,text='原位回中先卸扭矩、固定期望机械中位；不是让连杆大角度旋转。校零后重新记录该腿两点。',padding=5).pack(fill='x')
        log_container=ttk.Frame(content); content.add(log_container,minsize=110,stretch='always')
        log_options=ttk.Frame(log_container,padding=(8,2)); log_options.pack(fill='x')
        self.show_telemetry=tk.BooleanVar(value=False)
        self.auto_scroll=tk.BooleanVar(value=True)
        ttk.Checkbutton(log_options,text='显示周期状态',variable=self.show_telemetry).pack(side='left')
        ttk.Checkbutton(log_options,text='自动滚动',variable=self.auto_scroll).pack(side='left',padx=12)
        ttk.Button(log_options,text='清空日志',command=self.clear_log).pack(side='left',padx=4)
        ttk.Button(log_options,text='复制日志',command=self.copy_log).pack(side='left',padx=4)
        console=ttk.Frame(log_container,padding=8); console.pack(fill='both',expand=True)
        self.log=tk.Text(console,height=7,wrap='word',state='disabled',background='#0e1014',foreground='#959eae',insertbackground='#e5e8ee'); self.log.pack(side='left',fill='both',expand=True)
        scrollbar=ttk.Scrollbar(console,orient='vertical',command=self.log.yview)
        scrollbar.pack(side='right',fill='y'); self.log.configure(yscrollcommand=scrollbar.set)
        self.refresh(); root.after(25,self.tick)

    def append(self,text):
        # A held movement button can receive the same BLOCKED reply every 100ms.
        if text==self.last_log_line: return
        self.last_log_line=text
        self.log.configure(state='normal')
        self.log.insert('end',text+'\n')
        if self.auto_scroll.get(): self.log.see('end')
        if int(self.log.index('end-1c').split('.')[0])>500: self.log.delete('1.0','100.0')
        self.log.configure(state='disabled')

    def clear_log(self):
        self.log.configure(state='normal'); self.log.delete('1.0','end')
        self.log.configure(state='disabled'); self.last_log_line=None

    def copy_log(self):
        self.root.clipboard_clear(); self.root.clipboard_append(self.log.get('1.0','end-1c'))

    def handle_line(self,line):
        if line=='FEATURES WHEEL_SAVE_V1':
            self.wheel_save_supported=True; self.send('DIRSHOW')
        wheel=parse_wheel_profile(line)
        if wheel: self.update_wheel_row(wheel)
        if line=='FEATURES TEACHING_V1':
            self.teaching_supported=True; self.send('TEACHSHOW')
        record=parse_teaching(line)
        if record:
            self.teach_records[record['id']]=record
            self.update_teaching_row(record)
        if line.startswith(('TEACH captured','MIDPOINT CALIBRATED','MIDPOINT RESTORED')): self.fixed.set(False)
        # Keep receiving telemetry for the connection watchdog even when hidden.
        if line.startswith('role=FRONT_MASTER '):
            self.master=True; self.state.set(line)
            if self.snapshot_pending:
                self.snapshot_pending=False; self.snapshot_logging=True
        telemetry=line.startswith(('role=','wheel=','WDIR=','TEACHDATA ','DIRDATA ','HOST_ALIVE '))
        if not telemetry or self.show_telemetry.get() or self.snapshot_logging:
            self.append(line)
        if line.startswith('WDIR='): self.snapshot_logging=False

    def update_wheel_row(self,data):
        sid=data['id']; previous=self.wheel_profiles.get(sid)
        if self.wheel_loading or previous is None or self.wheel_values[sid].get()==str(previous['value']):
            self.wheel_values[sid].set(str(data['value']))
        self.wheel_profiles[sid]=data
        saved=str(data['saved_value']) if data['saved_present'] else '无'
        status='有未保存修改' if data['dirty'] else '已保存'
        confirmed='已确认' if data['verified'] else '尚未确认'
        self.wheel_labels[sid].set(f"当前 {data['value']:+d}（{confirmed}）／已存 {saved}／{status}")
        if self.wheel_loading and sid==4: self.wheel_loading=False

    def wheel_action(self,command):
        if not self.port or not self.master: self.append('请先连接并确认A板。'); return
        if not self.wheel_save_supported: self.append('请先更新A板固件以支持四轮独立保存。'); return
        if command=='SAVEWHEELS':
            if len(self.wheel_profiles)!=4:
                self.action('DIRSHOW'); self.append('请读取四轮状态后再保存。'); return
            if any(self.wheel_values[k].get()!=str(self.wheel_profiles[k]['value']) for k in range(1,5)):
                self.append('下拉框修改尚未应用，请逐行点击“应用并确认”后保存。'); return
        if command=='LOADWHEELS': self.wheel_loading=True
        self.action(command)

    def apply_wheel(self,sid):
        self.wheel_action(f'WDIR {sid} {self.wheel_values[sid].get()}')

    def update_teaching_row(self,data):
        def value(key): return str(data[key]) if data[key]>=0 else '—'
        def interval(lo,hi): return f'{data[lo]}～{data[hi]}' if data[lo]>=0 and data[hi]>=0 else '—'
        label=TEACH_STATES[data['state']]
        if data['near_zero'] and data['state'] not in ('WRAP','NEAR_ZERO'): label+='／当前位置近零点'
        if not data['feedback']: label+='／反馈未确认'
        elif data['age_ms']>2000: label+='／反馈较旧，请读取'
        direction={1:'＋1 到站立',-1:'－1 到站立'}.get(data['dir'],'待判定')
        names=['左前','右前','左后','右后']; sid=data['id']
        tag='ready' if data['state']=='VALID' and data['feedback'] and data['hw_known'] and not data['near_zero'] and data['age_ms']<=2000 else 'warning'
        self.teach_table.item(str(sid),values=(f'{sid} {names[sid-1]}',value('pos') if data['feedback'] else '未确认',
            interval('hw_min','hw_max') if data['hw_known'] else '待读取',
            value('stand'),value('lie'),interval('sw_min','sw_max'),direction,label),tags=(tag,))

    def cal_action(self,command,fixed=False):
        if not self.port or not self.master: self.append('请先连接A板。'); return
        if not self.teaching_supported: self.append('当前A板未确认标定新版，请先更新front_master固件。'); return
        if fixed and not self.fixed.get(): self.append('先支撑车身，摆好并固定所需腿姿，再勾选固定确认。'); return
        self.action(command)

    def export_teaching(self):
        if not self.teach_records: self.append('请先读取限位或点位，再导出。'); return
        folder=Path(__file__).resolve().parents[1]/'reports'; folder.mkdir(exist_ok=True)
        target=folder/('servo_teaching_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        target.write_text(json.dumps({'export_time':datetime.now().astimezone().isoformat(),
            'unit':'encoder counts, 4096 per revolution','joints':self.teach_records},ensure_ascii=False,indent=2),encoding='utf-8')
        self.append('标定记录已导出：'+str(target))

    def refresh(self):
        self.ports['values']=[p.device for p in list_ports.comports()]
        if not self.port_name.get() and self.ports['values']: self.port_name.set(self.ports['values'][0])

    def connect(self):
        self.disconnect()
        try:
            p=serial.Serial(port=None,baudrate=115200,timeout=0,write_timeout=0.2)
            p.dtr=False; p.rts=False; p.port=self.port_name.get(); p.open()
            self.port=p; self.master=False; self.received=time.monotonic(); self.buffer=''
            self.teaching_supported=False; self.last_hb=0
            self.snapshot_pending=True; self.snapshot_logging=False
            self.state.set('已连接，等待确认FRONT_MASTER角色；尚未使能')
            self.send('H'); self.send('REPORT ON'); self.send('S')
        except Exception as error: self.append('连接失败：'+str(error))

    def send(self,command):
        if not self.port: return
        try: self.port.write((command+'\n').encode('ascii'))
        except Exception as error:
            self.append('串口错误：'+str(error)); self.hold.command=None
            self.port.close(); self.port=None; self.master=False
            self.state.set('连接中断；主控运动指令租约到期后停轮')

    def action(self,command):
        if not self.port: self.append('请先连接A板串口。'); return
        if not self.master and command not in ('H','S','STOP','DISARM'):
            self.append('尚未确认A板固件，拒绝控制。'); return
        if command.strip().upper()=='S': self.snapshot_pending=True
        self.hold.release(); self.append('>>> '+command); self.send(command)

    def cockpit_drive(self,forward,turn):
        if not self.master or not self.port: self.append('请先连接并确认A板。'); return
        try:
            speed=float(self.cruise.get()); spin=float(self.rotation.get())
            if not 0.1<=speed<=12 or not 0.1<=spin<=4: raise ValueError()
        except ValueError: self.append('行驶速度须为0.1～12，旋转速度须为0.1～4 rad/s。'); return
        f,t=forward*speed,turn*spin
        peak=max(abs(f-t),abs(f+t))
        if peak>12: f,t=f*11.9998/peak,t*11.9998/peak
        self.hold.press(f'MOVE {f:.4f} {t:.4f}')

    def drive(self,command,sign=1):
        if not self.master or not self.port: self.append('请先连接并确认A板。'); return
        try:
            speed=float(self.speed.get())
            if not 0.1<=speed<=1: raise ValueError()
        except ValueError: self.append('速度须为0.1～1 rad/s。'); return
        text=f'W {self.wheel.get()} {speed*sign:.2f}' if command=='W' else f'{command} {speed:.2f}'
        self.hold.press(text)

    def pose_command(self):
        try:
            values=[float(v.get()) for v in self.pose]
            if not all(-15<=x<=15 for x in values): raise ValueError()
        except ValueError: self.append('POSE每项须为-15～15。'); return
        self.action('POSE '+' '.join(str(x) for x in values))

    def manual(self):
        text=self.entry.get().strip(); self.entry.delete(0,'end')
        if text: self.action(text)

    def stop(self):
        self.hold.release(); self.send('STOP')

    def focus_out(self,event):
        # Stop if focus moves outside this application, including lost mouse capture.
        self.root.after(50,lambda:self.hold.release() if self.root.focus_displayof() is None else None)

    def disconnect(self):
        self.hold.release()
        if self.port:
            self.send('STOP'); self.send('DISARM')
            if self.port: self.port.close()
        self.port=None; self.master=False; self.state.set('未连接；舵机可能继续保持原位置')
        self.teaching_supported=False; self.fixed.set(False)
        self.wheel_save_supported=False; self.wheel_profiles.clear(); self.wheel_loading=False
        for label in self.wheel_labels.values(): label.set('未连接／待读取A板保存状态')

    def close(self):
        self.disconnect(); self.root.destroy()

    def tick(self):
        if self.port:
            try:
                data=self.port.read(min(self.port.in_waiting,8192))
                if data:
                    self.received=time.monotonic(); self.buffer+=data.decode('utf-8',errors='replace')
                    while '\n' in self.buffer:
                        line,self.buffer=self.buffer.split('\n',1); line=line.strip()
                        if line:
                            self.handle_line(line)
                    self.buffer=self.buffer[-16384:]
                if self.master and time.monotonic()-self.received>3:
                    self.hold.release(); self.send('STOP'); self.send('DISARM'); self.master=False
                    self.state.set('超过3秒没有反馈，已停止续发运动指令；检查USB和A板')
            except Exception as error:
                self.append('读取错误：'+str(error)); self.disconnect()
        if self.master:
            self.hold.tick()
            if self.teaching_supported and time.monotonic()-self.last_hb>=0.5:
                self.last_hb=time.monotonic(); self.send('HB')
        self.root.after(25,self.tick)


if __name__=='__main__':
    window=tk.Tk(); Panel(window); window.mainloop()
