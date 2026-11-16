# 07 普通ESP32：1～4号总线舵机只读检测

适用于当前普通ESP32-WROOM及微雪Bus Servo Adapter(A)、四个STS3032总线舵机。与03的ESP32-S3运动调试工程分开，只有环境`servo_diagnostic`，不调用I²C、不启动WiFi、不初始化轮PWM，共用轮EN13保持低。

程序仅发送单舵机READ指令：不写目标、扭矩、ID、波特率、零偏或限位；不发送广播。不自动卸力，若舵机原本有扭矩，程序会读出torque=1并提示，保持现状。上电本身可能改变舵机状态，先用固定支撑托住车身。只读检测能判断通信/反馈状态，不能证明带载转动和机械机构正常。

## 直接打开串口：复制这两行到PowerShell

当前07只读检测v2已烧录到COM13，无需为了查看串口再次烧录。先关闭占用COM13的其他串口监视器或控制面板，再在VS Code终端执行：

```powershell
Set-Location -LiteralPath "C:\Users\32892\OneDrive\四轮足程序\07_ESP32_ServoDiagnostic"
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM13 -b 115200 --echo --eol LF --filter send_on_enter
```

看到串口监视器启动后，下面的检测命令直接输入同一个终端，每行按一次回车发送。**检测命令输入在串口监视器内，不是在PowerShell提示符后运行。**

```text
H
DEBUG ON
TEST 1
CHECK
```

`TEST 1`只读取1号，`CHECK`依次读取四个。查看不清时先输入`WATCH OFF`停止自动刷新。退出串口监视器按 **Ctrl+C**，回到PowerShell后才能执行编译、烧录等命令。若COM13不存在，先用下方“列出串口”命令查询实际端口，再替换命令里的COM13。

## 串口监视器内的完整命令表

| 输入命令（每行回车） | 功能 |
|---|---|
| `H` | 显示帮助 |
| `CHECK` 或 `S` | 依次读取1～4号ID、位置、电压、温度、状态及配置 |
| `SCAN` | 只读1～4号ID寄存器，确认真实应答，不扫描其他ID |
| `TEST 1` | 单独检测1号左前 |
| `TEST 2` | 单独检测2号右前 |
| `TEST 3` | 单独检测3号左后 |
| `TEST 4` | 单独检测4号右后 |
| `DEBUG ON` | 失败时输出BUS_TX/BUS_RX原始字节，排查回显 |
| `DEBUG OFF` | 关闭原始字节输出 |
| `WATCH ON` | 每2秒重复读取四个舵机 |
| `WATCH OFF` | 停止周期读取，方便查看和复制日志 |
| `RESETSTATS` | 清除累计失败、重试和回显计数，不修改舵机 |

本工程不支持`SERVOS ON`、`RELAX`、位置目标或修改ID等运动/写入命令。待机测量D-G电压时先`WATCH OFF`，暂不发送`CHECK`、`TEST`或`SCAN`。

## 接线与供电

| 连接端 | 对接端 |
|---|---|
| ESP32 GPIO5 | 微雪Adapter(A)标注TX |
| ESP32 GPIO34 | 微雪Adapter(A)标注RX |
| ESP32 GND | 微雪GND及6V负极共地 |
| 微雪舵机动力输入 | 正确极性的稳压6V |
| 微雪跳帽 | A档 |
| 微雪舵机总线 | ID1左前、2右前、3左后、4右后 |

以上沿用已验证的微雪接口标注，同名TX接TX、RX接RX；不要套用其他UART板的交叉接法。ESP32仍使用USB或独立稳压5V的一种，6V只供舵机侧，不能接ESP32的5V/3V3。GPIO34输入信号应为3.3V兼容电平。

6V关闭时四舵机通常不应答，不能当作损坏。先在动力关闭时烧录，托住车身，再给微雪板/舵机接通6V后检测。当前故障中的PCA9548A/IMU支路不参与本工程。

## 编译、查询串口和重新烧录：在PowerShell中运行

工程在OneDrive“四轮足程序”的`07_ESP32_ServoDiagnostic`，加入“四轮足软件.code-workspace”。在07工程终端运行，COM13按实际修改。关闭网页/串口监视器，避免占用同一端口：

```powershell
# 编译07工程
& "..\.tools\Scripts\python.exe" -m platformio run -e servo_diagnostic
# 列出电脑识别到的串口，不会使能舵机
& "..\.tools\Scripts\python.exe" -m platformio device list
# 仅在需要更新固件时烧录；先退出串口监视器并关闭6V
& "..\.tools\Scripts\python.exe" -m platformio run -e servo_diagnostic -t upload --upload-port COM13
# 烧录完成后打开串口监视器
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM13 -b 115200 --echo --eol LF --filter send_on_enter
```

串口每行一条：

```text
CHECK
```

会依次读取1～4号，输出4行`SERVO id=...`与汇总。`SCAN`只读取这四个预期ID的ID寄存器，`TEST 1`等只检测一个；`WATCH ON`每2秒重复，`WATCH OFF`停止刷新；`RESETSTATS`清错误计数；`H`查看帮助。退出监视器Ctrl+C。

## 结果解释

- `comm=FAIL`：三次未获得与预期ID匹配的ID寄存器应答；检查6V、共地、跳帽、TX/RX、总线插头、ID与原波特率1Mbps。
- `comm=ID_READ_OK feedback=FAIL`：真实ID读回但反馈读失败，不能当作完整通过。
- `comm=OK feedback=OK config=OK`：身份/反馈/所读配置都成功；校验应答ID、长度及校验和，丢弃不匹配帧。
- `pos/deg`位置，`V`供电电压，`C`内部温度，`status`舵机返回状态字节，`mode`模式（预期0），`torque`当前扭矩开关，`goal_raw`实际目标寄存器，`min/max`内部寄存器限位，不是实测机械止挡。
- `result=READ_ONLY_OK`要求上述读取成功、status=0、位置0～4095、供电5.0～6.8V、温度低于60°C、模式0。这是当前6V项目的检查窗口，不是厂家极限参数，也不代表已完成运动验收。
- `retries_now>0`说明本次曾读失败，重试成功也需关注连接稳定性；`failures`累计最终失败的查询数。连续数轮不增加计数更有参考价值。
- 同ID重复舵机无法可靠识别，不会自动改ID。缺少某ID时不要在整条总线上盲改ID。

检测结束后才能另行选择运动测试，当前工程没有运动命令。恢复05主工程需重新烧录`single_esp32`；本检测不擦除NVS或舵机标定。


## v2回显排除

初次现场结果是四个ID均显示PING_OK、status=0x01，但反馈全部失败。这可能是转接板发送回显：PING请求中的指令字节0x01与状态字节处在相同位置，单靠PING帧校验无法区分回显和真实状态。不能据此认定四台损坏或均报电压故障。

v2改为首先读取地址5的ID寄存器，要求返回正确ID、长度和校验和；发送请求的回显不能代替这个应答。识别完整TX回显、统计tx_echoes_now，并排除与请求完全相同的应答。`DEBUG ON`后`TEST 1`会输出失败事务的BUS_TX/BUS_RX，便于区分只有回显、无数据和有效返回。显示回显不等同于转接板损坏；回显之外仍需有真实舵机应答。

```text
DEBUG ON
TEST 1
CHECK
WATCH OFF
```

以上均只读。未获得真实ID读回前，不作“舵机正常”结论。
