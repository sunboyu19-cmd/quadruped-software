# ESP32-S3 + 微雪转接板 + STS3032 舵机调试

适配遥操手使用的ESP32-S3-WROOM-2-N32R16V（32MB Flash/16MB PSRAM），使用微雪 **Bus Servo Adapter (A)，SKU25514** 和飞特 **STS3032-C001 TTL总线舵机**。不是直接控制普通三线PWM舵机，也不是往微雪板里烧录固件。

程序烧录到ESP32-S3。启动只发送串口控制台提示，不自动运动、不写EEPROM、不主动使能扭矩。舵机上电本身的行为由其内部配置决定；先脱离腿部负载或拆下连杆调试。软件尚需实物验证。

## 接线

先断开USB和动力电源，拔掉轮电机驱动及编码器测试接线，轮电机动力保持关闭。本程序只测试舵机，不同时运行轮电机FOC。

| 位置 | 连接 |
|---|---|
| ESP32-S3 GPIO17（UART TX） | 微雪板UART的 **TX** |
| ESP32-S3 GPIO18（UART RX） | 微雪板UART的 **RX** |
| ESP32-S3 GND | 微雪板GND，动力电源负极共地 |
| 微雪板控制跳帽 | **A位（UART）** |
| 稳压6V正负极 | 微雪板绿色电源端子，按板上正负标识接 |
| STS3032原装3P线 | 微雪板总线舵机接口，核对G/V/S极性 |
| ESP32-S3标USB的原生接口 | USB数据线连接电脑，供电及程序控制台 |

**这块Adapter(A)官方要求TX接TX、RX接RX，不按一般UART交叉接线。** 使用GPIO编号，不是排针序号。微雪板USB-C在本方案不接电脑，避免和外部UART同时驱动。不要从ESP32的3V3/5V给舵机供电，也不要把6V接到ESP32。

首次只接一个舵机。四个未改过ID的舵机可能都是ID1，不能直接全接在同一总线上。S3 USB控制台115200，与舵机UART默认1000000是两个不同波特率。

## 桌面版本的命令行

三个工程共用上一级`.tools`虚拟环境。在VS Code选择“文件→打开文件夹”，打开本工程`03_ESP32S3_Servo`，再打开PowerShell终端。

```powershell
cd "E:\Users\32892\Desktop\机器狗\四轮足程序\03_ESP32S3_Servo"
```

查看串口：

```powershell
& "..\.tools\Scripts\python.exe" -m serial.tools.list_ports -v
```

本次S3原生USB通常为COM10（VID:PID 303A:1001），重插后可能变化。不要误用CH343的COM11。下面按实际串口替换COM10。

编译（不写板）：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_servo_test
```

烧录：先关闭舵机6V，保留ESP32 USB。若正在监视串口，Ctrl+C退出，避免串口占用。

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_servo_test -t upload --upload-port COM10
```

显示SUCCESS后，打开串口监视器：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM10 -b 115200 --echo --eol LF --filter send_on_enter
```

点击终端正文，英文输入设备指令，每条单独回车。退出监视器按Ctrl+C；无启动提示可短按RST/EN。无需Activate.ps1，无需修改PowerShell执行策略。

## 第一次测试

1. 固定舵机、卸下负载，接通6V，输入`H`回车验证控制台输入。
2. 输入`SCAN`，记录FOUND的ID；只读扫描0～253，不使用广播。扫描期间约数秒不处理其他指令，禁止带扭矩扫描。
3. 假设找到ID1，输入`I 1`，再输入`S`。确认位置、供电和温度合理。位置0～4095，一圈4096步；`speed_raw`、`load_raw`是原始量，不是rpm和电流。
4. 输入`A`：程序检查位置模式为0、位置限位、5.0～6.8V供电、温度低于60°C；先关闭扭矩，在当前反馈位置预置目标，设置20%运行扭矩上限，然后使能。不会主动回到2048，但使能时可能有微小位移。
5. 看到ARMED后输入`J 5`，相对上次目标增加5°；再输入`J -5`返回。每次最大±5°，相对本次使能位置累计最大±30°，不跨0/4095边界，也不绕过舵机内部限位。
6. `K`保持测试；30秒没有J/K会发送扭矩关闭命令。`D`立即请求关闭扭矩并读回验证，随后关闭6V。**卸扭矩后机构可能下落，必须先支撑。**

软件±30°只是调试窗口，不知道实际腿部机械限位。初次安装连杆前先确认方向、舵盘位置和可用行程；不要通过反复A重置中心来绕过机械限位。20%是舵机扭矩寄存器上限，不是经过标定的电流或力矩控制。

## 指令表

| 指令 | 含义 |
|---|---|
| `H` | 帮助 |
| `SCAN` | 当前波特率下扫描全部ID，不自动选择 |
| `I 1` | 选择ID1，只改主控选择，不改舵机ID |
| `PING` | 检查选中舵机是否应答 |
| `S` | 读取位置、速度原始值、负载原始值、电压、温度 |
| `A` | 在当前位置使能；仅支持模式0，其他模式不自动改写 |
| `J 5` / `J -5` | 相对目标小角度点动 |
| `K` | 刷新30秒超时，S/H不刷新 |
| `D` | 关闭选中舵机扭矩并读回验证 |
| `ID 2 SINGLE` | 将选中舵机永久改为ID2；必须物理只接一个舵机 |
| `B 115200` | 临时把主控舵机UART改为115200，不改舵机内部波特率 |

`B`可选38400、57600、76800、115200、128000、250000、500000、1000000。主控重启恢复1000000。只在未使能且卸扭矩已确认时允许切换ID、扫描、切换波特率。

## 四个舵机逐个编号

建议贴标签：1=右前、2=左前、3=右后、4=左后；编号是人为约定，可按实际布局调整。这个编号与轮电机AS5600的固定I²C地址无关，舵机ID可以永久修改。

每次只接一个舵机，先SCAN找到当前ID，例如1，发送`I 1`和`D`。若需要改成2，发送完整的`ID 2 SINGLE`。程序卸扭矩、解锁、写ID、验证新ID并重新锁定；不会广播改ID。显示成功后断电再上电，SCAN确认ID已保留。目标本来就是1时不需要再改。

每个舵机改好并贴上标签后再断电更换下一个。全部ID唯一后才可以连接在同一条总线上，用`I <id>`逐个选择测试。**SCAN无法发现两个相同ID的舵机：它们可能表现为一台或无应答。因此SINGLE是物理接线要求，程序无法验证。**

## 常见问题与限制

- 无应答：检查6V极性、跳帽A、TX-TX/RX-RX、GND共地、3P插头顺序、是否使用STS3032 TTL型号，以及是否接了同ID舵机。用B改变主控波特率后再SCAN。
- `BLOCKED: operating mode...`：不是位置模式0。程序不自动改模式/零点/角度限位；使用厂家工具核对该舵机的模式与型号。
- `BLOCKED: feedback...`：检查电压、温度、反馈位置和EEPROM限位。不要盲目提高电源电压。
- `torque_off_verified=0` / `off_pending=1`：无法确认卸扭矩，程序每500ms重试，禁止选其他ID；必要时物理断电，不能把软件打印DISABLED当作已卸扭矩。
- 使用过程中两次连续反馈/电压/温度检查失败，会请求卸扭矩。串口断开但主控仍运行，30秒后会超时；主控断电或重启后无法继续保证超时控制，舵机可能仍保持原目标。
- 修改ID失败时不要多次盲写；只接一个舵机，断电重启后SCAN核对。其他EEPROM配置不自动写入。
- 程序不写舵机零点、不强制回中、不切换电机连续旋转模式。ID命令会写EEPROM，这是唯一提供的持久配置操作。

## 官方参考与库来源

- [微雪Adapter(A)接线，UART跳帽A、TX接TX/RX接RX](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
- [微雪Adapter(A)规格，输入须匹配舵机电压](https://docs.waveshare.com/Bus_Servo_Adapter_A)
- [飞特STS3032-C001的6V规格](https://www.feetech.cn/Data/feetechrc/upload/file/20200611/6372749220559698399729459.pdf)
- [微雪ST系列协议](https://docs.waveshare.com/Servo_Communication_Protocol)
- [微雪ST系列寄存器说明](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation)
- [微雪官方Arduino库下载页](https://docs.waveshare.com/Bus_Servo_Adapter_A/Resources-And-Documents)

底层使用官方库的SMS_STS；库文件已放入lib/SCServo，保留原文件头和SOURCE说明，不需要另外安装SCServo库。芯片Flash/PSRAM配置沿用已测试的S3轮电机工程。


## 桌面整理版说明

本工程位于桌面“机器狗/四轮足程序”，三个工程共用上一级`.tools`，不需要激活虚拟环境。以`include/config.h`为实际编译配置；修改后需保存并重新烧录。此处是复制的独立工程，与原工作目录不会自动同步。
当前电脑的共享工具环境已准备好；换电脑时先安装Python，再在总目录执行`python -m venv .tools`和`.\.tools\Scripts\python.exe -m pip install platformio==6.1.18`。
