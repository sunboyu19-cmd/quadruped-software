# 遥操手 ESP32-S3 单轮FOC测试

独立工程，适配遥操手工程记录的 **ESP32-S3-WROOM-2-N32R16V**：32MB Octal Flash、16MB Octal PSRAM、原生USB Serial/JTAG。
原遥操手源码和原经典ESP32版FOC工程保持原状。2026-11-08已完成编码器读取、7极对校准检查及单轮低速转动测试。
以后烧录本工程会替换板上的遥操手固件；要恢复遥操手功能需重新烧录原工程。本程序不写入NVS或编码器OTP，也不执行整片擦除。

## 先换线，再换程序

先断电并拔掉遥操手的所有外设（尤其GPIO4手别跳线、GPIO8/9原AS5048B和舵机总线）。本工程读的是轮电机AS5600（固定地址0x36），不是遥操手AS5048B。
下表均为GPIO编号，不是排针上的第几针。旧经典ESP32版的25/26/27不能照搬到S3。

| ESP32-S3 | 连接 |
|---|---|
| 3V3 | AS5600模块确认可用的3.3V供电脚 |
| GND | AS5600 GND、MiniFOC GND、电机电源负极共地 |
| GPIO8 | AS5600 SDA |
| GPIO9 | AS5600 SCL |
| GPIO4 | MiniFOC IN1/PWM-A |
| GPIO5 | MiniFOC IN2/PWM-B |
| GPIO6 | MiniFOC IN3/PWM-C |
| GPIO7 | 已核对为高电平使能的EN/nSLEEP |
| MiniFOC U/V/W | 轮电机三根相线 |
| 动力电源正极→保险→开关 | MiniFOC VM/VIN |

编码器磁铁固定在转轴、居中且不倾斜；编码器板固定在静止支架。SDA/SCL只上拉到3.3V，已有合适板载上拉时不重复添加。
动力电源只接驱动板，不接ESP32的3V3或USB。GPIO19/20保留给USB；GPIO35/36/37为该WROOM-2存储器占用，不作外接控制。
MiniFOC商品标12–24V，本次用户使用6V完成了单轮低速测试；这不等于厂家确认整板支持6V全工况，也不证明此前电源反接未造成损伤。不要把DRV8316芯片范围当成整板范围。

## USB线与串口

使用这块S3板上标有 **USB** 的原生USB接口（VID:PID为303A:1001），数据线能传数据即可，不需要等待新开发板的Micro-USB线。
当前只读枚举看到原生USB为COM10、CH343为COM11；端口号可变化，也不能仅凭端口号确认是同一块板。
默认`Serial`走原生USB，不走CH343/UART口。上传/监视器需选原生USB口，先关闭遥操手控制台和其他占用串口的软件。
串口无输出时先确认USB插口/数据线，重启板后重新选择端口；程序不阻塞等待USB连接。

## 操作顺序

1. VS Code打开本目录，默认`s3_sensor_test`，只接USB和AS5600，驱动动力断开。Ctrl+Shift+B可编译；任务菜单提供上传编码器测试及串口监视器，上传会替换板上原固件。
2. 115200串口中看`a`角度和`w`角速度，手转一圈应连续变化。`mag`是十六进制状态寄存器，判断MD位为1且ML/MH位为0；本次实测显示`mag=67`，不能要求整个字节必须等于`20`。磁场状态不证明磁铁同轴精度。
3. 核对MiniFOC实际针脚和供电后，在`include/config.h`填写实测`SUPPLY_VOLTAGE`，再将`DRIVER_WIRING_AND_POWER_CONFIRMED`改为`true`。未确认时`A`会拒绝使能。
4. 选择`s3_motor_test`环境编译/上传，电机悬空固定。动力供电合规后输入`A`并回车校准；校准期间转子会动且串口指令阻塞数秒，物理开关可断电。
5. 输入`V1`：约9.55rpm；`V-1`反转；`V0`零速带电保持；`D`关闭驱动。10秒无`V`或`K`命令会关闭输出，长时间测试定期发`K`。`S`状态、`H`帮助。

## 初始参数

用户已核实电机为7极对。本次整理时`include/config.h`为对齐1.5V、输出电压指令限幅1.5V、目标速度±5rad/s，确认锁为`true`。如调整参数，必须保存并重新烧录才对板内程序生效；成功运行后也应保存对应参数。
当前供电参数按用户使用的6V设置；使用前仍需核对驱动整板支持6V，并以实测VM值为准。程序不自动测母线电压。
控制为AS5600编码器闭环速度、底层电压模式FOC，没有电流采样/电流闭环。不能把1.5V限幅当作硬件限流。

## 工具版本及构建

ESP32平台6.9.0 / Arduino-ESP32 2.0.17 / SimpleFOC 2.3.3；保留遥操手的OPI/OPI存储器与USB配置，未沿用遥操手串口控制协议。
使用本目录`../.tools/Scripts/python.exe -m platformio run -e s3_sensor_test -e s3_motor_test`构建。该独立虚拟环境不改遥操手Python环境。
新电脑可用Python创建`.venv`，再安装`platformio==6.1.18`，或使用VS Code的PlatformIO扩展。

## VS Code命令行操作

在VS Code中选择“终端 → 新建终端”，使用PowerShell。以下电脑命令都在显示`PS C:\...>`的终端中执行，每条输入后按回车。无需运行`Activate.ps1`或修改系统脚本执行策略，本工程直接调用虚拟环境内的Python。

### 1. 进入工程目录

```powershell
cd "E:\Users\32892\Desktop\机器狗\四轮足程序\02_ESP32S3_WheelFOC"
```

后面的命令均在此目录运行。`cd`切换目录，`&`执行引号里的程序，`.\`表示当前目录。

### 2. 查看串口

```powershell
& "..\.tools\Scripts\python.exe" -m serial.tools.list_ports -v
```

本次原生USB为COM10（VID:PID `303A:1001`），CH343为COM11。重插后编号可能变化；后面的COM10应替换为实际原生USB串口。程序控制台不走CH343接口。

### 3. 编译（不烧录）

只编译编码器版：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_sensor_test
```

只编译电机版：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_motor_test
```

`-m platformio`运行PlatformIO，`run`编译，`-e`选择版本。显示`SUCCESS`表示编译通过，不代表已经写入开发板或完成实物测试。

### 4. 烧录

先关闭电机动力电源、保留USB供电；若串口监视器正在运行，先按Ctrl+C退出并关闭其他占用该串口的软件。烧录会替换板内现有程序。

编码器版（不会初始化电机PWM）：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_sensor_test -t upload --upload-port COM10
```

电机版：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_motor_test -t upload --upload-port COM10
```

`-t upload`指定烧录，`--upload-port`指定串口。等`SUCCESS`后再打开监视器。修改配置文件后按Ctrl+S保存，再执行对应烧录命令；只编译不会更新板内固件。

### 5. 打开串口监视器

```powershell
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM10 -b 115200 --echo --eol LF --filter send_on_enter
```

`-p`指定串口，`-b`指定波特率，`--echo`显示键盘输入，`--eol LF`指定换行，`send_on_enter`让命令在回车后发送。没有启动输出时短按RST/EN复位。

**点击终端正文后输入设备指令，每条单独按回车。** 不要把设备指令输入到`PS ...>`提示符下，也不要把引号、反引号或字面量`\n`一起输入。退出监视器按Ctrl+C，回到PowerShell后才能运行编译、烧录命令。

## 发给ESP32的串口指令

这些是设备指令，不是PowerShell命令。

| 指令 | 作用 |
|---|---|
| `H` | 显示帮助，先用它确认指令能被ESP32收到 |
| `S` | 显示状态 |
| `A` | 校准并使能；校准时电机会动，之后目标为零速 |
| `V1` | 目标+1rad/s，约9.55rpm |
| `V-1` | 目标−1rad/s，反向低速 |
| `V0` | 主动保持零速，仍然带电，不等于禁用 |
| `K` | 刷新10秒指令超时，不改变目标速度 |
| `D` | 禁用驱动并清零目标；不能替代物理断电 |

`V`允许−5至+5rad/s。使能后10秒内没有新的`V`或`K`指令会自动禁用；重新启动需再次发送`A`校准。

### 首次低速测试顺序

1. 轮子悬空固定，核对供电极性、接线及参数，打开串口；先发送`H`、`S`。
2. 确认显示`mode=motor`，接通动力电源后发送`A`。校准期间程序可能数秒无法处理串口指令，异常时使用物理电源开关。
3. 应看到`PP check: OK!`、`MOT: Ready.`、`ARMED at zero speed`。当前版本在极对数检查失败时会禁用驱动；旧版即使库检查失败也可能显示ARMED，须发送`D`排查。
4. 看到ARMED后及时发送`V1`，短时观察转动及温升。需要继续观察时，在10秒内发送`K`。
5. 发送`V0`并等停稳，再发`V-1`测试反向；结束发送`D`，再关闭动力电源。

状态字段：`en`表示程序是否完成校准并使能，不是实测EN引脚电平；`a`为累计角度（rad），`w`为角速度（rad/s），`t`为目标角速度，`uq`为q轴电压指令。负角度和超过一圈的累计角度正常。

## 常见提示及排查

| 提示或现象 | 处理 |
|---|---|
| `Activate.ps1`被禁止 | 无需激活环境，直接使用上面的Python完整相对路径 |
| `A`无法识别为命令 | 当前是PowerShell；先打开串口监视器，再输入设备指令 |
| `Unexpected command arguments` | 一行内有多余字符；回车结束当前行，再用英文输入法输入单个`H`并回车 |
| 输入无反应 | 使用上述带LF和send_on_enter的监视器命令，点击终端正文，先测试`H`；本地回显不等于ESP32已收到 |
| `mode=sensor` | 当前为编码器版；运行电机需要明确烧录`s3_motor_test` |
| `BLOCKED: verify driver...` | 确认硬件后将配置中的确认锁设为`true`，保存并重新烧录电机版 |
| `Failed to notice movement` | 先禁用和断电，检查动力供电、相线、驱动模式、EN、机械阻力与磁铁是否随转子转动 |
| `PP check: fail` | 保持已核实的7极对；排查校准运动、磁铁同轴性和驱动力，不直接使用估算值改极对数 |
| `No current sense` | 当前电压模式无电流采样的正常提示；还需等待Ready及ARMED |
| `DISABLED: 10s without V or K command` | 指令超时；重新发送`A`，校准成功后再发速度指令 |
| 烧录串口被占用 | Ctrl+C退出监视器，关闭其他串口软件后重试 |
| 烧录停在Connecting | 按住BOOT、短按RST/EN、松开BOOT，重新检查串口号后重试 |

明显抖动、异响或异常发热时立即断开动力电源；不要通过持续加大电压强行运行。

## 左右编号与左腿调试（逐一测试）

编号约定：**1号=右腿/right，2号=左腿/left**。这是固件的软件标签，AS5600地址均为`0x36`，没有烧写芯片地址或永久编号。需在实物上贴R1、L2标签以防接反；左右由烧录版本指定，固件不会自动识别传感器身份。

先发送`D`，关闭动力和USB，再拔下右腿编码器及驱动的电源和信号线。左腿使用刚才同一组引脚，不使用GPIO10/11或GPIO12～15。每次只连接一套电机、驱动和编码器（全部为GPIO编号）：

| 信号 | 右腿1号 | 左腿2号 |
|---|---|---|
| AS5600 SDA | GPIO8 | GPIO8 |
| AS5600 SCL | GPIO9 | GPIO9 |
| AS5600地址 | 0x36 | 0x36 |
| 驱动PWM A/B/C | GPIO4/5/6 | GPIO4/5/6 |
| 驱动EN，高电平使能 | GPIO7 | GPIO7 |

AS5600用确认可用的3.3V供电脚，与驱动和ESP32共地，上拉只接3.3V。不要将两个AS5600并接在同一对SDA/SCL上。连接的电机、编码器和驱动必须属于同一条腿。驱动动力仍接6V，不能把6V接到编码器或ESP32。

原`s3_sensor_test`和`s3_motor_test`标记为右腿1号；新增`s3_left_sensor_test`和`s3_left_motor_test`标记为左腿2号。四个版本均只读一个编码器、控制一个驱动，不支持双腿同时连接测试。

### 先烧录左腿编码器测试版

在工程目录执行，烧录前关闭动力电源并退出监视器：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_left_sensor_test -t upload --upload-port COM10
```

打开监视器：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM10 -b 115200 --echo --eol LF --filter send_on_enter
```

输出应为`id=2 leg=left mode=sensor ...`。手转左轮检查`a`变化，停止后`w`应回到接近0。未找到传感器时启动会报告缺失，不能把零角度当作读取成功。

### 再烧录左腿电机版

Ctrl+C退出监视器，关闭动力电源，执行：

```powershell
& "..\.tools\Scripts\python.exe" -m platformio run -e s3_left_motor_test -t upload --upload-port COM10
```

使用上面的同一条监视器命令。确认显示`id=2 leg=left mode=motor`，左轮悬空，接通动力后发送`A`；看到极对数检查通过且ARMED，再发送`V1`。`D`禁用、`K`刷新超时。该版本的全部运动指令仅作用于左腿。

切回右腿时先断电换回右腿整套接线，再烧录`s3_motor_test`。左右校准每次由`A`重新执行，不沿用另一条腿的电角度零点；正方向由各自安装和校准决定，`V1`不保证左右轮在车身上同为前进方向。两腿当前共用`config.h`里的电机参数，更改参数会影响下一次编译的两腿版本；这不是每腿独立保存参数的程序。

参考：[ESP32-S3开发板官方引脚说明](https://docs.espressif.com/projects/esp-idf/en/v5.0.3/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html)、[SimpleFOC AS5600](https://docs.simplefoc.com/magnetic_sensor_i2c)。


## 桌面整理版说明

本工程位于桌面“机器狗/四轮足程序”，三个工程共用上一级`.tools`，不需要激活虚拟环境。以`include/config.h`为实际编译配置；修改后需保存并重新烧录。此处是复制的独立工程，与原工作目录不会自动同步。
当前电脑的共享工具环境已准备好；换电脑时先安装Python，再在总目录执行`python -m venv .tools`和`.\.tools\Scripts\python.exe -m pip install platformio==6.1.18`。
