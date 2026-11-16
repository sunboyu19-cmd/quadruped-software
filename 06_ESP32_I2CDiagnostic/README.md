# 四轮足独立I²C焊线检测

普通ESP32-WROOM，GPIO21=SDA，GPIO22=SCL，PCA9548A/TCA9548A默认地址0x70。仅运行I²C检测，GPIO13共用EN保持低；不初始化电机PWM、舵机串口或WiFi。**6V动力保持断开，USB及正确的3.3V逻辑供电即可。**

| 多路器通道（不是I²C地址） | 设备 | 预期I²C地址 |
|---:|---|---|
| 0 | MPU6050 / GY-521 | 0x68，AD0接GND；AD0高则0x69 |
| 1 | 轮4 右后AS5600 | 0x36 |
| 2 | 轮2 右前AS5600 | 0x36 |
| 6 | 轮3 左后AS5600 | 0x36 |
| 7 | 轮1 左前AS5600 | 0x36 |
| 3、4、5 | 暂空 | 无下游设备 |

多路器VCC=3.3V、GND共地；A0/A1/A2保持低得到0x70，RESET保持高。各模块电源按丝印连接，SDA/SCL上拉只到3.3V。只接正确的逻辑电源，不用6V给ESP32或传感器供电。软件保持EN低不能替代断开动力及实际硬件下拉。

## VS Code 操作

工程位于OneDrive“四轮足程序”的 `06_ESP32_I2CDiagnostic`，已加入“四轮足软件.code-workspace”。在该工程终端运行：

```powershell
# 编译
& "..\.tools\Scripts\python.exe" -m platformio run -e i2c_diagnostic
# 查串口
& "..\.tools\Scripts\python.exe" -m platformio device list
# 烧录：把COM13替换为实际端口，关闭占用端口的网页/串口监视器
& "..\.tools\Scripts\python.exe" -m platformio run -e i2c_diagnostic -t upload --upload-port COM13
# 打开串口监视器
& "..\.tools\Scripts\python.exe" -m platformio device monitor -p COM13 -b 115200 --echo --eol LF --filter send_on_enter
```

监视器中每条命令按回车发送。退出监视器用Ctrl+C。也可使用工作区任务 `I2C: Build / Upload / Serial monitor`，注意选择06工程任务。

v2上电只打印帮助和总线电平，不自动全地址扫描。`SCAN`仍可手动使用，遇超时/选通失败会中止，避免把未扫完误报为空通道。已知设备的逐路检查优先使用TEST。需要整体读数时可输入：

```text
SCAN
CHECK
IMU ON
WATCH ON
```

逐个手动转轮，确认对应 `raw`、`deg` 和 `delta` 改变，其他静止轮基本不变；轻轻倾斜IMU，查看ax/ay/az变化。`WATCH OFF`停止刷新，方便复制日志。IMU默认上电可能休眠，`sleep=1`但WHO正确说明通信成功，输入`IMU ON`只唤醒传感器，不驱动电机。

## 如何判断

- `UPSTREAM`正常只看到多路器0x70。扫描各通道会剔除上游设备，避免把每一路都能看到的多路器误认为下游传感器。
- `MUX FAIL`：多路器写选通/读取选通失败，先检查电源、公共地、GPIO21/22、RST及地址引脚。扫描会列出0x70～0x77的ACK候选；确认模块实际地址后可输入`MUX 71`等更改，不改硬件地址。
- `comm=OK`说明该次寄存器读成功；`comm=FAIL`及errors累计表示读失败，可能是焊接、电源、上拉或连线问题，单凭软件不能确定哪一焊点有问题。
- `magnet=OK`说明AS5600检测到磁场且强度不在过弱/过强标志范围；`NOT_DETECTED / TOO_WEAK / TOO_STRONG`需调整磁铁。磁场问题与I²C断线分别报告。
- 角度静止不变是正常现象，要手动转动对应电机转子验证。单个通道内如果并了两个相同地址设备，仅凭ACK不能识别重复地址，需核对实物接线。
- IMU `WHO=0x68`为预期MPU6050标识；地址可为0x68或0x69，WHO仍为0x68。`WRONG_ID`不能当作已验证MPU6050。温度是芯片内部温度。
- 默认先用100kHz排查焊线。100kHz正常后再输入`SPEED 400000`、`WATCH ON`，核对主工程需要的400kHz下errors不持续增加；只在100kHz通过还不代表主工程工作频率已通过。
- `RESETSTATS`只清累计错误及角度差值记录；`H`显示帮助。

烧录此检测程序会替换当前ESP32应用，不擦除NVS标定。检测结束后，关闭串口监视器再烧录05单主控工程；实际通道映射改变后应重新做FOC标定，不直接沿用原接线记录。此检测工程无网页控制入口。


## v2：重焊后间歇超时定位（2026-11-15）

收到重焊后日志：多路器读回及IMU/右前/右后出现`Error 263`。263=0x107=ESP_ERR_TIMEOUT，表示I²C操作超时；不能仅凭该错误断定芯片损坏。旧程序请求10ms超时，底层硬件/驱动异常时实际等待仍可能更长；v2不把增加等待时间当作修复。

升级v2后保持6V断开，100kHz逐行操作，每步看完结果再发下一条：

```text
WATCH OFF
OFF
LINES
TEST 7
TEST 6
TEST 2
TEST 1
TEST 0
```

- `OFF verified=1`：多路器全部通道关闭且控制字读回一致。
- `LINES`：在无I²C传输时SDA21/SCL22通常均为1。读到0表示当时该线被拉低，需检查短路、接反、异常设备/供电；读到1仅说明这一瞬间为高，不代表波形和接触一定正常。
- `TEST n`关闭其他通道，仅检查一个已知设备，然后再次关闭通道。记录出错通道、掩码、总线电平与耗时；`mask=0x01/02/04/40/80`分别对应通道0/1/2/6/7。
- 某通道出错后若`OFF`也失败或线持续为0，先停止，不继续推断后面通道。关闭USB及所有逻辑电源后检查焊点再重试；只按ESP32复位键不一定能复位仍有电的多路器/传感器。
- 若关闭全部通道也失败，优先检查ESP32到多路器的上游SDA21/SCL22、公共3.3V/GND、RST及上拉；若全关稳定、仅选某支路后失败，重点检查该支路。多个分支相互影响时也要排查通道间连锡。
- 所有补焊/测电阻或通断操作先断开全部供电。检查SDA-SCL、信号-电源/地有无短路；测到上拉电阻形成的有限阻值不等同于短路，应结合电路判断。

v2仍无电机PWM、舵机串口或WiFi，不改05主工程运动限幅、不自动恢复或隐藏总线故障。参考：[乐鑫错误码说明](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html)。
