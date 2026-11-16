# 来源

本目录的.h、.cpp、library.properties文件来自微雪官方资源页提供的Arduino库，保留原文件头。
本地修复：`SMS_STS::ReadMode(-1)`原先从56～70寄存器缓存读取33号寄存器，索引为负；现返回-1并标记错误。应用直接通过`readByte(id,33)`读取模式，不依赖这个缓存接口。其余库代码未修改。

- 资源页：https://docs.waveshare.com/Bus_Servo_Adapter_A/Resources-And-Documents
- 下载：https://files.waveshare.com/wiki/Bus-Servo-Adapter-%28A%29/SCSTServoLibrary.zip
- 取得日期：2026-11-08

应用使用SMS_STS（ST系列小端字节序），不使用SCSCL（SC系列大端字节序）。
