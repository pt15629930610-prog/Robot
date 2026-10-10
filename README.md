# Palm Robot

棕榈果采摘机器人软件项目。

## 当前功能
- 双轴关节电机 CAN 速度控制
- SMC60 摇杆输入
- MG 电机角度、速度和电流反馈
- 疑似干涉检测

## 硬件
- 横滚电机：CAN 0x141
- 俯仰电机：CAN 0x142
- 摇杆：CAN 0x101
- CAN 波特率：250000 bit/s
- 减速比：24:1

## 当前运行环境
Windows + Python + PEAK PCAN-USB

## 运行
先关闭 PCAN-View。

python legacy/dual_axis_control.py

## 说明
当前为实验性控制软件。
必须配备独立硬件急停，不得将电流检测视为安全认证功能。