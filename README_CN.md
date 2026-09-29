# DFRobot_9DOF_Optical_Odometer

SEN0776 九轴光学里程计主机库：通过 **I2C**（默认 **0x11**）读取光流累计位移、六轴 IMU、地磁与板载 Fusion 欧拉角/罗盘。

- [English](./README.md)

## 安装

将本库放入 Arduino `libraries` 目录，或在库管理器可用后在线安装。

## 接线（I2C）

| 模块 | Arduino UNO |
|------|-------------|
| SDA  | A4          |
| SCL  | A5          |
| VCC  | 5V          |
| GND  | GND         |

打开 `examples/` 下任一示例，串口监视器 **115200**。

## 示例

| 示例 | 说明 |
|------|------|
| getOdoAndImuData | 位移、欧拉角、六轴原始值 |
| getMagAndCompass | 磁力与罗盘角 |
| setCpiAndCalibration | CPI 与比例校准 |
| bmm350Calibration | BMM350 软铁校准流程占位 |
| fusionAttitude | 通过产品寄存器读取 Fusion 相关量 |
