# DFRobot_9DOF_Optical_Odometer

- [English Version](./README.md)

Gravity 九轴光学里程计主机库。模块读取光流、六轴和地磁，并在板上做 Fusion。主机用 I2C 或 UART 读同一套数据。

## 产品

SKU：SEN0776（Gravity）

## 目录

- [概述](#概述)
- [安装](#安装)
- [接口](#接口)
- [兼容性](#兼容性)
- [历史](#历史)
- [创作者](#创作者)

## 概述

位移单位厘米，断电后丢失。`begin()` 读到产品号 0x0776 才成功。判断朝向用罗盘，模块放平读取。roll 或 pitch 超过约 55° 后，yaw 不再用地磁。

- I2C：mode 拨码断开。addr 闭合为 0x11，断开为 0x12。
- UART：mode 拨码闭合。9600 8N1，从站地址与 I2C 相同。依赖 DFRobot_RTU。
- 拨码只在上电时读取，软件不能切换。上电约 1 秒后主机口才可用，改拨码后需重新上电。
- Gravity 线序：红 = VCC，黑 = GND，蓝 = SDA（UART 时为模块 TX），黄 = SCL（UART 时为模块 RX）。VCC 为 3.3 V 或 5 V。UART 交叉接线。Mega 示例用 Serial3：TX3=14 接模块黄/RX，RX3=15 接模块蓝/TX。UNO 没有 Serial1，请用 I2C。

## 安装

先安装 [DFRobot_RTU](https://github.com/DFRobot/DFRobot_RTU)。下载本库，放到 Arduino 的 `libraries` 目录，然后打开 `examples` 里的示例。

## 接口

```cpp
  /**
   * @brief 打开总线并核对产品号 0x0776
   * @n 只适用于当前拨码对应的接口。上电约 1 秒内会失败，示例里会重试
   * @return true 成功，false 无应答或身份不符
   */
  bool begin(void);

  /**
   * @brief 读取 PAA5160E1-Q 产品号
   * @n 正常为 0x004F
   * @return true 成功，false 失败
   */
  bool getPAA_ID(uint16_t &productId);

  /**
   * @brief 读取 PAA5160E1-Q 版本号，有效值在低 8 位
   * @return true 成功，false 失败
   */
  bool getPAA_RevisionID(uint16_t &revisionId);

  /**
   * @brief 读取累计 X/Y 位移，单位 cm
   * @n 不断电持续累加，断电丢失。本接口不清零
   * @return 0 成功，-1 失败
   */
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);

  /**
   * @brief 清零累计位移
   * @n 不影响校准系数和 CPI
   * @return 0 成功，-1 失败
   */
  int resetTotalDisplacement(void);

  /**
   * @brief 设置 X/Y 位移校准比例，掉电保存
   * @n 系数 = 实际距离 / 模块读数，范围 0.5~2.0。写入后已累计的厘米数按新系数重算
   * @return 0 成功，-1 超范围或失败
   */
  int setCalibrationScale(float scaleX, float scaleY);

  /**
   * @brief 设置光流 CPI 挡位，掉电保存
   * @n 挡位 1~200，counts/英寸 = 挡位 × 100。已累计位移按新挡位重算
   * @return 0 成功，-1 参数非法或失败
   */
  int setCpi(uint16_t cpi);

  /**
   * @brief 读取 SQUAL，原始值 0~255
   * @n 有效特征点数 = squal × 4。偏低表示表面不适合跟踪
   * @return 0 成功，-1 失败
   */
  int readSqual(uint8_t *squal);

  /**
   * @brief 设置光流方向：交换 XY、翻转 X、翻转 Y
   * @n 不掉电保存。同时开启时先交换再翻转
   * @return true 已写入，false 失败
   */
  bool setOrientation(bool swapXY, bool invertX, bool invertY);

  /**
   * @brief 读取加速度和陀螺仪原始 LSB
   * @n 上电默认 ±8 g、±2000 °/s。轴已按丝印重映射
   * @return 0 成功，-1 失败
   */
  int get6dofData(int16_t *accel, int16_t *gyro);

  /**
   * @brief 读取三轴地磁，单位 µT
   * @n 已按丝印重映射。写过校准后这里是校准值
   * @return 0 成功，-1 失败
   */
  int getGeomagneticData(int16_t *mag);

  /**
   * @brief 读取 Fusion 欧拉角，单位度
   * @n 平放时 roll、pitch 接近 0°。倾斜超过约 55° 后 yaw 不能当航向
   * @return 0 成功，-1 失败
   */
  int getEuler(float *roll, float *pitch, float *yaw);

  /**
   * @brief 读取罗盘航向 0~360°。判断朝向以本值为准
   * @n 0° = 丝印 +X 朝北，90° = 东。宜在放平时读取
   * @return 0 成功，-1 失败
   */
  int getCompassDegree(float *compassDeg);

  /**
   * @brief 设置 BMI323 模式
   * @n 0 睡眠，1 低功耗，2 正常（默认），3 高性能
   * @return true 已写入，false 失败
   */
  bool setBMI_Mode(eSensorMode_t mode);

  /**
   * @brief 软复位 BMI323，并按当前模式与量程重新配置
   * @return true 已写入，false 失败
   */
  bool reset_BMI(void);

  /**
   * @brief 设置加速度量程
   * @n 0 = ±2 g，1 = ±4 g，2 = ±8 g（默认），3 = ±16 g
   * @return true 已写入，false 失败
   */
  bool setAccelRange(eAccelRange_t range);

  /**
   * @brief 设置陀螺仪量程
   * @n 0 = ±125，1 = ±250，2 = ±500，3 = ±1000，4 = ±2000 °/s（默认）
   * @return true 已写入，false 失败
   */
  bool setGyroRange(eGyroRange_t range);

  /**
   * @brief 设置 BMM350 电源模式
   * @n 0 挂起，1 常规（默认），2 强制，3 强制快速
   * @return true 已写入，false 失败
   */
  bool setBMM_Mode(uint8_t opMode);

  /**
   * @brief 设置 BMM350 预设精度和采样率
   * @n presetMode：1 低功耗，2 普通，4 增强，3 高精度。rate = 6 为 25 Hz
   * @return true 已写入，false 失败
   */
  bool setBMM_PresetMode(uint8_t presetMode, uint8_t rate);

  /**
   * @brief 只修改 BMM350 采样率
   * @return true 已写入，false 失败
   */
  bool setBMM_Rate(uint8_t rate);

  /**
   * @brief 打开或关闭 BMM350 的 X/Y/Z 测量，非 0 为打开
   * @n 上电默认三轴都测量
   * @return true 已写入，false 失败
   */
  bool setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ);

  /**
   * @brief 读取 BMM350 是否有新样本
   * @return true 有新样本，false 未就绪或失败
   */
  bool getDataReadyState(void);

  /**
   * @brief 读取 BMM350 电源模式，含义同 setBMM_Mode
   * @return true 成功，false 失败
   */
  bool getBMM_Mode(uint16_t *mode);

  /**
   * @brief 读取 BMM350 采样率枚举
   * @return true 成功，false 失败
   */
  bool getBMM_Rate(uint8_t *rate);

  /**
   * @brief 读取姿态四元数，顺序 w、x、y、z
   * @return 0 成功，-1 失败
   */
  int fusionAhrsGetQuaternion(float q[4]);

  /**
   * @brief 读取重力向量，单位 g
   * @return 0 成功，-1 失败
   */
  int fusionAhrsGetGravity(float gravity[3]);

  /**
   * @brief 读取已去除重力的线性加速度，传感器坐标系，单位 g
   * @return 0 成功，-1 失败
   */
  int fusionAhrsGetLinearAcceleration(float linearAccel[3]);

  /**
   * @brief 读取已去除重力的线性加速度，北西天坐标系，单位 g
   * @return 0 成功，-1 失败
   */
  int fusionAhrsGetEarthAcceleration(float earthAccel[3]);

  /**
   * @brief 读取陀螺零偏，单位 °/s
   * @n 静止约 3 秒后开始估计，变化超过 0.05 °/s 且间隔不少于 120 秒时写入 Flash
   * @return 0 成功，-1 失败
   */
  int fusionBiasGetOffset(float gyroBias[3]);

  /**
   * @brief 写入硬铁偏移（µT）和 3×3 软铁矩阵，掉电保存
   * @n 先减硬铁，再乘软铁。未写过时硬铁为 0、软铁为单位阵
   * @return 0 已保存，-1 失败
   */
  int setMagCalibration(const float hardIron[3], const float softIron[3][3]);

  /**
   * @brief 预留。空实现，模式由 mode 拨码决定
   */
  void setHostModeSwitch(void);

  /**
   * @brief 预留。空实现，地址由 addr 拨码决定
   */
  void setI2cAddressSwitch(void);

  /**
   * @brief I2C 构造。i2cAddr 默认 0x11，addr 拨码断开时传 0x12
   * @n 不打开总线，在 setup() 里调用 begin()
   */
  explicit DFRobot_9DofODO_I2C(TwoWire *wire = &Wire, uint8_t i2cAddr = 0x11);
  bool begin(void);

  /**
   * @brief UART 构造。波特率固定 9600，unitId 默认 0x11
   * @n 有 Serial1 的板子默认 &Serial1。Mega 示例传入 &Serial3。帧格式 8N1
   */
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial = &Serial1, uint32_t baudRate = 9600, uint8_t unitId = 0x11);
  bool begin(void);
```

## 兼容性

| 主板 | 通过 | 未通过 | 未测试 | 备注 |
| --- | :--: | :----: | :----: | --- |
| Arduino UNO |  |  | √ | 仅 I2C，无 Serial1 |
| Arduino Mega2560 | √ |  |  | I2C 与 UART |
| Arduino Leonardo |  |  | √ |  |
| ESP32 |  |  | √ |  |

## 历史

- 2026/10/14 - V1.0.0

## 创作者

Written by wxzed(xiao.wu@dfrobot.com), 2026. (Welcome to our [website](https://www.dfrobot.com/))
