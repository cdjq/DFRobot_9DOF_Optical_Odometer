/*!
 * @file getOdoAndImuData.ino
 * @brief 主示例（I2C）：读取 Gravity 九轴光学里程计的总位移、表面质量与融合欧拉角。
 * @details setup 里 begin() 并清零累计位移；loop 里周期打印 X/Y 位移（cm）、SQUAL 与 roll/pitch/yaw（度）。
 * @n 接线（Gravity 线序：红=VCC，黑=GND，蓝=SDA，黄=SCL。本版固件固定 I2C 地址 0x11）：
 * @n UNO     : SDA=A4, SCL=A5, VCC=3.3V 或 5V, GND=GND
 * @n Mega2560: SDA=20, SCL=21, VCC=3.3V 或 5V, GND=GND
 * @n ESP32   : SDA=GPIO21, SCL=GPIO22, VCC=3.3V, GND=GND（Wire 默认脚）
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */
#include "DFRobot_9DofODO.h"

/* 默认 I2C 地址 0x11。本版固件不读地址拨码。 */
DFRobot_9DofODO_I2C odo;

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    Serial.println(F("Waiting SEN0776..."));
    delay(1000);
  }
  Serial.println(F("SEN0776 ready"));
  /* 累计位移在模块 RAM 中；主机复位而模块仍供电时会保留旧值，示例先清零便于观测。 */
  if (odo.resetTotalDisplacement() == 0) {
    Serial.println(F("total displacement reset"));
  }
}

void loop()
{
  float tx;
  float ty;
  float roll;
  float pitch;
  float yaw;
  int16_t acc[3];
  int16_t gyr[3];
  uint8_t squal;

  if (odo.getTotalDisplacement(&tx, &ty) == 0 && odo.getEuler(&roll, &pitch, &yaw) == 0 &&
      odo.get6dofData(acc, gyr) == 0 && odo.readSqual(&squal) == 0) {
    /* SQUAL 为原始值 0~255，有效特征点数 = squal × 4。yaw：0° = 丝印 +X 朝北。 */
    Serial.print(F("X = "));
    Serial.print(tx, 3);
    Serial.print(F(" cm, Y = "));
    Serial.print(ty, 3);
    Serial.print(F(" cm  |  SQUAL = "));
    Serial.print(squal);
    Serial.print(F(" (features = "));
    Serial.print((uint16_t)squal * 4u);
    Serial.println(F(")"));
    Serial.print(F("roll = "));
    Serial.print(roll, 2);
    Serial.print(F(" deg, pitch = "));
    Serial.print(pitch, 2);
    Serial.print(F(" deg, yaw = "));
    Serial.print(yaw, 2);
    Serial.println(F(" deg"));
  }
  delay(200);
}
