/*!
 * @file setCpiAndCalibration.ino
 * @brief 演示写入 CPI 挡位与 X/Y 位移校准系数，两者都掉电保存在模块 Flash。
 * @details setup 里 setCpi(50)（5000 counts/英寸）并把校准系数写成 1.0。改系数或挡位后，
 *          已累计的厘米数会按新参数重算。系数允许范围 0.5~2.0。
 * @n 实际距离与模块读数不一致时，系数取「实际距离 / 模块读数」。例如实际 100 cm、读数 109 cm，则取 100.0/109.0。
 * @n 接线（I2C，地址 0x11）：UNO SDA=A4/SCL=A5；Mega2560 SDA=20/SCL=21；ESP32 SDA=GPIO21/SCL=GPIO22。
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */
#include "DFRobot_9DofODO.h"

DFRobot_9DofODO_I2C odo;

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    delay(1000);
  }
  /* 挡位 50 = 5000 counts/英寸。setCpi(20) 才是 2000。 */
  odo.setCpi(50);
  if (odo.setCalibrationScale(1.0f, 1.0f) == 0) {
    Serial.println(F("CPI = 50 (5000 counts/inch), scale X = 1.00, Y = 1.00 saved"));
  } else {
    Serial.println(F("setCalibrationScale failed"));
  }
}

void loop()
{
  delay(1000);
}
