/*!
 * @file getOdoAndImuDataUART.ino
 * @brief UART 模式示例：以 MODBUS RTU 9600 8N1 读取总位移与融合欧拉角。
 * @details setup 里 begin() 并清零累计位移；loop 里周期打印 X/Y 位移（cm）与 roll/pitch/yaw（度）。
 *          从站地址 0x11（addr 拨码闭合）。需要带 Serial1 的板子，UNO 请改用 getOdoAndImuData。
 * @n 接线（UART 时 Gravity 线序：红=VCC，黑=GND，蓝=模块 TX，黄=模块 RX，交叉接到主机）：
 * @n Mega2560: 模块黄/RX ← Mega TX3(14)，模块蓝/TX → Mega RX3(15)，VCC=3.3V 或 5V，GND=GND
 * @n ESP32   : 模块黄/RX ← ESP32 TX，模块蓝/TX → ESP32 RX，VCC=3.3V，GND=GND（请按实际脚改 Serial1）
 * @n UNO     : 无 Serial1，请改用 getOdoAndImuData（I2C）
 * @n 使用前把 mode 拨码拨到闭合（UART）并重新上电。拨码只在上电时读取。
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */
#include "DFRobot_9DofODO.h"

#if !defined(HAVE_HWSERIAL1)
#error This example needs a board with Serial1. Use getOdoAndImuData on UNO.
#endif

/* 波特率固定 9600 8N1，从站地址 0x11；addr 拨码断开时改成 0x12。
 * Mega 用 Serial3（TX3=14，RX3=15）。 */
#if defined(ARDUINO_AVR_MEGA2560)
DFRobot_9DofODO_UART odo(&Serial3, 9600, 0x11);
#else
DFRobot_9DofODO_UART odo(&Serial1, 9600, 0x11);
#endif

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    Serial.println(F("Waiting SEN0776 UART..."));
    delay(1000);
  }
  odo.resetTotalDisplacement();
}

void loop()
{
  float tx;
  float ty;
  float roll;
  float pitch;
  float yaw;
  if (odo.getTotalDisplacement(&tx, &ty) == 0 && odo.getEuler(&roll, &pitch, &yaw) == 0) {
    /* yaw：0° = 丝印 +X 朝北。 */
    Serial.print(F("X = "));
    Serial.print(tx, 3);
    Serial.print(F(" cm, Y = "));
    Serial.print(ty, 3);
    Serial.println(F(" cm"));
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
