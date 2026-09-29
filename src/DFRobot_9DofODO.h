/**
 * @file DFRobot_9DofODO.h
 * @brief SEN0776 Gravity 九轴光学里程计主机库（I2C / UART 寄存器语义与 CS32 固件一致）
 */
#ifndef DFROBOT_9DOF_ODO_H
#define DFROBOT_9DOF_ODO_H

#include <Arduino.h>
#include <Wire.h>

#define DFROBOT_9DOF_PID 0x0776u
#define DFROBOT_9DOF_DEFAULT_I2C_ADDR 0x11u

class DFRobot_9DofODO_I2C {
public:
  explicit DFRobot_9DofODO_I2C(TwoWire *wire = &Wire, uint8_t i2cAddr = DFROBOT_9DOF_DEFAULT_I2C_ADDR);

  bool begin(void);
  bool getPAA_ID(uint16_t &productId);
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);
  int resetTotalDisplacement(void);
  int setCalibrationScale(float scaleX, float scaleY);
  int setCpi(uint16_t cpi);
  int readSqual(uint8_t *squal);
  bool setOrientation(bool swapXY, bool invertX, bool invertY);
  int get6dofData(int16_t *accel, int16_t *gyro);
  int getGeomagneticData(int16_t *mag);
  int getEuler(float *roll, float *pitch, float *yaw);
  int getCompassDegree(float *compassDeg);

  void setHostModeSwitch(void) {}
  void setI2cAddressSwitch(void) {}

private:
  TwoWire *_wire;
  uint8_t _addr;
  bool _ready;

  bool readReg16(uint16_t reg, uint16_t *value);
  bool writeReg16(uint16_t reg, uint16_t value);
  bool readFloat32(uint16_t regHi, float *out);
  bool writeFloat32(uint16_t regHi, float value);
};

class DFRobot_9DofODO_UART {
public:
#if defined(HAVE_HWSERIAL1)
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial = &Serial1, uint32_t baudRate = 9600);
#else
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial, uint32_t baudRate = 9600);
#endif

  bool begin(void);
  void setHostModeSwitch(void) {}
  void setI2cAddressSwitch(void) {}

private:
  HardwareSerial *_serial;
  uint32_t _baud;
};

#endif
