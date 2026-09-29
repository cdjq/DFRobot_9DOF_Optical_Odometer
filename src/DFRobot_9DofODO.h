/*!
 * @file DFRobot_9DofODO.h
 * @brief Host library for SEN0776 Gravity 9-DOF optical odometer
 * @details Register map matches CS32 firmware and dfrobot-general-rtu@1.0 (16-bit big-endian).
 * @n Default I2C slave address 0x11; UART RTU class is reserved for future firmware support.
 * @copyright Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @version V1.0.0
 * @date 2026-09-29
 */
#ifndef DFROBOT_9DOF_ODO_H
#define DFROBOT_9DOF_ODO_H ///< Header guard

#include <Arduino.h>
#include <Wire.h>

#define DFROBOT_9DOF_PID 0x0776u ///< Product ID (SEN0776)
#define DFROBOT_9DOF_DEFAULT_I2C_ADDR 0x11u ///< Default 7-bit I2C slave address

class DFRobot_9DofODO_I2C {
public:
  /**
   * @fn DFRobot_9DofODO_I2C
   * @brief Construct host driver on TwoWire
   * @n Does not call begin(); user must call begin() in setup().
   * @param wire I2C bus pointer, default &Wire
   * @param i2cAddr 7-bit module address, default 0x11
   */
  explicit DFRobot_9DofODO_I2C(TwoWire *wire = &Wire, uint8_t i2cAddr = DFROBOT_9DOF_DEFAULT_I2C_ADDR);

  /**
   * @fn begin
   * @brief Initialize I2C and verify module PID 0x0776
   * @n Call once in setup() before other APIs.
   * @return true if the module responds with expected PID
   */
  bool begin(void);

  /**
   * @fn getPAA_ID
   * @brief Read PAA5160 product id (input reg 0x0006)
   * @n Expected value 0x004F on SEN0776.
   * @param productId output 16-bit id
   * @return true on successful read
   */
  bool getPAA_ID(uint16_t &productId);

  /**
   * @fn getTotalDisplacement
   * @brief Read accumulated displacement in centimeters
   * @n Values are float32 in input regs 0x000A~0x000D.
   * @param totalX_cm X total or NULL
   * @param totalY_cm Y total or NULL
   * @return 0 on success, negative on error
   */
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);

  /**
   * @fn resetTotalDisplacement
   * @brief Clear accumulated displacement (holding cmd 0x0100)
   * @n Does not reset live delta counts.
   * @return 0 on success
   */
  int resetTotalDisplacement(void);

  /**
   * @fn setCalibrationScale
   * @brief Write cal_scale_x/y (0.5~2.0), stored in module Flash
   * @n Invalid values are rejected by firmware holding write.
   * @param scaleX X scale factor
   * @param scaleY Y scale factor
   * @return 0 on success
   */
  int setCalibrationScale(float scaleX, float scaleY);

  /**
   * @fn setCpi
   * @brief Write CPI setting 1~200 (holding 0x0105)
   * @n Display CPI is (cpi+1)*100 in input 0x0011.
   * @param cpi counts-per-inch setting
   * @return 0 on success
   */
  int setCpi(uint16_t cpi);

  /**
   * @fn readSqual
   * @brief Read surface quality (firmware already x4)
   * @n Low squal on glossy or lifted sensor may cause drift.
   * @param squal output 0~255
   * @return 0 on success
   */
  int readSqual(uint8_t *squal);

  /**
   * @fn setOrientation
   * @brief Configure optical axis swap/invert (holding 0x0106)
   * @n Matches PAA5160 mounting direction on the PCB.
   * @param swapXY swap X/Y deltas before integration
   * @param invertX invert X
   * @param invertY invert Y
   * @return true on success
   */
  bool setOrientation(bool swapXY, bool invertX, bool invertY);

  /**
   * @fn get6dofData
   * @brief Read BMI323 accel/gyro raw int16 (0x0012~0x0017)
   * @n Units follow firmware BMI323 configuration (8g / 2000dps range).
   * @param accel length-3 array or NULL
   * @param gyro length-3 array or NULL
   * @return 0 on success
   */
  int get6dofData(int16_t *accel, int16_t *gyro);

  /**
   * @fn getGeomagneticData
   * @brief Read BMM350 magnetometer int16 (0x0018~0x001A)
   * @n Axes are remapped in firmware to match module silkscreen.
   * @param mag length-3 array or NULL
   * @return 0 on success
   */
  int getGeomagneticData(int16_t *mag);

  /**
   * @fn getEuler
   * @brief Read Fusion roll/pitch/yaw in degrees
   * @n Registers 0x001B~0x0020 as float32 pairs.
   * @param roll roll degrees or NULL
   * @param pitch pitch degrees or NULL
   * @param yaw yaw degrees or NULL
   * @return 0 on success
   */
  int getEuler(float *roll, float *pitch, float *yaw);

  /**
   * @fn getCompassDegree
   * @brief Read compass heading 0~360 degrees (0x0021)
   * @n Requires valid magnetometer data and Fusion update on module.
   * @param compassDeg output angle or NULL
   * @return 0 on success
   */
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
