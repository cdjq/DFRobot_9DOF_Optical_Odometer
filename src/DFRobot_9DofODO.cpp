/*!
 * @file DFRobot_9DofODO.cpp
 * @brief Gravity 九轴光学里程计的 I2C / UART 接口实现。
 * @details 寄存器为 16 位大端；float32 占连续两个寄存器，高字在前。
 *          UART 的 CRC、组帧和超时由 DFRobot_RTU 完成，本文件只做产品寄存器读写。
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */

#include "DFRobot_9DofODO.h"

DFRobot_9DofODO_I2C::DFRobot_9DofODO_I2C(TwoWire *wire, uint8_t i2cAddr)
  : _wire(wire), _addr(i2cAddr), _ready(false)
{
}

bool DFRobot_9DofODO_I2C::readReg16(uint16_t reg, uint16_t *value)
{
  uint8_t addrBuf[2];
  if (value == NULL) {
    return false;
  }
  addrBuf[0] = (uint8_t)(reg >> 8);
  addrBuf[1] = (uint8_t)(reg & 0xFFu);
  _wire->beginTransmission(_addr);
  if (_wire->write(addrBuf, 2) != 2) {
    return false;
  }
  if (_wire->endTransmission(false) != 0) {
    return false;
  }
  if (_wire->requestFrom((int)_addr, 2) != 2) {
    return false;
  }
  *value = ((uint16_t)_wire->read() << 8) | _wire->read();
  return true;
}

bool DFRobot_9DofODO_I2C::writeReg16(uint16_t reg, uint16_t value)
{
  uint8_t buf[4];
  buf[0] = (uint8_t)(reg >> 8);
  buf[1] = (uint8_t)(reg & 0xFFu);
  buf[2] = (uint8_t)(value >> 8);
  buf[3] = (uint8_t)(value & 0xFFu);
  _wire->beginTransmission(_addr);
  if (_wire->write(buf, 4) != 4) {
    return false;
  }
  return (_wire->endTransmission() == 0);
}

bool DFRobot_9DofODO_I2C::readFloat32(uint16_t regHi, float *out)
{
  uint16_t hi;
  uint16_t lo;
  union {
    float f;
    uint32_t u;
  } v;
  if (out == NULL) {
    return false;
  }
  if (!readReg16(regHi, &hi) || !readReg16(regHi + 1u, &lo)) {
    return false;
  }
  v.u = ((uint32_t)hi << 16) | lo;
  *out = v.f;
  return true;
}

bool DFRobot_9DofODO_I2C::writeFloat32(uint16_t regHi, float value)
{
  union {
    float f;
    uint32_t u;
  } v;
  v.f = value;
  if (!writeReg16(regHi, (uint16_t)(v.u >> 16))) {
    return false;
  }
  return writeReg16(regHi + 1u, (uint16_t)(v.u & 0xFFFFu));
}

bool DFRobot_9DofODO_I2C::begin(void)
{
  uint16_t pid = 0;
  _wire->begin();
  _ready = readReg16(0x0000u, &pid) && (pid == DFROBOT_9DOF_PID);
  return _ready;
}

bool DFRobot_9DofODO_I2C::getPAA_ID(uint16_t &productId)
{
  return readReg16(0x0006u, &productId);
}

bool DFRobot_9DofODO_I2C::getPAA_RevisionID(uint16_t &revisionId)
{
  return readReg16(0x0007u, &revisionId);
}

int DFRobot_9DofODO_I2C::getTotalDisplacement(float *totalX_cm, float *totalY_cm)
{
  if (!_ready || totalX_cm == NULL || totalY_cm == NULL) {
    return -1;
  }
  if (!readFloat32(0x000Au, totalX_cm) || !readFloat32(0x000Cu, totalY_cm)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::resetTotalDisplacement(void)
{
  if (!_ready) {
    return -1;
  }
  return writeReg16(0x0100u, 1u) ? 0 : -1;
}

int DFRobot_9DofODO_I2C::setCalibrationScale(float scaleX, float scaleY)
{
  if (!_ready || scaleX < 0.5f || scaleX > 2.0f || scaleY < 0.5f || scaleY > 2.0f) {
    return -1;
  }
  if (!writeFloat32(0x0101u, scaleX) || !writeFloat32(0x0103u, scaleY)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::setCpi(uint16_t cpi)
{
  if (!_ready || cpi < 1u || cpi > 200u) {
    return -1;
  }
  return writeReg16(0x0105u, cpi) ? 0 : -1;
}

int DFRobot_9DofODO_I2C::readSqual(uint8_t *squal)
{
  uint16_t v;
  if (squal == NULL || !readReg16(0x000Eu, &v)) {
    return -1;
  }
  *squal = (uint8_t)v;
  return 0;
}

bool DFRobot_9DofODO_I2C::setOrientation(bool swapXY, bool invertX, bool invertY)
{
  uint16_t v = (swapXY ? 1u : 0u) | (invertX ? 2u : 0u) | (invertY ? 4u : 0u);
  return writeReg16(0x0106u, v);
}

int DFRobot_9DofODO_I2C::get6dofData(int16_t *accel, int16_t *gyro)
{
  uint16_t i;
  uint16_t v;
  if (accel == NULL || gyro == NULL) {
    return -1;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16(0x0012u + i, &v)) {
      return -1;
    }
    accel[i] = (int16_t)v;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16(0x0015u + i, &v)) {
      return -1;
    }
    gyro[i] = (int16_t)v;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::getGeomagneticData(int16_t *mag)
{
  uint16_t i;
  uint16_t v;
  if (mag == NULL) {
    return -1;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16(0x0018u + i, &v)) {
      return -1;
    }
    mag[i] = (int16_t)v;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::getEuler(float *roll, float *pitch, float *yaw)
{
  if (roll == NULL || pitch == NULL || yaw == NULL) {
    return -1;
  }
  if (!readFloat32(0x001Bu, roll) || !readFloat32(0x001Du, pitch) || !readFloat32(0x001Fu, yaw)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::getCompassDegree(float *compassDeg)
{
  return readFloat32(0x0021u, compassDeg) ? 0 : -1;
}

bool DFRobot_9DofODO_I2C::setBMI_Mode(eSensorMode_t mode)
{
  return _ready && writeReg16(0x0107u, (uint16_t)mode);
}

bool DFRobot_9DofODO_I2C::reset_BMI(void)
{
  return _ready && writeReg16(0x010Eu, 1u);
}

bool DFRobot_9DofODO_I2C::setAccelRange(eAccelRange_t range)
{
  return _ready && writeReg16(0x0108u, (uint16_t)range);
}

bool DFRobot_9DofODO_I2C::setGyroRange(eGyroRange_t range)
{
  return _ready && writeReg16(0x0109u, (uint16_t)range);
}

bool DFRobot_9DofODO_I2C::setBMM_Mode(uint8_t opMode)
{
  return _ready && writeReg16(0x010Au, opMode);
}

bool DFRobot_9DofODO_I2C::setBMM_PresetMode(uint8_t presetMode, uint8_t rate)
{
  if (!_ready || !writeReg16(0x010Cu, rate)) {
    return false;
  }
  return writeReg16(0x010Bu, presetMode);
}

bool DFRobot_9DofODO_I2C::setBMM_Rate(uint8_t rate)
{
  return _ready && writeReg16(0x010Cu, rate);
}

bool DFRobot_9DofODO_I2C::setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ)
{
  uint16_t axes = (enX ? 1u : 0u) | (enY ? 2u : 0u) | (enZ ? 4u : 0u);
  return _ready && writeReg16(0x010Du, axes);
}

bool DFRobot_9DofODO_I2C::getDataReadyState(void)
{
  uint16_t v = 0;
  return _ready && readReg16(0x003Du, &v) && v != 0u;
}

bool DFRobot_9DofODO_I2C::getBMM_Mode(uint16_t *mode)
{
  return mode != NULL && _ready && readReg16(0x003Eu, mode);
}

bool DFRobot_9DofODO_I2C::getBMM_Rate(uint8_t *rate)
{
  uint16_t v = 0;
  if (rate == NULL || !_ready || !readReg16(0x003Fu, &v)) {
    return false;
  }
  *rate = (uint8_t)v;
  return true;
}

int DFRobot_9DofODO_I2C::fusionAhrsGetQuaternion(float q[4])
{
  if (q == NULL || !readFloat32(0x0023u, &q[0]) || !readFloat32(0x0025u, &q[1]) ||
      !readFloat32(0x0027u, &q[2]) || !readFloat32(0x0029u, &q[3])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::fusionAhrsGetGravity(float gravity[3])
{
  if (gravity == NULL || !readFloat32(0x002Bu, &gravity[0]) || !readFloat32(0x002Du, &gravity[1]) ||
      !readFloat32(0x002Fu, &gravity[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::fusionAhrsGetLinearAcceleration(float linearAccel[3])
{
  if (linearAccel == NULL || !readFloat32(0x0031u, &linearAccel[0]) ||
      !readFloat32(0x0033u, &linearAccel[1]) || !readFloat32(0x0035u, &linearAccel[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::fusionAhrsGetEarthAcceleration(float earthAccel[3])
{
  if (earthAccel == NULL || !readFloat32(0x0037u, &earthAccel[0]) ||
      !readFloat32(0x0039u, &earthAccel[1]) || !readFloat32(0x003Bu, &earthAccel[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::fusionBiasGetOffset(float gyroBias[3])
{
  if (gyroBias == NULL || !readFloat32(0x0041u, &gyroBias[0]) || !readFloat32(0x0043u, &gyroBias[1]) ||
      !readFloat32(0x0045u, &gyroBias[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_I2C::setMagCalibration(const float hardIron[3], const float softIron[3][3])
{
  uint8_t r;
  uint8_t c;
  if (!_ready || hardIron == NULL || softIron == NULL) {
    return -1;
  }
  if (!writeFloat32(0x0110u, hardIron[0]) || !writeFloat32(0x0112u, hardIron[1]) ||
      !writeFloat32(0x0114u, hardIron[2])) {
    return -1;
  }
  for (r = 0; r < 3; r++) {
    for (c = 0; c < 3; c++) {
      uint16_t addr = (uint16_t)(0x0116u + ((uint16_t)r * 3u + c) * 2u);
      if (!writeFloat32(addr, softIron[r][c])) {
        return -1;
      }
    }
  }
  return writeReg16(0x0128u, 1u) ? 0 : -1;
}

DFRobot_9DofODO_UART::DFRobot_9DofODO_UART(HardwareSerial *serial,
                                           uint32_t baudRate,
                                           uint8_t unitId)
  : DFRobot_RTU((Stream *)serial),
    _serial(serial),
    _baud(baudRate),
    _addr(unitId),
    _ready(false)
{
}

bool DFRobot_9DofODO_UART::readReg16Input(uint16_t reg, uint16_t *value)
{
  uint16_t v;
  if (value == NULL) {
    return false;
  }
  v = readInputRegister(_addr, reg);
  *value = v;
  return true;
}

bool DFRobot_9DofODO_UART::writeReg16Holding(uint16_t reg, uint16_t value)
{
  return (writeHoldingRegister(_addr, reg, value) == 0);
}

bool DFRobot_9DofODO_UART::readFloat32Input(uint16_t regHi, float *out)
{
  uint16_t hi;
  uint16_t lo;
  union {
    float f;
    uint32_t u;
  } v;
  if (out == NULL) {
    return false;
  }
  if (!readReg16Input(regHi, &hi) || !readReg16Input(regHi + 1u, &lo)) {
    return false;
  }
  v.u = ((uint32_t)hi << 16) | lo;
  *out = v.f;
  return true;
}

bool DFRobot_9DofODO_UART::writeFloat32Holding(uint16_t regHi, float value)
{
  union {
    float f;
    uint32_t u;
  } v;
  v.f = value;
  if (!writeReg16Holding(regHi, (uint16_t)(v.u >> 16))) {
    return false;
  }
  return writeReg16Holding(regHi + 1u, (uint16_t)(v.u & 0xFFFFu));
}

bool DFRobot_9DofODO_UART::begin(void)
{
  uint16_t pid = 0;
  _serial->begin(_baud);
  setTimeoutTimeMs(100);
  _ready = readReg16Input(0x0000u, &pid) && (pid == DFROBOT_9DOF_PID);
  return _ready;
}

bool DFRobot_9DofODO_UART::getPAA_ID(uint16_t &productId)
{
  return readReg16Input(0x0006u, &productId);
}

bool DFRobot_9DofODO_UART::getPAA_RevisionID(uint16_t &revisionId)
{
  return _ready && readReg16Input(0x0007u, &revisionId);
}

int DFRobot_9DofODO_UART::getTotalDisplacement(float *totalX_cm, float *totalY_cm)
{
  if (!_ready || totalX_cm == NULL || totalY_cm == NULL) {
    return -1;
  }
  if (!readFloat32Input(0x000Au, totalX_cm) || !readFloat32Input(0x000Cu, totalY_cm)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::resetTotalDisplacement(void)
{
  if (!_ready) {
    return -1;
  }
  return writeReg16Holding(0x0100u, 1u) ? 0 : -1;
}

int DFRobot_9DofODO_UART::setCalibrationScale(float scaleX, float scaleY)
{
  if (!_ready || scaleX < 0.5f || scaleX > 2.0f || scaleY < 0.5f || scaleY > 2.0f) {
    return -1;
  }
  if (!writeFloat32Holding(0x0101u, scaleX) || !writeFloat32Holding(0x0103u, scaleY)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::setCpi(uint16_t cpi)
{
  if (!_ready || cpi < 1u || cpi > 200u) {
    return -1;
  }
  return writeReg16Holding(0x0105u, cpi) ? 0 : -1;
}

int DFRobot_9DofODO_UART::readSqual(uint8_t *squal)
{
  uint16_t v;
  if (squal == NULL || !readReg16Input(0x000Eu, &v)) {
    return -1;
  }
  *squal = (uint8_t)v;
  return 0;
}

bool DFRobot_9DofODO_UART::setOrientation(bool swapXY, bool invertX, bool invertY)
{
  uint16_t v = (swapXY ? 1u : 0u) | (invertX ? 2u : 0u) | (invertY ? 4u : 0u);
  return writeReg16Holding(0x0106u, v);
}

int DFRobot_9DofODO_UART::get6dofData(int16_t *accel, int16_t *gyro)
{
  uint16_t i;
  uint16_t v;
  if (!_ready || accel == NULL || gyro == NULL) {
    return -1;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16Input(0x0012u + i, &v)) {
      return -1;
    }
    accel[i] = (int16_t)v;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16Input(0x0015u + i, &v)) {
      return -1;
    }
    gyro[i] = (int16_t)v;
  }
  return 0;
}

int DFRobot_9DofODO_UART::getGeomagneticData(int16_t *mag)
{
  uint16_t i;
  uint16_t v;
  if (!_ready || mag == NULL) {
    return -1;
  }
  for (i = 0; i < 3u; i++) {
    if (!readReg16Input(0x0018u + i, &v)) {
      return -1;
    }
    mag[i] = (int16_t)v;
  }
  return 0;
}

int DFRobot_9DofODO_UART::getEuler(float *roll, float *pitch, float *yaw)
{
  if (!_ready || roll == NULL || pitch == NULL || yaw == NULL) {
    return -1;
  }
  if (!readFloat32Input(0x001Bu, roll) || !readFloat32Input(0x001Du, pitch) ||
      !readFloat32Input(0x001Fu, yaw)) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::getCompassDegree(float *compassDeg)
{
  if (!_ready || compassDeg == NULL) {
    return -1;
  }
  return readFloat32Input(0x0021u, compassDeg) ? 0 : -1;
}

bool DFRobot_9DofODO_UART::setBMI_Mode(eSensorMode_t mode)
{
  return _ready && writeReg16Holding(0x0107u, (uint16_t)mode);
}

bool DFRobot_9DofODO_UART::reset_BMI(void)
{
  return _ready && writeReg16Holding(0x010Eu, 1u);
}

bool DFRobot_9DofODO_UART::setAccelRange(eAccelRange_t range)
{
  return _ready && writeReg16Holding(0x0108u, (uint16_t)range);
}

bool DFRobot_9DofODO_UART::setGyroRange(eGyroRange_t range)
{
  return _ready && writeReg16Holding(0x0109u, (uint16_t)range);
}

bool DFRobot_9DofODO_UART::setBMM_Mode(uint8_t opMode)
{
  return _ready && writeReg16Holding(0x010Au, opMode);
}

bool DFRobot_9DofODO_UART::setBMM_PresetMode(uint8_t presetMode, uint8_t rate)
{
  if (!_ready || !writeReg16Holding(0x010Cu, rate)) {
    return false;
  }
  return writeReg16Holding(0x010Bu, presetMode);
}

bool DFRobot_9DofODO_UART::setBMM_Rate(uint8_t rate)
{
  return _ready && writeReg16Holding(0x010Cu, rate);
}

bool DFRobot_9DofODO_UART::setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ)
{
  uint16_t axes = (enX ? 1u : 0u) | (enY ? 2u : 0u) | (enZ ? 4u : 0u);
  return _ready && writeReg16Holding(0x010Du, axes);
}

bool DFRobot_9DofODO_UART::getDataReadyState(void)
{
  uint16_t v = 0;
  return _ready && readReg16Input(0x003Du, &v) && v != 0u;
}

bool DFRobot_9DofODO_UART::getBMM_Mode(uint16_t *mode)
{
  return mode != NULL && _ready && readReg16Input(0x003Eu, mode);
}

bool DFRobot_9DofODO_UART::getBMM_Rate(uint8_t *rate)
{
  uint16_t v = 0;
  if (rate == NULL || !_ready || !readReg16Input(0x003Fu, &v)) {
    return false;
  }
  *rate = (uint8_t)v;
  return true;
}

int DFRobot_9DofODO_UART::fusionAhrsGetQuaternion(float q[4])
{
  if (!_ready || q == NULL || !readFloat32Input(0x0023u, &q[0]) || !readFloat32Input(0x0025u, &q[1]) ||
      !readFloat32Input(0x0027u, &q[2]) || !readFloat32Input(0x0029u, &q[3])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::fusionAhrsGetGravity(float gravity[3])
{
  if (!_ready || gravity == NULL || !readFloat32Input(0x002Bu, &gravity[0]) ||
      !readFloat32Input(0x002Du, &gravity[1]) || !readFloat32Input(0x002Fu, &gravity[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::fusionAhrsGetLinearAcceleration(float linearAccel[3])
{
  if (!_ready || linearAccel == NULL || !readFloat32Input(0x0031u, &linearAccel[0]) ||
      !readFloat32Input(0x0033u, &linearAccel[1]) || !readFloat32Input(0x0035u, &linearAccel[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::fusionAhrsGetEarthAcceleration(float earthAccel[3])
{
  if (!_ready || earthAccel == NULL || !readFloat32Input(0x0037u, &earthAccel[0]) ||
      !readFloat32Input(0x0039u, &earthAccel[1]) || !readFloat32Input(0x003Bu, &earthAccel[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::fusionBiasGetOffset(float gyroBias[3])
{
  if (!_ready || gyroBias == NULL || !readFloat32Input(0x0041u, &gyroBias[0]) ||
      !readFloat32Input(0x0043u, &gyroBias[1]) || !readFloat32Input(0x0045u, &gyroBias[2])) {
    return -1;
  }
  return 0;
}

int DFRobot_9DofODO_UART::setMagCalibration(const float hardIron[3], const float softIron[3][3])
{
  uint8_t r;
  uint8_t c;
  if (!_ready || hardIron == NULL || softIron == NULL) {
    return -1;
  }
  if (!writeFloat32Holding(0x0110u, hardIron[0]) || !writeFloat32Holding(0x0112u, hardIron[1]) ||
      !writeFloat32Holding(0x0114u, hardIron[2])) {
    return -1;
  }
  for (r = 0; r < 3; r++) {
    for (c = 0; c < 3; c++) {
      uint16_t addr = (uint16_t)(0x0116u + ((uint16_t)r * 3u + c) * 2u);
      if (!writeFloat32Holding(addr, softIron[r][c])) {
        return -1;
      }
    }
  }
  return writeReg16Holding(0x0128u, 1u) ? 0 : -1;
}
