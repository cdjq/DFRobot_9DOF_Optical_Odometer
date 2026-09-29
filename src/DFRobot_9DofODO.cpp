/**
 * @file DFRobot_9DofODO.cpp
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
    return 0;
  }
  return writeReg16(0x0105u, cpi) ? 1 : 0;
}

int DFRobot_9DofODO_I2C::readSqual(uint8_t *squal)
{
  uint16_t v;
  if (squal == NULL || !readReg16(0x000Eu, &v)) {
    return -1;
  }
  *squal = (uint8_t)(v / 4u);
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
  *squal = (uint8_t)(v / 4u);
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
