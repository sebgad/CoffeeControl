// Code based on Retian (version 1.0)
// https://arduino-projekte.webnode.at/meine-libraries/adc-ads1115/

#include "Arduino.h"
#include "ADS1115.h"

ADS1115::ADS1115(TwoWire * _obj_i2c) {
  _objI2C = _obj_i2c;
}

bool ADS1115::begin() {
  bool b_success = false;
  _iI2cAddress = ADS1115_I2CADD_DEFAULT;
  _objI2C->begin(_iI2cAddress);

  _objI2C->beginTransmission(_iI2cAddress);
  if (_objI2C->endTransmission()== 0) {
    b_success = true;
  }
  return b_success;
}

bool ADS1115::begin(uint8_t i_i2c_address) {
  bool b_success = false;
  _iI2cAddress  = i_i2c_address;
  _objI2C->begin(_iI2cAddress);

  _objI2C->beginTransmission(_iI2cAddress);
  if (_objI2C->endTransmission()== 0) {
    b_success = true;
  }
  return b_success;
}

bool ADS1115::begin(int i_sda_pin, int i_scl_pin) {
  bool b_success = false;
  _iI2cAddress = ADS1115_I2CADD_DEFAULT;
  _objI2C->begin(i_sda_pin, i_scl_pin);

  _objI2C->beginTransmission(_iI2cAddress);
  if (_objI2C->endTransmission()== 0) {
    b_success = true;
  }
  return b_success;
}

bool ADS1115::begin(int i_sda, int i_scl, uint8_t i_i2c_address) {
  bool b_success = false;
  _iI2cAddress  = i_i2c_address;
  _objI2C->begin(i_sda, i_scl, _iI2cAddress);

  _objI2C->beginTransmission(_iI2cAddress);
  if (_objI2C->endTransmission()== 0) {
    b_success = true;
  }
  return b_success;
}

void ADS1115::init() {
  byte param;

  param = getGain();
  if (param != ADS1115_PGA_2P048) setGain(ADS1115_PGA_2P048);
  if (bitNumbering != ADS1115_LSB_2P048) bitNumbering = ADS1115_LSB_2P048;
  param = getMux();
  if (param != ADS1115_MUX_AIN0_AIN1) setMux(ADS1115_MUX_AIN0_AIN1);
  param = getRate();
  if (param != ADS1115_RATE_128) setRate(ADS1115_RATE_128);
  param = getMode();
  if (param != ADS1115_MODE_SINGLESHOT) setMode(ADS1115_MODE_SINGLESHOT);
}


void ADS1115::setRate(byte rate) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = bitRead(rate, 2);
  bool b1 = bitRead(rate, 1);
  bool b0 = bitRead(rate, 0);

  bitWrite(iConfigReg, ADS1115_DR2, b2);
  bitWrite(iConfigReg, ADS1115_DR1, b1);
  bitWrite(iConfigReg, ADS1115_DR0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getRate() { 
  return (read16(ADS1115_CONFIG_REG) & 0b11100000) >> 5;
}


void ADS1115::setMux(byte mux) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = bitRead(mux, 2);
  bool b1 = bitRead(mux, 1);
  bool b0 = bitRead(mux, 0);

  bitWrite(iConfigReg, ADS1115_MUX2, b2);
  bitWrite(iConfigReg, ADS1115_MUX1, b1);
  bitWrite(iConfigReg, ADS1115_MUX0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
  delay(ADS1115_DELAY_AFTER_MUX_CHANGE);
}


byte ADS1115::getMux() { 
  return (read16(ADS1115_CONFIG_REG) & 0b111000000000000) >> 12;
}


void ADS1115::setGain(byte b_gain) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = bitRead(b_gain, 2);
  bool b1 = bitRead(b_gain, 1);
  bool b0 = bitRead(b_gain, 0);

  bitWrite(iConfigReg, ADS1115_PGA2, b2);
  bitWrite(iConfigReg, ADS1115_PGA1, b1);
  bitWrite(iConfigReg, ADS1115_PGA0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);

  switch (b_gain) {
    case ADS1115_PGA_6P144:
      bitNumbering = ADS1115_LSB_6P144;
      break;
    case ADS1115_PGA_4P096:
      bitNumbering = ADS1115_LSB_4P096;
      break;
    case ADS1115_PGA_2P048:
      bitNumbering = ADS1115_LSB_2P048;
      break;
    case ADS1115_PGA_1P024:
      bitNumbering = ADS1115_LSB_1P024;
      break;
    case ADS1115_PGA_0P512:
      bitNumbering = ADS1115_LSB_0P512;
      break;
    case ADS1115_PGA_0P256:
      bitNumbering = ADS1115_LSB_0P256;
      break;
  }
}


byte ADS1115::getGain() { 
  return (read16(ADS1115_CONFIG_REG) & 0b111000000000) >> 9;
}


void ADS1115::setMode(bool mode) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bitWrite(iConfigReg, ADS1115_CMP_MDE, mode);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getMode() { 
  return (read16(ADS1115_CONFIG_REG) & 0b100000000) >> 8;
}


void ADS1115::setPolarity(bool b_polarity) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bitWrite(iConfigReg, ADS1115_CMP_POL, b_polarity);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}

byte ADS1115::getPolarity() {
  return (read16(ADS1115_CONFIG_REG) & (1<<ADS1115_CMP_POL)) >> ADS1115_CMP_POL;
}

void ADS1115::setLatchingMode(bool b_mode) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bitWrite(iConfigReg, ADS1115_CMP_LAT, b_mode);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}

byte ADS1115::getLatchingMode() {
  return (read16(ADS1115_CONFIG_REG) & (1<<ADS1115_CMP_LAT)) >> ADS1115_CMP_LAT;
}


void ADS1115::setQueueMode(byte b_mode) {
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b1 = bitRead(b_mode, 1);
  bool b0 = bitRead(b_mode, 0);

  bitWrite(iConfigReg, ADS1115_CMP_QUE1, b1);
  bitWrite(iConfigReg, ADS1115_CMP_QUE0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
}


void ADS1115::setPinRdyMode(bool b_activate){
  iLowThreshReg = read16(ADS1115_LOW_THRESH_REG);
  iHighThreshReg = read16(ADS1115_HIGH_THRESH_REG);

  if (b_activate){
    // MSB to 1 if PinReadyMode is activated
    bitWrite(iLowThreshReg, 15, 0);
    bitWrite(iHighThreshReg, 15, 1);
  } else {
    bitWrite(iLowThreshReg, 15, 1);
    bitWrite(iHighThreshReg, 15, 0);
  }
  write16(ADS1115_LOW_THRESH_REG, iLowThreshReg);
  write16(ADS1115_HIGH_THRESH_REG, iHighThreshReg);
}

bool ADS1115::getPinRdyMode() {
  iLowThreshReg = read16(ADS1115_LOW_THRESH_REG);
  iHighThreshReg = read16(ADS1115_HIGH_THRESH_REG);

  if (~bitRead(iLowThreshReg, 15) && bitRead(iHighThreshReg, 15)) {
    return true;
  } else {
    return false;
  }
}

byte ADS1115::getQueueMode() {
  return (read16(ADS1115_CONFIG_REG) & (0b11));
}


void ADS1115::startSingleMeas() {
  if (getMode() != ADS1115_MODE_SINGLESHOT) setMode(ADS1115_MODE_SINGLESHOT);
  iConfigReg = read16(ADS1115_CONFIG_REG) | (1 << ADS1115_OS);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


bool ADS1115::conversionReady() {
  bool convReady;

  iConfigReg = read16(ADS1115_CONFIG_REG);
  convReady = (iConfigReg >> ADS1115_OS);
  if (convReady) return true;
  else return false;
}


int ADS1115::readConversion() {
  return (int)read16(ADS1115_CONVERSION_REG);
}


float ADS1115::readVoltage() {
  return (int)read16(ADS1115_CONVERSION_REG) * bitNumbering;
}


void ADS1115::printConfigReg() {
  Serial.print("Conf.Reg.: ");
  iConfigReg = read16(ADS1115_CONFIG_REG);
  Serial.println(iConfigReg, BIN);
}


void ADS1115::write16(byte reg, unsigned int val) {
  byte _reg = reg;
  unsigned int _val = val;

  _objI2C->beginTransmission(_iI2cAddress);
  _objI2C->write(_reg);
  _objI2C->write((byte)highByte(_val));
  _objI2C->write((byte)lowByte(_val));
  _objI2C->endTransmission();
}


unsigned int ADS1115::read16(byte reg) {
  byte _reg = reg;
  byte hByte, lByte;

  _objI2C->beginTransmission(_iI2cAddress);
  _objI2C->write(_reg);
  _objI2C->endTransmission();
  delay(1);
  _objI2C->requestFrom((int)_iI2cAddress, 2);
  if (_objI2C->available())
  {
    hByte = _objI2C->read();
    lByte = _objI2C->read();
  }
  return ((int)(hByte << 8) + lByte);
}
