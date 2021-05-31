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
  byte _rate = rate;

  configReg = read16(ADS1115_CONFIG_REG);
  if (bitRead(_rate, 2)) configReg |= (1 << ADS1115_DR2);
  else configReg &= ~(1 << ADS1115_DR2);
  if (bitRead(_rate, 1)) configReg |= (1 << ADS1115_DR1);
  else configReg &= ~(1 << ADS1115_DR1);
  if (bitRead(_rate, 0)) configReg |= (1 << ADS1115_DR0);
  else configReg &= ~(1 << ADS1115_DR0);
  write16(ADS1115_CONFIG_REG, configReg);
}


byte ADS1115::getRate() { 
  return (read16(ADS1115_CONFIG_REG) & 0b11100000) >> 5;
}


void ADS1115::setMux(byte mux) {
  byte _mux = mux;

  configReg = read16(ADS1115_CONFIG_REG);
  if (bitRead(_mux, 2)) configReg |= (1 << ADS1115_MUX2);
  else configReg &= ~(1 << ADS1115_MUX2);
  if (bitRead(_mux, 1)) configReg |= (1 << ADS1115_MUX1);
  else configReg &= ~(1 << ADS1115_MUX1);
  if (bitRead(_mux, 0)) configReg |= (1 << ADS1115_MUX0);
  else configReg &= ~(1 << ADS1115_MUX0);
  write16(ADS1115_CONFIG_REG, configReg);
  delay(ADS1115_DELAY_AFTER_MUX_CHANGE);
}


byte ADS1115::getMux() { 
  return (read16(ADS1115_CONFIG_REG) & 0b111000000000000) >> 12;
}


void ADS1115::setGain(byte b_gain) {
  byte _b_gain = b_gain;
  
  // read out config Register from ADS1115
  configReg = read16(ADS1115_CONFIG_REG);

  if (bitRead(_b_gain, 2)) {
    // write 1 to config register at position ADS1115_PGA2
    configReg |= (1 << ADS1115_PGA2);
  } else {
    // write 0 to config register at position ADS1115_PGA2
    configReg &= ~(1 << ADS1115_PGA2);
  }
  
  if (bitRead(_b_gain, 1)) configReg |= (1 << ADS1115_PGA1);
  else configReg &= ~(1 << ADS1115_PGA1);
  if (bitRead(_b_gain, 0)) configReg |= (1 << ADS1115_PGA0);
  else configReg &= ~(1 << ADS1115_PGA0);

  // write back modified config register
  write16(ADS1115_CONFIG_REG, configReg);
  
  switch (_b_gain) {
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
  bool _mode = mode;
  
  configReg = read16(ADS1115_CONFIG_REG);
  if (_mode) configReg |= (1 << ADS1115_MODE);
  else configReg &= ~(1 << ADS1115_MODE);
  write16(ADS1115_CONFIG_REG, configReg);
}


byte ADS1115::getMode() { 
  return (read16(ADS1115_CONFIG_REG) & 0b100000000) >> 8;
}


void ADS1115::setPolarity(bool b_polarity) {
  // read out config Register from ADS1115
  configReg = read16(ADS1115_CONFIG_REG);

  bool _b_pol = b_polarity;

  if (_b_pol) {
    configReg |= (1<<ADS1115_CMP_POL);
  } else {
    configReg &= (1<<ADS1115_CMP_POL);
  }

  write16(ADS1115_CONFIG_REG, configReg);
}


byte ADS1115::getPolarity() {
  return (read16(ADS1115_CONFIG_REG) & (1<<ADS1115_CMP_POL)) >> ADS1115_CMP_POL;
}


void ADS1115::startSingleMeas() {
  if (getMode() != ADS1115_MODE_SINGLESHOT) setMode(ADS1115_MODE_SINGLESHOT);
  configReg = read16(ADS1115_CONFIG_REG) | (1 << ADS1115_OS);
  write16(ADS1115_CONFIG_REG, configReg);
}


bool ADS1115::conversionReady() {
  bool convReady;

  configReg = read16(ADS1115_CONFIG_REG);
  convReady = (configReg >> ADS1115_OS);
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
  configReg = read16(ADS1115_CONFIG_REG);
  Serial.println(configReg, BIN);
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
