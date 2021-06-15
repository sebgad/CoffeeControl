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

void ADS1115::setDefault() {
  /**
   * Bring ADS1115 back to default settings
  */
  setMux(ADS1115_MUX_AIN0_AIN1);
  setPGA(ADS1115_LSB_2P048);
  setOpMode(ADS1115_MODE_SINGLESHOT);
  setRate(ADS1115_RATE_128);
  setCompMode(ADS1115_CMP_MODE_TRADITIONAL);
  setCompPolarity(ADS1115_CMP_POL_ACTIVE_LOW);
  setCompLatchingMode(ADS1115_CMP_LAT_NOT_ACTIVE);
  setCompQueueMode(ADS1115_CMP_DISABLE);
}


void ADS1115::startSingleShotMeas(bool b_status) {
  /**
   * Single-shot conversion start
   * This bit determines the operational status of the device. OS can only be written when in power-down state and has 
   * no effect when a conversion is ongoing.
   * 
   * @param b_status: ADS1115_MODE_SINGLESHOT : Start a single conversion (when in power-down state)
   * 
  */
  if (b_status) {
    iConfigReg = read16(ADS1115_CONFIG_REG);
    writeBitIn2Byte(iConfigReg, ADS1115_OS, b_status);
    write16(ADS1115_CONFIG_REG, iConfigReg);
  }
  
};

bool ADS1115::getOpStatus(void){
  /** 
   * Get Operational status
   * @return: 0 : Device is currently performing a conversion, 1 : Device is not currently performing a conversion
  */
  return (read16(ADS1115_CONFIG_REG) & 1<<ADS1115_OS) >> ADS1115_OS;
}


void ADS1115::setMux(byte b_mux) {
  /**
   * Set input multiplexer configuration
   * @param b_mux:
   *    ADS1115_MUX_AIN0_AIN1 AINp = AIN0 and AINn = AIN1
   *    ADS1115_MUX_AIN0_AIN3 AINp = AIN0 and AINn = AIN3
   *    ADS1115_MUX_AIN1_AIN3 AINp = AIN1 and AINn = AIN3
   *    ADS1115_MUX_AIN2_AIN3 AINp = AIN2 and AINn = AIN3
   *    ADS1115_MUX_AIN0_GND AINp = AIN0 and AINn = GND
   *    ADS1115_MUX_AIN1_GND AINp = AIN1 and AINn = GND
   *    ADS1115_MUX_AIN2_GND AINp = AIN2 and AINn = GND
   *    ADS1115_MUX_AIN3_GND AINp = AIN3 and AINn = GND
  */ 
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = readBitIn2Byte(b_mux, 2);
  bool b1 = readBitIn2Byte(b_mux, 1);
  bool b0 = readBitIn2Byte(b_mux, 0);

  writeBitIn2Byte(iConfigReg, ADS1115_MUX2, b2);
  writeBitIn2Byte(iConfigReg, ADS1115_MUX1, b1);
  writeBitIn2Byte(iConfigReg, ADS1115_MUX0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
  delay(ADS1115_DELAY_AFTER_MUX_CHANGE);
}


byte ADS1115::getMux() { 
  /**
   * Get Input multiplexer configuration
   * @return:
   *    0b000 AINp = AIN0 and AINn = AIN1
   *    0b001 AINp = AIN0 and AINn = AIN3
   *    0b010 AINp = AIN1 and AINn = AIN3
   *    0b011 AINp = AIN2 and AINn = AIN3
   *    0b100 AINp = AIN0 and AINn = GND
   *    0b101 AINp = AIN1 and AINn = GND
   *    0b110 AINp = AIN2 and AINn = GND
   *    0b111 AINp = AIN3 and AINn = GND
  */ 
  return (read16(ADS1115_CONFIG_REG) & 0b111<<ADS1115_MUX0) >> ADS1115_MUX0;
}


void ADS1115::setPGA(byte b_gain) {
  /**
   * Set the FSR of the programmable gain amplifier
   * @param b_gain:
   *    ADS1115_PGA_6P144 : FSR = +-6.144V
   *    ADS1115_PGA_4P096 : FSR = +-4.096V
   *    ADS1115_PGA_2P048 : FSR = +-2.048V
   *    ADS1115_PGA_1P024 : FSR = +-1.024V
   *    ADS1115_PGA_0P512 : FSR = +-0.512V
   *    ADS1115_PGA_0P256 : FSR = +-0.256V
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = readBitIn2Byte(b_gain, 2);
  bool b1 = readBitIn2Byte(b_gain, 1);
  bool b0 = readBitIn2Byte(b_gain, 0);

  writeBitIn2Byte(iConfigReg, b2, ADS1115_PGA2);
  writeBitIn2Byte(iConfigReg, b1, ADS1115_PGA1);
  writeBitIn2Byte(iConfigReg, b0, ADS1115_PGA0);

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



byte ADS1115::getPGA() { 
  /**
   * Get the FSR of the programmable gain amplifier
   * @return:
   *    0b000 : FSR = +-6.144V
   *    0b001 : FSR = +-4.096V
   *    0b010 : FSR = +-2.048V
   *    0b011 : FSR = +-1.024V
   *    0b100 : FSR = +-0.512V
   *    0b101 : FSR = +-0.256V
  */
  return (read16(ADS1115_CONFIG_REG) & 0b111<<ADS1115_PGA0) >> ADS1115_PGA0;
}


void ADS1115::setOpMode(bool b_mode) {
  /**
   * Set the operating mode
   * @param b_mode
   *    ADS1115_MODE_CONTINUOUS : Continuous-conversion mode
   *    ADS1115_MODE_SINGLESHOT : Single-shot mode or power-down state (default)
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  writeBitIn2Byte(iConfigReg, ADS1115_MODE, b_mode);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getOpMode() {
  /**
   * get operating mode
   * @return
   *    0 : Continuous-conversion mode
   *    1 : Single-shot mode or power-down state (default)
  */
  return (read16(ADS1115_CONFIG_REG) & 1<<ADS1115_MODE) >> ADS1115_MODE;
}

void ADS1115::setRate(byte b_rate) {
  /**
   * Set the data rate
   * @param b_rate
   *    ADS1115_RATE_8    : 8 samples per second  (SPS)
   *    ADS1115_RATE_16   : 16 SPS
   *    ADS1115_RATE_32   : 32 SPS
   *    ADS1115_RATE_64   : 64 SPS
   *    ADS1115_RATE_128  : 128 SPS (default)
   *    ADS1115_RATE_250  : 250 SPS
   *    ADS1115_RATE_475  : 475 SPS
   *    ADS1115_RATE_860  : 860 SPS
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b2 = readBitIn2Byte(b_rate, 2);
  bool b1 = readBitIn2Byte(b_rate, 1);
  bool b0 = readBitIn2Byte(b_rate, 0);

  writeBitIn2Byte(iConfigReg, ADS1115_DR2, b2);
  writeBitIn2Byte(iConfigReg, ADS1115_DR1, b1);
  writeBitIn2Byte(iConfigReg, ADS1115_DR0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getRate() {
  /**
   * Get the data rate
   * @return
   *    0b000 : 8 samples per second  (SPS)
   *    0b001 : 16 SPS
   *    0b010 : 32 SPS
   *    0b011 : 64 SPS
   *    0b100 : 128 SPS (default)
   *    0b101 : 250 SPS
   *    0b110 : 475 SPS
   *    0b111 : 860 SPS
  */
  return (read16(ADS1115_CONFIG_REG) & 0b111<<ADS1115_DR0) >> ADS1115_DR0;
}


void ADS1115::setCompMode(bool b_mode) {
  /**
   * Set the comparator operating mode
   * @b_mode: 
   *    ADS1115_CMP_MODE_TRADITIONAL  : Traditional comparator (default)
   *    ADS1115_CMP_MODE_WINDOW       : Window comparator
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  writeBitIn2Byte(iConfigReg, ADS1115_CMP_MDE, b_mode);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getCompMode() { 
   /**
   * Get the comparator operating mode
   * @return: 
   *    0  : Traditional comparator (default)
   *    1  : Window comparator
  */
  return (read16(ADS1115_CONFIG_REG) & 1<<ADS1115_CMP_MDE) >> ADS1115_CMP_MDE;
}


void ADS1115::setCompPolarity(bool b_polarity) {
  /**
   * Set polarity of the ALERT/RDY pin
   * @param b_polarity
   *    ADS1115_CMP_POL_ACTIVE_LOW  : Active low (default)
   *    ADS1115_CMP_POL_ACTIVE_HIGH : Active high
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  writeBitIn2Byte(iConfigReg, ADS1115_CMP_POL, b_polarity);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getCompPolarity() {
  /**
   * Get polarity of the ALERT/RDY pin
   * @return
   *    0  : Active low (default)
   *    1  : Active high
  */
  return (read16(ADS1115_CONFIG_REG) & (1<<ADS1115_CMP_POL)) >> ADS1115_CMP_POL;
}

void ADS1115::setCompLatchingMode(bool b_mode) {
  /**
   * Set whether the ALERT/RDY pin latches after being asserted or clears after conversions are within the margin of 
   * the upper and lower threshold values.
   * @param b_mode
   *    ADS1115_CMP_LAT_NOT_ACTIVE : Nonlatching comparator . The ALERT/RDY pin does not latch when asserted (default).
   *    ADS1115_CMP_LAT_ACTIVE     : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data 
   *                                 are read by the master or an appropriate SMBus alert response is sent by the master. 
   *                                 The device responds with its address, and it is the lowest address currently asserting the 
   *                                 ALERT/RDY bus line.
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  writeBitIn2Byte(iConfigReg, ADS1115_CMP_LAT, b_mode);
  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getCompLatchingMode() {
   /**
   * Get whether the ALERT/RDY pin latches after being asserted or clears after conversions are within the margin of
   * the upper and lower threshold values.
   * @return
   *    0 : Nonlatching comparator. The ALERT/RDY pin does not latch when asserted (default).
   *    1 : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data 
   *        are read by the master or an appropriate SMBus alert response is sent by the master. 
   *        The device responds with its address, and it is the lowest address currently asserting the 
   *        ALERT/RDY bus line.
  */
  return (read16(ADS1115_CONFIG_REG) & (1<<ADS1115_CMP_LAT)) >> ADS1115_CMP_LAT;
}


void ADS1115::setCompQueueMode(byte b_mode) {
  /**
   * Perform two functions. When set to ADS1115_CMP_DISABLE, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state. 
   * When set to any other value, the ALERT/RDY pin and the comparator function are enabled, and the set value determines the 
   * number of successive conversions exceeding the upper or lower threshold required before asserting the ALERT/RDY pin.
   * @param b_mode
   *    ADS1115_CMP_QUE_ASSERT_1_CONV : Assert after one conversion
   *    ADS1115_CMP_QUE_ASSERT_2_CONV : Assert after two conversions
   *    ADS1115_CMP_QUE_ASSERT_4_CONV : Assert after four conversions
   *    ADS1115_CMP_DISABLE           : Disable comparator and set ALERT/RDY pin to high-impedance (default)
  */
  iConfigReg = read16(ADS1115_CONFIG_REG);
  bool b1 = readBitIn2Byte(b_mode, 1);
  bool b0 = readBitIn2Byte(b_mode, 0);

  writeBitIn2Byte(iConfigReg, ADS1115_CMP_QUE1, b1);
  writeBitIn2Byte(iConfigReg, ADS1115_CMP_QUE0, b0);

  write16(ADS1115_CONFIG_REG, iConfigReg);
}


byte ADS1115::getCompQueueMode() {
  /**
   * Get Queue mode of comparator. When set to 11, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state. 
   * When set to any other value, the ALERT/RDY pin and the comparator function are enabled, and the set value determines the 
   * number of successive conversions exceeding the upper or lower threshold required before asserting the ALERT/RDY pin.
   * @param b_mode
   *    0b00 : Assert after one conversion
   *    0b01 : Assert after two conversions
   *    0b10 : Assert after four conversions
   *    0b11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)
  */
  return (read16(ADS1115_CONFIG_REG) & (0b11<<ADS1115_CMP_QUE0)>>ADS1115_CMP_QUE0);
}


void ADS1115::setCompLowThreshBit(bool b_value, int i_bit_num){
  /**
   * Set the lower threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore, 
   * the values in these registers must be updated whenever the PGA settings are changed. 
   * @param b_value: value for the low threshold register
   * @param i_bit_num: bit number to change, LSF bit is 0
  */
  iLowThreshReg = read16(ADS1115_LOW_THRESH_REG);
  writeBitIn2Byte(iLowThreshReg, i_bit_num, b_value);
  write16(ADS1115_LOW_THRESH_REG, iLowThreshReg);
}

byte ADS1115::getCompLowThreshBit(int i_bit_num){
  /**
   * Get the lower threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore, 
   * the values in these registers must be updated whenever the PGA settings are changed. 
   * @param i_bit_num: bit number to read, LSF bit is 0
   * @return bit value on i_bit_num
  */
  iLowThreshReg = read16(ADS1115_LOW_THRESH_REG);
  readBitIn2Byte(iLowThreshReg, i_bit_num);
  return iLowThreshReg;
}


void ADS1115::setCompHighThreshBit(bool b_value, int i_bit_num){
  /**
   * Set the higher threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore, 
   * the values in these registers must be updated whenever the PGA settings are changed. 
   * @param b_value: value for the high threshold register
   * @param i_bit_num: bit number to change, LSF bit is 0
  */
  iHighThreshReg = read16(ADS1115_HIGH_THRESH_REG);
  writeBitIn2Byte(iHighThreshReg, i_bit_num, b_value);
  write16(ADS1115_HIGH_THRESH_REG, iHighThreshReg);
}

byte ADS1115::getCompHighThreshBit(int i_bit_num){
  /**
   * Get the high threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore, 
   * the values in these registers must be updated whenever the PGA settings are changed. 
   * @param i_bit_num: bit number to read, LSF bit is 0
   * @return bit value on i_bit_num
  */
  iHighThreshReg = read16(ADS1115_HIGH_THRESH_REG);
  readBitIn2Byte(iHighThreshReg, i_bit_num);
  return iHighThreshReg;
}


void ADS1115::setPinRdyMode(bool b_activate, byte b_comp_queue_mode){
  /**
   * Set pin ready mode. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a 
   * continuous-conversion ready pulse when in continuous-conversion mode. Latching comparator is activated in this mode.
   * @param b_activate
   *    ADS1115_CONV_READY_ACTIVE     : pin ready mode is activated
   *    ADS1115_CONV_READY_NOT_ACTIVE : pin ready mode is deactivated
  */
  setCompLowThreshBit(0, 15);
  setCompHighThreshBit(1, 15);

  setCompQueueMode(b_comp_queue_mode);
}


bool ADS1115::getPinRdyMode() {
  /**
   * Set pin ready mode. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a 
   * continuous-conversion ready pulse when in continuous-conversion mode. Latching comparator is activated in this mode.
   * @return
   *    true     : pin ready mode is activated
   *    false    : pin ready mode is deactivated
  */
  byte b_cmp_queue_mode = getCompQueueMode();

  iLowThreshReg = read16(ADS1115_LOW_THRESH_REG);
  iHighThreshReg = read16(ADS1115_HIGH_THRESH_REG);

  if (~readBitIn2Byte(iLowThreshReg, 15) && readBitIn2Byte(iHighThreshReg, 15) && ~(b_cmp_queue_mode==ADS1115_CMP_DISABLE)) {
    return true;
  } else {
    return false;
  }
}

void ADS1115::writeBitIn2Byte(uint16_t &i_register, int i_pos, bool b_value){
  /** Write a bit in a uint16_t number at a given position
   * @param i_register: uint16_t register as reference
   * @param i_pos: position of the bit
   * @param b_value: value of the bit, 0 or 1
  */
 if (b_value) {
   i_register |= (1<<i_pos);
 } else {
   i_register &= ~(1<<i_pos);
 }
}

bool ADS1115::readBitIn2Byte(uint16_t i_register, int i_pos){
  /** 
   * read bit in uint16_t number at a given position
   * @param: i_register: number where to read out the bit
   * @param i_pos: bit position which should be read out
   * @return: bit value of given position
  */
  return ((i_register & (1<<i_pos)) >> i_pos);
}

int ADS1115::readConversion() {
  /**
   * read conversion data from the conversion register as int value. Size can be maximum 16bit due to register length of the ADS1115
   * @return int in range (0...2^16-1)
  */ 
  return read16(ADS1115_CONVERSION_REG);
}


float ADS1115::readVoltage() {
  /**
   * read voltage level, based on the adc value of the ADS1115
   * @return measured voltage
  */ 
  int16_t meas;
  float meas_f;
  meas = read16(ADS1115_CONVERSION_REG);
  meas_f = meas * bitNumbering;  
  return meas_f;
}


void ADS1115::setPhysicalConversion(float fgradient, float f_offset) {
  /**
   * set factors for conversion from voltage to physical value
   * @param f_gradient: gradient of the conversion function
   * @param f_offset: (y-)Offset of the conversion function
  */ 
  _fGradient = fgradient;
  _fOffset = f_offset;
  _iConvMethod = ADS1115_CONV_METHOD_SINGLE;
}

void ADS1115::_initConvTable(size_t i_size_conv) {
  /**
   * Initialize pointer for conversion table
   * @param i_size_conv: row of the conversion table
  */
  
  // Make (row) size of conversion table in class available
  _iSizeConvTable=i_size_conv;
  // assign memory to the pointer, pointer in pointer element
  _ptrConvTable = new float*[i_size_conv];
  
  // assign second pointer in pointer to get a 2dim field
  for(int i_row=0;i_row<i_size_conv;i_row++) {
    _ptrConvTable[i_row]=new float[2];
  }
}

void ADS1115::setPhysicalConversion(float arr_conv_table[][2], size_t i_size_conv) {
  /**
   * set factors for conversion from voltage to physical value
   * @param arr_conv_table: table for conversion, 1st dim is x value, 2nd dim is y value
   * @param i_size_conv: (row) size of conversion table
  */

  // Initialize member _ptrConvTable
  _initConvTable(i_size_conv);
  
  // copy array entries to member _ptrConvTable
  for (int i_row=0; i_row<i_size_conv; i_row++){
    for (int i_col=0; i_col<2; i_col++){
      _ptrConvTable[i_row][i_col] = arr_conv_table[i_row][i_col];
    }
  }
  _iConvMethod = ADS1115_CONV_METHOD_TABLE;
}


float ADS1115::readPhysical(void){
  /**
   * read physical value
   * @return: physical value based on voltage read out
  */
 float f_voltage = readVoltage();

 if (_iConvMethod == ADS1115_CONV_METHOD_SINGLE) {
   return f_voltage * _fGradient + _fOffset;
 } else if (_iConvMethod == ADS1115_CONV_METHOD_TABLE) {
    float f_prev_x;
    float f_act_x;
    float f_prev_y;
    float f_act_y;
    
    // TODO: define behaviour when it is outside the defined range.
    if (f_voltage < _ptrConvTable[0][0]) {
      // left outside

    } else if (f_voltage > _ptrConvTable[_iSizeConvTable - 1][0]) {
      // right outside

    }

    for (int i_idx = 1; i_idx < _iSizeConvTable; i_idx++) {
      f_prev_x = _ptrConvTable[i_idx-1][0];
      f_act_x = _ptrConvTable[i_idx][0];
      f_prev_y = _ptrConvTable[i_idx-1][1];
      f_act_y = _ptrConvTable[i_idx][1];
      
      if( (f_voltage >= f_prev_x) && (f_voltage < f_act_x) ) {
        // inside interval
        float f_gradient = (f_act_y-f_prev_y)/(f_act_x-f_prev_x);
        float f_val = f_gradient * ( f_voltage - f_prev_x) + f_prev_y;
        return  f_val;
      } 
    }
 }
}


void ADS1115::printConfigReg() {
  /**
   * Dump Config register to Serial output
  */
  Serial.print("ADS1115 Conf.Reg.: ");
  iConfigReg = read16(ADS1115_CONFIG_REG);
  Serial.println(iConfigReg, BIN);
}


void ADS1115::write16(byte reg, uint16_t val) {
  byte _reg = reg;

  _objI2C->beginTransmission(_iI2cAddress);
  _objI2C->write(_reg);
  _objI2C->write((byte)highByte(val));
  _objI2C->write((byte)lowByte(val));
  _objI2C->endTransmission();
}


int16_t ADS1115::read16(byte reg) {
  byte _reg = reg;
  byte hByte, lByte;
  int16_t i_conv;
  String str_conv;

  _objI2C->beginTransmission(_iI2cAddress);
  _objI2C->write(_reg);
  _objI2C->endTransmission();
  delay(1);
  _objI2C->requestFrom((int)_iI2cAddress, 2);
  if (_objI2C->available())
  {
    hByte = _objI2C->read();
    lByte = _objI2C->read();

    i_conv = (hByte << 8) | lByte;
  }
  return i_conv;
}
