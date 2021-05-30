// Code based on Retian (version 1.0), Implementation extended and comments translated into english
// https://arduino-projekte.webnode.at/meine-libraries/adc-ads1115/

#ifndef ADS1115_h
#define ADS1115_h

#include <Wire.h>

#define ADS1115_I2CADD_DEFAULT  0x48 //ADDR-Pin on GND
#define ADS1115_I2CADD_ADDR_VDD 0x49 //ADDR-Pin on VDD
#define ADS1115_I2CADD_ADDR_SDA 0x4A //ADDR-Pin on SDA
#define ADS1115_I2CADD_ADDR_SCL 0x4B //ADDR-Pin on SCL

//ADS1115 register
#define ADS1115_CONVERSION_REG 0x00 // measurement is stored in this register
#define ADS1115_CONFIG_REG 0x01 // configuration register of the ADS1115

// bit number of ADS1115_CONFIG_REG 
#define ADS1115_DR0  0x05
#define ADS1115_DR1  0x06
#define ADS1115_DR2  0x07
#define ADS1115_MODE 0x08
#define ADS1115_PGA0 0x09
#define ADS1115_PGA1 0x0A
#define ADS1115_PGA2 0x0B
#define ADS1115_MUX0 0x0C
#define ADS1115_MUX1 0x0D
#define ADS1115_MUX2 0x0E
#define ADS1115_OS   0x0F

//ADS1115 input multiplexer settings
#define ADS1115_MUX_AIN0_AIN1 0b000 // default
#define ADS1115_MUX_AIN0_AIN3 0b001
#define ADS1115_MUX_AIN1_AIN3 0b010
#define ADS1115_MUX_AIN2_AIN3 0b011
#define ADS1115_MUX_AIN0_GND  0b100
#define ADS1115_MUX_AIN1_GND  0b101
#define ADS1115_MUX_AIN2_GND  0b110
#define ADS1115_MUX_AIN3_GND  0b111

// Programmable Gain Amplifier
#define ADS1115_PGA_6P144 0b000
#define ADS1115_PGA_4P096 0b001
#define ADS1115_PGA_2P048 0b010 // default
#define ADS1115_PGA_1P024 0b011
#define ADS1115_PGA_0P512 0b100
#define ADS1115_PGA_0P256 0b101

// measurement mode
#define ADS1115_MODE_CONTINUOUS 0
#define ADS1115_MODE_SINGLESHOT 1 // default

// data rate (SPS - samples per second)
#define ADS1115_RATE_8   0b000 
#define ADS1115_RATE_16  0b001
#define ADS1115_RATE_32  0b010
#define ADS1115_RATE_64  0b011
#define ADS1115_RATE_128 0b100 // default
#define ADS1115_RATE_250 0b101
#define ADS1115_RATE_475 0b110
#define ADS1115_RATE_860 0b111

// bit significance in mV depending on chosen PGA 
#define ADS1115_LSB_6P144  0.187500
#define ADS1115_LSB_4P096  0.125000
#define ADS1115_LSB_2P048  0.062500 // default
#define ADS1115_LSB_1P024  0.031250
#define ADS1115_LSB_0P512  0.015625
#define ADS1115_LSB_0P256  0.007813

#define ADS1115_DELAY_AFTER_MUX_CHANGE 5 //5 ms

#include "Arduino.h"

class ADS1115
{
  public:
    ADS1115(TwoWire*);
    bool begin(void);
    bool begin(uint8_t);
    bool begin(int, int);
    bool begin(int, int, uint8_t);
    void init(void);
    void setGain(byte);
    byte getGain(void);
    void setMux(byte);
    byte getMux(void);
    void setRate(byte);
    byte getRate(void);
    void setMode(bool);
    byte getMode(void);
    void startSingleMeas(void);
    bool conversionReady(void);
    int readConversion(void);
    float readVoltage(void);
    void printConfigReg(void);
        
  private:
    unsigned int read16(byte);
    void write16(byte, unsigned int);

    int _iSdaPin;
    int _iSclPin;
    uint8_t _iI2cAddress;
    TwoWire * _objI2C;
    float bitNumbering;
    unsigned int configReg;
};

#endif