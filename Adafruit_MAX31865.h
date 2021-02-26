/**********Adafruit_MAX31865 modified by Sylvain Boyer 17/02/2021
 * 
 * 17/02/2021 - added some code from Jack Davies and J-M-L (arduino forum)
 *  https://forum.arduino.cc/index.php?topic=703346.msg4772457#msg4772457
 *    added: readRTDAsync() and temperatureAsync
 *    this allow to keep the code running rather than being stuck by the delays
 *  17/02/2021 - Fork from https://github.com/budulinek/Adafruit_MAX31865.git
 *    the fork contains code improvement to allow continuous measurement rather than single shot measurement
 *    this will improve reading rate but will cause self heating.
 *    the code also allow the use of the 50Hz filter with the appropriate delay.


/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_MAX31865_H
#define ADAFRUIT_MAX31865_H

//#define MAX31865_DEBUG_LIBRARY

#define MAX31865_CONFIG_REG 0x00
#define MAX31865_CONFIG_BIAS 0x80
#define MAX31865_CONFIG_MODEAUTO 0x40
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20
#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ 0x01
#define MAX31865_CONFIG_FILT60HZ 0x00

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_RTDLSB_REG 0x02
#define MAX31865_HFAULTMSB_REG 0x03
#define MAX31865_HFAULTLSB_REG 0x04
#define MAX31865_LFAULTMSB_REG 0x05
#define MAX31865_LFAULTLSB_REG 0x06
#define MAX31865_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_SPIDevice.h>

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/*! Interface class for the MAX31865 RTD Sensor reader */
class Adafruit_MAX31865 {
public:
  Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso,
                    int8_t spi_clk);
  Adafruit_MAX31865(int8_t spi_cs);

  bool begin(max31865_numwires_t x = MAX31865_2WIRE);

  uint8_t readFault(void);
  void clearFault(void);
  uint16_t readRTD();

  bool readRTDAsync(uint16_t& rtd); //added by JD
  float temperatureAsync(float Rt, float RTDnominal, float refResistor); //added by JD

  void setWires(max31865_numwires_t wires);
  void autoConvert(bool b);
  void enable50Hz(bool b);
  void enableBias(bool b);

  float temperature(float RTDnominal, float refResistor);

  #ifdef MAX31865_DEBUG_LIBRARY
    uint8_t debugConfigRegister(void);
  #endif

private:
  Adafruit_SPIDevice spi_dev;

  void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);

  uint8_t readRegister8(uint8_t addr);
  uint16_t readRegister16(uint8_t addr);

  void writeRegister8(uint8_t addr, uint8_t reg);
  
  // bias voltage
  bool bias;
  
  // continuous conversion
  bool continuous;
  
  // 50Hz filter
  bool filter50Hz;     
};

#endif
