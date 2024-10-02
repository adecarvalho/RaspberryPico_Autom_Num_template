#include <sys/_stdint.h>
#include <sys/types.h>
#include <stdint.h>
#ifndef PICO_LIB_H
#define PICO_LIB_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
//*****************************
//DigitalOut
//*****************************
class DigitalOut {
public:
  DigitalOut();
  virtual ~DigitalOut();

  void begin(uint8_t pin, uint8_t initState = 0);
  void write(uint8_t state);
  uint8_t getState() const;
  void toggle();

private:
  uint8_t _pin;
  uint8_t _state;
};
//*****************************
//Button
//*****************************
class Button {
public:
  Button();
  virtual ~Button();

  void begin(uint8_t pin);

  void read();
  bool isPressed();
  bool isReleased();
  bool isHeld(uint16_t count = 0);

private:
  uint8_t _pin;
  bool _lastReading;
  uint32_t _lastDebounceTime;
  uint32_t _debounceDelay;
  uint16_t _state;
};
//*****************************
//SoftTimer
//*****************************
class SoftTimer {
public:
  SoftTimer();
  virtual ~SoftTimer();

  void start();
  void stop();
  void update();
  void setPeriod(uint32_t ms);
  void attach(void (*callback)(void), uint32_t ms);

private:
  uint32_t _period;
  uint32_t _lasttime;
  bool _enable;
  void (*_callback)(void);
};
//*******************************
// MCP3208
//********************************
class MCP3208 {
public:
  static const uint8_t IN_A = 0;
  static const uint8_t IN_B = 1;

  const uint8_t SPI0_SCK_PIN = 2;
  const uint8_t SPI0_TX_PIN = 3;
  const uint8_t SPI0_RX_PIN = 4;
  const uint8_t SPI0_CS_PIN = 5;

  MCP3208();

  virtual ~MCP3208();

  void begin();  //dans le setup

  uint16_t read(uint8_t channel);

  void close();

private:
  SPIClassRP2040* _spi0 = nullptr;
};
//**************************
// MCP4922
//**************************
class MCP4922 {
public:
  const uint8_t SPI1_SCK_PIN = 10;
  const uint8_t SPI1_TX_PIN = 11;
  const uint8_t SPI1_RX_PIN = 12;
  const uint8_t SPI1_CS_PIN = 13;

  static const uint8_t OUT_A = 0;
  static const uint8_t OUT_B = 1;

  MCP4922();
  virtual ~MCP4922();

  void begin();

  void write(uint8_t channel, int value);

  void reset();

  void close();
private:
  SPIClassRP2040* _spi1 = nullptr;
};
//******************************
// MCP9808
//******************************
class MCP9808 {
public:

  static const uint8_t TEMPERATURE_REG = 0x05;
  static const uint8_t RESOLUTION_REG = 0x08;
  static const uint8_t MANUFACTURER_ID_REG = 0x06;
  static const uint16_t MANUFACTURER_ID = 0x0054;

  MCP9808();
  virtual ~MCP9808();

  bool begin(TwoWire* i2c,  uint8_t adress);

  void setResolution(uint8_t resolution);

  uint8_t getResolution();

  float getTemperature();

private:
  uint8_t _adress;

  TwoWire *_i2c;
};
//*************************
// DHT22
//*************************
enum DHT_STATUS
{
    DHT_OK = 0,
    DHT_ERROR_RESPONSE,
    DHT_ERROR_CHECK_SUM,
    DHT_ERROR_TIMEOUT
};
//
class DHT22
{
public:
    DHT22();
    
    void begin(uint8_t dhtPin);

    DHT_STATUS readValues();

    float getTemperature();

    float getHumidity();

    float getDewPoint();
    
private:
    DHT_STATUS _startConversion();
    uint8_t _readByte();
    uint16_t _readWord();
    uint32_t _readLong();
    float _readDewPoint(float temp, float hum);

    uint8_t _pin;
    float _temperature;
    float _humidity;
    float _dewPoint;
};
#endif