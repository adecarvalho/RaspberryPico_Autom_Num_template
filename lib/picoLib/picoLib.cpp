#include <sys/_stdint.h>

#include <stdint.h>
#include "Arduino.h"

#include "picoLib.h"
//
DigitalOut::DigitalOut() {
  _pin = 0x00;
  _state = 0x00;
}
//
DigitalOut::~DigitalOut() {}
//
void DigitalOut::begin(uint8_t pin, uint8_t initState) {
  _pin = pin;
  _state = initState;
  //
  pinMode(_pin, OUTPUT);
  digitalWriteFast(_pin, _state);
}
//
void DigitalOut::write(uint8_t state) {
  if (_state == state) {
    return;
  } else {
    _state = state;
    digitalWriteFast(_pin, _state);
  }
}
//
uint8_t DigitalOut::getState() const {
  return _state;
}
//
void DigitalOut::toggle() {
  if (_state == 0x00) {
    _state = 0x01;
  } else {
    _state = 0x00;
  }
  digitalWriteFast(_pin, _state);
}
//
Button::Button() {
  _pin = 0x00;
  _lastReading = HIGH;
  _lastDebounceTime = 0;
  _debounceDelay = 10;
  _state = 0;
}
//
Button::~Button() {}
//
void Button::begin(uint8_t pin) {
  _pin = pin;
  pinMode(_pin, INPUT);
}
//
void Button::read() {
  bool reading = digitalRead(_pin);

  if (reading != _lastReading) {
    _lastDebounceTime = millis();
  }
  //
  if ((millis() - _lastDebounceTime) > _debounceDelay) {
    bool pressed = reading == HIGH;
    if (pressed) {
      if (_state < 0xFFFE) {
        _state++;
      } else if (_state == 0xFFFE) {
        _state = 2;
      }
    } else if (_state) {
      _state = (_state == 0xFFFF) ? 0 : 0xFFFF;
    }
  }
  //
  _lastReading = reading;
}
//
bool Button::isPressed() {
  return _state == 1;
}
//
bool Button::isReleased() {
  return _state == 0xFFFF;
}
//
bool Button::isHeld(uint16_t count) {
  return _state > (1 + count) && _state < 0xFFFF;
}
//
SoftTimer::SoftTimer() {
  _period = 0;
  _lasttime = 0;
  _enable = false;
  _callback = nullptr;
}
//
SoftTimer::~SoftTimer() {
  _period = 0;
  _lasttime = 0;
  _enable = false;
  _callback = nullptr;
}
//
void SoftTimer::start() {
  _enable = true;
}
//
void SoftTimer::stop() {
  _enable = false;
}
//
void SoftTimer::update() {
  uint32_t now = millis();

  if (_enable) {
    if ((now - _lasttime) >= _period) {
      _lasttime = now;
      if (_callback != nullptr) {
        _callback();
      }
    }
  }
}
//
void SoftTimer::setPeriod(uint32_t ms) {
  _period = ms;
}
//
void SoftTimer::attach(void (*callback)(void), uint32_t ms) {
  _period = ms;
  _callback = callback;
}
//**************************
MCP3208::MCP3208() {
  _spi0 = nullptr;
}
//
MCP3208::~MCP3208() {
  if (_spi0 != nullptr) {
    delete _spi0;
    _spi0 = nullptr;
  }
}
//
void MCP3208::begin() {
  _spi0 = &SPI;
  _spi0->setSCK(SPI0_SCK_PIN);
  _spi0->setTX(SPI0_TX_PIN);
  _spi0->setRX(SPI0_RX_PIN);
  _spi0->setCS(SPI0_CS_PIN);
  _spi0->begin();
  //
  pinMode(SPI0_CS_PIN, OUTPUT);
  digitalWrite(SPI0_CS_PIN, 1);
}
//
uint16_t MCP3208::read(uint8_t channel) {
  uint16_t res = 0x0000;
  uint16_t data = 0x0600;

  uint8_t msb = 0x00;
  uint8_t mid = 0x00;
  uint8_t lsb = 0x00;

  uint8_t r0 = 0x00;
  uint8_t r1 = 0x00;
  uint8_t r2 = 0x00;

  //
  if (channel > 7) {
    channel = 7;
  }
  //
  data = data | (channel << 6);

  msb = (data & 0xFF00) >> 8;
  mid = (data & 0x00FF);
  lsb = 0x00;
  //
  digitalWrite(SPI0_CS_PIN, 0);
  _spi0->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  r2 = _spi0->transfer(msb);
  r1 = _spi0->transfer(mid);
  r0 = _spi0->transfer(lsb);
  _spi0->endTransaction();
  digitalWrite(SPI0_CS_PIN, 1);
  //

  res = res | (r1 << 8) | r0;
  return res;
}
//
void MCP3208::close() {

  if (_spi0 != nullptr) {
    delete _spi0;
    _spi0 = nullptr;
  }
}
//*****************************
MCP4922::MCP4922() {
  _spi1 = nullptr;
}
//
MCP4922::~MCP4922() {
  if (_spi1 != nullptr) {
    delete _spi1;
    _spi1 = nullptr;
  }
}
//
void MCP4922::begin() {
  _spi1 = &SPI1;
  _spi1->setSCK(SPI1_SCK_PIN);
  _spi1->setTX(SPI1_TX_PIN);
  _spi1->setRX(SPI1_RX_PIN);
  _spi1->setCS(SPI1_CS_PIN);
  //
  _spi1->begin();
  pinMode(SPI1_CS_PIN, OUTPUT);
  digitalWrite(SPI1_CS_PIN, 1);
}
//
void MCP4922::write(uint8_t channel, int value) {
  uint16_t data = 0x0000;
  uint8_t msb = 0x00;
  uint8_t lsb = 0x00;

  //saturation
  if (value > 4095) {
    value = 4095;
  }
  if (value < 0) {
    value = 0;
  }
  //
  if (channel == MCP4922::OUT_A) {
    data = 0x7000;
  } else {
    data = 0xF000;
  }
  //
  data = data | value;
  msb = (data & 0xFF00) >> 8;
  lsb = (data & 0x00FF);
  //
  digitalWrite(SPI1_CS_PIN, 0);
  _spi1->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  _spi1->transfer(msb);
  _spi1->transfer(lsb);
  _spi1->endTransaction();
  digitalWrite(SPI1_CS_PIN, 1);
}
//
void MCP4922::reset() {
  write(MCP3208::IN_A, 0);
  write(MCP3208::IN_B, 0);
}
//
void MCP4922::close() {
  if (_spi1 != nullptr) {
    delete _spi1;
    _spi1 = nullptr;
  }
}
//*******************************
MCP9808::MCP9808() {
  _adress = 0x00;
  _i2c = nullptr;
}
//
MCP9808::~MCP9808() {
}
//
bool MCP9808::begin(TwoWire* i2c, uint8_t adress) {
  _adress = adress;
  _i2c = i2c;
  //
  uint8_t msb = 0x00;
  uint8_t lsb = 0x00;
  uint16_t res = 0x0000;
  bool val = false;
  //
  _i2c->beginTransmission(_adress);
  _i2c->write(MCP9808::MANUFACTURER_ID_REG);
  _i2c->endTransmission();

  _i2c->requestFrom(_adress, byte(2));
  msb = _i2c->read();
  lsb = _i2c->read();
  _i2c->endTransmission();

  //
  res = (msb << 8) | lsb;

  if (res == MCP9808::MANUFACTURER_ID) {
    val = true;
  } else {
    val = false;
  }

  return val;
}
void MCP9808::setResolution(uint8_t resolution) {
  if (resolution > 3) {
    resolution = 3;
  }
  //
  _i2c->beginTransmission(_adress);
  _i2c->write(MCP9808::RESOLUTION_REG);
  _i2c->write(resolution);
  _i2c->endTransmission();
}
//
uint8_t MCP9808::getResolution() {
  uint8_t res = 0;

  _i2c->beginTransmission(_adress);
  _i2c->write(MCP9808::RESOLUTION_REG);
  _i2c->endTransmission();
  //
  _i2c->requestFrom(_adress, byte(1));
  res = _i2c->read();
  _i2c->endTransmission();

  return res;
}
//
float MCP9808::getTemperature() {
  float temp = 0;
  uint8_t upper = 0x00;
  uint8_t lower = 0x00;
  uint8_t signe = 0x00;
  //
  _i2c->beginTransmission(_adress);
  _i2c->write(MCP9808::TEMPERATURE_REG);
  _i2c->endTransmission();
  //
  _i2c->requestFrom(_adress, byte(2));
  upper = _i2c->read();
  lower = _i2c->read();
  _i2c->endTransmission();
  //
  signe = upper & 0b00010000;
  upper = upper & 0b00011111;

  temp = (upper * 16.0) + (lower / 16.0);

  if (signe) {
    temp = 256 - temp;
  }

  return temp;
}
//
DHT22::DHT22() {
  _pin = 0;
  _temperature = 0;
  _humidity = 0;
  _dewPoint = 0;
}
//
void DHT22::begin(uint8_t dhtPin) {
  _pin = dhtPin;
  pinMode(_pin, INPUT);
}
//
float DHT22::_readDewPoint(float temp, float hum) {
  float dew = 0;
  float a = 17.27;
  float b = 237.7;
  float phi = 0;

  if (hum > 0) {
    phi = (a * temp) / (b + temp);
    phi = phi + log(hum / 100);
    //
    dew = (b * phi) / (a - phi);
  }
  return dew;
}
//
float DHT22::getDewPoint() {
  return _dewPoint;
}
//
DHT_STATUS DHT22::readValues() {
  DHT_STATUS res = DHT_OK;

  uint8_t signe = 0;
  uint8_t check = 0;
  uint8_t sum = 0;
  uint8_t hum_upper = 0;
  uint8_t hum_lower = 0;
  uint8_t temp_upper = 0;
  uint8_t temp_lower = 0;

  uint16_t hum_16 = 0x0000;
  uint16_t temp_16 = 0x0000;

  uint32_t datas = 0x00000000;

  //
  res = _startConversion();

  if (res == DHT_OK) {
    //hum_16 = _readWord();
    //temp_16 = _readWord();
    datas = _readLong();
    sum = _readByte();

    hum_16 = (datas & 0xFFFF0000) >> 16;
    temp_16 = (datas & 0x0000FFFF);

    hum_upper = (hum_16 & 0xFF00) >> 8;
    hum_lower = (hum_16 & 0x00FF);

    temp_upper = (temp_16 & 0xFF00) >> 8;
    temp_lower = (temp_16 & 0x00FF);

    //
    check = hum_upper + hum_lower + temp_upper + temp_lower;
    //
    if (check == sum) {
      res = DHT_OK;
      //
      _humidity = (float)(hum_16 / 10.0);

      signe = temp_upper & 0b10000000;

      if (signe > 0) {
        temp_upper = temp_upper & 0b01111111;
        temp_16 = (temp_upper << 8) | (temp_lower);
        _temperature = (float)(temp_16 / 10.0) * (-1);

      } else {
        _temperature = (float)(temp_16 / 10.0);
      }

      _dewPoint = _readDewPoint(_temperature, _humidity);

    } else {
      res = DHT_ERROR_CHECK_SUM;
    }
  }
  return res;
}
//
float DHT22::getTemperature() {
  return _temperature;
}
//
float DHT22::getHumidity() {
  return _humidity;
}
//
DHT_STATUS DHT22::_startConversion() {
  DHT_STATUS res = DHT_OK;

  pinMode(_pin, OUTPUT);
  //
  digitalWrite(_pin, 0);
  delayMicroseconds(1200);
  //
  digitalWrite(_pin, 1);
  delayMicroseconds(30);
  //
  pinMode(_pin, INPUT);
  delayMicroseconds(40);

  if (digitalRead(_pin) == 0) {
    delayMicroseconds(80);
    //
    if (digitalRead(_pin) == 1) {
      res = DHT_OK;
    } else {
      res = DHT_ERROR_RESPONSE;
    }
  }
  uint32_t now = millis();
  //
  while (digitalRead(_pin) == 1) {
    if ((millis() - now) > 20) {
      return DHT_ERROR_TIMEOUT;
    }
  }

  return res;
}
//
uint32_t DHT22::_readLong() {
  uint32_t res = 0x00000000;

  for (int i = 0; i < 32; i++) {
    while (digitalRead(_pin) == 0) {
    }
    //
    delayMicroseconds(40);
    //
    if (digitalRead(_pin) == 1) {
      res = res | (1 << (31 - i));
    }
    //
    while (digitalRead(_pin) == 1) {
    }
  }
  //
  return res;
}
//
uint16_t DHT22::_readWord() {
  uint16_t res = 0x0000;

  for (int i = 0; i < 16; i++) {
    while (digitalRead(_pin) == 0) {
    }
    //
    delayMicroseconds(40);
    //
    if (digitalRead(_pin) == 1) {
      res = res | (1 << (15 - i));
    }
    //
    while (digitalRead(_pin) == 1) {
    }
  }
  //
  return res;
}
//
uint8_t DHT22::_readByte() {
  uint8_t res = 0x00;

  for (int i = 0; i < 8; i++) {
    while (digitalRead(_pin) == 0) {
    }
    //
    delayMicroseconds(40);
    //
    if (digitalRead(_pin) == 1) {
      res = res | (1 << (7 - i));
    }
    //
    while (digitalRead(_pin) == 1) {
    }
  }
  //
  return res;
}
