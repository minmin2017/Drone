#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

class BMP280Reader {
public:
  explicit BMP280Reader(float seaLevel_hPa = 1013.25f)
  : _seaLevel_hPa(seaLevel_hPa), _addr(0), _ok(false), _wire(nullptr) {}

  bool begin(TwoWire& wire = Wire, int sda = -1, int scl = -1, uint32_t i2cFreq = 100000) {
    _wire = &wire;

#if defined(ESP32)
    if (sda >= 0 && scl >= 0) _wire->begin(sda, scl, i2cFreq);
    else _wire->begin();
    _wire->setClock(i2cFreq);
#else
    (void)sda; (void)scl;
    _wire->begin();
    _wire->setClock(i2cFreq);
#endif

    _wire->begin(sda, scl, freq);   // เรียก I2C ก่อน
    if (_bmp.begin(0x76))
 { _addr = 0x76; _ok = true; return true; }
    if (_bmp.begin(0x77)) { _addr = 0x77; _ok = true; return true; }

    _ok = false;
    return false;
  }

  bool ok() const { return _ok; }
  uint8_t address() const { return _addr; }

  float temperatureC() {
    if (!_ok) return NAN;
    return _bmp.readTemperature();
  }

  float pressurePa() {
    if (!_ok) return NAN;
    return _bmp.readPressure();
  }

  float pressurehPa() {
    float p = pressurePa();
    if (isnan(p)) return NAN;
    return p / 100.0f;
  }

  float altitudeM() {
    if (!_ok) return NAN;
    return _bmp.readAltitude(_seaLevel_hPa);
  }

  void configure() {
    if (!_ok) return;

    _bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,   // Temp
      Adafruit_BMP280::SAMPLING_X16,  // Pressure
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_125
    );
  }

private:
  Adafruit_BMP280 _bmp;
  float _seaLevel_hPa;
  uint8_t _addr;
  bool _ok;
  TwoWire* _wire;
};
