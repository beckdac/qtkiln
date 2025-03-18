#include "qtkiln_thermo.h"

QTKilnThermo::QTKilnThermo(uint16_t interval_ms, MAX31855 *max31855, MAX6675 *max6675) {
  _interval_ms = interval_ms;
  _max31855 = max31855;
  _max6675 = max6675;
  _lastTime = 0;
  _lastTempC = -1;
  if (_max31855 || _max6675) {
    _err = false;
  } else {
    _err = true;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::begin(void) {
  if (_max31855) {
    _max31855->begin();
    while(_max31855->getChipID(MAX31855_FORCE_READ_DATA) != MAX31855_ID) {
      Serial.println("unable to retrieve ID from MAX31855");
      delay(5000);
    }
    Serial.println("MAX31855 connection verified");
  } else if (_max6675) {
    _max6675->begin();
    while (_max6675->getChipID(MAX6675_FORCE_READ_DATA) != MAX6675_ID) {
      Serial.println("unable to detect MAX6675");
      delay(5000);
    }
    Serial.println("MAX6675 connection verified");
  } else {
    _err = true;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::enable(void) {
  _enabled = true;
  _doRead();
}

void QTKilnThermo::disable(void) {
  _enabled = false;
}

bool QTKilnThermo::isEnabled(void) {
  return _enabled;
}

unsigned long QTKilnThermo::msSinceLastUpdate(void) {
  unsigned long now = millis();
  return now - _lastTime;
}

unsigned long QTKilnThermo::lastTime(void) {
  return _lastTime;
}

float QTKilnThermo::getTemperatureC(void) {
  if (!_lastTime) {
    _doRead();
  }
  return _lastTempC;
}

void QTKilnThermo::_MAX31855_verbose_diagnose(uint8_t code) {
    switch (code)
    {
      case MAX31855_THERMOCOUPLE_SHORT_TO_VCC:
        Serial.println(F("Thermocouple short to VCC"));
        break;

      case MAX31855_THERMOCOUPLE_SHORT_TO_GND:
        Serial.println(F("Thermocouple short to GND"));
        break;

      case MAX31855_THERMOCOUPLE_NOT_CONNECTED:
        Serial.println(F("Thermocouple not connected"));
        break;

      case MAX31855_THERMOCOUPLE_UNKNOWN:
        Serial.println(F("Thermocouple unknown error"));
        break;

      //case MAX31855_THERMOCOUPLE_READ_FAIL:
       // Serial.println(F("Thermocouple read error, check chip & spi cable"));
        //break;
    }
}

void QTKilnThermo::_doRead(void) {
  if (_max31855) {
    if (_max31855->detectThermocouple(MAX31855_FORCE_READ_DATA) == MAX31855_THERMOCOUPLE_OK) {
      double tmp =_max31855->getTemperature(MAX31855_FORCE_READ_DATA);
      if (tmp != MAX31855_ERROR) {
	_lastTempC = tmp;
    	_lastTime = millis();
      } else {
	_err = true;
	_errno = QTKILN_ERRNO_READ_ERROR;
      }
    } else {
      _err = true;
      _errno = QTKILN_ERRNO_MAX31855_NOT_DETECTED;
    }
  } else if (_max6675) {
    double tmp = _max6675->getTemperature(MAX6675_FORCE_READ_DATA);
    if (tmp != MAX6675_ERROR) {
      _lastTempC = tmp;
      _lastTime = millis();
    } else {
      _err = true;
      _errno = QTKILN_ERRNO_READ_ERROR;
    }
  } else {
    _err = true;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::loop(void) {
  unsigned long now = millis();

  if ((now - _lastTime) >= _interval_ms && _enabled)
    _doRead();
}
