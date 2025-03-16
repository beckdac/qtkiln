#include "Arduino.h"
#include "qtkiln_thermo.h"

QTKilnThermo::QTKilnThermo(uint16_t interval_ms, float (* read_fptr)(void)) {
  _interval_ms = interval_ms;
  _read_fptr = read_fptr;
}

void QTKilnThermo::begin(void) {
  _lastTime = millis();
}

void QTKilnThermo::enable(void) {
  _enabled = true;
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

float QTKilnThermo::readCelsius(void) {
  return _lastTempC;
}

void QTKilnThermo::loop(void) {
  unsigned long now = millis();

  if ((now - _lastTime) >= _interval_ms && _enabled) {
    _lastTempC = (_read_fptr ? _read_fptr() : -1);
    _lastTime = millis();
  }
}
