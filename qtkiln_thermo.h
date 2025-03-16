#ifndef QTKILN_THERMO_H
#define QTKILN_THERMO_H

#include "Arduino.h"

class QTKilnThermo
{
  public:
    QTKilnThermo(uint16_t interval_ms, float (* read_fptr)(void));

    void begin(void);
    void loop(void);
    unsigned long msSinceLastUpdate(void);
    void enable(void);
    void disable(void);
    bool isEnabled(void);
    float readCelsius(void);
    unsigned long lastTime(void);

  private:
    uint16_t _interval_ms = 250;	// default update period
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    float _lastTempC = .0;		// last temperature reading in C
    float _readCelsius(void);		// return the temperature in C
    float (*_read_fptr)(void) = NULL;	// underlying thermocouple read function ptr
};

#endif
