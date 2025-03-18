#ifndef QTKILN_THERMO_H
#define QTKILN_THERMO_H

#include "Arduino.h"
#include "MAX6675.h"
#include "MAX31855.h"

#define QTKILN_ERRNO_READ_ERROR 1
#define QTKILN_ERRNO_INVALID_TYPE 2
#define QTKILN_ERRNO_MAX31855_NOT_DETECTED 3

class QTKilnThermo
{
  public:
    QTKilnThermo(uint16_t interval_ms, MAX31855 *max31855, MAX6675 *max6675);

    void begin(void);
    void loop(void);
    unsigned long msSinceLastUpdate(void);
    void enable(void);
    void disable(void);
    bool isEnabled(void);
    float getTemperatureC(void);
    unsigned long lastTime(void);

  private:
    void _MAX31855_verbose_diagnose(uint8_t code);
    uint16_t _interval_ms = 250;	// default update period
    MAX31855 *_max31855;		// if this is not null, then it is 
    MAX6675 *_max6675;			//	this type
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    bool _err = false;			// has the system encountered an error
    uint8_t _errno = 0;			// what error has occurred
    float _lastTempC = .0;		// last temperature reading in C
    void _doRead(void);			// do the read via the phy
};

#endif
