#ifndef QTKILN_THERMO_H
#define QTKILN_THERMO_H

#include "Arduino.h"
#include "MAX6675.h"
#include "MAX31855.h"

#define QTKILN_ERRNO_READ_ERROR 1
#define QTKILN_ERRNO_INVALID_TYPE 2
#define QTKILN_ERRNO_MAX31855_NOT_DETECTED 3

#define QTKILN_THERMO_TASK_STACK_SIZE 2024+512
#define QTKILN_THERMO_TASK_PRI tskIDLE_PRIORITY + 1

#define QTKILN_THERMO_DEFAULT_UPDATE_INTERVAL_MS 250

extern "C" void thermoTaskFunction(void *pvParameter);

class QTKilnThermo
{
  public:
    QTKilnThermo(uint16_t updateInterval_ms, MAX31855 *max31855, MAX6675 *max6675);

    void begin(void);
    unsigned long msSinceLastUpdate(void);
    void enable(void);
    void disable(void);
    bool isEnabled(void);
    float getTemperature_C(void);
    unsigned long getLastTime(void);
    void thread(void);
    TaskHandle_t getTask(void);
    UBaseType_t getTaskHighWaterMark(void);
    uint16_t getUpdateInterval_ms(void);
    void setUpdateInterval_ms(uint16_t updateInterval_ms);
    unsigned int getErrorCount(void);
    float getFilteredTemperature_C(void);
    float getFilterCutoffFrequency_Hz(void);
    void setFilterCutoffFrequency_Hz(float cutoffFrequency_Hz);

  private:
    void _MAX31855_verbose_diagnose(uint8_t code);
    uint16_t _updateInterval_ms = 250;	// default update period
    MAX31855 *_max31855;		// if this is not null, then it is 
    MAX6675 *_max6675;			//	this type
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    unsigned int _err = 0;		// count of errors
    uint8_t _errno = 0;			// what error has occurred last
    float _lastTemp_C = .0;		// last temperature reading in C
    void _doRead(void);			// do the read via the phy
    struct QTKilnThermoLowPassFilter {
      float cutoffFrequency_Hz = 1.;
      float prevSample_C = 0;
      float filtered_C = 0;
      double ts = 0;
    } _lowPassFilter;
    void _filter(float sample);		// process a new sample into the filter
    TaskHandle_t _taskHandle = NULL;	// for managing the task later
};

#endif
