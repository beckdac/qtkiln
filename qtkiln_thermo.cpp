#include "qtkiln_thermo.h"
#include "qtkiln_log.h"
#include "qtkiln.h"

extern QTKilnLog qtklog;

// use a static function to be the entry point for the task
void thermoTaskFunction(void *pvParameter) {
  QTKilnThermo *thermo = static_cast<QTKilnThermo *>(pvParameter);

  thermo->thread();
}

QTKilnThermo::QTKilnThermo(uint16_t updateInterval_ms, MAX31855 *max31855, const char *name) {
  _name = name;
  _updateInterval_ms = updateInterval_ms;
  _max31855 = max31855;
  _lastTime = 0;
  _lastTemp_C = 0;
  _taskHandle = NULL;
  _lowPassFilter.cutoffFrequency_Hz = 1.;
  if (_max31855) {
    _err = 0;
  } else {
    _err++;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::setUpdateInterval_ms(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
  qtklog.debug(QTKLOG_DBG_PRIO_HIGH, "thermo update interval modified to %d ms", _updateInterval_ms);
}

uint16_t QTKilnThermo::getUpdateInterval_ms(void) {
  return _updateInterval_ms;
}

void QTKilnThermo::begin(void) {
  _lowPassFilter.ts = 0;


  if (_max31855) {
    _max31855->begin();
    int _max31855_read = _max31855->read();
    while(_max31855_read != STATUS_OK) {
      qtklog.warn("unable to retrieve reading from MAX31855 named %s", _name);
      _err++;
      delay(1000);
      _max31855_read = _max31855->read();
    }
    qtklog.print("MAX31855 connection verified after %d failures", _err);
  } else {
    _err++;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }

  // start on core 1
  BaseType_t rc = xTaskCreatePinnedToCore(thermoTaskFunction, _name,
		  QTKILN_THERMO_TASK_STACK_SIZE, (void *)this, QTKILN_THERMO_TASK_PRI,
		  &_taskHandle, QTKILN_TASK_CORE);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for %s", _name);
}

TaskHandle_t QTKilnThermo::getTask(void) {
  return _taskHandle;
}

void QTKilnThermo::enable(void) {
  _enabled = true;
  qtklog.debug(QTKLOG_DBG_PRIO_HIGH, "enabling thermo");
  if (_lastTime == 0) {
    _doRead();
    qtklog.debug(QTKLOG_DBG_PRIO_LOW, "read thermo done");
  }
}

void QTKilnThermo::disable(void) {
  _enabled = false;
  qtklog.debug(QTKLOG_DBG_PRIO_HIGH, "disabling thermo");
}

bool QTKilnThermo::isEnabled(void) {
  return _enabled;
}

unsigned long QTKilnThermo::msSinceLastUpdate(void) {
  unsigned long now = millis();
  return now - _lastTime;
}

unsigned long QTKilnThermo::getLastTime(void) {
  return _lastTime;
}

float QTKilnThermo::getFilteredTemperature_C(void) {
  if (_lastTime == 0) {
    _doRead();
  }
  return _lowPassFilter.filtered_C;
}

float QTKilnThermo::getTemperature_C(void) {
  if (!_lastTime) {
    _doRead();
  }
  return _lastTemp_C;
}

// simple first order low pass filter in Hz
void QTKilnThermo::_filter(float sample) {
  double ts = (millis() - _lowPassFilter.ts) * 1e-3;

  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "sample = %.2f, previous sample = %.2f", sample, _lowPassFilter.prevSample_C);
	
  float a_0 = -(((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz - 1.) 
		  / ((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz + 1.));
  float b_0 = ((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz) 
	  	/ (1. + (ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz);

  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "a_0 = %g b_0 = %g", a_0, b_0);

  _lowPassFilter.filtered_C = a_0 * _lowPassFilter.filtered_C + 
	  b_0 * sample + b_0 * sample;
  _lowPassFilter.prevSample_C = sample;
  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "sample = %.2f, filtered = %.2f", sample, _lowPassFilter.filtered_C);
  _lowPassFilter.ts = millis();
}

void QTKilnThermo::_doRead(void) {
  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "_doRead()");
  if (_max31855) {
    uint8_t status = _max31855->read();
    if (status == STATUS_OK) {
      double tmp =_max31855->getTemperature();
      qtklog.debug(QTKLOG_DBG_PRIO_LOW, "%s max31855 = %g", _name, tmp);
      _lastTemp_C = tmp;
      _lastTime = millis();
    } else {
      if (_max31855->shortToGND())
        qtklog.warn("%s thermocouple has SHORT TO GROUND");
      if (_max31855->shortToVCC())
        qtklog.warn("%s thermocouple has SHORT TO VCC");
      if (_max31855->openCircuit())  
        qtklog.warn("%s thermocouple has OPEN CIRCUIT");
      if (_max31855->genericError()) 
        qtklog.warn("%s thermocouple has GENERIC ERROR");
      if (_max31855->noRead())       
        qtklog.warn("%s thermocouple has NO READ");
      if (_max31855->noCommunication()) 
        qtklog.warn("%s thermocouple has NO COMMUNICATION");
      _err++;
      _errno = QTKILN_ERRNO_MAX31855_NOT_DETECTED;
      qtklog.warn("MAX31855 thermocouple not detected");
      return;
    }
  } else {
    _err++;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
    qtklog.warn("no thermocouple object available");
    return;
  }
  _filter(_lastTemp_C);
}

const char *QTKilnThermo::getName(void) {
  return _name;
}

void QTKilnThermo::thread(void) {
  TickType_t xDelay;

  while (1) {
    if (_enabled) {
      _doRead();
    }
    xDelay = pdMS_TO_TICKS(_updateInterval_ms);
    vTaskDelay(xDelay);
  }
}

UBaseType_t QTKilnThermo::getTaskHighWaterMark(void) {
  if (_taskHandle)
    return uxTaskGetStackHighWaterMark(_taskHandle);
  qtklog.warn("no task associated with qtkiln thermo in high watermark test");
  return 0;
}

unsigned int QTKilnThermo::getErrorCount(void) {
  return _err;
}

float QTKilnThermo::getFilterCutoffFrequency_Hz(void) {
  return _lowPassFilter.cutoffFrequency_Hz;
}

void QTKilnThermo::setFilterCutoffFrequency_Hz(float cutoffFrequency_Hz) {
  _lowPassFilter.cutoffFrequency_Hz = cutoffFrequency_Hz;
  _lowPassFilter.ts = 0; // should this be reset? surely a non zero valu is a better estimate?
}
