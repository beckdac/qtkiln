#include "qtkiln_thermo.h"
#include "qtkiln_log.h"
#include "qtkiln.h"

extern QTKilnLog qtklog;

// use a static function to be the entry point for the task
void thermoTaskFunction(void *pvParameter) {
  QTKilnThermo *thermo = static_cast<QTKilnThermo *>(pvParameter);

  thermo->thread();
}

QTKilnThermo::QTKilnThermo(uint16_t updateInterval_ms, MAX31855 *max31855, MAX6675 *max6675) {
  _updateInterval_ms = updateInterval_ms;
  _max31855 = max31855;
  _max6675 = max6675;
  _lastTime = 0;
  _lastTemp_C = 0;
  _taskHandle = NULL;
  _lowPassFilter.cutoffFrequency_Hz = 1.;
  if (_max31855 || _max6675) {
    _err = 0;
  } else {
    _err++;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::setUpdateInterval_ms(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
  qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "thermo update interval modified to %d ms", _updateInterval_ms);
}

uint16_t QTKilnThermo::getUpdateInterval_ms(void) {
  return _updateInterval_ms;
}

void QTKilnThermo::begin(void) {
  _lowPassFilter.ts = 0;


  if (_max31855) {
    _max31855->begin();
    while(_max31855->getChipID(MAX31855_FORCE_READ_DATA) != MAX31855_ID) {
      qtklog.warn("unable to retrieve ID from MAX31855");
      delay(5000);
    }
    qtklog.print("MAX31855 connection verified");
  } else if (_max6675) {
    _max6675->begin();
    while (_max6675->getChipID(MAX6675_FORCE_READ_DATA) != MAX6675_ID) {
      qtklog.warn("unable to detect MAX6675");
      delay(5000);
    }
    qtklog.print("MAX6675 connection verified");
  } else {
    _err++;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }

  // start on core 1
  BaseType_t rc = xTaskCreatePinnedToCore(thermoTaskFunction, (_max31855 ? "max31855" : "max65675"),
		  QTKILN_THERMO_TASK_STACK_SIZE, (void *)this, QTKILN_THERMO_TASK_PRI,
		  &_taskHandle, QTKILN_TASK_CORE);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for %s", (_max31855 ? "max31855" : "max65675"));
}

TaskHandle_t QTKilnThermo::getTask(void) {
  return _taskHandle;
}

void QTKilnThermo::enable(void) {
  _enabled = true;
  qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "enabling thermo");
  if (_lastTime == 0) {
    _doRead();
    qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "read thermo done");
  }
}

void QTKilnThermo::disable(void) {
  _enabled = false;
  qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "disabling thermo");
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

void QTKilnThermo::_MAX31855_verbose_diagnose(uint8_t code) {
    switch (code)
    {
      case MAX31855_THERMOCOUPLE_SHORT_TO_VCC:
        qtklog.warn("thermocouple short to VCC");
        break;

      case MAX31855_THERMOCOUPLE_SHORT_TO_GND:
        qtklog.warn("thermocouple short to GND");
        break;

      case MAX31855_THERMOCOUPLE_NOT_CONNECTED:
        qtklog.warn("thermocouple not connected");
        break;

      case MAX31855_THERMOCOUPLE_UNKNOWN:
        qtklog.warn("thermocouple unknown error");
        break;

      //case MAX31855_THERMOCOUPLE_READ_FAIL:
       // qtklog.print("Thermocouple read error, check chip & spi cable");
        //break;
    }
}

// simple first order low pass filter in Hz
void QTKilnThermo::_filter(float sample) {
  double ts = (millis() - _lowPassFilter.ts) * 1e-3;

  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "sample = %.2f, previous sample = %.2f", sample, _lowPassFilter.prevSample_C);
	
  float a_0 = -(((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz - 1.) 
		  / ((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz + 1.));
  float b_0 = ((ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz) 
	  	/ (1. + (ts / 2.) * 2. * PI * _lowPassFilter.cutoffFrequency_Hz);

  //qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "a_0 = %g b_0 = %g", a_0, b_0);

  _lowPassFilter.filtered_C = a_0 * _lowPassFilter.filtered_C + 
	  b_0 * sample + b_0 * sample;
  _lowPassFilter.prevSample_C = sample;
  //qtklog.debug(QTKLOG_DBG_PRIO_LOW, "sample = %.2f, filtered = %.2f", sample, _lowPassFilter.filtered_C);
  _lowPassFilter.ts = millis();
}

void QTKilnThermo::_doRead(void) {
  //qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "_doRead()");
  if (_max31855) {
    //qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "_max31855 0x%x", _max31855);
    if (_max31855->detectThermocouple(MAX31855_FORCE_READ_DATA) == MAX31855_THERMOCOUPLE_OK) {
      double tmp =_max31855->getTemperature(MAX31855_FORCE_READ_DATA);
      if (tmp != MAX31855_ERROR) {
	qtklog.debug(QTKLOG_DBG_PRIO_LOW, "max31855 = %g", tmp);
	_lastTemp_C = tmp;
    	_lastTime = millis();
      } else {
	_err++;
	_errno = QTKILN_ERRNO_READ_ERROR;
        qtklog.warn("MAX31855 thermocouple read error");
        return;
      }
    } else {
      _err++;
      _errno = QTKILN_ERRNO_MAX31855_NOT_DETECTED;
      qtklog.warn("MAX31855 thermocouple not detected");
      return;
    }
  } else if (_max6675) {
    //qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "max6675 0x%x", _max6675);
    double tmp = _max6675->getTemperature(MAX6675_FORCE_READ_DATA);
    if (tmp != MAX6675_ERROR) {
      qtklog.debug(QTKLOG_DBG_PRIO_LOW, "max6675 = %g", tmp);
      _lastTemp_C = tmp;
      _lastTime = millis();
    } else {
      _err++;
      _errno = QTKILN_ERRNO_READ_ERROR;
      qtklog.warn("MAX6675 thermocouple not responding");
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
