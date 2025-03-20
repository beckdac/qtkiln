#include "qtkiln_thermo.h"
#include "qtkiln_log.h"

extern QTKilnLog qtklog;

// use a static function to be the entry point for the task
void thermoTaskFunction(void *pvParameter) {
  QTKilnThermo *thermo = static_cast<QTKilnThermo *>(pvParameter);

  thermo->thread();
}

QTKilnThermo::QTKilnThermo(uint16_t interval_ms, MAX31855 *max31855, MAX6675 *max6675) {
  _interval_ms = interval_ms;
  _max31855 = max31855;
  _max6675 = max6675;
  _lastTime = 0;
  _lastTempC = -1;
  _taskHandle = NULL;
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
    _err = true;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }

  BaseType_t rc = xTaskCreate(thermoTaskFunction, (_max31855 ? "max31855" : "max65675"),
		  QTKILN_THERMO_TASK_STACK_SIZE, (void *)this, QTKILN_THERMO_TASK_PRI,
		  &_taskHandle);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for %s", (_max31855 ? "max31855" : "max65675"));
}

TaskHandle_t QTKilnThermo::getTask(void) {
  return _taskHandle;
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

void QTKilnThermo::_doRead(void) {
  if (_max31855) {
    if (_max31855->detectThermocouple(MAX31855_FORCE_READ_DATA) == MAX31855_THERMOCOUPLE_OK) {
      double tmp =_max31855->getTemperature(MAX31855_FORCE_READ_DATA);
      if (tmp != MAX31855_ERROR) {
	qtklog.debug(QTKLOG_DBG_PRIO_LOW, "max31855 = %g", tmp);
	_lastTempC = tmp;
    	_lastTime = millis();
      } else {
	_err = true;
	_errno = QTKILN_ERRNO_READ_ERROR;
        qtklog.warn("MAX31855 thermocouple read error");
      }
    } else {
      _err = true;
      _errno = QTKILN_ERRNO_MAX31855_NOT_DETECTED;
      qtklog.warn("MAX31855 thermocouple not detected");
    }
  } else if (_max6675) {
    double tmp = _max6675->getTemperature(MAX6675_FORCE_READ_DATA);
    if (tmp != MAX6675_ERROR) {
      qtklog.debug(QTKLOG_DBG_PRIO_LOW, "max6675 = %g", tmp);
      _lastTempC = tmp;
      _lastTime = millis();
    } else {
      _err = true;
      _errno = QTKILN_ERRNO_READ_ERROR;
      qtklog.warn("MAX6675 thermocouple not responding");
    }
  } else {
    _err = true;
    _errno = QTKILN_ERRNO_INVALID_TYPE;
  }
}

void QTKilnThermo::thread(void) {
  TickType_t xDelay;

  while (1) {
    if (_enabled) {
      _doRead();
    }
    xDelay = pdMS_TO_TICKS(_interval_ms);
    vTaskDelay(xDelay);
  }
}

UBaseType_t QTKilnThermo::getTaskHighWaterMark(void) {
  if (_taskHandle)
    return uxTaskGetStackHighWaterMark(_taskHandle);
  qtklog.warn("no task associated with qtkiln thermo in high watermark test");
  return 0;
}
