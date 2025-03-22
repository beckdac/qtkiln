#include "qtkiln_pwm.h"
#include "qtkiln_thermo.h"
#include "qtkiln_log.h"
#include "qtkiln_program.h"
#include "qtkiln.h"

#include <PID_v2.h>

extern Config config;
extern QTKilnLog qtklog;
extern QTKilnThermo *kiln_thermo;
extern QTKilnThermo *housing_thermo;
extern QTKilnProgram program;

// use a static function to be the entry point for the task
void pwmPWMFunction(void *pvParameter) {
  QTKilnPWM *pwm = static_cast<QTKilnPWM *>(pvParameter);

  pwm->thread();
}

QTKilnPWM::QTKilnPWM(uint16_t windowSize_ms) {
  _windowSize_ms = windowSize_ms;
  _lastTime = 0;
  _taskHandle = NULL;
}

void QTKilnPWM::setWindowSize_ms(uint16_t windowSize_ms) {
  _windowSize_ms = windowSize_ms;
  _pid->SetOutputLimits(0, _windowSize_ms);
  qtklog.debug(0, "pwm update interval modified to %d ms", _windowSize_ms);
}

uint16_t QTKilnPWM::getWindowSize_ms(void) {
  return _windowSize_ms;
}

void QTKilnPWM::begin(void) {
  qtklog.print("intializing new pid/pwm with window size %d", _windowSize_ms);
  _pid = new PID_v2(config.Kp, config.Ki, config.Kd, PID::Direct);
  _pid->SetOutputLimits(0, config.pwmWindow_ms);

  BaseType_t rc = xTaskCreatePinnedToCore(pwmPWMFunction, "pwm",
		  QTKILN_PWM_TASK_STACK_SIZE, (void *)this, QTKILN_PWM_TASK_PRI,
		  &_taskHandle, QTKILN_TASK_CORE);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for pwm");
}

TaskHandle_t QTKilnPWM::getTask(void) {
  return _taskHandle;
}

void QTKilnPWM::enable(void) {
  bool wasEnabled = _enabled;
  _enabled = true;
  if (!wasEnabled) {
    _pid->SetTunings(config.Kp, config.Ki, config.Kd);
    _pid->SetSampleTime(_windowSize_ms);
    _pid->Start(kiln_thermo->getTemperature_C(), 0, _targetTemperature_C);
    _windowStartTime = millis();
    qtklog.debug(0, "PID is being enabled for the PWM task");
  }
}

void QTKilnPWM::disable(void) {
  bool wasEnabled = _enabled;
  _enabled = false;
  if (wasEnabled) {
    ssr_off();
    qtklog.debug(0, "PID is being disabled for the PWM task");
  }
}

bool QTKilnPWM::isEnabled(void) {
  return _enabled;
}

uint16_t QTKilnPWM::getTargetTemperature_C() {
  return _targetTemperature_C;
}

void QTKilnPWM::setTargetTemperature_C(uint16_t targetTemperature_C) {
  _targetTemperature_C = targetTemperature_C;
}

unsigned long QTKilnPWM::msSinceLastUpdate(void) {
  unsigned long now = millis();
  return now - _lastTime;
}

unsigned long QTKilnPWM::getLastTime(void) {
  return _lastTime;
}

float QTKilnPWM::getDutyCycle(void) {
  return 100. * (float)_output_ms / (float)_windowSize_ms;
}

void QTKilnPWM::thread(void) {
  TickType_t xDelay;
  double tmp;
  unsigned long now;

  while (1) {
    if (_enabled && _pid) {
      now = millis();
      while (now - _windowStartTime > _windowSize_ms) {
        _windowStartTime += _windowSize_ms;
      }
      tmp = _pid->Run(kiln_thermo->getTemperature_C());
      if (isnan(tmp)) {
         qtklog.warn("nan detected for output of PID, disabling pid");
         disable();
	 qtklog.warn("trying to shutdown any running program from pwm loop");
	 if (program.isRunning()) {
	   program.stop();
	 }
      }
      _output_ms = tmp;
      if (now - _windowStartTime < _output_ms)
        ssr_on();
      else
        ssr_off();
    }
    xDelay = pdMS_TO_TICKS(_updateInterval_ms);
    vTaskDelay(xDelay);
  }
}

UBaseType_t QTKilnPWM::getTaskHighWaterMark(void) {
  if (_taskHandle)
    return uxTaskGetStackHighWaterMark(_taskHandle);
  qtklog.warn("no task associated with qtkiln pwm in high watermark test");
  return 0;
}

double QTKilnPWM::getKp(void) {
  return _pid->GetKp();
}

double QTKilnPWM::getKi(void) {
  return _pid->GetKi();
}

double QTKilnPWM::getKd(void) {
  return _pid->GetKd();
}

void QTKilnPWM::setTunings(double Kp, double Ki, double Kd) {
  _pid->SetTunings(Kp, Ki, Kd);
}

void QTKilnPWM::setUpdateInterval_ms(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
}

uint16_t QTKilnPWM::getUpdateInterval_ms(void) {
  return _updateInterval_ms;
}
