#include "qtkiln_pwm.h"
#include "qtkiln_thermo.h"
#include "qtkiln_log.h"
#include "qtkiln_program.h"
#include "qtkiln.h"

#include <QuickPID.h>
#include <sTune.h>

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
  _input = 0;
  _output = 0;
  _tuning.tuner = NULL;
  _tuning.enabled = false;
  _targetTemperature_C = 0;
  _setpoint_flt_C = 0;
  _taskHandle = NULL;
}

void QTKilnPWM::setWindowSize_ms(uint16_t windowSize_ms) {
  _windowSize_ms = windowSize_ms;
  _pid->SetOutputLimits(0, _windowSize_ms);
  _pid->SetSampleTimeUs(_windowSize_ms * 1000);
  qtklog.debug(0, "pwm update interval modified to %d ms", _windowSize_ms);
}

uint16_t QTKilnPWM::getWindowSize_ms(void) {
  return _windowSize_ms;
}

void QTKilnPWM::begin(void) {
  qtklog.print("intializing new pid/pwm with window size %d", _windowSize_ms);
  _pid = new QuickPID(&_input, &_output, &_setpoint_flt_C, config.Kp, config.Ki, config.Kd, 
		  QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp,
		  QuickPID::Action::direct);
  _pid->SetOutputLimits(0, _windowSize_ms);
  _pid->SetSampleTimeUs(_windowSize_ms * 1000);

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
    _input = kiln_thermo->getFilteredTemperature_C();
    _setpoint_flt_C = _targetTemperature_C;
    _pid->SetMode(QuickPID::Control::automatic);
    //_pid->SetProportionalMode(QuickPID::pMode::pOnMeas);
    _pid->Initialize();
    //_pid->Compute();
    _windowStartTime = millis();
    qtklog.debug(0, "PID is being enabled for the PWM task");
  }
}

void QTKilnPWM::resetPID(void) {
  if (_enabled) {
    qtklog.warn("cannot reset the PID parameters while control is enabled");
    return;
  }
  if (_pid)
    _pid->Reset();
}

void QTKilnPWM::disable(void) {
  bool wasEnabled = _enabled;
  _enabled = false;
  if (wasEnabled) {
    _pid->SetMode(QuickPID::Control::manual);
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
  _setpoint_flt_C = _targetTemperature_C;
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

uint16_t QTKilnPWM::getOutput_ms(void) {
  return _output_ms;
}

void QTKilnPWM::thread(void) {
  TickType_t xDelay;
  unsigned long now;

  while (1) {
    if (_enabled && _pid) {
      if (kiln_thermo)
        _input = kiln_thermo->getFilteredTemperature_C();
      else
	qtklog.warn("kiln_thermo not available in the pwm main loop");
      now = millis();
      while (now - _windowStartTime > _windowSize_ms) {
        _windowStartTime += _windowSize_ms;
      }
      if (!_tuning.enabled) {
	_pid->Compute(); // most of these will be noop with false return
      } else {
	switch (_tuning.tuner->Run()) {
          case sTune::TunerStatus::sample:  // once per sample during test
	    _input = kiln_thermo->getFilteredTemperature_C(); // already done above
	    break;
	  case sTune::TunerStatus::tunings: // done when the tuning is complete
	    _tuning.tuner->GetAutoTunings(&_Kp, &_Ki, &_Kd);
	    qtklog.print("PID auto tuning complete and Kp = %g, Ki = %g, Kd = %d", _Kp, _Ki, _Kd);
	    _pid->Reset();
	    qtklog.print("updating configuration with initial tunings, consider saving");
            config_setPidInitialKp(_Kp);
	    config_setPidInitialKi(_Ki);
            config_setPidInitialKd(_Kd);
	    qtklog.print("returning control to PID with new tunings");
	    _pid->SetMode(QuickPID::Control::automatic);
	    _pid->SetProportionalMode(QuickPID::pMode::pOnMeas);
	    _pid->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
	    _pid->SetTunings(_Kp, _Ki, _Kd);
	    break;
	  case sTune::TunerStatus::runPid: // once per sample after tuning
	    _pid->Compute(); // most of these will be noop with false return
	    break;
	}
      }
      // compute will update _output
      // check to make sure we haven't lost numerical stability
      if (isnan(_output)) {
         qtklog.warn("nan detected for output of PID, disabling pid");
	 if (_tuning.enabled)
	   stopTuning();
         if (isEnabled())
	   disable();
	 qtklog.warn("trying to shutdown any running program from pwm loop");
	 if (program.isRunning()) {
	   program.stop();
	 }
      }
      // can't be lower than 0; this should really be min of update_ms
      // update the internal state variable version
      if (_output < 0) {
	_output_ms = 0;
	_output = 0;
      } else
	_output_ms = _output + 0.5; // rounding via 0.5
      // decide if we should be turning on the ssr
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

void QTKilnPWM::startTuning(void) {
  if (!_pid) {
    qtklog.warn("PID tuner invoked with no PID controller available");
    return;
  }
  if (getTargetTemperature_C() == 0) {
    qtklog.warn("current target temperature is 0, tuning not available");
    return;
  }
  // create a new tuner
  if (!_tuning.tuner)
    _tuning.tuner = new sTune(&_input, &_output, sTune::TuningMethod::ZN_PID, 
                        sTune::Action::directIP, sTune::SerialMode::printOFF);
#if 0
  // per the sTune instructions
  _pid->SetMode(QuickPID::Control::manual); // but I don't see this in the example
#endif
  _tuning.outputSpan = _windowSize_ms;
  _tuning.tuner->Configure(_tuning.inputSpan, _tuning.outputSpan, 
                           _tuning.outputStart, _tuning.outputStep,
		           _tuning.testTimeSec, _tuning.settleTimeSec,
		           _tuning.samples);
  _tuning.tuner->SetEmergencyStop(_tuning.tempLimit);

  enable();
  _tuning.enabled = true;
}

void QTKilnPWM::stopTuning(void) {
  disable();
  _tuning.enabled = false;
}

bool QTKilnPWM::isTuning(void) {
  return _tuning.enabled;
}
