#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include "qtkiln.h"
#include "qtkiln_log.h"
#include "qtkiln_program.h"
#include "qtkiln_pwm.h"
#include "qtkiln_thermo.h"

extern Preferences preferences;
extern QTKilnLog qtklog;
extern QTKilnPWM pwm;
extern QTKilnThermo *kiln_thermo, *housing_thermo;

// use a static function to be the entry point for the task
void programTaskFunction(void *pvParameter) {
  QTKilnProgram *program = static_cast<QTKilnProgram *>(pvParameter);

  program->thread();
}

QTKilnProgram::QTKilnProgram(void) {
  _updateInterval_ms = 1000;
  _running = false;
  _paused = false;
  _currentProgram = NULL;
  _currentStep = 0;
  _taskHandle = NULL;
}

void QTKilnProgram::begin(void) {
  // put this task on cpu 1 which is where everything else is for simplicity
  BaseType_t rc = xTaskCreatePinnedToCore(programTaskFunction, "program",
                  QTKILN_PROGRAM_TASK_STACK_SIZE, (void *)this, QTKILN_PROGRAM_TASK_PRI,
                  &_taskHandle, QTKILN_TASK_CORE);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for program");
}

void QTKilnProgram::_resetProgramStepVariables(void) {
  _currentAFAPUp = true; // always start trying to increase the temperature
  _inDwell = false;
  _stepStartTime_ms = millis();
}

void QTKilnProgram::_resetProgramVariables(void) {
  _currentStep = 0;
  _running = false;
  _paused = false;
  _resetProgramStepVariables();
}

void QTKilnProgram::thread(void) {
  TickType_t xDelay;
  unsigned int now;

  while (1) {
    if (_running) {
      qtklog.debug(0, "running program %s at step %d", _currentProgram->name, _currentStep);
      // check to make sure that the data structures are reasonable intact
      // before doing anything
      if (!_currentProgram) {
	qtklog.warn("program main thread called after program ended");
	_resetProgramVariables();
      } else if (_currentStep >= _currentProgram->steps) {
	qtklog.warn("program main thread called after program ended");
	_resetProgramVariables();
      }
      // running and not paused
      if (!_paused) {
	now = millis();
	float nowTemp = kiln_thermo->getFilteredTemperature_C();
	// check if we need to move to the next step
	// this can happen if we are at the next time start, or
	// in fast as possible and at target temperature
	bool metAFAPConditions = false;
	// if we are in the dwell period there is no afap
	if (_inDwell) {
	  metAFAPConditions = false;
	// if we are in AFAP
        } else if (_currentProgram->step[_currentStep].asFastAsPossible) {
	  // and temperature is in up mode
	  if (_currentAFAPUp)
	    metAFAPConditions = nowTemp > _currentProgram->step[_currentStep].targetTemperature_C;
	  else // down mode
	    metAFAPConditions = nowTemp < _currentProgram->step[_currentStep].targetTemperature_C;
	  // if we have met the temperature conditions and there is a dwell time set
	  // transition to dwell mode
	  if (metAFAPConditions && _currentProgram->step[_currentStep].dwell_ms > 0) {
	     // this is when we need to move to the next step
	     _nextStepChangeTime_ms = now + _currentProgram->step[_currentStep].dwell_ms;
	     // make sure to signal we are in dwell mode so that AFAP is no longer relevant
	     _inDwell = true;
	     // AFAP is no longer applicable for this pass through the loop
	     metAFAPConditions = false;
	     // finalize the temperature setting at the set point
	     pwm.setTargetTemperature_C(_currentProgram->step[_currentStep].targetTemperature_C);
	  }
	}
	// ready for a step change?
	if (now >= _nextStepChangeTime_ms || metAFAPConditions) {
          qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "changing step to %d", _currentStep + 1);
	  // if we have reached the end of the program stop execution
	  if (_currentStep + 1 >= _currentProgram->steps) {
            qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "end of program reached, calling stop");
	    stop();
	  } else {
	    // increment the step counter
	    _currentStep++;
	    _resetProgramStepVariables(); // sets, for example: _stepStartTime_ms
	    _stepStartTemp_C = kiln_thermo->getFilteredTemperature_C();
	    // is the next time change in AFAP
            if (_currentProgram->step[_currentStep].asFastAsPossible) {
	      _nextStepChangeTime_ms = ULONG_MAX; // end of time for this micro
	      if (_currentProgram->step[_currentStep].targetTemperature_C >
		    _currentProgram->step[_currentStep-1].targetTemperature_C)
	        _currentAFAPUp = true;
              else
	        _currentAFAPUp = false;
	      // set temperature
	      pwm.setTargetTemperature_C(_currentProgram->step[_currentStep].targetTemperature_C);
            } else {
	      // calculate the next update interval which is the transition time 
	      // plus the dwell time
	      _nextStepChangeTime_ms = now + 
	          _currentProgram->step[_currentStep].transitionWindow_ms + 
		  _currentProgram->step[_currentStep].dwell_ms;
	    }
	  }
	}
	if (!_inDwell) {
	  // update the temperature for this step
	  unsigned long now = millis();
	  // time since the step start
	  unsigned long deltaT_ms = now - _stepStartTime_ms;
	  // temperature change over the transition window for this step
	  float deltaTemp_C = _currentProgram->step[_currentStep].targetTemperature_C - _stepStartTemp_C;
	  // degree change in C per millisecond, not data type
	  float dTms = deltaTemp_C / _currentProgram->step[_currentStep].transitionWindow_ms;
	  // new temperature from y = mx+b
	  float newT_C = _stepStartTemp_C + (dTms * deltaT_ms);
	  qtklog.debug(QTKLOG_DBG_PRIO_LOW, "running program %s changing set point to %g", _currentProgram->name, newT_C);
	  pwm.setTargetTemperature_C(newT_C);
	} else {
	  // redundant
	  // pwm.setTargetTemperature_C(_currentProgram->step[_currentStep].targetTemperature_C);
	}
      }
    }
    xDelay = pdMS_TO_TICKS(_updateInterval_ms);
    vTaskDelay(xDelay);
  }
}

TaskHandle_t QTKilnProgram::getTask(void) {
  return _taskHandle;
}

UBaseType_t QTKilnProgram::getTaskHighWaterMark(void) {
  if (_taskHandle)
    return uxTaskGetStackHighWaterMark(_taskHandle);
  qtklog.warn("no task associated with qtkiln program in high watermark test");
  return 0;
}

struct QTKilnProgramStruct *QTKilnProgram::_parseProgram(const String &program) {
  JsonDocument doc;

  qtklog.debug(0, "parsing program from JSON: %s", program.c_str());

  DeserializationError error = deserializeJson(doc, program);

  if (error) {
    qtklog.warn("unable to parse JSON: %s", program.c_str());
    qtklog.warn("the following error occurred: %s", error.c_str());
    return NULL;
  }

  // make sure there is a name entity and it is valid
  if (!doc["name"].is<const char *>()) {
    qtklog.warn("unable to find the program name in the program");
    return NULL;
  }
  const char *name = doc["name"] | "";
  if (strlen(name) > QTKILN_MAX_PROGRAM_NAME_LEN) {
    qtklog.warn("program name %s is too long, max length is %d characters", name, QTKILN_MAX_PROGRAM_NAME_LEN);
    return NULL;
  }
  // find the number of steps so they can be allocated
  uint8_t steps = doc["steps"] | 0;
  if (!steps) {
    qtklog.warn("unable to find the number of steps in the program");
    return NULL;
  }
  QTKilnProgramStruct *new_program = (QTKilnProgramStruct *)malloc(sizeof(struct QTKilnProgramStruct));
  strncpy(new_program->name, name, QTKILN_MAX_PROGRAM_NAME_LEN);
  new_program->steps = steps;
  new_program->step = (QTKilnProgramStructStep *)malloc(sizeof(struct QTKilnProgramStructStep) * steps);
  memset(new_program->step, 0, sizeof(struct QTKilnProgramStructStep));
  // parse the individual steps out of the document
  for (int i = 0; i < steps; ++i) {
    // find the target temperature for this step
    if (!doc["step"][i][PGM_TARGET_TEMP].is<uint16_t>()){
      qtklog.warn("failed to parse program at step %d because the target temperature was missing", i);
      free(new_program->step);
      free(new_program);
      return NULL;
    }
    new_program->step[i].targetTemperature_C = doc["step"][i][PGM_TARGET_TEMP];
    // set up the transition time
    if (!doc["step"][i][PGM_TRANS_WINDOW_MIN].is<uint16_t>()){
      new_program->step[i].asFastAsPossible = doc["step"][i][PGM_AFAP] | false;
      // no transition window time and not AFAP, something is wrong
      if (!new_program->step[i].asFastAsPossible) {
	qtklog.warn("couldn't find transition time or As Fast As Possible flag parsing step %d", i);
	free(new_program->step);
	free(new_program);
	return NULL;
      }
      new_program->step[i].transitionWindow_ms = 0;
    } else {
      new_program->step[i].transitionWindow_ms = doc["step"][i][PGM_TRANS_WINDOW_MIN] | 0;
      new_program->step[i].transitionWindow_ms *= 1000 * 60; // convert min to ms
      new_program->step[i].asFastAsPossible = doc["step"][i][PGM_AFAP] | false;
      if (new_program->step[i].asFastAsPossible) {
	qtklog.warn("transition time window and as fast as possible can't both be specificed for step %d", i);
	free(new_program->step);
	free(new_program);
	return NULL;
      }
    }
    new_program->step[i].dwell_ms = doc["step"][i][PGM_DWELL_MIN] | 0;
    new_program->step[i].dwell_ms *= 1000 * 60; // convert min to ms
  }
  _debugPrint(new_program);
  return new_program;
}

bool QTKilnProgram::isProgramLoaded(void) {
  return (_currentProgram != NULL);
}

const char *QTKilnProgram::getLoadedProgramName(void) {
  return _currentProgram->name;
}

void QTKilnProgram::_debugPrint(QTKilnProgramStruct *prg) {
  if (!prg)
    qtklog.debug(0, "program structure missing when calling debugPrint of program");
  qtklog.debug(0, "program has %d steps", prg->steps);
  qtklog.debug(0, "step  %10s  %10s  %10s  %10s", PGM_TARGET_TEMP, PGM_AFAP, PGM_TRANS_WINDOW_MIN, PGM_DWELL_MIN);
  for (uint16_t i = 0; i < prg->steps; ++i) {
    qtklog.debug(0, "%4d  %10d  %10d  %10d  %10d", i, prg->step[i].targetTemperature_C, prg->step[i].asFastAsPossible, prg->step[i].transitionWindow_ms / 1000 / 60, prg->step[i].dwell_ms / 1000 / 60);
  }
}

void QTKilnProgram::_saveProgram(const String &name, const String &program) {
  preferences.begin(QTPROGRAM_NAMESPACE, false);
  preferences.putString(name.c_str(), program);
  preferences.end();
}

bool QTKilnProgram::set(const String &name, const String &program){
  struct QTKilnProgramStruct *validProgram;
  validProgram = _parseProgram(program);
  if (validProgram) {
    qtklog.print("saving program %s to memory with contents %s", name.c_str(), program.c_str());
    _saveProgram(name, program);
  } else {
    qtklog.warn("unabled to save program %s because it failed verification", name.c_str());
    return false;
  }
  return true;
}

String QTKilnProgram::getJSON(const char *name) {
  String program;

  preferences.begin(QTPROGRAM_NAMESPACE, true);
  if (preferences.isKey(name))
    program = preferences.getString(name);
  else {
    qtklog.warn("unable to find program in storage named %s", name);
  }
  preferences.end();

  return program;
}

struct QTKilnProgramStruct *QTKilnProgram::get(const char *name) {
  struct QTKilnProgramStruct *validProgram = NULL;
  String program;

  program = getJSON(name);
  if (program.length() > 2) // at least an open and close brace
    validProgram = _parseProgram(program);
  return validProgram;
}

void QTKilnProgram::loadProgram(const char *name) {
  struct QTKilnProgramStruct *program = NULL;

  qtklog.print("loading program %s from memory", name);

  program = get(name);
  if (!program) {
    qtklog.print("failed to load program %s", name);
    return;
  }
  if (_currentProgram) {
    free(_currentProgram->step);
    free(_currentProgram);
  }
  _currentProgram = program;
}

void QTKilnProgram::start(void) {
  if (!_running) {
    qtklog.print("starting program execution");
    _running = true;
    _currentStep = 0;
    _resetProgramVariables();
    _resetProgramStepVariables();
    _stepStartTemp_C = kiln_thermo->getFilteredTemperature_C();
    _nextStepChangeTime_ms = millis() + 
	          _currentProgram->step[_currentStep].transitionWindow_ms + 
		  _currentProgram->step[_currentStep].dwell_ms;
    pwm.enable();
  }
}

void QTKilnProgram::stop(void) {
  if (_running) {
    qtklog.print("stopping program execution");
    _running = false;
    _currentStep = 0;
    _resetProgramStepVariables();
    _resetProgramVariables();
    pwm.disable();
    ssr_off();
  }
}

bool QTKilnProgram::isPaused(void) {
  if (_paused)
    return true;
  return false;
}

void QTKilnProgram::pause(void) {
  if (_running) {
    qtklog.print("pausing program execution at step %d", _currentStep);
    _paused = true;
  }
}

void QTKilnProgram::unPause(void) {
  if (_running) {
    qtklog.print("unpausing program execution at step %d", _currentStep);
    _paused = true;
  }
}

void QTKilnProgram::setUpdateInterval_ms(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
  qtklog.debug(0, "program update interval modified to %d ms", _updateInterval_ms);
}

uint16_t QTKilnProgram::getUpdateInterval_ms(void) {
  return _updateInterval_ms;
}

bool QTKilnProgram::isRunning(void) {
  return _running;
}
