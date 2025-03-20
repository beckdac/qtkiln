#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include "qtkiln.h"
#include "qtkiln_log.h"
#include "qtkiln_program.h"

extern Preferences preferences;
extern QTKilnLog qtklog;

QTKilnProgram::QTKilnProgram(void) {
  _running = false;
  _paused = false;
  _currentProgram = NULL;
  _currentStep = 0;
}

void QTKilnProgram::begin(void) {
  qtklog.print("program runner initialized");
}

void QTKilnProgram::loop(void) {
  if (_running) {
    if (!_paused) {
    }
  }
}

struct QTKilnProgramStruct *QTKilnProgram::_parseProgram(const String &program) {
  JsonDocument doc;

  qtklog.debug(10, "parsing program from JSON: %s", program.c_str());

  DeserializationError error = deserializeJson(doc, program);

  if (error) {
    qtklog.warn("unable to parse JSON: %s", program.c_str());
    qtklog.warn("the following error occurred: %s", error.c_str());
    return NULL;
  }
  uint8_t steps = doc["steps"] | 0;
  if (!steps) {
    qtklog.warn("unable to find the number of steps in the program");
    return NULL;
  }
  QTKilnProgramStruct *new_program = (QTKilnProgramStruct *)malloc(sizeof(struct QTKilnProgramStruct));
  new_program->steps = steps;
  new_program->step = (QTKilnProgramStructStep *)malloc(sizeof(struct QTKilnProgramStructStep) * steps);
  // parse the individual steps out of the document
  for (int i = 0; i < steps; ++i) {
    memset(new_program->step, 0, sizeof(struct QTKilnProgramStructStep));
    // find the target temperature for this step
    JsonVariant tmpObj = doc[i + 1][PGM_TARGET_TEMP];
    if (tmpObj.isNull()) {
      qtklog.warn("failed to parse program at step %d because the target temperature was missing", i+1);
      free(new_program->step);
      free(new_program);
      return NULL;
    }
    new_program->step[i].targetTemperature_C = doc[i + 1][PGM_TARGET_TEMP];
    // set up the transition time
    tmpObj = doc[i + 1][PGM_TRANS_WINDOW_MIN];
    if (tmpObj.isNull()) {
      new_program->step[i].asFastAsPossible = doc[i + 1][PGM_AFAP] | false;
      // no transition window time and not AFAP, something is wrong
      if (!new_program->step[i].asFastAsPossible) {
	qtklog.warn("couldn't find transition time or As Fast As Possible flag parsing step %d", i+1);
	free(new_program->step);
	free(new_program);
	return NULL;
      }
    } else {
      new_program->step[i].transitionWindow_ms = (doc[i + 1][PGM_TRANS_WINDOW_MIN]);
      new_program->step[i].transitionWindow_ms *= 1000 * 60; // convert min to ms
      new_program->step[i].asFastAsPossible = doc[i + 1][PGM_AFAP] | false;
      if (new_program->step[i].asFastAsPossible) {
	qtklog.warn("transition time window and as fast as possible can't both be specificed for step %d", i+1);
	free(new_program->step);
	free(new_program);
	return NULL;
      }
    }
    new_program->step[i].dwell_ms = doc[PGM_DWELL_MIN] | 0;
    new_program->step[i].dwell_ms *= 1000 * 60; // convert min to ms
  }
  return new_program;
}

void QTKilnProgram::_saveProgram(const String &name, const String &program) {
  preferences.begin(QTPROGRAM_NAMESPACE, false);
  preferences.putString(name.c_str(), program);
  preferences.end();
}

void QTKilnProgram::set(const String &name, const String &program){
  struct QTKilnProgramStruct *validProgram;
  validProgram = _parseProgram(program);
  if (validProgram) {
    qtklog.print("saving program %s to memory with contents %s", name.c_str(), program.c_str());
    _saveProgram(name, program);
  } else {
    qtklog.warn("unabled to save program %s because it failed verification", name);
  }
}

String QTKilnProgram::getJSON(const String &name) {
  String program;

  preferences.begin(QTPROGRAM_NAMESPACE, true);
  if (preferences.isKey(name.c_str()))
    program = preferences.getString(name.c_str());
  else {
    qtklog.warn("unable to find program in storage named %s", name.c_str());
  }
  preferences.end();

  return program;
}

struct QTKilnProgramStruct *QTKilnProgram::get(const String &name) {
  struct QTKilnProgramStruct *validProgram = NULL;
  String program;

  program = getJSON(name);
  if (program.length() > 2) // at least an open and close brace
    validProgram = _parseProgram(program);
  return validProgram;
}

void QTKilnProgram::load(const String &name) {
  struct QTKilnProgramStruct *program = NULL;

  qtklog.print("loading program %s from memory", name.c_str());

  program = get(name);
  if (!program) {
    qtklog.print("failed to load program %s", name.c_str());
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
  }
}

void QTKilnProgram::stop(void) {
  if (_running) {
    qtklog.print("stopping program execution");
    _running = false;
    _currentStep = 0;
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
