#include <Arduino.h>
#include <ArduinoJson.h>

#include "qtkiln_program.h"


QTKilnProgram::QTKilnProgram(void) {
}

void QTKilnProgram::begin(void) {
  Serial.println("program runner initialized");
}

void QTKilnProgram::loop(void) {
  if (_running) {
  }
}

bool QTKilnProgram::_verifyProgram(const char *program) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    Serial.print("unable to parse program: ");
    Serial.println(program);
    Serial.print("the following error occurred: ");
    Serial.println(error.f_str());
    return;
  }
}

void QTKilnProgram::set(const char *name, const char *program){
  bool validProgram = _verifyProgram(program);
  if (validProgram) {
    Serial.print("saving program ");
    Serial.print(name);
    Serial.println(" to memory");
  } else {
    Serial.print("unabled to save program ");
    Serial.print(name);
    Serial.println(" to memory because it failed verification");
  }
}

struct QTKilnProgramStruct *QTKilnProgram::get(const char *name) {
}

void QTKilnProgram::run(struct QTKilnProgramStruct *program) {
}

void QTKilnProgram::start(void) {
}

void QTKilnProgram::end(void) {
}

void QTKilnProgram::pause(void) {
}
