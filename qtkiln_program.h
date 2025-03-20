#ifndef _QTKILN_PROGRAM_H_
#define _QTKILN_PROGRAM_H_

#include "Arduino.h"

#define QTPROGRAM_NAMESPACE "qtkiln_pgm"

#define PGM_TARGET_TEMP "targetTemperature_C"
#define PGM_AFAP "AFAP"
#define PGM_TRANS_WINDOW_MIN "transitionWindow_min"
#define PGM_DWELL_MIN "dwell_min"

struct QTKilnProgramStructStep {
  int16_t targetTemperature_C;
  bool asFastAsPossible;
  unsigned long transitionWindow_ms;
  unsigned long dwell_ms;
};

struct QTKilnProgramStruct {
  uint8_t steps;
  struct QTKilnProgramStructStep *step;
};

class QTKilnProgram
{
  public:
    QTKilnProgram();
    void begin();
    void loop();
    void set(const String &name, const String &program);
    struct QTKilnProgramStruct *get(const String &name);
    String getJSON(const String &name);
    void start();
    void load(const String &name);
    void stop();
    void pause();
    bool isPaused();
    void unPause();

  private:
    struct QTKilnProgramStruct *_parseProgram(const String &program);
    void _saveProgram(const String &name, const String &program);
    bool _verifyProgram(const String &program);
    struct QTKilnProgramStruct *_currentProgram;
    bool _running;
    bool _paused;
    uint8_t _currentStep;
};

#endif
