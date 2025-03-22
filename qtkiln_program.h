#ifndef _QTKILN_PROGRAM_H_
#define _QTKILN_PROGRAM_H_

#include "Arduino.h"

#define QTPROGRAM_NAMESPACE "qtkiln_pgm"

#define PGM_TARGET_TEMP "tgtT_C"
#define PGM_AFAP "AFAP"
#define PGM_TRANS_WINDOW_MIN "deltaT_min"
#define PGM_DWELL_MIN "dwell_min"

#define QTKILN_PROGRAM_TASK_STACK_SIZE 2048
#define QTKILN_PROGRAM_TASK_PRI tskIDLE_PRIORITY + 3

#define QTKILN_PROGRAM_DEFAULT_UPDATE_INTERVAL_MS 1000

extern "C" void programTaskFunction(void *pvParameter);

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
    bool isRunning();
    bool isPaused();
    void unPause();
    void thread(void);
    TaskHandle_t getTask(void);
    UBaseType_t getTaskHighWaterMark(void);
    void setUpdateInterval_ms(uint16_t updateInterval_ms);
    uint16_t getUpdateInterval_ms(void);

  private:
    struct QTKilnProgramStruct *_parseProgram(const String &program);
    void _debugPrint(QTKilnProgramStruct *prg); // <-- this belongs on the struct!!
    void _saveProgram(const String &name, const String &program);
    bool _verifyProgram(const String &program);
    struct QTKilnProgramStruct *_currentProgram;
    uint16_t _updateInterval_ms;	// the delay time in ms between task updates
    bool _running;			// is this program running
    bool _paused;			// is it paused at a certain state
    uint8_t _currentStep;		// current step
    TaskHandle_t _taskHandle = NULL;    // for managing the task later
};

#endif
