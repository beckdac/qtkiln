#ifndef _QTKILN_PROGRAM_H_
#define _QTKILN_PROGRAM_H_

#include "Arduino.h"

#define QTPROGRAM_NAMESPACE "qtkiln_pgm"

#define PGM_TARGET_TEMP "tgtT_C"
#define PGM_AFAP "AFAP"
#define PGM_TRANS_WINDOW_MIN "deltaT_min"
#define PGM_DWELL_MIN "dwell_min"

#define QTKILN_PROGRAM_TASK_STACK_SIZE 6144
#define QTKILN_PROGRAM_TASK_PRI tskIDLE_PRIORITY + 4

#define QTKILN_PROGRAM_DEFAULT_UPDATE_INTERVAL_MS 1000

extern "C" void programTaskFunction(void *pvParameter);

struct QTKilnProgramStructStep {
  int16_t targetTemperature_C;
  bool asFastAsPossible;
  unsigned long transitionWindow_ms;
  unsigned long dwell_ms;
};

// limit from preferences key length
#define QTKILN_MAX_PROGRAM_NAME_LEN 15
struct QTKilnProgramStruct {
  char name[QTKILN_MAX_PROGRAM_NAME_LEN];
  uint8_t steps;
  struct QTKilnProgramStructStep *step;
};

class QTKilnProgram
{
  public:
    QTKilnProgram();
    void begin();
    void loop();
    bool set(const String &name, const String &program);
    struct QTKilnProgramStruct *get(const char *name);
    String getJSON(const char *name);
    void start();
    void loadProgram(const char *name);
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
    bool isProgramLoaded(void);
    const char *getLoadedProgramName(void);
    uint8_t getCurrentStep(void);
    uint8_t getCurrentProgramSteps(void);
    bool isDwell(void);
    unsigned long getNextStepChangeTime_ms(void);
    unsigned long getStepStartTime_ms(void);
    uint16_t getStepStartTemp_C();
    char** getProgramNames(uint8_t*);

  private:
    struct QTKilnProgramStruct *_parseProgram(const String &program);
    void _debugPrint(QTKilnProgramStruct *prg); // <-- this belongs on the struct!!
    void _saveProgram(const String &name, const String &program);
    bool _verifyProgram(const String &program);
    struct QTKilnProgramStruct *_currentProgram;
    void _resetProgramVariables(void);  // when a new program is read
    void _resetProgramStepVariables(void);  // for each new step in a program
    uint16_t _updateInterval_ms;	// the delay time in ms between task updates
    bool _running;			// is this program running
    bool _paused;			// is it paused at a certain state
    uint8_t _currentStep;		// current step
    bool _currentAFAPUp = true;		// is our current as fast as possible about going up in T?
    unsigned long _nextStepChangeTime_ms;// time in device ms that we change to the next step
    unsigned long _stepStartTime_ms;	// when we started this step
    float _stepStartTemp_C;		// temp when this step was started
    bool _inDwell = false;		// are we in the dwell state of this step
    TaskHandle_t _taskHandle = NULL;    // for managing the task later
};

#endif
