#ifndef QTKILN_PWM_H
#define QTKILN_PWM_H

#include <Arduino.h>
#include <QuickPID.h>
#include <sTune.h>

#define QTKILN_PWM_TASK_STACK_SIZE 8192
#define QTKILN_PWM_TASK_PRI tskIDLE_PRIORITY + 3

#define QTKILN_PWM_DEFAULT_WINDOW_SIZE 5000


extern "C" void pwmTaskFunction(void *pvParameter);

class QTKilnPWM
{
  public:
    QTKilnPWM(uint16_t windowSize_ms);

    void begin(void);
    unsigned long msSinceLastUpdate(void);
    void enable(void);
    void disable(void);
    bool isEnabled(void);
    void setOutput_ms(uint16_t output_ms);
    uint16_t getOutput_ms(void);
    unsigned long getLastTime(void);
    void thread(void);
    TaskHandle_t getTask(void);
    UBaseType_t getTaskHighWaterMark(void);
    uint16_t getWindowSize_ms(void);
    void setWindowSize_ms(uint16_t windowSize_ms);
    void setTargetTemperature_C(uint16_t targetTemperature_C);
    uint16_t getTargetTemperature_C(void);
    float getDutyCycle(void);
    double getKp(void);
    double getKi(void);
    double getKd(void);
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setTunings(double Kp, double Ki, double Kd);
    void setUpdateInterval_ms(uint16_t updateInterval_ms);
    uint16_t getUpdateInterval_ms(void);
    void resetPID(void);
    void startTuning(void);
    void stopTuning(void);
    bool isTuning(void);
    double getTuningKp(void);
    double getTuningKi(void);
    double getTuningKd(void);

  private:
    uint16_t _updateInterval_ms = 20;	// how often the pid manager will run
    					//	10ms is the SSR-DA response time, so twice is minimum
    unsigned long _windowStartTime = 0; // start of this next PWM window
    uint16_t _windowSize_ms = 5000;	// the size of the PWM frequency window in ms
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    uint16_t _output_ms = 0;		// where in the frequency window the high state should turn off
    QuickPID *_pid;			// PID control object
    uint16_t _targetTemperature_C = 0;	// target temperature for PID
    float _Kp, _Ki, _Kd;		// internal variables for linking tuner to PID controller
    float _input, _output, _setpoint_flt_C;
                                        // internal variables for PID control
    struct QTKilnPWM_tuning {
      bool enabled = false;             // are we in auto tune mode
      sTune *tuner;			// PID tuner
      // sTune settings
      uint32_t settleTimeSec = 200;	// 40 samples
      uint16_t samples = 500;
      uint32_t testTimeSec = 500*5;  // runPid interval = testTimeSec / samples
      float inputSpan = 1100;
      float outputSpan = QTKILN_PWM_DEFAULT_WINDOW_SIZE;
      float outputStart = 0;
      float outputStep = 50;
      float tempLimit = 450;
      // 
    } _tuning;
    TaskHandle_t _taskHandle = NULL;	// for managing the task later
};

#endif
