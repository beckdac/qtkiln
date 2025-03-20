#ifndef QTKILN_PWM_H
#define QTKILN_PWM_H

#include "Arduino.h"
#include "PID_v2.h"

#define QTKILN_PWM_TASK_STACK_SIZE 2048
#define QTKILN_PWM_TASK_PRI tskIDLE_PRIORITY + 2

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
    uint16_t getUpdateInterval_ms(void);
    void setUpdateInterval_ms(uint16_t windowSize_ms);
    void setTargetTemperature_C(uint16_t targetTemperature_C);
    uint16_t getTargetTemperature_C(void);
    float getDutyCycle(void);
    double getKp(void);
    double getKi(void);
    double getKd(void);
    void setKp(double);
    void setKi(double);
    void setKd(double);

  private:
    unsigned long _windowStartTime = 0; // start of this next PWM window
    uint16_t _windowSize_ms = 5000;	// the size of the PWM frequency window in ms
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    uint16_t _output_ms = 0;		// where in the frequency window the high state should turn off
    PID_v2 *_pid;			// PID control object
    uint16_t _targetTemperature_C = 0;	// target temperature for PID
    TaskHandle_t _taskHandle = NULL;	// for managing the task later
};

#endif
