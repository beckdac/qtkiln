#ifndef QTKILN_MQTT_H
#define QTKILN_MQTT_H

#include "Arduino.h"
#include "EspMQTTClient.h"

#define QTKILN_MQTT_TASK_STACK_SIZE 5120
#define QTKILN_MQTT_TASK_PRI tskIDLE_PRIORITY + 2

#define QTKILN_MQTT_DEFAULT_UPDATE_INTERVAL_MS 1000
#define QTKILN_MQTT_DEEP_STATE_UPDATE_COUNT 100

extern "C" void mqttTaskFunction(void *pvParameter);

class QTKilnMQTT
{
  public:
    QTKilnMQTT(void);

    void begin(uint16_t updateInterval_ms, EspMQTTClient *mqttCli);
    unsigned long msSinceLastUpdate(void);
    void enable(void);
    void disable(void);
    bool isEnabled(void);
    void thread(void);
    unsigned long getLastTime(void);
    TaskHandle_t getTask(void);
    UBaseType_t getTaskHighWaterMark(void);
    uint16_t getUpdateInterval_ms(void);
    void setUpdateInterval_ms(uint16_t updateInterval_ms);

  private:
    void _publish_state(bool active, bool pidCurrent, bool deepState);
    uint16_t _updateInterval_ms = 250;	// default update period
    unsigned long _lastTime = 0;	// the last time the system was updated
    bool _enabled = false;		// is the system running
    TaskHandle_t _taskHandle = NULL;	// for managing the task later
    EspMQTTClient *_mqttCli = NULL;	// pubsub thing
};

#endif
