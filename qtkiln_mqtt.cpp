#include <Arduino.h>
#include <ArduinoJson.h>

#include "qtkiln_mqtt.h"
#include "qtkiln_thermo.h"
#include "qtkiln_program.h"
#include "qtkiln_pwm.h"
#include "qtkiln_log.h"
#include "qtkiln.h"

extern Config config;

extern QTKilnLog qtklog;
extern char buf1[MAX_BUF];

extern QTKilnThermo *kiln_thermo;
extern QTKilnThermo *housing_thermo;

extern QTKilnPWM pwm;
extern QTKilnProgram program;

// use a static function to be the entry point for the task
void mqttTaskFunction(void *pvParameter) {
  QTKilnMQTT *mqtt = static_cast<QTKilnMQTT *>(pvParameter);

  mqtt->thread();
}

QTKilnMQTT::QTKilnMQTT(void) {
  _updateInterval_ms = QTKILN_MQTT_DEFAULT_UPDATE_INTERVAL_MS;
  _lastTime = 0;
  _taskHandle = NULL;
}

void QTKilnMQTT::setUpdateInterval_ms(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
  qtklog.debug(0, "mqtt update interval modified to %d ms", _updateInterval_ms);
}

uint16_t QTKilnMQTT::getUpdateInterval_ms(void) {
  return _updateInterval_ms;
}

void QTKilnMQTT::begin(uint16_t updateInterval_ms, EspMQTTClient *mqttCli) {
  if (!mqttCli)
    qtklog.error("mqtt client passwd to mqtt task was null");
  _updateInterval_ms = updateInterval_ms;
  _mqttCli = mqttCli;

  _mqttCli->setMaxPacketSize(8192);
  //_mqttCli->setMaxOutPacketSize(8192);

  // start on core 1 which means no need for mutex since all tasks callin this are too
  BaseType_t rc = xTaskCreatePinnedToCore(mqttTaskFunction, "mqtt",
		  QTKILN_MQTT_TASK_STACK_SIZE, (void *)this, QTKILN_MQTT_TASK_PRI,
		  &_taskHandle, QTKILN_TASK_CORE);
  if (rc != pdPASS || !_taskHandle)
    qtklog.error("unable to create task handle for mqtt");
}

TaskHandle_t QTKilnMQTT::getTask(void) {
  return _taskHandle;
}

void QTKilnMQTT::enable(void) {
  _enabled = true;
}

void QTKilnMQTT::disable(void) {
  _enabled = false;
}

bool QTKilnMQTT::isEnabled(void) {
  return _enabled;
}

unsigned long QTKilnMQTT::msSinceLastUpdate(void) {
  unsigned long now = millis();
  return now - _lastTime;
}

unsigned long QTKilnMQTT::getLastTime(void) {
  return _lastTime;
}

void QTKilnMQTT::_publish_state(bool active, bool pid_current, bool deepState) {
  JsonDocument doc;
  String jsonString;

  doc["time_ms"] = millis();
  doc["kiln"]["temp_C"] = kiln_thermo->getFilteredTemperature_C();
  doc["housing"]["temp_C"] = housing_thermo->getFilteredTemperature_C();
  if (pwm.isPwmEnabled() || active || deepState) {
    doc["pwm"]["enabled"] = pwm.isPwmEnabled();
    doc["pwm"]["dutyCycle_%"] = pwm.getDutyCycle();
    doc["pwm"]["output_ms"] = pwm.getOutput_ms();
    if (pwm.isPidEnabled() || active || deepState) {
      doc["pid"]["enabled"] = pwm.isPidEnabled();
      doc["pid"]["targetTemp_C"] = pwm.getTargetTemperature_C();
    }
  }
  if (pwm.isPidEnabled() || pid_current || deepState) {
    double Kp = pwm.getKp(), Ki = pwm.getKi(), Kd = pwm.getKd();
    doc["pid"]["Kp"] = Kp;
    doc["pid"]["Ki"] = Ki;
    doc["pid"]["Kd"] = Kd;
  }
  if (pwm.isTuning() || deepState) {
    double tKp = pwm.getTuningKp();
    double tKi = pwm.getTuningKi();
    double tKd = pwm.getTuningKd();
    doc["pid"]["tuning"]["Kp"] = tKp;
    doc["pid"]["tuning"]["Ki"] = tKi;
    doc["pid"]["tuning"]["Kd"] = tKd;
  }
  if (program.isProgramLoaded()) {
    doc["program"]["name"] = program.getLoadedProgramName();
    doc["program"]["running"] = program.isRunning();
    doc["program"]["steps"] = program.getCurrentProgramSteps();
    doc["program"]["step"] = program.getCurrentStep();
    doc["program"]["paused"] = program.isPaused();
    doc["program"]["inDwell"] = program.isDwell();
    doc["program"]["nextStepChangeTime_ms"] = program.getNextStepChangeTime_ms();
    doc["program"]["stepStartTime_ms"] = program.getStepStartTime_ms();
    doc["program"]["stepStartTemp_C"] = program.getStepStartTemp_C();
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  _mqttCli->publish(buf1, jsonString);
}

void QTKilnMQTT::thread(void) {
  TickType_t xDelay;
  uint32_t deepStateCounter = 0;
  bool deepStateUpdate = true;

  while (1) {
    if (_enabled) {
      if (++deepStateCounter % QTKILN_MQTT_DEEP_STATE_UPDATE_COUNT == 0)
        deepStateUpdate = true;
      else
        deepStateUpdate = false;
      _publish_state(false, false, deepStateUpdate);
    }
    xDelay = pdMS_TO_TICKS(_updateInterval_ms);
    vTaskDelay(xDelay);
  }
}

UBaseType_t QTKilnMQTT::getTaskHighWaterMark(void) {
  if (_taskHandle)
    return uxTaskGetStackHighWaterMark(_taskHandle);
  qtklog.warn("no task associated with qtkiln mqtt in high watermark test");
  return 0;
}
