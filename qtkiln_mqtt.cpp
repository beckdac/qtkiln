#include <Arduino.h>
#include <ArduinoJson.h>

#include "qtkiln_mqtt.h"
#include "qtkiln_thermo.h"
#include "qtkiln_pwm.h"
#include "qtkiln_log.h"
#include "qtkiln.h"

extern QTKilnLog qtklog;
extern char buf1[MAX_BUF];

extern QTKilnThermo *kiln_thermo;
extern QTKilnThermo *housing_thermo;

extern QTKilnPWM *PWM;

// use a static function to be the entry point for the task
void mqttTaskFunction(void *pvParameter) {
  QTKilnMQTT *mqtt = static_cast<QTKilnMQTT *>(pvParameter);

  mqtt->thread();
}

QTKilnMQTT::QTKilnMQTT(uint16_t updateInterval_ms) {
  _updateInterval_ms = updateInterval_ms;
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

void QTKilnMQTT::begin(void) {
  BaseType_t rc = xTaskCreate(mqttTaskFunction, "mqtt",
		  QTKILN_MQTT_TASK_STACK_SIZE, (void *)this, QTKILN_MQTT_TASK_PRI,
		  &_taskHandle);
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

void QTKilnMQTT::_publish_state(bool active, bool pid_current) {
  JsonDocument doc;
  String jsonString;

  doc["kiln"]["time_ms"] = kiln_thermo->lastTime();
  doc["kiln"]["temperature_C"] = kiln_thermo->getTemperature_C();
  doc["housing"]["time_ms"] = housing_thermo->lastTime();
  doc["housing"]["temperature_C"] = housing_thermo->getTemperature_C();
  if (PWM.isEnabled() || active) {
    doc["pidEnabled"] = PWM.isEnabled();
    doc["targetTemperatureC"] = PWM.getTargetTemperature_C();
    doc["dutyCycle"] = PWM.getDutyCycle();
  }
  if (PWM.isEnabled() || pid_current) {
    double Kp = pid->GetKp(), Ki = pid->GetKi(), Kd = pid->GetKd();
    doc["pid"]["Kp"] = Kp;
    doc["pid"]["Ki"] = Ki;
    doc["pid"]["Kd"] = Kd;
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  mqtt_cli->publish(buf1, jsonString);
}

void QTKilnMQTT::thread(void) {
  TickType_t xDelay;

  while (1) {
    if (_enabled) {
      mqtt_publish_state(false, false)
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
