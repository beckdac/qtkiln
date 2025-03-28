#include <Arduino.h>
#include <ArduinoJson.h>

#include "qtkiln_mqtt.h"
#include "qtkiln_thermo.h"
#include "qtkiln_pwm.h"
#include "qtkiln_log.h"
#include "qtkiln.h"

extern Config config;

extern QTKilnLog qtklog;
extern char buf1[MAX_BUF];

extern QTKilnThermo *kiln_thermo;
extern QTKilnThermo *housing_thermo;

extern QTKilnPWM pwm;

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

void QTKilnMQTT::_publish_state(bool active, bool pid_current) {
  JsonDocument doc;
  String jsonString;

  doc["kiln"]["time_ms"] = kiln_thermo->getLastTime();
  doc["kiln"]["temp_C"] = kiln_thermo->getFilteredTemperature_C();
  doc["housing"]["time_ms"] = housing_thermo->getLastTime();
  doc["housing"]["temp_C"] = housing_thermo->getFilteredTemperature_C();
  if (pwm.isEnabled() || active) {
    doc["pidEnabled"] = pwm.isEnabled();
    doc["targetTemp_C"] = pwm.getTargetTemperature_C();
    doc["dutyCycle_%"] = pwm.getDutyCycle();
    doc["output_ms"] = pwm.getOutput_ms();
  }
  if (pwm.isEnabled() || pid_current) {
    double Kp = pwm.getKp(), Ki = pwm.getKi(), Kd = pwm.getKd();
    doc["pid"]["Kp"] = Kp;
    doc["pid"]["Ki"] = Ki;
    doc["pid"]["Kd"] = Kd;
    if (pwm.isTuning()) {
      Kp = pwm.getTuningKp();
      Ki = pwm.getTuningKi();
      Kd = pwm.getTuningKd();
      doc["pid"]["tuning"]["Kp"] = Kp;
      doc["pid"]["tuning"]["Ki"] = Ki;
      doc["pid"]["tuning"]["Kd"] = Kd;
    }
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  _mqttCli->publish(buf1, jsonString);
}

void QTKilnMQTT::thread(void) {
  TickType_t xDelay;

  while (1) {
    if (_enabled) {
      _publish_state(false, false);
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
