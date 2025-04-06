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
extern QTKilnMQTT mqtt;

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

// home assistant
void QTKilnMQTT::homeAssistant_begin() {
  JsonDocument doc;
  String jsonString;
  char devId[MAX_BUF], cmpId[MAX_BUF];

  snprintf(devId, MAX_BUF, config.homeAss.deviceIdFmt, config.mac);
  doc["qos"] = 2;
  doc["dev"]["ids"] = devId;
  doc["dev"]["name"] = devId;
  doc["dev"]["mf"] = config.homeAss.manufacturer;
  doc["dev"]["mdl"] = config.homeAss.model;
  doc["dev"]["sw"] = config.homeAss.softwareRev;
  doc["dev"]["hw"] = config.homeAss.hardwareRev;
  doc["dev"]["sn"] = config.mac;
  doc["o"]["name"] = config.homeAss.originName;
  doc["o"]["sw"] = config.homeAss.softwareRev;

  // kiln
  snprintf(cmpId, MAX_BUF, "%s_kiln", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "Kiln Temperature";
  doc["cmps"][cmpId]["device_class"] = "temperature";
  doc["cmps"][cmpId]["unit_of_measurement"] = "°C";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.kiln.temp_C }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // housing
  snprintf(cmpId, MAX_BUF, "%s_housing", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "Housing Temperature";
  doc["cmps"][cmpId]["device_class"] = "temperature";
  doc["cmps"][cmpId]["unit_of_measurement"] = "°C";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.housing.temp_C }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // pwm output
  snprintf(cmpId, MAX_BUF, "%s_output_ms", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "number";
  doc["cmps"][cmpId]["name"] = "PWM Output";
  doc["cmps"][cmpId]["unit_of_measurement"] = "milliseconds";
  doc["cmps"][cmpId]["min"] = 0;
  doc["cmps"][cmpId]["max"] = pwm.getWindowSize_ms();
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.output_ms }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PWM);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  doc["cmps"][cmpId]["command_template"] = "{ \"output_ms\":{{ value }} }";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;
  // duty cycle
  snprintf(cmpId, MAX_BUF, "%s_dutyCycle_percent", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "PWM Duty Cycle";
  doc["cmps"][cmpId]["device_class"]= "power_factor";
  doc["cmps"][cmpId]["unit_of_measurement"] = "%";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.dutyCycle }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PWM);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // pwm enabled
  snprintf(cmpId, MAX_BUF, "%s_pwmEnabled", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "switch";
  doc["cmps"][cmpId]["name"] = "PWM Enabled";
  doc["cmps"][cmpId]["payload_on"] = "{ \"pwmEnabled\":true }";
  doc["cmps"][cmpId]["payload_off"] = "{ \"pwmEnabled\":false }";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.pwmEnabled }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PWM);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;
  // pid enabled
  snprintf(cmpId, MAX_BUF, "%s_pidEnabled", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "switch";
  doc["cmps"][cmpId]["name"] = "PID Enabled";
  doc["cmps"][cmpId]["payload_on"] = "{ \"pidEnabled\":true }";
  doc["cmps"][cmpId]["payload_off"] = "{ \"pidEnabled\":false }";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.pidEnabled }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PID);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;
  // pid target temperature
  snprintf(cmpId, MAX_BUF, "%s_targetTemp_C", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "number";
  doc["cmps"][cmpId]["name"] = "Target Temperature";
  doc["cmps"][cmpId]["unit_of_measurement"] = "°C";
  doc["cmps"][cmpId]["min"] = 0;
  doc["cmps"][cmpId]["max"] = TARGET_TEMP_MAX_C;
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.targetTemp_C }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PID);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  doc["cmps"][cmpId]["command_template"] = "{ \"targetTemp_C\":{{ value }} }";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;

  // program
  //snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PROGRAM);
  //doc["state_topic"] = buf2;
  snprintf(cmpId, MAX_BUF, "%s_programName", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "text";
  doc["cmps"][cmpId]["name"] = "Loaded Program";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.name }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONTROL);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  doc["cmps"][cmpId]["command_template"] = "{ \"program\":\"{{ value }}\" }";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;
  // running a program
  snprintf(cmpId, MAX_BUF, "%s_runProgram", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "switch";
  doc["cmps"][cmpId]["name"] = "Program Running";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.running }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONTROL);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  doc["cmps"][cmpId]["payload_on"] = "{ \"runProgram\":true }";
  doc["cmps"][cmpId]["payload_off"] = "{ \"runProgram\":false }";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "set");
  doc["cmps"][cmpId]["command_topic"] = buf1;

  // mem
  // parse min free as free memory for simplicity
  snprintf(cmpId, MAX_BUF, "%s_mem", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "Free Memory";
  doc["cmps"][cmpId]["device_class"] = "data_size";
  doc["cmps"][cmpId]["unit_of_measurement"] = "B";
  doc["cmps"][cmpId]["max"] = "128";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.minFree }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_MEM);
  doc["cmps"][cmpId]["state_topic"] = buf1;

  // info
  // kiln thermo read errors
  snprintf(cmpId, MAX_BUF, "%s_kiln_errors", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "Kiln Thermocouple Read Errors";
  //doc["cmps"][cmpId]["device_class"] = "number";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.kilnErrorCount }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_INFO);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // housing thermo read errors
  snprintf(cmpId, MAX_BUF, "%s_housing_errors", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "Housing Thermocouple Read Errors";
  //doc["cmps"][cmpId]["device_class"] = "number";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.housingErrorCount }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_INFO);
  doc["cmps"][cmpId]["state_topic"] = buf1;

  // wifi
  // rssi
  snprintf(cmpId, MAX_BUF, "%s_wifiRSSI", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "sensor";
  doc["cmps"][cmpId]["name"] = "WiFi Signal Strength (dbm)";
  doc["cmps"][cmpId]["device_class"] = "signal_strength";
  doc["cmps"][cmpId]["unit_of_measurement"] = "dBm";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.RSSI_dBm }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_WIFI);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // ip
  snprintf(cmpId, MAX_BUF, "%s_wifiIP", devId);
  doc["cmps"][cmpId]["unique_id"] = cmpId;
  doc["cmps"][cmpId]["p"] = "text";
  doc["cmps"][cmpId]["name"] = "WiFi IP Address";
  doc["cmps"][cmpId]["value_template"] = "{{ value_json.ip }}";
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_WIFI);
  doc["cmps"][cmpId]["state_topic"] = buf1;
  // this is not supported and these messages will be ignored
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, "ignored");
  doc["cmps"][cmpId]["command_topic"] = buf1;

  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, config.homeAss.configTopicFmt, "device", devId);
  _mqttCli->publish(buf1, jsonString, true);
}

void QTKilnMQTT::_publishState(bool active, bool pid_current, bool deepState) {
  JsonDocument doc;
  String jsonString;

  doc["time_ms"] = millis();
  doc["kiln"]["temp_C"] = kiln_thermo->getFilteredTemperature_C();
  doc["housing"]["temp_C"] = housing_thermo->getFilteredTemperature_C();
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  _mqttCli->publish(buf1, jsonString);

  if (pwm.isPwmEnabled() || active || deepState) {
    doc.clear();
    doc["time_ms"] = millis();
    doc["pwmEnabled"] = pwm.isPwmEnabled();
    doc["output_ms"] = pwm.getOutput_ms();
    doc["dutyCycle"] = pwm.getDutyCycle();
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PWM);
    _mqttCli->publish(buf1, jsonString);
  }

  if (pwm.isPidEnabled() || pid_current || deepState) {
    double Kp = pwm.getKp(), Ki = pwm.getKi(), Kd = pwm.getKd();
    doc.clear();
    doc["time_ms"] = millis();
    doc["pidEnabled"] = pwm.isPidEnabled();
    doc["targetTemp_C"] = pwm.getTargetTemperature_C();
    doc["Kp"] = Kp;
    doc["Ki"] = Ki;
    doc["Kd"] = Kd;
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PID);
    _mqttCli->publish(buf1, jsonString);
  }

  if (program.isProgramLoaded()) {
    doc.clear();
    doc["time_ms"] = millis();
    doc["name"] = program.getLoadedProgramName();
    doc["running"] = program.isRunning();
    doc["steps"] = program.getCurrentProgramSteps();
    doc["step"] = program.getCurrentStep();
    doc["paused"] = program.isPaused();
    doc["inDwell"] = program.isDwell();
    doc["nextStepChangeTime_ms"] = program.getNextStepChangeTime_ms();
    doc["stepStartTime_ms"] = program.getStepStartTime_ms();
    doc["stepStartTemp_C"] = program.getStepStartTemp_C();
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONTROL);
    _mqttCli->publish(buf1, jsonString);
  }

  if (deepState) {
    multi_heap_info_t info;

    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    doc.clear();
    doc["time_ms"] = millis();
    doc["totalFree"] =  info.total_free_bytes;   // total currently free in all non-continues blocks
    doc["minFree"] = info.minimum_free_bytes;  // minimum free ever
    doc["largestFreeBlock"] = info.largest_free_block;   // largest continues block to allocate big array
    doc["kilnThermo"] = kiln_thermo->getTaskHighWaterMark();
    doc["housingThermo"] = housing_thermo->getTaskHighWaterMark();
    doc["program"] = program.getTaskHighWaterMark();
    doc["pwm"] = pwm.getTaskHighWaterMark();
    doc["mqtt"] = mqtt.getTaskHighWaterMark();
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_MEM);
    _mqttCli->publish(buf1, jsonString);

    doc.clear();
    doc["time_ms"] = millis();
    doc["debugPriorityCutoff"] = qtklog.getDebugPriorityCutoff();
    doc["reallocationCount"] = qtklog.getReallocationCount();
    doc["kilnErrorCount"] = kiln_thermo->getErrorCount();
    doc["housingErrorCount"] = housing_thermo->getErrorCount();
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_INFO);
    _mqttCli->publish(buf1, jsonString);

    doc.clear();
    doc["time_ms"] = millis();
    doc["RSSI_dBm"] = WiFi.RSSI();
    doc["ip"] = WiFi.localIP().toString().c_str();
    serializeJson(doc, jsonString);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_WIFI);
    _mqttCli->publish(buf1, jsonString);
  }
}

#if 0
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
  if (deepState) {
    multi_heap_info_t info;

    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    doc["mem"]["totalFree"] =  info.total_free_bytes;   // total currently free in all non-continues blocks
    doc["mem"]["minFree"] = info.minimum_free_bytes;  // minimum free ever
    doc["mem"]["largestFreeBlock"] = info.largest_free_block;   // largest continues block to allocate big array
    doc["mem"]["kilnThermo"]["highWaterMark"] = kiln_thermo->getTaskHighWaterMark();
    doc["mem"]["housingThermo"]["highWaterMark"] = housing_thermo->getTaskHighWaterMark();
    doc["mem"]["program"]["highWaterMark"] = program.getTaskHighWaterMark();
    doc["mem"]["pwm"]["highWaterMark"] = pwm.getTaskHighWaterMark();
    doc["mem"]["mqtt"]["highWaterMark"] = mqtt.getTaskHighWaterMark();
    doc["info"]["debugPriorityCutoff"] = qtklog.getDebugPriorityCutoff();
    doc["info"]["reallocationCount"] = qtklog.getReallocationCount();
    doc["info"]["kilnErrorCount"] = kiln_thermo->getErrorCount();
    doc["info"]["housingErrorCount"] = housing_thermo->getErrorCount();
    doc["wifi"]["RSSI_dBm"] = WiFi.RSSI();
    doc["wifi"]["ip"] = WiFi.localIP().toString().c_str();
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  _mqttCli->publish(buf1, jsonString);
}
#endif

void QTKilnMQTT::thread(void) {
  TickType_t xDelay;
  uint32_t deepStateCounter = 0;
  bool deepStateUpdate = true;

  while (1) {
#if 0
    continue;
#endif
    if (_enabled) {
      if (++deepStateCounter % QTKILN_MQTT_DEEP_STATE_UPDATE_COUNT == 0)
        deepStateUpdate = true;
      else
        deepStateUpdate = false;
      _publishState(false, false, deepStateUpdate);
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
