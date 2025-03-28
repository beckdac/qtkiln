#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <Preferences.h>
#include <time.h>
#include "esp_chip_info.h"
#include "ESP.h"

#include <MAX31855.h>
#include <EspMQTTClient.h>
#include <TM1637Display.h>
#include <ArduinoJson.h>

#include "qtkiln.h"
#include "qtkiln_log.h"
#include "qtkiln_thermo.h"
#include "qtkiln_program.h"
#include "qtkiln_pwm.h"
#include "qtkiln_mqtt.h"

char buf1[MAX_BUF];

void config_setThermoUpdateInterval_ms(QTKilnThermo *thermo, uint16_t thermo_update_int_ms);
void config_setThermoFilterCutoffFrequency_Hz(QTKilnThermo *thermo, float cutoffFrequency_Hz);

// setup the basic log for serial logging
QTKilnLog qtklog(true);

// alarm pin
#define ALARM_PIN 26
bool alarm_state = false;

// thermocouple phy
#define MAXDO_PIN 19
#define MAXCS0_PIN 17 // MAX31855 kiln
#define MAXCS1_PIN 16 // MAX31855 housing
#define MAXSCK_PIN 18
SPIClass *vSPI = new SPIClass(VSPI);
MAX31855 kiln_MAX31855(MAXCS0_PIN, vSPI);
MAX31855 housing_MAX31855(MAXCS1_PIN, vSPI);
QTKilnThermo *kiln_thermo = NULL;
QTKilnThermo *housing_thermo = NULL;

// MQTT server obj and process
EspMQTTClient *mqttCli = NULL;
QTKilnMQTT mqtt;

// PWM object
#define SSR_PIN 25
bool ssr_state = false;
QTKilnPWM pwm(QTKILN_PWM_DEFAULT_WINDOW_SIZE);

// LCD
#define LCD_CLK 33
#define LCD_DIO 32
TM1637Display lcd(LCD_CLK, LCD_DIO);
const uint8_t LCD_BOOT[] = {
  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,          // b
  SEG_C | SEG_D | SEG_E | SEG_G,                  // o
  SEG_C | SEG_D | SEG_E | SEG_G,                  // o
  SEG_D | SEG_E | SEG_F | SEG_G      		  // t
};
uint8_t lcd_val = 0;
bool dots[4] = { false, false, false, false };

// Program execution engine
QTKilnProgram program;

// Preferences interface
Preferences preferences;
struct Config config;

void configLoad(const String &jsonString, bool justThermos=false) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    qtklog.warn("deserializeJson() failed: %s", error.c_str());
    return;
  }
  if (!justThermos) {
    config_setWifiSsid(doc[PREFS_WIFI_SSID] | config.wifiSsid);
    config_setWifiPassword(doc[PREFS_WIFI_PWD] | config.wifiPassword);
    config_setMqttHostIp(doc[PREFS_MQTT_HOSTIP] | config.mqttHostIp);
    config_setMqttUsername(doc[PREFS_MQTT_USERNAME] | config.mqttUsername);
    config_setMqttPassword(doc[PREFS_MQTT_PWD] | config.mqttPassword);
    config_setMqttPort(doc[PREFS_MQTT_PORT] | config.mqttPort);
    config_setHostname(doc[PREFS_HOSTNAME] | config.hostname);
    config_setPwmWindow_ms(doc[PREFS_PWM_WINDOW_MS] | config.pwmWindow_ms);
    config_setMqttUpdateInterval_ms(doc[PREFS_MQTT_UPD_INT_MS] | config.mqttUpdateInterval_ms);
    config_setMqttEnableDebugMessages(doc[PREFS_MQTT_ENABLE_DBG] | config.mqttEnableDebugMessages);
    config_setProgramUpdateInterval_ms(doc[PREFS_PGM_UPD_INT_MS] | config.programUpdateInterval_ms);
    config_setPidInitialKp(doc[PREFS_PID_KP] | config.Kp);
    config_setPidInitialKi(doc[PREFS_PID_KI] | config.Ki);
    config_setPidInitialKd(doc[PREFS_PID_KD] | config.Kd);
    config_setTimezone(doc[PREFS_TIMEZONE] | config.timezone);
    config_setDebugPriority(doc[PREFS_DBG_PRIORITY] | config.debugPriority);
  }
  // we can only set these after they are configured 
  // so check if they are allocated before setting these
  if (doc[PREFS_KILN].is<JsonObject>() && kiln_thermo) {
    config_setThermoUpdateInterval_ms(kiln_thermo, doc[PREFS_KILN][PREFS_THRM_UPD_INT_MS] | config.kiln.updateInterval_ms);
    config_setThermoFilterCutoffFrequency_Hz(kiln_thermo, doc[PREFS_KILN][PREFS_FLTR_CUT_FREQ_HZ] | config.kiln.filterCutoffFrequency_Hz);
  }
  if (doc[PREFS_HOUSING].is<JsonObject>() && housing_thermo) {
    config_setThermoUpdateInterval_ms(housing_thermo, doc[PREFS_HOUSING][PREFS_THRM_UPD_INT_MS] | config.housing.updateInterval_ms);
    config_setThermoFilterCutoffFrequency_Hz(housing_thermo, doc[PREFS_HOUSING][PREFS_FLTR_CUT_FREQ_HZ] | config.housing.filterCutoffFrequency_Hz);
  }
  if (justThermos)
    return;
  if (doc[CONFIG_RESET] | false) {
    configReset();
    esp_restart();
  } else if (doc[CONFIG_SAVE] | false) {
    configUpdatePrefs();
  }
}

String configSerialize(void) {
  JsonDocument doc;
  String jsonString;

  doc[PREFS_DBG_PRIORITY] = config.debugPriority;
  doc[PREFS_HOSTNAME] = config.hostname;
  doc[PREFS_WIFI_SSID] = config.wifiSsid;
  //doc[PREFS_WIFI_PWD] = config.wifiPassword;  don't show this for security
  doc[PREFS_MQTT_HOSTIP] = config.mqttHostIp;
  doc[PREFS_MQTT_USERNAME] = config.mqttUsername;
  //doc[PREFS_MQTT_PWD] = config.mqttPassword;  don't show this for security
  doc[PREFS_MQTT_PORT] = config.mqttPort;
  doc[PREFS_PWM_WINDOW_MS] = config.pwmWindow_ms;
  doc[PREFS_PGM_UPD_INT_MS] = config.programUpdateInterval_ms;
  doc[PREFS_MQTT_UPD_INT_MS] = config.mqttUpdateInterval_ms;
  doc[PREFS_MQTT_ENABLE_DBG] = config.mqttEnableDebugMessages;
  doc[PREFS_PID_KP] = config.Kp;
  doc[PREFS_PID_KI] = config.Ki;
  doc[PREFS_PID_KD] = config.Kd;
  doc[PREFS_TIMEZONE] = config.timezone;
  doc[PREFS_KILN][PREFS_FLTR_CUT_FREQ_HZ] = config.kiln.filterCutoffFrequency_Hz;
  doc[PREFS_KILN][PREFS_THRM_UPD_INT_MS] = config.kiln.updateInterval_ms;
  doc[PREFS_HOUSING][PREFS_FLTR_CUT_FREQ_HZ] = config.housing.filterCutoffFrequency_Hz;
  doc[PREFS_HOUSING][PREFS_THRM_UPD_INT_MS] = config.kiln.updateInterval_ms;

  serializeJson(doc, jsonString);

  return jsonString;
}

void configReset(void) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  preferences.begin(QTPROGRAM_NAMESPACE, false);
  preferences.clear();
  preferences.end();
}

void configLoadPrefs(bool justThermos) {
  preferences.begin(PREFS_NAMESPACE, true);
  String jsonString = preferences.getString(PREFS_CONFIG_JSON, String("{}"));
  preferences.end();
  qtklog.print("read config in JSON to preferences: %s", jsonString.c_str());
  configLoad(jsonString, justThermos);
}

void configUpdatePrefs(void) {
  String jsonString = configSerialize();
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putString(PREFS_CONFIG_JSON, jsonString);
  preferences.end();
  qtklog.print("write config in JSON to preferences: %s", jsonString.c_str());
}

// state variables are found above the loop function

// initialize the hardware and provide for any startup
// delays according to manufacturer data sheets
void setup() {
  uint8_t u8mac[6];

  // no matter what this is the firs thing we do
  pinMode(SSR_PIN, OUTPUT);
  // always turn it off incase we are coming back from a reset
  ssr_state=true; // setting this to true causes it to be forced off
  ssr_off();
  // always turn it off incase we are coming back from a reset
  pinMode(ALARM_PIN, OUTPUT);
  alarm_state=false;
  alarm_off();

  // set these to output prior to use
  pinMode(MAXCS0_PIN, OUTPUT);
  pinMode(MAXCS1_PIN, OUTPUT);

  // initialize the serial for 115200 baud
  Serial.begin(115200);

  // introduce a delay for settling
  delay(QTKILN_BOOT_DELAY_MS);

  // lcd setup
  lcd.setBrightness(1);
  lcd.setSegments(LCD_BOOT);

  // start the logger
  qtklog.begin();
  qtklog.print("QTK log has started at ms %lu...", millis());

  // load the preferences
  configLoadPrefs(false);

  // connect to wifi
  WiFi.setHostname(config.hostname);
  WiFi.begin(config.wifiSsid, config.wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_SETUP_DELAY_MS);
  }

  // get the mac
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, u8mac);
  if (ret == ESP_OK) {
    snprintf(config.mac, MAX_CFG_STR, MAC_FMT_STR,
	u8mac[0], u8mac[1], u8mac[2],
	u8mac[3], u8mac[4], u8mac[5]);
    qtklog.print("MAC for this device is %s", config.mac);
  } else {
    qtklog.error("failed to read MAC address");
  }
  // if we still have the default hostname change it and reboot
  if (strncmp(config.hostname, HOSTNAME_DEFAULT, MAX_CFG_STR) == 0) {
    char buf[MAX_CFG_STR];
    snprintf(buf, MAX_CFG_STR, "qtkiln_%s", config.mac);
    qtklog.warn("default hostname of %s found, setting it to %s and resetting device", config.hostname, buf);
    config_setHostname(buf);
    configUpdatePrefs();
    esp_restart();
  }
  // setup the topic based on the mac
  snprintf(config.topic, MAX_CFG_STR, MQTT_TOPIC_FMT,
	MQTT_TOPIC_BASE, config.mac);
  qtklog.print("topic root for this device is %s", config.topic);

  // setup PID
  pwm.begin();
 
  // start the program runner task
  program.begin(); 

  // start the mqtt client
  mqttCli = new EspMQTTClient(config.wifiSsid, config.wifiPassword, config.mqttHostIp,
                   config.mqttUsername, config.mqttPassword, config.hostname, config.mqttPort);
  qtklog.print("MQTT client connected");
  // send a last will message that removes the entity
  // not exactly sure how to set this up
  if (config.homeAss.enabled) {
    qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "enabling last will for HomeAssistant entity management");
    snprintf(buf1, MAX_BUF, config.homeAss.configTopicFmt, "sensor", "qtkiln", config.mac);
    mqttCli->enableLastWillMessage(strdup(buf1), "{}");
  }
  mqttCli->enableDebuggingMessages(config.mqttEnableDebugMessages);
  mqttCli->enableOTA(config.topic);  // make hacking a little challenging
  mqtt.begin(config.mqttUpdateInterval_ms, mqttCli);
  qtklog.mqttOutputEnable(true);

  // initialize the thermocouples and get the first readingings
  // setup SPI
  vSPI->begin();
  //
  // kiln phy
  kiln_MAX31855.begin(); 
  kiln_MAX31855.setSPIspeed(16000000);
  // kiln process
  kiln_thermo = new QTKilnThermo(config.kiln.updateInterval_ms, &kiln_MAX31855, "kiln");
  kiln_thermo->begin();
  //
  // housing phy
  housing_MAX31855.begin();
  housing_MAX31855.setSPIspeed(16000000);
  // housing process
  housing_thermo = new QTKilnThermo(config.housing.updateInterval_ms, &housing_MAX31855, "housing");
  housing_thermo->begin();
  // this is done again here to force these thermocouple setup
  configLoadPrefs(true);
  // turn them on
  kiln_thermo->enable();
  housing_thermo->enable();

  // lcd setup
  //lcd_update(kiln_thermo->getFilteredTemperature_C(), false, false);

  // turn this on at the end
  mqtt.enable();
}

void lcd_update(float temp, bool bold, bool colon) {
  // 0.5 is for rounding up
  uint16_t val = temp + 0.5;

  if (temp < 0) {
    val = 0;
    qtklog.warn("negative temperature sent to lcd_update, set to 0");
  } else if (temp > 9999) {
    val = 9999;
    qtklog.warn("max temperature reached, is everything OK");
  }
  lcd_val = val;

  if (bold)
    lcd.setBrightness(7);
  else
    lcd.setBrightness(1);
  lcd.showNumberDecEx(val, (colon ? 0xff: 0));
}

// state or preallocated variables for loop

void mqtt_publish_statistics(void) {
  JsonDocument doc;
  String jsonString;
  multi_heap_info_t info;

  doc["time"] = millis();
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
  doc["mem"]["totalFree"] =  info.total_free_bytes;   // total currently free in all non-continues blocks
  doc["mem"]["minFree"] = info.minimum_free_bytes;  // minimum free ever
  doc["mem"]["largestFreeBlock"] = info.largest_free_block;   // largest continues block to allocate big array

  doc["kilnThermo"]["highWaterMark"] = kiln_thermo->getTaskHighWaterMark();
  doc["housingThermo"]["highWaterMark"] = housing_thermo->getTaskHighWaterMark();
  doc["program"]["highWaterMark"] = program.getTaskHighWaterMark();
  doc["pwm"]["highWaterMark"] = pwm.getTaskHighWaterMark();
  doc["mqtt"]["highWaterMark"] = mqtt.getTaskHighWaterMark();
  doc["state"]["debugPriorityCutoff"] = qtklog.getDebugPriorityCutoff();
  doc["statistics"]["reallocationCount"] = qtklog.getReallocationCount();
  doc["statistics"]["kilnErrorCount"] = kiln_thermo->getErrorCount();
  doc["statistics"]["housingErrorCount"] = housing_thermo->getErrorCount();

  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  mqttCli->publish(buf1, jsonString);
}

bool programNameInList(const char *name) {
  JsonDocument doc;
  String jsonString;

  preferences.begin(PREFS_NAMESPACE, true);
  jsonString = preferences.getString(PREFS_PROGRAM_LIST, String("{}"));
  preferences.end();
  qtklog.print("read program list in JSON from preferences to look for: %s", name);

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    qtklog.warn("unable to deserialize JSON: %s", error.c_str());
    return false;
  }

  JsonArray arr = doc.as<JsonArray>();

  for (JsonVariant value : arr) {
    const char *this_name = value.as<const char *>();
    if (strcmp(name, this_name) == 0)
      return true;
  }
  return false;
}

void addProgramNameToList(const char *name) {
  JsonDocument doc;
  String jsonIn, jsonOut;

  // get the json from the prefs
  preferences.begin(PREFS_NAMESPACE, true);
  jsonIn = preferences.getString(PREFS_PROGRAM_LIST, String("[]"));
  preferences.end();
  qtklog.print("read program list in JSON from preferences: %s", jsonIn.c_str());

  // convert to the doc
  DeserializationError error = deserializeJson(doc, jsonIn);

  if (error) {
    qtklog.warn("unable to deserialize JSON: %s", error.c_str());
    return;
  }

  // add the name
  doc.add(name);
  // serialize it back out
  serializeJson(doc, jsonOut);

  // save to preferences
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putString(PREFS_PROGRAM_LIST, jsonOut);
  preferences.end();
  qtklog.print("wrote program list in JSON to preferences: %s", jsonOut.c_str());
}

void mqtt_publish_config(void) {
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONFIG);
  mqttCli->publish(buf1, configSerialize());
}

void mqtt_publish_programs(void) {
  JsonDocument doc;
  String jsonString;

  preferences.begin(PREFS_NAMESPACE, true);
  jsonString = preferences.getString(PREFS_PROGRAM_LIST, String("{}"));
  preferences.end();
  qtklog.print("read program list in JSON: %s", jsonString.c_str());

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    qtklog.warn("unable to deserialize JSON: %s", error.c_str());
    return;
  }

  JsonArray arr = doc.as<JsonArray>();

  for (JsonVariant value : arr) {
    const char *name = value.as<const char *>();
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PROGRAM);
    jsonString = program.getJSON(name);
    mqttCli->publish(buf1, jsonString);
  }
}

void alarm_on(void) {
  qtklog.debug(QTKLOG_DBG_PRIO_HIGH, "alarm on called");
  alarm_state = true;
  digitalWrite(ALARM_PIN, LOW);
}

void alarm_off(void) {
  qtklog.debug(QTKLOG_DBG_PRIO_HIGH, "alarm off called");
  alarm_state = false;
  digitalWrite(ALARM_PIN, HIGH);
}

void ssr_on(void) {
  if (ssr_state)
    return;
  if (config.alarmOnSSR)
    alarm_on();
  ssr_state = true;
  digitalWrite(SSR_PIN, LOW);
}

void ssr_off(void) {
  if (ssr_state) {
    if (config.alarmOnSSR)
      alarm_off();
    ssr_state = false;
    digitalWrite(SSR_PIN, HIGH);
  }
}

// main loop, this is a freertos task
void loop() {
  TickType_t xDelay;

#define QTKILN_MAIN_LOOP_STARTUP_DELAY 1000
  xDelay = pdMS_TO_TICKS(QTKILN_MAIN_LOOP_STARTUP_DELAY);
  vTaskDelay(xDelay);

  while (1) {
    lcd_update(kiln_thermo->getFilteredTemperature_C(), ssr_state, ssr_state);
    // run handlers for subprocesses 
    mqttCli->loop();

    xDelay = pdMS_TO_TICKS(config.mainLoop_ms);
    vTaskDelay(xDelay);
  }
}

// set configuration variables after checking them
void config_setHostname(const char *hostname) {
  if (!hostname) {
    qtklog.warn("empty hostname passed to %s", __func__);
    return;
  }
  strncpy(config.hostname, hostname, MAX_CFG_STR);
  qtklog.print("hostname set to %s", config.hostname);
}
void config_setThermoUpdateInterval_ms(QTKilnThermo *thermo, uint16_t thermoUpdateInterval_ms) {
  if (thermoUpdateInterval_ms < THERMO_MIN_UPDATE_MS) {
    qtklog.warn("thermocouple update interval must be > %d ms ... forcing to min (%d)",
    	THERMO_MIN_UPDATE_MS, THERMO_MIN_UPDATE_MS);
    thermoUpdateInterval_ms = THERMO_MIN_UPDATE_MS;
  }
  if (thermoUpdateInterval_ms > THERMO_MAX_UPDATE_MS) {
    qtklog.warn("thermocouple update interval must be < %d ms ... forcing to max (%d)", 
	THERMO_MAX_UPDATE_MS, THERMO_MAX_UPDATE_MS);
    thermoUpdateInterval_ms = THERMO_MAX_UPDATE_MS;
  }
  if (thermo == kiln_thermo) {
    config.kiln.updateInterval_ms = thermoUpdateInterval_ms;
    kiln_thermo->setUpdateInterval_ms(config.kiln.updateInterval_ms);
  } else if (thermo == housing_thermo) {
    config.housing.updateInterval_ms = thermoUpdateInterval_ms;
    housing_thermo->setUpdateInterval_ms(config.housing.updateInterval_ms);
  } else {
    qtklog.warn("unknown thermocouple passed to config set thermocouple update interval");
    return;
  }
  qtklog.print("%s thermocouple update interval set to %d ms", 
        (thermo == kiln_thermo ? PREFS_KILN : PREFS_HOUSING),
	thermo->getUpdateInterval_ms());
}
// the filter cutoff has to be faster than the sampling rate
// and slower than the pwm rate
#define THERMO_MIN_CUTOFF_HZ (1./(config.pwmWindow_ms / 1000.))
#define THERMO_MAX_CUTOFF_HZ (1./(thermo->getUpdateInterval_ms() / 1000.))
void config_setThermoFilterCutoffFrequency_Hz(QTKilnThermo *thermo, float cutoffFrequency_Hz) {
  if (cutoffFrequency_Hz < THERMO_MIN_CUTOFF_HZ) {
    qtklog.warn("thermocouple update interval must be > %d ms ... forcing to min (%d)",
    	THERMO_MIN_CUTOFF_HZ, THERMO_MIN_CUTOFF_HZ);
    cutoffFrequency_Hz = THERMO_MIN_CUTOFF_HZ;
  }
  if (cutoffFrequency_Hz > THERMO_MAX_CUTOFF_HZ) {
    qtklog.warn("thermocouple update interval must be < %d ms ... forcing to max (%d)", 
	THERMO_MAX_CUTOFF_HZ, THERMO_MAX_CUTOFF_HZ);
    cutoffFrequency_Hz = THERMO_MAX_CUTOFF_HZ;
  }
  if (thermo == kiln_thermo) {
    config.kiln.filterCutoffFrequency_Hz = cutoffFrequency_Hz;
    kiln_thermo->setFilterCutoffFrequency_Hz(cutoffFrequency_Hz);
  } else if (thermo == housing_thermo) {
    config.housing.filterCutoffFrequency_Hz = cutoffFrequency_Hz;
    housing_thermo->setFilterCutoffFrequency_Hz(cutoffFrequency_Hz);
  } else {
    qtklog.warn("unknown thermocouple passed to config set thermocouple update interval");
    return;
  }
  qtklog.print("%s thermocouple low pass filter frequency cutoff set to %g Hz",
        (thermo == kiln_thermo ? PREFS_KILN : PREFS_HOUSING),
	thermo->getFilterCutoffFrequency_Hz());
}
void config_setPwmWindow_ms(uint16_t pwmWindow_ms) {
  if (pwmWindow_ms < PWM_MIN_WINDOW_MS) {
    qtklog.warn("PWM window must be > %d ms ... forcing to min (%d)",
	PWM_MIN_WINDOW_MS, PWM_MIN_WINDOW_MS);
    pwmWindow_ms = PWM_MIN_WINDOW_MS;
  }
  if (pwmWindow_ms > PWM_MAX_WINDOW_MS) {
    qtklog.warn("PWM window must be < %d ms ... forcing to max (%d)",
	PWM_MIN_WINDOW_MS, PWM_MIN_WINDOW_MS);
    pwmWindow_ms = PWM_MAX_WINDOW_MS;
  }
  config.pwmWindow_ms = pwmWindow_ms;
  qtklog.print("PWM update interval = %d ms", config.pwmWindow_ms);
}
const char *config_setWifiSsid(void) {
  return config.wifiSsid;
}
void config_setWifiSsid(const char *wifiSsid) {
  strncpy(config.wifiSsid, wifiSsid, MAX_CFG_STR);
}
const char *config_setWifiPassword(void) {
  return config.wifiPassword;
}
void config_setWifiPassword(const char *wifiPassword) {
  strncpy(config.wifiPassword, wifiPassword, MAX_CFG_STR);
}
const char *config_getMqttHostIp(void) {
  return config.mqttHostIp;
}
void config_setMqttHostIp(const char *hostIp) {
  strncpy(config.mqttHostIp, hostIp, MAX_CFG_STR);
}
const char *config_getMqttPassword(void) {
  return config.mqttPassword;
}
void config_setMqttPassword(const char *password) {
  strncpy(config.mqttPassword, password, MAX_CFG_STR);
}
const char *config_getMqttUsername(void) {
  return config.mqttUsername;
}
void config_setMqttUsername(const char *username) {
  strncpy(config.mqttUsername, username, MAX_CFG_STR);
}
uint16_t config_getMqttPort(void) {
  return config.mqttPort;
}
void config_setMqttPort(uint16_t port) {
  config.mqttPort = port;
}
void config_setMqttUpdateInterval_ms(uint16_t mqttUpdateInterval_ms) {
  if (mqttUpdateInterval_ms < MQTT_MIN_UPDATE_MS) {
    qtklog.warn("mqtt update interval must be > %d ms .. forcing to min (%d)",
	MQTT_MIN_UPDATE_MS, MQTT_MIN_UPDATE_MS);
    mqttUpdateInterval_ms = MQTT_MIN_UPDATE_MS;
  }
  if (mqttUpdateInterval_ms > MQTT_MAX_UPDATE_MS) {
    qtklog.warn("MQTT update interval must be < %d ms ... forcing to max (%d)",
	MQTT_MAX_UPDATE_MS, MQTT_MAX_UPDATE_MS);
    mqttUpdateInterval_ms = MQTT_MAX_UPDATE_MS;
  }
  config.mqttUpdateInterval_ms = mqttUpdateInterval_ms;
  mqtt.setUpdateInterval_ms(config.mqttUpdateInterval_ms);
  qtklog.print("mqtt update interval  = %d ms", config.mqttUpdateInterval_ms);
}
void config_setMqttEnableDebugMessages(bool enable_messages) {
  config.mqttEnableDebugMessages = enable_messages;
  if (mqttCli)
    mqttCli->enableDebuggingMessages(config.mqttEnableDebugMessages);
}
void config_setProgramUpdateInterval_ms(uint16_t programUpdateInterval_ms) {
  if (programUpdateInterval_ms < PGM_MIN_UPDATE_MS) {
    qtklog.warn("program update interval must be > %d ms ... forcing to min (%d)",
	PGM_MIN_UPDATE_MS, PGM_MIN_UPDATE_MS);
    programUpdateInterval_ms = PGM_MIN_UPDATE_MS;
  }
  if (programUpdateInterval_ms > PGM_MAX_UPDATE_MS) {
    qtklog.warn("program update interval must be < %d ms ... forcing to max (%d)",
	PGM_MAX_UPDATE_MS, PGM_MAX_UPDATE_MS);
    programUpdateInterval_ms = PGM_MAX_UPDATE_MS;
  }
  config.programUpdateInterval_ms = programUpdateInterval_ms;
  program.setUpdateInterval_ms(config.programUpdateInterval_ms);
  qtklog.print("program update interval = %d ms", config.programUpdateInterval_ms);
}
void config_setPidInitialKp(double Kp) {
  if (Kp < 0)
    Kp = 0;
  config.Kp = Kp;
  qtklog.print("PID initial Kp set to %g", config.Kp);
}
void config_setPidInitialKi(double Ki) {
  if (Ki < 0)
    Ki = 0;
  config.Ki = Ki;
  qtklog.print("PID initial Ki set to %g", config.Ki);
}
void config_setPidInitialKd(double Kd) {
  if (Kd < 0)
    Kd = 0;
  config.Kd = Kd;
  qtklog.print("PID initial Kd set to %g", config.Kd);
}
void config_setDebugPriority(uint16_t debugPriority) {
  qtklog.setDebugPriorityCutoff(debugPriority);
}
void config_setTimezone(const char *timeZone) {
  time_t now;
  struct tm timeinfo;
  char buf[MAX_BUF];

  if (!timeZone) {
    qtklog.warn("empty timeZone passed to set time zone");
    return;
  }

  setenv("TZ", timeZone, 1);
  tzset();

  qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "setting the ntp server to pool.ntp.org");
  configTime(0, 0, "pool.ntp.org");
  if (!localtime_r(&now, &timeinfo)) {
    qtklog.warn("unable to obtain time");
  }

  // localtime triggers calls to ntp and other things
  localtime_r(&now, &timeinfo);
  strftime(buf, MAX_BUF, "%c", &timeinfo);
  qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "time zone set to %s where the current time is %s", timeZone, buf);
}


// handle mqtt config messages
void onConfigMessageReceived(const String &message) {
  qtklog.print("processing config update in json: %s", message.c_str());
  configLoad(message, false);
}

// handle mqtt state messages
void onSetStateMessageReceived(const String &message) {
  JsonDocument doc;
  double Kp = pwm.getKp(), Ki = pwm.getKi(), Kd = pwm.getKd();
  bool pidEnabledAtStart = pwm.isEnabled(), startPid = false;
  bool updateTunings = false;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    qtklog.warn("unable to deserialize JSON: %s", error.c_str());
    return;
  }
  // if this request turns on the pid enable, then
  // do it first before parsing others, like target temp
  // if it turns it off, make sure the ssr is off
  if (doc[MSG_PID_ENABLED].is<bool>()) {
    bool pidEnabled = doc[MSG_PID_ENABLED];
    if (!pidEnabled && pidEnabledAtStart) {
      pwm.disable();
      qtklog.print("stopping PID control");
    } else if (pidEnabled && !pidEnabledAtStart) {
      startPid = true;
    }
  }
  // go through the keys and handle each
  for (JsonPair kv : doc.as<JsonObject>()) {
    if (strcmp(kv.key().c_str(), MSG_TARGET_TEMP) == 0) {
      double tmp = doc[MSG_TARGET_TEMP];
      if (tmp < TARGET_TEMP_MIN) {
        tmp = TARGET_TEMP_MIN;
      } else if (tmp > TARGET_TEMP_MAX) {
        tmp = TARGET_TEMP_MAX;
      }
      pwm.setTargetTemperature_C(tmp);
      if (pwm.isEnabled()) { // pid was already enabled, just updating the set point
        qtklog.print("adjusting current PID target temperature to %d C", pwm.getTargetTemperature_C());
      } else {
        qtklog.print("setting PID target temperature to %d C", pwm.getTargetTemperature_C());
      }
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KP) == 0 && pwm.isEnabled()) {
      Kp = doc[PREFS_PID_KP];
      if (Kp < 0)
        Kp = 0;
      updateTunings = true;
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KI) == 0 && pwm.isEnabled()) {
      Ki = doc[PREFS_PID_KI];
      if (Ki < 0)
        Ki = 0;
      updateTunings = true;
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KD) == 0 && pwm.isEnabled()) {
      Kd = doc[PREFS_PID_KD];
      if (Kd < 0)
        Kd = 0;
      updateTunings = true;
    } else if (strcmp(kv.key().c_str(), "alarm") == 0) {
      bool alarm_set = doc["alarm"] | false;
      if (alarm_set)
        alarm_on();
      else
        alarm_off();
    } else if (strcmp(kv.key().c_str(), "alarmOnSSR") == 0) {
      config.alarmOnSSR = doc["alarmOnSSR"] | false;
      qtklog.print("setting alarmOnSSR flag to %s", (config.alarmOnSSR ? "true" : "false"));
    } else if (strcmp(kv.key().c_str(), "pidTuning") == 0) {
      if (doc["pidTuning"] | false)
        pwm.startTuning();
      else
        pwm.stopTuning();
      qtklog.print("setting PID tuning flag to %s", (pwm.isTuning() ? "true" : "false"));
    } else if (strcmp(kv.key().c_str(), "restart") == 0) {
      bool restart = doc["restart"] | false;
      qtklog.warn("received restart request");
      if (restart) // and we're out
        esp_restart();
    } else if (strcmp(kv.key().c_str(), "program") == 0) {
      const char *name = doc["program"];
      program.loadProgram(name);
    } else if (strcmp(kv.key().c_str(), "runProgram") == 0) {
      bool state = doc["runProgram"] | false;
      if (state && !program.isProgramLoaded()) {
        qtklog.warn("runProgram sent but no program loaded");
      } else if (state && !program.isRunning()) {
        qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "starting to run program %s", program.getLoadedProgramName());
        program.start();
      } else if (!state && program.isRunning()) {
        qtklog.debug(QTKLOG_DBG_PRIO_ALWAYS, "stopping program %s", program.getLoadedProgramName());
        program.stop();
      }
    }
  }
  if (startPid) {
      pwm.enable();
      qtklog.print("starting PID with target temperature of %d C", pwm.getTargetTemperature_C());
  }
  if (updateTunings) {
    qtklog.print("updating live tunings to (Kp = %g, Ki = %g, Kd = %g)", Kp, Ki, Kd);
    pwm.setTunings(Kp, Ki, Kd);
  }
}
void onGetStateMessageReceived(const String &message) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    qtklog.warn("deserializeJson() failed: %s", error.c_str());
    return;
  }
  // if it has a name object then we are setting the program
  if (doc["statistics"] | false) {
    qtklog.print("statistics requested via mqtt");
    mqtt_publish_statistics();
  }
  if (doc["programs"] | false) {
    qtklog.print("programs requested via mqtt");
    mqtt_publish_programs();
  }
  if (doc["config"] | false) {
    qtklog.print("config requested via mqtt");
    mqtt_publish_config();
  }
}
void onProgramMessageReceived(const String &message) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    qtklog.warn("deserializeJson() failed: %s", error.c_str());
    return;
  }
  // if it has a name object then we are setting the program
// need to remove the strings from this
  if (doc["name"].is<const char*>()) {
    const char *name = doc["name"];
    bool validProgram = program.set(name, message);
    if (validProgram && !programNameInList(name)) {
       addProgramNameToList(name);
    } 
  }
}

// need to remove the strings from this
void homeAss_begin(void) {
  JsonDocument doc;
  String jsonString;
  char buf[MAX_BUF], buf2[MAX_BUF];

  snprintf(buf, MAX_BUF, config.homeAss.deviceIdFmt, config.mac);
  doc["dev"]["ids"] = buf;
  doc["dev"]["name"] = buf;
  doc["dev"]["mf"] = config.homeAss.manufacturer;
  doc["dev"]["mdl"] = config.homeAss.model;
  doc["dev"]["sw"] = config.homeAss.softwareRev;
  doc["dev"]["hw"] = config.homeAss.hardwareRev;
  doc["dev"]["sn"] = config.mac;
  doc["o"]["name"] = config.homeAss.originName;
  doc["o"]["sw"] = config.homeAss.softwareRev;
  snprintf(buf, MAX_BUF, config.homeAss.componentFmt, config.mac, "kiln");
  doc[buf]["p"] = "sensor";
  doc[buf]["device_class"] = "temperature";
  doc[buf]["unit_of_measurement"] = "°C";
  doc[buf]["value_template"] = "{{ value_json.kiln.temp_C }}";
  snprintf(buf2, MAX_BUF, "%s_t", buf);
  doc[buf]["unique_id"] = buf2;
  snprintf(buf, MAX_BUF, config.homeAss.componentFmt, config.mac, "housing");
  doc[buf]["p"] = "sensor";
  doc[buf]["device_class"] = "temperature";
  doc[buf]["unit_of_measurement"] = "°C";
  doc[buf]["value_template"] = "{{ value_json.housing.temp_C }}";
  snprintf(buf2, MAX_BUF, "%s_t", buf);
  doc[buf]["unique_id"] = buf2;
  snprintf(buf, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  doc["state_topic"] = buf;
  doc["qos"] = 2;

  serializeJson(doc, jsonString);

  snprintf(buf1, MAX_BUF, config.homeAss.configTopicFmt, "sensor", "qtkiln", config.mac);
  mqttCli->publish(buf1, jsonString, true);
}

// when the connection to the mqtt has completed
void onConnectionEstablished(void) {
  char topic[MAX_BUF];

  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONFIG);
  mqttCli->subscribe(topic, onConfigMessageReceived);
  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_GET);
  mqttCli->subscribe(topic, onGetStateMessageReceived);
  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_PROGRAM);
  mqttCli->subscribe(topic, onProgramMessageReceived);
  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_SET);
  mqttCli->subscribe(topic, onSetStateMessageReceived);

  // send out the home assistant mqtt discovery messages
  if (config.homeAss.enabled) {
    homeAss_begin();
  }
}
