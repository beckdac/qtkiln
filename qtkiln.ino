#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <Preferences.h>

#include <PID_v2.h>
#include <MAX31855.h>
#include <MAX6675.h>
#include <EspMQTTClient.h>
#include <TM1637Display.h>
#include <ArduinoJson.h>

#include "qtkiln.h"
#include "qtkiln_log.h"
#include "qtkiln_thermo.h"

// setup the basic log for serial logging
QTKilnLog qtklog(true);

// thermocouple phy
#define MAXDO_PIN 5
#define MAXCS0_PIN 3 // MAX31855 kiln
#define MAXCS1_PIN 2 // MAX6675 housing
#define MAXSCK_PIN 4
MAX31855 kiln_thermocouple(MAXCS0_PIN);
MAX6675 housing_thermocouple(MAXCS1_PIN);
QTKilnThermo *kiln_thermo = NULL;
QTKilnThermo *housing_thermo = NULL;

// WiFi credentials
#include "wifi_cred.h"
const char *ssid = WIFI_SSID;
const char *sspw = WIFI_PASS;

// MQTT server
#include "mqtt_cred.h"
EspMQTTClient *mqtt_cli = NULL;

// PWM object
#define SSR_PIN 8
bool ssr_state = false;
unsigned long pwm_window_start_time = 0;

// PID object
PID_v2 *pid = NULL;
boolean pid_enabled = false;
uint16_t targetTemperatureC = 0;
unsigned long pid_output = 0;

// LCD
#define LCD_CLK 1
#define LCD_DIO 0
TM1637Display lcd(LCD_CLK, LCD_DIO);
const uint8_t LCD_BOOT[] = {
  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,          // b
  SEG_C | SEG_D | SEG_E | SEG_G,                  // o
  SEG_C | SEG_D | SEG_E | SEG_G,                  // o
  SEG_D | SEG_E | SEG_F | SEG_G      		  // t
};
uint8_t lcd_val = 0;
bool dots[4] = { false, false, false, false };

// Preferences interface
Preferences preferences;
struct Config config;

void configLoad(const String &jsonString) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    qtklog.warn("deserializeJson() failed: %s", error.c_str());
    return;
  }
  config_set_hostname(doc[PREFS_HOSTNAME] | config.hostname);
  config_set_thermoUpdateInterval_ms(doc[PREFS_THRM_UPD_INT_MS] | config.thermoUpdateInterval_ms);
  config_set_pwmWindow_ms(doc[PREFS_PWM_WINDOW_MS] | config.pwmWindow_ms);
  config_set_mqttUpdateInterval_ms(doc[PREFS_MQTT_UPD_INT_MS] | config.mqttUpdateInterval_ms);
  config_set_mqtt_enable_debug_messages(doc[PREFS_MQTT_ENABLE_DBG] | config.mqtt_enable_debug_messages);
  config_set_programUpdateInterval_ms(doc[PREFS_PGM_UPD_INT_MS] | config.programUpdateInterval_ms);
  config_set_pid_init_Kp(doc[PREFS_PID_KP] | config.Kp);
  config_set_pid_init_Ki(doc[PREFS_PID_KI] | config.Ki);
  config_set_pid_init_Kd(doc[PREFS_PID_KD] | config.Kd);
  if (doc[CONFIG_RESET] | false) {
    configReset();
    esp_restart();
  }
}

String configSerialize(void) {
  JsonDocument doc;
  String jsonString;

  doc[PREFS_HOSTNAME] = config.hostname;
  doc[PREFS_THRM_UPD_INT_MS] = config.thermoUpdateInterval_ms;
  doc[PREFS_PWM_WINDOW_MS] = config.pwmWindow_ms;
  doc[PREFS_PGM_UPD_INT_MS] = config.programUpdateInterval_ms;
  doc[PREFS_MQTT_UPD_INT_MS] = config.mqttUpdateInterval_ms;
  doc[PREFS_MQTT_ENABLE_DBG] = config.mqtt_enable_debug_messages;
  doc[PREFS_PID_KP] = config.Kp;
  doc[PREFS_PID_KI] = config.Ki;
  doc[PREFS_PID_KD] = config.Kd;

  serializeJson(doc, jsonString);

  return jsonString;
}

void configReset(void) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.clear();
  preferences.end();
}

void configLoadPrefs(void) {
  preferences.begin(PREFS_NAMESPACE, true);
  String jsonString = preferences.getString(PREFS_CONFIG_JSON, String("{}"));
  preferences.end();
  qtklog.print("read config in JSON to preferences: %s", jsonString.c_str());
  configLoad(jsonString);
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

  // initialize the serial for 115200 baud
  Serial.begin(115200);

  delay(QTKILN_BOOT_DELAY_MS);

  // lcd setup
  lcd.setBrightness(1);
  lcd.setSegments(LCD_BOOT);

  // start the logger
  qtklog.begin();
  qtklog.print("QTK log has started at ms %lu...", millis());

  // load the preferences
  configLoadPrefs();

  // connect to wifi
  WiFi.setHostname(config.hostname);
  WiFi.begin(ssid, sspw);
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
    config_set_hostname(buf);
    configUpdatePrefs();
    esp_restart();
  }
  // setup the topic based on the mac
  snprintf(config.topic, MAX_CFG_STR, MQTT_TOPIC_FMT,
	MQTT_TOPIC_BASE, config.mac);
  qtklog.print("topic root for this device is %s", config.topic);

  // setup PWM
  pinMode(SSR_PIN, OUTPUT);
  // always turn it off incase we are coming back from a reset
  digitalWrite(SSR_PIN, LOW);
  qtklog.print("PWM cycle window in ms is %d", config.pwmWindow_ms);
  pwm_window_start_time = millis();

  // setup PID
  pid = new PID_v2(config.Kp, config.Ki, config.Kd, PID::Direct);
  pid->SetOutputLimits(0, config.pwmWindow_ms);
  

  // start the mqtt client
  mqtt_cli = new EspMQTTClient(WIFI_SSID, WIFI_PASS, MQTT_BROKER,
    MQTT_USER, MQTT_PASS, config.mac, MQTT_PORT);
  qtklog.print("MQTT client connected");
  mqtt_cli->enableDebuggingMessages(config.mqtt_enable_debug_messages);

  // initialize the thermocouples and get the first readingings
  // kiln
  kiln_thermo = new QTKilnThermo(config.thermoUpdateInterval_ms, &kiln_thermocouple, NULL);
  kiln_thermo->begin();
  kiln_thermo->enable();
  // housing
  housing_thermo = new QTKilnThermo(config.thermoUpdateInterval_ms, NULL, &housing_thermocouple);
  housing_thermo->begin();
  housing_thermo->enable();
  // get the readings
  qtklog.print("kiln = %g C and housing = %g C",
        kiln_thermo->getTemperatureC(),
  	housing_thermo->getTemperatureC());

  // lcd setup
  // 0.5 is for rounding up
  lcd_update(kiln_thermo->getTemperatureC() + 0.5, false, false);
}

void lcd_update(uint16_t val, bool bold, bool colon) {
  lcd_val = val;
  if (bold)
    lcd.setBrightness(7);
  else
    lcd.setBrightness(1);
  lcd.showNumberDecEx(val, (colon ? 0xff: 0));
}

// state or preallocated variables for loop
unsigned long last_time = 0, now, delta_t;
char buf1[MAX_BUF], buf2[MAX_BUF];

void mqtt_publish_state(bool active=false, bool pid_current=false, bool statistics=false) {
  JsonDocument doc;
  String jsonString;

  doc["kiln"]["time_ms"] = kiln_thermo->lastTime();
  doc["kiln"]["temperature_C"] = kiln_thermo->getTemperatureC();
  doc["housing"]["time_ms"] = housing_thermo->lastTime();
  doc["housing"]["temperature_C"] = housing_thermo->getTemperatureC();
  if (pid_enabled || active) {
    doc["pidEnabled"] = pid_enabled;
    doc["targetTemperatureC"] = targetTemperatureC;
    doc["dutyCycle"] = 100. * (double)pid_output / (double)config.pwmWindow_ms;
  }
  if (pid_enabled || pid_current) {
    double Kp = pid->GetKp(), Ki = pid->GetKi(), Kd = pid->GetKd();
    doc["pid"]["Kp"] = Kp;
    doc["pid"]["Ki"] = Ki;
    doc["pid"]["Kd"] = Kd;
  }
  if (statistics) {
    doc["log"]["reallocationCount"] = qtklog.getReallocationCount();
    doc["log"]["debugPriorityCutoff"] = qtklog.getDebugPriorityCutoff();
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  mqtt_cli->publish(buf1, jsonString);
}

void mqtt_publish_programs(void) {

}

void ssr_on(void) {
  if (ssr_state)
    return;
  ssr_state = true;
  digitalWrite(SSR_PIN, HIGH);
}

void ssr_off(void) {
  if (ssr_state) {
    ssr_state = false;
    digitalWrite(SSR_PIN, LOW);
  }
}

void loop() {
  now = millis();
  if (pid_enabled) {
    // shift the next start time into the future
    while (now - pwm_window_start_time > config.pwmWindow_ms) {
      pwm_window_start_time += config.pwmWindow_ms;
    }
    pid_output = pid->Run(kiln_thermo->getTemperatureC());
    qtklog.debug(QTKLOG_DBG_PRIO_LOW, "pid_output = %d", pid_output);
    if (now - pwm_window_start_time < pid_output)
      ssr_on();
    else
      ssr_off();
  }
  // 0.5 is for rounding up
  lcd_update(kiln_thermo->getTemperatureC() + 0.5, ssr_state, ssr_state);

  // run handlers for subprocesses
  kiln_thermo->loop();
  housing_thermo->loop();
  mqtt_cli->loop();

  now = millis();
  delta_t = now - last_time;
  // if we have waited long enough, update the thermos
  if (delta_t >= config.mqttUpdateInterval_ms) {
    mqtt_publish_state();
    last_time = millis();
  }
  // do a minimal delay for the PWM and other service loops
  delay(config.min_loop_ms);
}

// set configuration variables after checking them
void config_set_hostname(const char *hostname) {
  if (!hostname) {
    qtklog.warn("empty hostname passed to %s", __func__);
    return;
  }
  strncpy(config.hostname, hostname, MAX_CFG_STR);
  qtklog.print("hostname set to %s", config.hostname);
}
void config_set_thermoUpdateInterval_ms(uint16_t thermoUpdateInterval_ms) {
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
  config.thermoUpdateInterval_ms = thermoUpdateInterval_ms;
  qtklog.print("thermocouple update interval = %d ms", config.thermoUpdateInterval_ms);
}
void config_set_pwmWindow_ms(uint16_t pwmWindow_ms) {
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
void config_set_mqttUpdateInterval_ms(uint16_t mqttUpdateInterval_ms) {
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
  qtklog.print("mqtt update interval  = %d ms", config.mqttUpdateInterval_ms);
}
void config_set_mqtt_enable_debug_messages(bool enable_messages) {
  config.mqtt_enable_debug_messages = enable_messages;
  if (mqtt_cli)
    mqtt_cli->enableDebuggingMessages(config.mqtt_enable_debug_messages);
}
void config_set_programUpdateInterval_ms(uint16_t programUpdateInterval_ms) {
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
  qtklog.print("program update interval = %d ms", config.programUpdateInterval_ms);
}
void config_set_pid_init_Kp(double Kp) {
  if (Kp < 0)
    Kp = 0;
  config.Kp = Kp;
  qtklog.print("PID initial Kp set to %g", config.Kp);
}
void config_set_pid_init_Ki(double Ki) {
  if (Ki < 0)
    Ki = 0;
  config.Ki = Ki;
  qtklog.print("PID initial Ki set to %g", config.Ki);
}
void config_set_pid_init_Kd(double Kd) {
  if (Kd < 0)
    Kd = 0;
  config.Kd = Kd;
  qtklog.print("PID initial Kd set to %g", config.Kd);
}


// handle mqtt config messages
void onConfigMessageReceived(const String &message) {
  qtklog.print("processing config update in json: %s", message.c_str());
  configLoad(message);
}

// handle mqtt state messages
void onSetStateMessageReceived(const String &message) {
  JsonDocument doc;
  double Kp = pid->GetKp(), Ki = pid->GetKi(), Kd = pid->GetKd();
  bool pid_enabled_at_start = pid_enabled;
  bool updateTunings = false;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    qtklog.warn("unable to deserialize JSON: %s", error.c_str());
    return;
  }
  // if this request turns on the pid enable, then
  // do it first before parsing others, like target temp
  // if it turns it off, make sure the ssr is off
  JsonVariant tmpObj = doc[MSG_PID_ENABLED];
  if (!tmpObj.isNull()) {
    pid_enabled = doc[MSG_PID_ENABLED];
    if (!pid_enabled && pid_enabled_at_start) {
      ssr_off();
      qtklog.print("stopping PID control");
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
      targetTemperatureC = tmp;
      if (pid_enabled && !pid_enabled_at_start) { // pid was off, start it 
        pid->SetTunings(config.Kp, config.Ki, config.Kd);
        pid->SetSampleTime(config.min_loop_ms); // rough estimate of how often run gets called
        pid->Start(kiln_thermo->getTemperatureC(), 0, targetTemperatureC);
        qtklog.print("starting PID with target temperature of %d C", targetTemperatureC);
      } else if (pid_enabled) { // pid was already enabled, just updating the set point
        pid->Setpoint(targetTemperatureC);
        qtklog.print("adjusting PID target temperature to %d C", targetTemperatureC);
      } else {
        qtklog.print("setting PID target temperature to %d C", targetTemperatureC);
      }
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KP) == 0 && pid_enabled) {
      Kp = doc[PREFS_PID_KP];
      if (Kp < 0)
        Kp = 0;
      updateTunings = true;
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KI) == 0 && pid_enabled) {
      Ki = doc[PREFS_PID_KI];
      if (Ki < 0)
        Ki = 0;
      updateTunings = true;
    } else if (strcmp(kv.key().c_str(), PREFS_PID_KD) == 0 && pid_enabled) {
      Kd = doc[PREFS_PID_KD];
      if (Kd < 0)
        Kd = 0;
      updateTunings = true;
    }
  }
  if (updateTunings && pid) {
    qtklog.print("updating live tunings to (Kp = %g, Ki = %g, Kd = %g)", Kp, Ki, Kd);
    pid->SetTunings(Kp, Ki, Kd);
  }
}
void onGetStateMessageReceived(const String &message) {
}

// when the connection to the mqtt has completed
void onConnectionEstablished(void) {
  char topic[MAX_BUF];

  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_CONFIG);
  mqtt_cli->subscribe(topic, onConfigMessageReceived);
  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_GET);
  mqtt_cli->subscribe(topic, onGetStateMessageReceived);
  snprintf(topic, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_SET);
  mqtt_cli->subscribe(topic, onSetStateMessageReceived);

  mqtt_publish_state();
}
