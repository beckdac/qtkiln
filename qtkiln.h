#ifndef _QTKILN_H_
#define _QTKILN_H_

#include "wifi_cred.h"
#include "mqtt_cred.h"

#define NTP_SERVER "pool.ntp.org"

// prototypes
void config_setPwmWindow_ms(uint16_t pwm_update_int_ms);
void config_setMqttUpdateInterval_ms(uint16_t mqtt_update_int_ms);
void config_setProgramUpdateInterval_ms(uint16_t program_update_int_ms);
void config_setPidInitialKi(double Ki);
void config_setPidInitialKd(double Kd);
void config_setPidInitialKp(double Kp);
void config_setTimezone(const char *timeZone);
void config_setHostname(const char *hostname);
void onConnectionEstablished(void);
void onConfigMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);
void lcd_update(float temp, bool bold, bool colon);
void ssr_on(void);
void ssr_off(void);

// max typical string buffer
#define MAX_BUF 256

// default initial PID params
#define PID_KP 20.0
#define PID_KI 0.2
#define PID_KD 0.1

#define QTKILN_TASK_CORE 1
// config structure used by all the modules
#define MAX_CFG_STR 64
#define MAC_DEFAULT "c0:ff:ee:ca:fe:42"
#define HOSTNAME_DEFAULT "qtkiln_coffeecafe42"
struct Config {
  char wifiSsid[MAX_CFG_STR] = WIFI_SSID;
  char wifiPassword[MAX_CFG_STR] = WIFI_PASS;
  char mqttHostIp[MAX_CFG_STR] = MQTT_BROKER;
  uint16_t mqttPort = MQTT_PORT;
  char mqttUsername[MAX_CFG_STR] = MQTT_USER;
  char mqttPassword[MAX_CFG_STR] = MQTT_PASS;
  char hostname[MAX_CFG_STR] = HOSTNAME_DEFAULT;
  char mac[MAX_CFG_STR] = MAC_DEFAULT;
  char topic[MAX_CFG_STR] = "";
  char timezone[MAX_CFG_STR] = "PST8PDT,M3.2.0,M11.1.0";  // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
  uint16_t debugPriority = 0;
  uint16_t pwmWindow_ms = 5000;
  uint16_t mqttUpdateInterval_ms = 1000;
  uint16_t programUpdateInterval_ms = 1000;
  bool mqttEnableDebugMessages = false;
  uint8_t mainLoop_ms = 5;
  double Kp = PID_KP, Ki = PID_KI, Kd = PID_KD;
  struct ConfigThermo {
    uint16_t updateInterval_ms = 250;
    float filterCutoffFrequency_Hz = 1.;
  } kiln, housing;
  bool alarmOnSSR = false;			// should the alarm sound when the SSR is on, mostly for testing
  struct HomeAssistantConfig {
    bool enabled = true;
    char configTopicFmt[MAX_CFG_STR] = "homeassistant/%s/%s_%s/config";
    char deviceIdFmt[MAX_CFG_STR] = "qtkiln_%s";
    char deviceNameFmt[MAX_CFG_STR] = "qtkiln_%s";
    char componentFmt[MAX_CFG_STR] = "qtkiln_%s_cmp_%s";
    char model[MAX_CFG_STR] = "qtkiln basic";
    char manufacturer[MAX_CFG_STR] = "koren labs foundry";
    char softwareRev[MAX_CFG_STR] = "basic v1.0";
    char hardwareRev[MAX_CFG_STR] = "qtkiln basic";
    char originName[MAX_CFG_STR] = "qtkiln basic v1.0";
    char originSoftwareRev[MAX_CFG_STR] = "qtkiln basic software v1.0";
    char originUrl[MAX_CFG_STR] = "https://github.com/beckdac/qtkiln";
  } homeAss;
};

// global temperature maxes
#define TARGET_TEMP_MIN_C 0
#define TARGET_TEMP_MAX_C 1000
#define ALARM_TEMP_C 900
#define EMERGENCY_SHUTDOWN_TEMP_C 1100
#define ALARM_HOUSING_TEMP_C 60
#define EMERGENCY_HOUSING_SHUTDOWN_TEMP_C 80

// MQTT keywords
#define MQTT_TOPIC_FMT "%s/%s"
#define MQTT_TOPIC_BASE "qtkiln"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CONFIG "config"
#define MQTT_TOPIC_PROGRAM "program"
#define MQTT_TOPIC_SET "set"
#define MQTT_TOPIC_GET "get"

#define MSG_TEMP "temp_C"
#define MSG_TARGET_TEMP "targetTemp_C"
#define MSG_PID_ENABLED "pidEnabled"

// configuration
#define MAC_FMT_STR "%02X%02X%02X%02X%02X%02X"  
#define QTKILN_BOOT_DELAY_MS 2000
// preferences and json names
#define PREFS_WIFI_SSID "wifiSsid"
#define PREFS_WIFI_PWD "wifiPassword"
#define PREFS_MQTT_HOSTIP "mqttHostIp"
#define PREFS_MQTT_USERNAME "mqttUsername"
#define PREFS_MQTT_PWD "mqttPassword"
#define PREFS_MQTT_PORT "mqttPort"
#define PREFS_DBG_PRIORITY "debugPriority"
#define PREFS_CONFIG_JSON "config.json"
#define PREFS_PROGRAM_LIST "programs.json"
#define CONFIG_RESET "reset"
#define CONFIG_SAVE "save"
#define PREFS_NAMESPACE MQTT_TOPIC_BASE
#define PREFS_PID_KP "Kp"
#define PREFS_PID_KI "Ki"
#define PREFS_PID_KD "Kd"
#define PREFS_HOSTNAME "hostname"
#define PREFS_KILN "kiln"
#define PREFS_HOUSING "housing"
#define PREFS_TIMEZONE "tz"
#define PREFS_FLTR_CUT_FREQ_HZ "filterCutoffFrequency_Hz"
#define PREFS_THRM_UPD_INT_MS "thermocoupleUpdateInterval_ms"
#define PREFS_PWM_WINDOW_MS "PWMWindow_ms"
#define PREFS_MQTT_UPD_INT_MS "mqttUpdateInterval_ms"
#define PREFS_MQTT_ENABLE_DBG "mqttEnableDebugMessages"
#define PREFS_PGM_UPD_INT_MS "programUpdateInterval_ms"

// timings
#define WIFI_SETUP_DELAY_MS 250
#define MQTT_MIN_UPDATE_MS 1000
#define MQTT_MAX_UPDATE_MS 15000
#define THERMO_MIN_UPDATE_MS 250
#define THERMO_MAX_UPDATE_MS 5000
#define PWM_MIN_WINDOW_MS 5000
#define PWM_MAX_WINDOW_MS 20000
#define PGM_MIN_UPDATE_MS 1000
#define PGM_MAX_UPDATE_MS 60000

#endif
