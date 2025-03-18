#ifndef _QTKILN_H_
#define _QTKILN_H_

// prototypes
void config_set_thermo_update_int_ms(uint16_t thermo_update_int_ms);
void config_set_pwm_update_int_ms(uint16_t pwm_update_int_ms);
void config_set_mqtt_update_int_ms(uint16_t mqtt_update_int_ms);
void config_set_pid_init_Ki(double Ki);
void config_set_pid_init_Kd(double Kd);
void config_set_pid_init_Kp(double Kp);
void onConnectionEstablished(void);
void onConfigMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);
void lcd_update(uint16_t val, bool bold, bool colon);

// max typical string buffer
#define MAX_BUF 256

// default initial PID params
#define PID_KP 10.0
#define PID_KI 0.2
#define PID_KD 0.1

// global temperature maxes
#define TARGET_TEMP_MIN 0
#define TARGET_TEMP_MAX 1100

// MQTT keywords
#define MQTT_TOPIC_FMT "%s/%s"
#define MQTT_TOPIC_BASE "qtkiln"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CONFIG "config"
#define MQTT_TOPIC_SET "set"
#define MQTT_TOPIC_GET "get"

#define MSG_TEMP "temperatureC"
#define MSG_TARGET_TEMP "targetTemperatureC"
#define MSG_PID_ENABLED "pidEnabled"

// configuration
#define MAC_DEFAULT "c0:ff:ee:ca:fe:42"
#define MAX_CFG_STR 32
#define MAC_FMT_STR "%02X%02X%02X%02X%02X%02X"  
// preferences and json names
#define PREFS_CONFIG_JSON "config.json"
#define PREFS_NAMESPACE MQTT_TOPIC_BASE
#define PREFS_PID_KI "Ki"
#define PREFS_PID_KP "Kp"
#define PREFS_PID_KD "Kd"
#define PREFS_THRM_UPD_INT_MS "thermocoupleUpdateInterval_ms"
#define PREFS_PWM_WINDOW_MS "PWMWindow_ms"
#define PREFS_MQTT_UPD_INT_MS "mqttUpdateInterval_ms"
#define PREFS_MQTT_ENABLE_DBG "mqttEnableDebugMessages"

// timings
#define WIFI_SETUP_DELAY_MS 250
#define MQTT_MIN_UPDATE_MS 1000
#define MQTT_MAX_UPDATE_MS 15000
#define THERMO_MIN_UPDATE_MS 250
#define THERMO_MAX_UPDATE_MS 5000
#define PWM_MIN_WINDOW_MS 5000
#define PWM_MAX_WINDOW_MS 20000

#endif
