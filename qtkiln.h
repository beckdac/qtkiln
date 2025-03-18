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

#define PID_KP 10.0
#define PID_KI 0.2
#define PID_KD 0.1

#define MIN_TARGET_TEMP 0
#define MAX_TARGET_TEMP 1100

#define MQTT_TOPIC_FMT "%s/%s"
#define MQTT_TOPIC_BASE "qtkiln"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CONFIG "config"
#define MQTT_TOPIC_SET "set"
#define MQTT_TOPIC_GET "get"

#define WIFI_SETUP_DELAY_MS 250

// configuration
#define PRFS_NAMESPACE MQTT_TOPIC_BASE
#define PRFS_PID_KI "initial_Ki"
#define PRFS_PID_KP "initial_Kp"
#define PRFS_PID_KD "initial_Kd"
#define PRFS_THRM_UPD_INT_MS "thrm_upd_int_ms"
#define PRFS_PWM_UPD_INT_MS "pwm_upd_int_ms"
#define PRFS_MQTT_UPD_INT_MS "mqtt_upd_int_ms"

#define MAX_CFG_STR 32
#define MAC_FMT_STR "%02X%02X%02X%02X%02X%02X"  
#define MIN_THERMO_UPDATE_MS 250
#define MAX_THERMO_UPDATE_MS 5000
#define MIN_PWM_UPDATE_MS 5000
#define MAX_PWM_UPDATE_MS 20000
#define MIN_MQTT_UPDATE_MS 250
#define MAX_MQTT_UPDATE_MS 15000

#define MAC_DEFAULT "c0:ff:ee:ca:fe:42"
#define PRFS_CONFIG_JSON "config.json"

#define MAX_BUF 256
#define MAX_MSG_BUF 256

#define MQTT_GET_MSG_TEMP "temperature_C"
#define MQTT_SET_MSG_TARGET_TEMP "target_temperature_C"
#define MQTT_GET_MSG_TARGET_TEMP MQTT_SET_MSG_TARGET_TEMP
#define MQTT_SET_MSG_PID_ENABLED "pid_enabled"
#define MQTT_GET_MSG_PID_ENABLED MQTT_SET_MSG_PID_ENABLED
#define MQTT_SET_MSG_PID_SETTINGS "pid_settings_Kp_Ki_Kd"
#define MQTT_GET_MSG_PID_SETTINGS MQTT_SET_MSG_PID_SETTINGS

#endif
