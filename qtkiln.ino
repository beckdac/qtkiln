#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <Preferences.h>

#include <PID_v2.h>
#include <max6675.h>
#include <EspMQTTClient.h>

#include "qtkiln_thermo.h"

// thermocouple phy
#define MAXDO_PIN 5
#define MAXCS0_PIN 3
#define MAXCS1_PIN 2
#define MAXSCK_PIN 4
MAX6675 kiln_thermocouple(MAXSCK_PIN, MAXCS0_PIN, MAXDO_PIN);
MAX6675 housing_thermocouple(MAXSCK_PIN, MAXCS1_PIN, MAXDO_PIN);
QTKilnThermo *kiln_thermo;
QTKilnThermo *housing_thermo;
// wrappers to circumvent address of bound member function
float kiln_readCelsius(void) {
  return kiln_thermocouple.readCelsius();
}
float housing_readCelsius(void) {
  return housing_thermocouple.readCelsius();
}

// WiFi credentials
#define WIFI_SETUP_DELAY_MS 250
#include "wifi_cred.h"
const char *ssid = WIFI_SSID;
const char *sspw = WIFI_PASS;

// MQTT server
#define MQTT_MAX_TOPIC_STR 256
#define MQTT_TOPIC_BASE "qtkiln"
#define MQTT_TOPIC_FMT "%s/%s"
#define MQTT_TOPIC_KILN_FMT "%s/kiln"
#define MQTT_KILN_TEMP_FMT "%lu %0.2f"
#define MQTT_TOPIC_HOUSING_FMT "%s/housing"
#define MQTT_HOUSING_TEMP_FMT "%lu %0.2f"
#define MQTT_TOPIC_PID_ENABLED_FMT "%s/pid_enabled"
#define MQTT_PID_ENABLED_FMT "%u"
#define MQTT_TOPIC_PID_SETTINGS_FMT "%s/pid_settings_Kp_Ki_Kd"
#define MQTT_PID_SETTINGS_FMT "%g;%g;%g"
#define MQTT_SET_MSG_PID_SETTINGS_FMT MQTT_PID_SETTINGS_FMT
#define MQTT_SUBTOPIC_CFG_FMT "%s/config"
#define MQTT_SUBTOPIC_GET_FMT "%s/get"
#define MQTT_SUBTOPIC_SET_FMT "%s/set"
#define MQTT_TOPIC_DUTY_CYCLE_FMT "%s/duty_cycle"
#define MQTT_DUTY_CYCLE_FMT "%g"
#define MQTT_GET_MSG_DUTY_CYCLE "duty_cycle"
#define MQTT_TOPIC_TARGET_TEMP_FMT "%s/target_temperature_C"
#define MQTT_TARGET_TEMP_FMT "%lu"
#define MQTT_ON_CONN_MSG "connected"
#include "mqtt_cred.h"
EspMQTTClient *mqtt_cli = NULL;

// PWM object
#define SSR_PIN 8
unsigned long pwm_window_start_time = 0;

// PID object
PID_v2 *pid = NULL;
#define PID_KP 10.0
#define PID_KI 0.2
#define PID_KD 0.1
boolean pid_enabled = false;
uint16_t target_temperature_C = 0;
unsigned long pid_output = 0;
#define MIN_TARGET_TEMP 0
#define MAX_TARGET_TEMP 1100

// configuration
#define PRFS_PID_KI "initial_Ki"
#define PRFS_PID_KP "initial_Kp"
#define PRFS_PID_KD "initial_Kd"
#define PRFS_THRM_UPD_INT_MS "thrm_upd_int_ms"
#define PRFS_PWM_UPD_INT_MS "pwm_upd_int_ms"
#define PRFS_MQTT_UPD_INT_MS "mqtt_upd_int_ms"
Preferences preferences;
#define MAX_CFG_STR 32
#define MAC_FMT_STR "%02X%02X%02X%02X%02X%02X"  
#define MIN_THERMO_UPDATE_MS 250
#define MAX_THERMO_UPDATE_MS 5000
#define MIN_PWM_UPDATE_MS 5000
#define MAX_PWM_UPDATE_MS 20000
#define MIN_MQTT_UPDATE_MS 250
#define MAX_MQTT_UPDATE_MS 15000
struct config {
  char mac[MAX_CFG_STR] = "c0:ff:ee:ca:fe:42";
  char topic[MAX_CFG_STR] = "";
  uint16_t thermo_update_int_ms = 250;
  uint16_t pwm_update_int_ms = 5000;
  uint16_t mqtt_update_int_ms = 1000;
  uint8_t min_loop_ms = 5;
  double Kp = 0, Ki = 0, Kd = 0;
} config;

// state variables are found above the loop function

// prototypes
void config_set_thermo_update_int_ms(uint16_t thermo_update_int_ms, bool update_prefs);
void config_set_pwm_update_int_ms(uint16_t pwm_update_int_ms, bool update_prefs);
void config_set_mqtt_update_int_ms(uint16_t mqtt_update_int_ms, bool update_prefs);
void config_set_pid_init_Ki(double Ki, bool update_prefs);
void config_set_pid_init_Kd(double Kd, bool update_prefs);
void config_set_pid_init_Kp(double Kp, bool update_prefs);
void onConnectionEstablished(void);
void onConfigMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);
void onStateSetMessageReceived(const String &message);

// initialize the hardware and provide for any startup
// delays according to manufacturer data sheets
void setup() {
  uint8_t u8mac[6];

  // load up the preferences so we can overwrite
  // the config defaults
  preferences.begin("qtkiln", false);
  //preferences.clear();

  // wait for MAX chip to stabilize
  delay(500);

  // initialize the serial for 115200 baud
  Serial.begin(115200);

  // connect to wifi
  WiFi.begin(ssid, sspw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_SETUP_DELAY_MS);
  }

  // setup the config structure

  // get the mac
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, u8mac);
  if (ret == ESP_OK) {
    snprintf(config.mac, MAX_CFG_STR, MAC_FMT_STR,
	u8mac[0], u8mac[1], u8mac[2],
	u8mac[3], u8mac[4], u8mac[5]);
    Serial.print("MAC for this device is ");
    Serial.println(config.mac);
  } else {
    Serial.println("failed to read MAC address");
  }
  // setup the topic based on the mac
  snprintf(config.topic, MAX_CFG_STR, MQTT_TOPIC_FMT,
	MQTT_TOPIC_BASE, config.mac);
  Serial.print("topic root for this device is ");
  Serial.println(config.topic);

  // set the variables from defaults and init pieces
  config_set_thermo_update_int_ms(
     preferences.getUShort(PRFS_THRM_UPD_INT_MS, MIN_THERMO_UPDATE_MS), false);
  config_set_pwm_update_int_ms(
     preferences.getUShort(PRFS_PWM_UPD_INT_MS, MIN_PWM_UPDATE_MS), false);
  config_set_mqtt_update_int_ms(
     preferences.getUShort(PRFS_MQTT_UPD_INT_MS, MIN_MQTT_UPDATE_MS), false);
  config_set_pid_init_Kp(preferences.getDouble(PRFS_PID_KP, PID_KP), false);
  config_set_pid_init_Ki(preferences.getDouble(PRFS_PID_KI, PID_KI), false);
  config_set_pid_init_Kd(preferences.getDouble(PRFS_PID_KD, PID_KD), false);

  // setup PWM
  pinMode(SSR_PIN, OUTPUT);
  // always turn it off incase we are coming back from a reset
  digitalWrite(SSR_PIN, LOW);
  Serial.print("PWM cycle window in ms is ");
  Serial.println(config.pwm_update_int_ms);
  pwm_window_start_time = millis();

  // setup PID
  pid = new PID_v2(config.Kp, config.Ki, config.Kd, PID::Direct);
  pid->SetOutputLimits(0, config.pwm_update_int_ms);
  

  // start the mqtt client
  mqtt_cli = new EspMQTTClient(WIFI_SSID, WIFI_PASS, MQTT_BROKER,
    MQTT_USER, MQTT_PASS, config.mac, MQTT_PORT);
  Serial.println("MQTT client connected");
  //mqtt_cli->enableDebuggingMessages(true);
  mqtt_cli->enableHTTPWebUpdater("/");
  Serial.println("web updater listening");

  // do first thermocouple reading
  kiln_thermo = new QTKilnThermo(config.thermo_update_int_ms, &kiln_readCelsius);
  kiln_thermo->begin();
  kiln_thermo->enable();
  housing_thermo = new QTKilnThermo(config.thermo_update_int_ms, &housing_readCelsius);
  housing_thermo->begin();
  housing_thermo->enable();
  Serial.print("kiln C = "); 
  Serial.print(kiln_thermo->readCelsius());
  Serial.print(" housing C = ");
  Serial.println(housing_thermo->readCelsius());
}

// state or preallocated variables for loop
unsigned long last_time = 0, now, delta_t;
#define MAX_BUF 256
char buf1[MAX_BUF], buf2[MAX_BUF];

void mqtt_publish_temps(void) {
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_KILN_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_KILN_TEMP_FMT, kiln_thermo->lastTime(), kiln_thermo->readCelsius());
    mqtt_cli->publish(buf1, buf2);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_HOUSING_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_HOUSING_TEMP_FMT, kiln_thermo->lastTime(), housing_thermo->readCelsius());
    mqtt_cli->publish(buf1, buf2);
}
void mqtt_publish_pid_enabled(void) {
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_PID_ENABLED_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_PID_ENABLED_FMT, pid_enabled);
    mqtt_cli->publish(buf1, buf2);
}
void mqtt_publish_pid_settings(void) {
    double Kp = pid->GetKp(), Ki = pid->GetKi(), Kd = pid->GetKd();

    snprintf(buf1, MAX_BUF, MQTT_TOPIC_PID_SETTINGS_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_PID_SETTINGS_FMT, Kp, Ki, Kd);
    mqtt_cli->publish(buf1, buf2);
}
void mqtt_publish_duty_cycle(void) {
    double duty_cycle = 100. * (double)pid_output / (double)config.pwm_update_int_ms;
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_DUTY_CYCLE_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_DUTY_CYCLE_FMT, duty_cycle);
    mqtt_cli->publish(buf1, buf2);
}
void mqtt_publish_target_temp(void) {
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_TARGET_TEMP_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_TARGET_TEMP_FMT, target_temperature_C);
    mqtt_cli->publish(buf1, buf2);
}

bool ssr_state = false;
void ssr_on(void) {
  if (ssr_state)
    return;
  //Serial.print("SSR ");
  //Serial.print(millis());
  //Serial.println(" on");
  ssr_state = true;
  digitalWrite(SSR_PIN, HIGH);
}

void ssr_off(void) {
  if (ssr_state) {
    //Serial.print("SSR ");
    //Serial.print(millis());
    //Serial.println(" off");
    ssr_state = false;
    digitalWrite(SSR_PIN, LOW);
  }
}

void loop() {
  now = millis();
  if (pid_enabled) {
    // shift the next start time into the future
    while (now - pwm_window_start_time > config.pwm_update_int_ms) {
      pwm_window_start_time += config.pwm_update_int_ms;
    }
    pid_output = pid->Run(kiln_thermo->readCelsius());
    if (now - pwm_window_start_time < pid_output)
      ssr_on();
    else
      ssr_off();
  }

  // run handlers for subprocesses
  kiln_thermo->loop();
  housing_thermo->loop();
  mqtt_cli->loop();

  now = millis();
  delta_t = now - last_time;
  // if we have waited long enough, update the thermos
  if (delta_t >= config.mqtt_update_int_ms) {
    mqtt_publish_temps();
    last_time = millis();
  }
  // do a minimal delay for the PWM and other service loops
  delay(config.min_loop_ms);
}

// set configuration variables after checking them
void config_set_thermo_update_int_ms(uint16_t thermo_update_int_ms, bool update_prefs) {
  if (thermo_update_int_ms < MIN_THERMO_UPDATE_MS) {
    Serial.print("thermocouple update interval must be > ");
    Serial.print(MIN_THERMO_UPDATE_MS);
    Serial.println(" ms ... forcing to min");
    thermo_update_int_ms = MIN_THERMO_UPDATE_MS;
  }
  if (thermo_update_int_ms > MAX_THERMO_UPDATE_MS) {
    Serial.print("thermocouple update interval must be < ");
    Serial.print(MAX_THERMO_UPDATE_MS);
    Serial.println(" ms ... forcing to max");
    thermo_update_int_ms = MAX_THERMO_UPDATE_MS;
  }
  config.thermo_update_int_ms = thermo_update_int_ms;
  Serial.print("thermocouple update interval (ms) = ");
  Serial.println(config.thermo_update_int_ms);
  if (update_prefs)
     preferences.putUShort(PRFS_THRM_UPD_INT_MS, config.thermo_update_int_ms);
}
void config_set_pwm_update_int_ms(uint16_t pwm_update_int_ms, bool update_prefs) {
  if (pwm_update_int_ms < MIN_PWM_UPDATE_MS) {
    Serial.print("PWM update interval must be > ");
    Serial.print(MIN_PWM_UPDATE_MS);
    Serial.println(" ms ... forcing to min");
    pwm_update_int_ms = MIN_PWM_UPDATE_MS;
  }
  if (pwm_update_int_ms > MAX_PWM_UPDATE_MS) {
    Serial.print("PWM update interval must be < ");
    Serial.print(MAX_PWM_UPDATE_MS);
    Serial.println(" ms ... forcing to max");
    pwm_update_int_ms = MAX_PWM_UPDATE_MS;
  }
  config.pwm_update_int_ms = pwm_update_int_ms;
  Serial.print("PWM update interval (ms) = ");
  Serial.println(config.pwm_update_int_ms);
  if (update_prefs)
     preferences.putUShort(PRFS_PWM_UPD_INT_MS, config.pwm_update_int_ms);
}
void config_set_mqtt_update_int_ms(uint16_t mqtt_update_int_ms, bool update_prefs) {
  if (mqtt_update_int_ms < MIN_MQTT_UPDATE_MS) {
    Serial.print("mqtt update interval must be > ");
    Serial.print(MIN_MQTT_UPDATE_MS);
    Serial.println(" ms ... forcing to min");
    mqtt_update_int_ms = MIN_MQTT_UPDATE_MS;
  }
  if (mqtt_update_int_ms > MAX_MQTT_UPDATE_MS) {
    Serial.print("MQTT update interval must be < ");
    Serial.print(MAX_MQTT_UPDATE_MS);
    Serial.println(" ms ... forcing to max");
    mqtt_update_int_ms = MAX_MQTT_UPDATE_MS;
  }
  config.mqtt_update_int_ms = mqtt_update_int_ms;
  Serial.print("mqtt update interval (ms) = ");
  Serial.println(config.mqtt_update_int_ms);
  if (update_prefs)
     preferences.putUShort(PRFS_MQTT_UPD_INT_MS, config.mqtt_update_int_ms);
}
void config_set_pid_init_Kp(double Kp, bool update_prefs) {
  config.Kp = Kp;
  Serial.print("PID initial Kp set to ");
  Serial.println(config.Kp);
  if (update_prefs)
     preferences.putDouble(PRFS_PID_KP, config.Kp);
}
void config_set_pid_init_Ki(double Ki, bool update_prefs) {
  config.Ki = Ki;
  Serial.print("PID initial Ki set to ");
  Serial.println(config.Ki);
  if (update_prefs)
     preferences.putDouble(PRFS_PID_KI, config.Ki);
}
void config_set_pid_init_Kd(double Kd, bool update_prefs) {
  config.Kd = Kd;
  Serial.print("PID initial Kd set to ");
  Serial.println(config.Kd);
  if (update_prefs)
     preferences.putDouble(PRFS_PID_KD, config.Kd);
}


// handle mqtt config messages
#define MAX_MSG_BUF 256
void onConfigMessageReceived(const String &message) {
  bool config_updated = false;
  char msg[MAX_MSG_BUF];
  message.toCharArray(msg, MAX_MSG_BUF);

  // the format should be key=value so if no = found, bad msg
  char *eqptr = strchr(msg, '=');
  if (eqptr) {
    char *valptr = eqptr+1;
    *eqptr = NULL;
    if (strcmp(msg, PRFS_THRM_UPD_INT_MS) == 0) {
      unsigned long val = strtoul(valptr, NULL, 0);
      config_set_thermo_update_int_ms(val, true);
      config_updated = true;
    } else if (strcmp(msg, PRFS_PWM_UPD_INT_MS) == 0) {
      unsigned long val = strtoul(valptr, NULL, 0);
      config_set_pwm_update_int_ms(val, true);
      config_updated = true;
    } else if (strcmp(msg, PRFS_MQTT_UPD_INT_MS) == 0) {
      unsigned long val = strtoul(valptr, NULL, 0);
      config_set_mqtt_update_int_ms(val, true);
      config_updated = true;
    } else if (strcmp(msg, PRFS_PID_KP) == 0) {
      double val = strtod(valptr, NULL);
      config_set_pid_init_Kp(val, true);
      config_updated = true;
    } else if (strcmp(msg, PRFS_PID_KI) == 0) {
      double val = strtod(valptr, NULL);
      config_set_pid_init_Ki(val, true);
      config_updated = true;
    } else if (strcmp(msg, PRFS_PID_KD) == 0) {
      double val = strtod(valptr, NULL);
      config_set_pid_init_Kd(val, true);
      config_updated = true;
    }
  }

  if (config_updated) {
    Serial.println("system configuration updated, restarting...");
    delay(1000);
    esp_restart();
  }
}

#define MQTT_GET_MSG_TEMP "temperature_C"
#define MQTT_SET_MSG_TARGET_TEMP "target_temperature_C"
#define MQTT_GET_MSG_TARGET_TEMP MQTT_SET_MSG_TARGET_TEMP
#define MQTT_SET_MSG_PID_ENABLED "pid_enabled"
#define MQTT_GET_MSG_PID_ENABLED MQTT_SET_MSG_PID_ENABLED
#define MQTT_SET_MSG_PID_SETTINGS "pid_settings_Kp_Ki_Kd"
#define MQTT_GET_MSG_PID_SETTINGS MQTT_SET_MSG_PID_SETTINGS
// handle mqtt state messages
void onSetStateMessageReceived(const String &message) {
  char msg[MAX_MSG_BUF];
  message.toCharArray(msg, MAX_MSG_BUF);

  // the format should be key=value so if no = found, bad msg
  char *eqptr = strchr(msg, '=');
  if (eqptr) {
    char *valptr = eqptr+1;
    *eqptr = NULL;
    if (strcmp(msg, MQTT_SET_MSG_TARGET_TEMP) == 0) {
      unsigned long val = strtoul(valptr, NULL, 0);
      if (val < MIN_TARGET_TEMP) {
        val = MIN_TARGET_TEMP;
      } else if (val > MAX_TARGET_TEMP) {
        val = MAX_TARGET_TEMP;
      }
      Serial.print("target temperature set via mqtt to (C) ");
      Serial.println(val);
      target_temperature_C = val;
      pid->Setpoint(target_temperature_C);
    } else if (strcmp(msg, MQTT_SET_MSG_PID_ENABLED) == 0) {
      bool val = strtoul(valptr, NULL, 0);
      Serial.print("pid enabled is set to ");
      Serial.println(val);
      if (!pid_enabled && val) {
        // turning on
        Serial.print("pid turning on with target temperature of (C) ");
	Serial.println(target_temperature_C);
        pid->Start(kiln_thermo->readCelsius(), 0, target_temperature_C);
      } else if (pid_enabled && !val) {
        // turning off
        Serial.println("pid turning off");
        ssr_off();
      }
      pid_enabled=val;
    } else if (strcmp(msg, MQTT_SET_MSG_PID_SETTINGS) == 0) {
      double Kp, Ki, Kd;
      uint8_t parsed = sscanf(valptr, MQTT_SET_MSG_PID_SETTINGS_FMT, &Kp, &Ki, &Kd);
      if ((Kp >= 0 & Ki >= 0 && Kd >= 0) && parsed == 3) {
	Serial.print("updating instaneous tunings from mqtt Kp = ");
	Serial.print(Kp);
	Serial.print(" Ki = ");
	Serial.print(Ki);
	Serial.print(" Kd = ");
	Serial.println(Kd);
	if (pid)
	  pid->SetTunings(Kp, Ki, Kd);
      }
    }
  }
}
void onGetStateMessageReceived(const String &message) {
  char msg[MAX_MSG_BUF];
  message.toCharArray(msg, MAX_MSG_BUF);

  // the format should be key=value so if no = found, bad msg
  if (strcmp(msg, MQTT_GET_MSG_TEMP) == 0) {
    mqtt_publish_temps();
  } else if (strcmp(msg, MQTT_GET_MSG_TARGET_TEMP) == 0) {
    mqtt_publish_target_temp();
  } else if (strcmp(msg, MQTT_GET_MSG_PID_ENABLED) == 0) {
    mqtt_publish_pid_enabled();
  } else if (strcmp(msg, MQTT_GET_MSG_PID_SETTINGS) == 0) {
    mqtt_publish_pid_settings();
  } else if (strcmp(msg, MQTT_GET_MSG_DUTY_CYCLE) == 0) {
    mqtt_publish_duty_cycle();
  }
}

// when the connection to the mqtt has completed
void onConnectionEstablished(void) {
  char topic[MQTT_MAX_TOPIC_STR];

  snprintf(topic, MQTT_MAX_TOPIC_STR, MQTT_SUBTOPIC_CFG_FMT, config.topic);
  mqtt_cli->subscribe(topic, onConfigMessageReceived);

  snprintf(topic, MQTT_MAX_TOPIC_STR, MQTT_SUBTOPIC_GET_FMT, config.topic);
  mqtt_cli->subscribe(topic, onGetStateMessageReceived);
  snprintf(topic, MQTT_MAX_TOPIC_STR, MQTT_SUBTOPIC_SET_FMT, config.topic);
  mqtt_cli->subscribe(topic, onSetStateMessageReceived);

  mqtt_cli->publish(config.topic, MQTT_ON_CONN_MSG);
}
