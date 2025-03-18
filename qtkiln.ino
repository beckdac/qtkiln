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
#include "qtkiln_thermo.h"

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
unsigned long pwm_window_start_time = 0;

// PID object
PID_v2 *pid = NULL;
boolean pid_enabled = false;
uint16_t target_temperature_C = 0;
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
struct Config {
  char mac[MAX_CFG_STR] = MAC_DEFAULT;
  char topic[MAX_CFG_STR] = "";
  uint16_t thermo_update_int_ms = 250;
  uint16_t pwm_update_int_ms = 5000;
  uint16_t mqtt_update_int_ms = 1000;
  uint8_t min_loop_ms = 5;
  double Kp = PID_KP, Ki = PID_KI, Kd = PID_KD;
} config;

void configLoad(const String &jsonString) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  config_set_thermo_update_int_ms(doc[PRFS_THRM_UPD_INT_MS] | config.thermo_update_int_ms);
  config_set_pwm_update_int_ms(doc[PRFS_PWM_UPD_INT_MS] | config.pwm_update_int_ms);
  config_set_mqtt_update_int_ms(doc[PRFS_MQTT_UPD_INT_MS] | config.mqtt_update_int_ms);
  config_set_pid_init_Kp(doc[PRFS_PID_KP] | config.Kp);
  config_set_pid_init_Ki(doc[PRFS_PID_KI] | config.Ki);
  config_set_pid_init_Kp(doc[PRFS_PID_KD] | config.Kd);
}

String configSerialize(void) {
  JsonDocument doc;
  String jsonString;

  doc[PRFS_THRM_UPD_INT_MS] = config.thermo_update_int_ms;
  doc[PRFS_PWM_UPD_INT_MS] = config.pwm_update_int_ms;
  doc[PRFS_MQTT_UPD_INT_MS] = config.mqtt_update_int_ms;
  doc[PRFS_PID_KP] = config.Kp;
  doc[PRFS_PID_KI] = config.Ki;
  doc[PRFS_PID_KD] = config.Kd;

  serializeJson(doc, jsonString);

  return jsonString;
}

void configLoadPrefs(void) {
  preferences.begin("qtkiln", true);
  String jsonString = preferences.getString(PRFS_CONFIG_JSON, String("{}"));
  preferences.end();
  Serial.print("read config: ");
  Serial.println(jsonString);
  configLoad(jsonString);
}

void configUpdatePrefs(void) {
  String jsonString = configSerialize();
  preferences.begin("qtkiln", false);
  preferences.putString(PRFS_CONFIG_JSON, jsonString);
  preferences.end();
  Serial.print("write config: ");
  Serial.println(jsonString);
}

// state variables are found above the loop function

// initialize the hardware and provide for any startup
// delays according to manufacturer data sheets
void setup() {
  uint8_t u8mac[6];

  delay(100);

  // initialize the serial for 115200 baud
  Serial.begin(115200);

  // lcd setup
  lcd.setBrightness(1);
  lcd.setSegments(LCD_BOOT);

  configLoadPrefs();

  // connect to wifi
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
  //mqtt_cli->enableHTTPWebUpdater("/");
  Serial.println("web updater listening");

  // initialize the thermocouples and get the first readingings
  // kiln
  kiln_thermo = new QTKilnThermo(config.thermo_update_int_ms, &kiln_thermocouple, NULL);
  kiln_thermo->begin();
  kiln_thermo->enable();
  // housing
  housing_thermo = new QTKilnThermo(config.thermo_update_int_ms, NULL, &housing_thermocouple);
  housing_thermo->begin();
  housing_thermo->enable();
  // get the readings
  Serial.print("kiln C = "); 
  Serial.print(kiln_thermo->getTemperatureC());
  Serial.print(" housing C = ");
  Serial.println(housing_thermo->getTemperatureC());

  // lcd setup
  // 0.5 is for rounding up
  lcd_update(kiln_thermo->getTemperatureC() + 0.5, false, false);

  configSerialize();
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

void mqtt_publish_state(bool active=false, bool pid_current=false) {
  JsonDocument doc;
  String jsonString;

  doc["kiln"]["time_ms"] = kiln_thermo->lastTime();
  doc["kiln"]["temperature_C"] = kiln_thermo->getTemperatureC();
  doc["housing"]["time_ms"] = kiln_thermo->lastTime();
  doc["housing"]["temperature_C"] = kiln_thermo->getTemperatureC();
  if (pid_enabled || active) {
    doc["pid_enabled"] = pid_enabled;
    doc["target_temperature_C"] = target_temperature_C;
    doc["duty_cycle"] = 100. * (double)pid_output / (double)config.pwm_update_int_ms;
  }
  if (pid_enabled || pid_current) {
    double Kp = pid->GetKp(), Ki = pid->GetKi(), Kd = pid->GetKd();
    doc["pid"]["Kp"] = Kp;
    doc["pid"]["Ki"] = Ki;
    doc["pid"]["Kd"] = Kd;
  }
  serializeJson(doc, jsonString);
  snprintf(buf1, MAX_BUF, MQTT_TOPIC_FMT, config.topic, MQTT_TOPIC_STATE);
  mqtt_cli->publish(buf1, jsonString);
}

void mqtt_publish_programs(void) {

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
    pid_output = pid->Run(kiln_thermo->getTemperatureC());
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
  if (delta_t >= config.mqtt_update_int_ms) {
    mqtt_publish_state();
    last_time = millis();
  }
  // do a minimal delay for the PWM and other service loops
  delay(config.min_loop_ms);
}

// set configuration variables after checking them
void config_set_thermo_update_int_ms(uint16_t thermo_update_int_ms) {
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
}
void config_set_pwm_update_int_ms(uint16_t pwm_update_int_ms) {
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
}
void config_set_mqtt_update_int_ms(uint16_t mqtt_update_int_ms) {
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
}
void config_set_pid_init_Kp(double Kp) {
  config.Kp = Kp;
  Serial.print("PID initial Kp set to ");
  Serial.println(config.Kp);
}
void config_set_pid_init_Ki(double Ki) {
  config.Ki = Ki;
  Serial.print("PID initial Ki set to ");
  Serial.println(config.Ki);
}
void config_set_pid_init_Kd(double Kd) {
  config.Kd = Kd;
  Serial.print("PID initial Kd set to ");
  Serial.println(config.Kd);
}


// handle mqtt config messages
void onConfigMessageReceived(const String &message) {
  Serial.print("processing config update in json: ");
  Serial.println(message);
  configLoad(message);
}

// handle mqtt state messages
void onSetStateMessageReceived(const String &message) {
  char msg[MAX_MSG_BUF];
  message.toCharArray(msg, MAX_MSG_BUF);

  // the format should be key=value so if no = found, bad msg
      //if (pid_enabled)
      //  pid->Setpoint(target_temperature_C);
      //mqtt_publish_target_temp();
      
  //if (!pid_enabled && val) {
        // turning on
        //pid->Start(kiln_thermo->getTemperatureC(), 0, target_temperature_C);
      //} else if (pid_enabled && !val) {
        // turning off
        //ssr_off();
      //pid_enabled=val;\

      //double Kp, Ki, Kd;
      //uint8_t parsed = sscanf(valptr, MQTT_SET_MSG_PID_SETTINGS_FMT, &Kp, &Ki, &Kd);
      //if ((Kp >= 0 & Ki >= 0 && Kd >= 0) && parsed == 3) {
//	Serial.print("updating instaneous tunings from mqtt Kp = ");
	//if (pid)
	 // pid->SetTunings(Kp, Ki, Kd);
}
void onGetStateMessageReceived(const String &message) {
  char msg[MAX_MSG_BUF];
  message.toCharArray(msg, MAX_MSG_BUF);

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
