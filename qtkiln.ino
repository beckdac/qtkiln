#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <Preferences.h>

#include <max6675.h>
#include <SlowPWM.h>
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
#define MQTT_TOPIC_KILN_TEMP_FMT "%lu %0.2f"
#define MQTT_TOPIC_HOUSING_FMT "%s/housing"
#define MQTT_TOPIC_HOUSING_TEMP_FMT "%lu %0.2f"
#define MQTT_SUBTOPIC_CFG_FMT "%s/config"
#define MQTT_SUBTOPIC_SET_FMT "%s/get"
#define MQTT_SUBTOPIC_GET_FMT "%s/set"
#define MQTT_ON_CONN_MSG "connected"
#include "mqtt_cred.h"
EspMQTTClient *mqtt_cli = NULL;

// PWM object
#define SSR_PIN 8
S_PWM *pwm = NULL;

// configuration
#define PRFS_THRM_UPD_INT_MS_FMT "thrm_upd_int_ms"
#define PRFS_PWM_UPD_INT_MS_FMT "pwm_upd_int_ms"
#define PRFS_MQTT_UPD_INT_MS_FMT "mqtt_upd_int_ms"
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
  char topic[MAX_CFG_STR];
  uint16_t thermo_update_int_ms = 250;
  uint16_t pwm_update_int_ms = 5000;
  uint16_t mqtt_update_int_ms = 1000;
  uint8_t min_loop_ms = 5;
} config;

// state variables are found above the loop function

// prototypes
void thermocouple_update(void);
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

  // check some config variables against mins
  set_thermo_update_int_ms(
     preferences.getUInt(PRFS_THRM_UPD_INT_MS_FMT, MIN_THERMO_UPDATE_MS));
  set_pwm_update_int_ms(
     preferences.getUInt(PRFS_PWM_UPD_INT_MS_FMT, MIN_PWM_UPDATE_MS));
  set_mqtt_update_int_ms(
     preferences.getUInt(PRFS_MQTT_UPD_INT_MS_FMT, MIN_MQTT_UPDATE_MS));

  // setup PWM
  pwm = new S_PWM(SSR_PIN, config.pwm_update_int_ms);
  pwm->setDuty(0); // force duty cycle to 0
  pwm->begin();
  Serial.println("PWM is running");

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

void loop() {
  // run handlers for subprocesses
  pwm->pwmLoop();
  kiln_thermo->loop();
  housing_thermo->loop();
  mqtt_cli->loop();

  now = millis();
  delta_t = now - last_time;
  // if we have waited long enough, update the thermos
  if (delta_t >= config.mqtt_update_int_ms) {
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_KILN_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_TOPIC_KILN_TEMP_FMT, kiln_thermo->lastTime(), kiln_thermo->readCelsius());
    mqtt_cli->publish(buf1, buf2);
    snprintf(buf1, MAX_BUF, MQTT_TOPIC_HOUSING_FMT, config.topic);
    snprintf(buf2, MAX_BUF, MQTT_TOPIC_HOUSING_TEMP_FMT, kiln_thermo->lastTime(), housing_thermo->readCelsius());
    mqtt_cli->publish(buf1, buf2);
    last_time = millis();
  }
  // do a minimal delay for the PWM and other service loops
  delay(config.min_loop_ms);
}

// set configuration variables after checking them
void set_thermo_update_int_ms(uint16_t thermo_update_int_ms) {
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
void set_pwm_update_int_ms(uint16_t pwm_update_int_ms) {
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
void set_mqtt_update_int_ms(uint16_t mqtt_update_int_ms) {
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


// handle mqtt config messages
void onConfigMessageReceived(const String &message) {
}

// handle mqtt state messages
void onSetStateMessageReceived(const String &message) {
}
void onGetStateMessageReceived(const String &message) {
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
