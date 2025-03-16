#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <max6675.h>
#include <SlowPWM.h>
#include <EspMQTTClient.h>

// thermocouple phy
#define MAXDO_PIN 5
#define MAXCS0_PIN 3
#define MAXCS1_PIN 2
#define MAXSCK_PIN 4
MAX6675 kiln_thermocouple(MAXSCK_PIN, MAXCS0_PIN, MAXDO_PIN);
MAX6675 housing_thermocouple(MAXSCK_PIN, MAXCS1_PIN, MAXDO_PIN);

// WiFi credentials
#include "wifi_cred.h"
const char *ssid = WIFI_SSID;
const char *sspw = WIFI_PASS;

// MQTT server
#define MQTT_MAX_TOPIC_STR 256
#define MQTT_TOPIC_BASE "qtkiln"
#define MQTT_TOPIC_FMT "qtkiln/%s"
#define MQTT_SUBTOPIC_CFG_FMT "%s/config"
#define MQTT_SUBTOPIC_SET_FMT "%s/get"
#define MQTT_SUBTOPIC_GET_FMT "%s/set"
#include "mqtt_cred.h"
EspMQTTClient *mqtt_cli = NULL;

// PWM object
#define SSR_PIN 8
S_PWM *pwm = NULL;

// configuration
#define MAX_CFG_STR 24
struct config {
  char mac[MAX_CFG_STR] = "c0:ff:ee:ca:fe:42";
  char topic[MAX_CFG_STR];
  uint16_t thermo_update_int_ms = 250;
  uint16_t PWM_update_int_ms = 5000;
  uint8_t min_loop_ms = 5;
} config;
#define MIN_THERMO_UPDATE_MS 250
#define MAX_THERMO_UPDATE_MS 5000
#define MIN_PWM_UPDATE_MS 5000
#define MAX_PWM_UPDATE_MS 20000

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

  // wait for MAX chip to stabilize
  delay(500);

  // initialize the serial for 115200 baud
  Serial.begin(115200);

  // connect to wifi
  WiFi.begin(ssid, sspw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  // setup the config structure

  // get the mac
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, u8mac);
  if (ret == ESP_OK) {
    snprintf(config.mac, MAX_CFG_STR, 
      "%02x:%02x:%02x:%02x:%02x:%02x",
	u8mac[0], u8mac[1], u8mac[2],
	u8mac[3], u8mac[4], u8mac[5]);
  } else {
    Serial.println("failed to read MAC address");
  }
  // setup the topic based on the mac
  snprintf(config.topic, MAX_CFG_STR, "%s/%s",
	TOPIC_BASE, config.mac);

  // check some config variables against mins
  if (config.thermo_update_int_ms < MIN_THERMO_UPDATE_MS) {
    Serial.print("thermocouple update interval must be > ");
    Serial.print(MIN_THERMO_UPDATE_MS);
    Serial.println(" ms ... forcing to min");
    config.thermo_update_int_ms = MIN_THERMO_UPDATE_MS;
  }
  if (config.thermo_update_int_ms > MAX_THERMO_UPDATE_MS) {
    Serial.print("thermocouple update interval must be < ");
    Serial.print(MAX_THERMO_UPDATE_MS);
    Serial.println(" ms ... forcing to max");
    config.thermo_update_int_ms = MAX_THERMO_UPDATE_MS;
  }
  if (config.PWM_update_int_ms < MIN_PWM_UPDATE_MS) {
    Serial.print("PWM update interval must be > ");
    Serial.print(MIN_PWM_UPDATE_MS);
    Serial.println(" ms ... forcing to min");
    config.PWM_update_int_ms = MIN_PWM_UPDATE_MS;
  }
  if (config.PWM_update_int_ms > MAX_PWM_UPDATE_MS) {
    Serial.print("PWM update interval must be < ");
    Serial.print(MAX_PWM_UPDATE_MS);
    Serial.println(" ms ... forcing to max");
    config.PWM_update_int_ms = MAX_PWM_UPDATE_MS;
  }

  // setup PWM
  pwm = new S_PWM(SSR_PIN, config.PWM_update_int_ms);
  pwm->setDuty(0); // force duty cycle to 0
  pwm->begin();

  // start the mqtt client
  mqtt_cli = new EspMQTTClient(WIFI_SSID, WIFI_PASS, MQTT_BROKER,
    MQTT_USER, MQTT_PASS, config.mac, MQTT_PORT);

  // do first thermocouple reading
  thermocouple_update();
}

// state variables associated with the loop
float kiln_temperature, housing_temperature;
// some of these could be declared in loop, but then
// they would take allocated time on the heap each time
// so we do them here instead
unsigned long last_time, now, delta_t;

void loop() {
  pwm->pwmLoop(); // run the PWM handler

  now = millis();
  delta_t = now - last_time;
  // if we have waited long enough, update the thermos
  if (delta_t >= config.thermo_update_int_ms) {
    thermocouple_update();

    Serial.print("kiln C = "); 
    Serial.print(kiln_temperature);
    Serial.print(" housing C = ");
    Serial.println(housing_temperature);
    now = millis();
  }
  // do a minimal delay for the PWM and other service loops
  delay(config.min_loop_ms);
}

// read the thermocouples and update hte last updated 
void thermocouple_update(void) {
  kiln_temperature = kiln_thermocouple.readCelsius();
  housing_temperature = housing_thermocouple.readCelsius();
  last_time = millis();
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
  char *buf[MAX_TOPIC_STR];
  snprintf(buf, MAX_TOPIC_STR, MQTT_SUBTOPIC_CFG_FMT, config.topic);
  mqtt_cli.subscribe(
"mytopic/test", [] (const String &payload)  {
    Serial.println(payload);
  });

  mqtt_cli.publish("mytopic/test", "This is a message");
}
