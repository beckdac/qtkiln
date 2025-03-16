#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <max6675.h>

// thermocouple phy
const unsigned int MAXDO = 5;
const unsigned int MAXCS0 = 3;
const unsigned int MAXCS1 = 2;
const unsigned int MAXSCK = 4;
MAX6675 kiln_thermocouple(MAXSCK, MAXCS0, MAXDO);
MAX6675 housing_thermocouple(MAXSCK, MAXCS1, MAXDO);

// WiFi credentials
#include "wifi_cred.h"
const char *ssid = WIFI_SSID;
const char *sspw = WIFI_PASS;

// MQTT server
const char *mqtt_broker = "192.168.1.3";
const char *topic_base = "qtkiln";
const char *mqtt_username = "qtkiln";
const char *mqtt_password = "hotashell";
const int mqtt_port = 1883;

// configuration
#define MAX_CFG_STR 24
struct config {
  char mac[MAX_CFG_STR] = "c0:ff:ee:ca:fe:42";
  char topic[MAX_CFG_STR];
  uint16_t update_interval_ms;
} config;

// state variables are found above the loop function

// prototypes
void thermocouple_update(void);

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
	topic_base, config.mac);

  // check some config variables against mins
  if (config.update_interval_ms < 250) {
    Serial.println("MAX6675 update interval must be > 250 ms, forcing to 250 ms");
    config.update_interval_ms = 250;
  }

  // do first thermocouple reading
  thermocouple_update();
}

// state variables associated with the loop
float kiln_temperature, housing_temperature;
unsigned long last_time, now, delta_t;

void loop() {
  now = millis();
  delta_t = now - last_time;
  // if we have waited long enough, update the thermos
  if (delta_t >= config.update_interval_ms) {
    thermocouple_update();

    Serial.print("kiln C = "); 
    Serial.print(kiln_temperature);
    Serial.print(" housing C = ");
    Serial.println(housing_temperature);
    now = millis();
  }
  delay(config.update_interval_ms - (now-last_time));
}

// read the thermocouples and update hte last updated 
void thermocouple_update(void) {
  kiln_temperature = kiln_thermocouple.readCelsius();
  housing_temperature = housing_thermocouple.readCelsius();
  last_time = millis();
}

