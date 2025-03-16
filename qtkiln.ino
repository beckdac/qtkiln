#include <WiFi.h>
#include <esp_wifi.h>
#include "Arduino.h"
#include "max6675.h"

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
struct config {
  char mac[24] = "c0:ff:ee:ca:fe:42";
  char topic[24];
  uint16_t sample_kiln_ms;
  uint16_t sample_housing_ms;
} config;

// state variables
float kiln_temperature, housing_temperature;

// initialize the hardware and provide for any startup
// delays according to manufacturer data sheets
void setup() {
  uint8_t u8mac[6];

  // wait for MAX chip to stabilize
  delay(500);

  // initialize the serial for 9600 baud
  Serial.begin(9600);

  // connect to wifi
  WiFi.begin(ssid, sspw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  // setup the config structure
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    snprintf(config.mac, 24, 
		  "%02x:%02x:%02x:%02x:%02x:%02x",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("failed to read MAC address");
  }
}

void loop() {
  kiln_temperature = kiln_thermocouple.readCelsius();
  housing_temperature = housing_thermocouple.readCelsius();
  Serial.print("kiln C = "); 
  Serial.print(kiln_temperature);
  Serial.print(" housing C = ");
  Serial.println(housing_temperature);
  
  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(250);
}

