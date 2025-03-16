#include <WiFi.h>
#include "Arduino.h"
#include "max6675.h"

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
const char *topic = "qtkiln";
const char *mqtt_username = "qtkiln";
const char *mqtt_password = "hotashell";
const int mqtt_port = 1883;

float kiln_temperature, housing_temperature;

void setup() {
  // wait for MAX chip to stabilize
  delay(500);

  Serial.begin(9600);

  // Connect to WiFi
  WiFi.begin(ssid, sspw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
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
