# qtkiln
Open source controller for a table top kiln


`wifi_cred.h` should look something like:
```
#define WIFI_SSID "ssid"
#define WIFI_PASS "password"
```

`mqtt_cred.h` should look something like:
```
#define MQTT_BROKER "192.168.1.42"
#define MQTT_USER "qtkiln"
#define MQTT_PASS "password"
#define MQTT_PORT 1883
```

Using `mosquitto` pub and sub clients on linux, you can modify the settings and listen to the devices, e.g.:
```
# listen to all traffic from kilns
mosquitto_sub -v -u mqtt_user -P mqtt_password -h mqtt_host -p 1883 -t 'qtkiln/#'
# change the config for a kiln and reset it
# for example, change the mqtt update frequency to once per 3 seconds
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"mqttUpdateInterval_ms":3000}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"reset":true}'
# change the set point for the temperature control
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"targetTemperature_C":42}'
# turn on the pid control
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"pidEnabled":1}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/get -m '{"targetTemperature_C":42}'
```
