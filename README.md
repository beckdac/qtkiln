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
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"targetTemp_C":42}'
# turn on the pid control
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"pidEnabled":1}'
//mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/get -m '{"targetTemp_C"}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/get -m 'statistics'
```

### Debug / test protocole
```
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"reset":true}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"mqttUpdateInterval_ms":3000}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"save":true}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"targetTemp_C":42}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"targetTemp_C":42,"pidEnabled":true}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/get -m '{"statistics":true}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"debugPriority":100}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"alarmOnSSR":true}'
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"pidEnabled":false}'
```

### PID tuning protocole
* Get a high enough output so it can be recorded to a file and parsed out later
```
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/config -m '{"mqttUpdateInterval_ms":1000}'
sleep 1
* Find a duty cycle or output_ms that is small and low temp
```
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"setOutput_ms":150}'
```
* Let the system stablize at this temperature an use it for the values in `qtkiln_pwm.h` under `tuning`; this includes the startstep which should be the output_ms like 150 above the stepsize which is startstep + the size so 200 for a step size of 50
* Make sure the time windows are long enough that the system has fully equilibrated to the 150 before the quiet time is over and the test begins and the entire transition can happen in the sample window specified
```
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"alarmOnSSR":true}'
sleep 1
mosquitto_pub -L mqtt://mqtt_user:mqtt_password@mqtt_host/qtkiln/ECDA3BC01AB4/set -m '{"pidTuning":true}'
```

### Current dependency set:
```
  WiFi at version 3.1.3
  Networking at version 3.1.3
  Preferences at version 3.1.3
  MAX31855_RT at version 0.6.1
  SPI at version 3.1.3
  EspMQTTClientFork at version 1.13.4
  ArduinoOTA at version 3.1.3
  Update at version 3.1.3
  PubSubClient at version 2.8
  WebServer at version 3.1.3
  FS at version 3.1.3
  ESPmDNS at version 3.1.3
  TM1637 at version 1.2.0
  ArduinoJson at version 7.3.1
  QuickPID at version 3.1.9
  sTune at version 2.4.0
```
