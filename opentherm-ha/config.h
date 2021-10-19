/*
  Topic structure is built with get/set mechanizm for
  compatibility with home assistant and to allow external
  device control
  _GET_ topics are used to publish current thermostat state
  _SET_ topics are used to control the thermostat
*/

// Your WiFi credentials.
// Set password to "" for open networks.
const char* ssid = "xxxxxx";
const char* pass = "xxxxxx";

// Your MQTT broker address and credentials
const char* mqtt_server = "xxx.xxx.xxx.xxx";
const char* mqtt_user = "xxxxxx";
const char* mqtt_password = "xxxxxx";
const int mqtt_port = 1883;

// Master OpenTherm Shield pins configuration
const int OT_IN_PIN = 4;  //for Arduino, 4 for ESP8266 (D2), 21 for ESP32
const int OT_OUT_PIN = 5; //for Arduino, 5 for ESP8266 (D1), 22 for ESP32

// Temperature sensor pin
const int ROOM_TEMP_SENSOR_PIN = 14; //for Arduino, 14 for ESP8266 (D5), 18 for ESP32

/*
   current temperature topics
   if setter is used - thermostat works with external values, bypassing built-in sensor
   if no values on setter for more than 1 minute - thermostat falls back to built-in sensor
*/

const String BASE_TOPIC = "opentherm-thermostat/";

const String CURRENT_TEMP_GET_TOPIC = BASE_TOPIC + "current-temperature/get";
const String CURRENT_TEMP_SET_TOPIC = BASE_TOPIC + "current-temperature/set";

// current temperature topics
const String TEMP_SETPOINT_GET_TOPIC = BASE_TOPIC + "setpoint-temperature/get";
const String TEMP_SETPOINT_SET_TOPIC = BASE_TOPIC + "setpoint-temperature/set";

// working mode topics
const String MODE_GET_TOPIC = BASE_TOPIC + "mode/get";
const String MODE_SET_TOPIC = BASE_TOPIC + "mode/set";

// boiler water temperature topic
const String TEMP_BOILER_GET_TOPIC = BASE_TOPIC + "boiler-temperature/get";
const String TEMP_BOILER_TARGET_GET_TOPIC = BASE_TOPIC + "boiler-target-temperature/get";

// debug data
const String INTEGRAL_ERROR_GET_TOPIC = BASE_TOPIC + "integral-error/get";
const String FLAME_STATUS_GET_TOPIC = BASE_TOPIC + "flame-status/get";
const String FLAME_LEVEL_GET_TOPIC = BASE_TOPIC + "flame-level/get";

// domestic hot water temperature topic
const String TEMP_DHW_GET_TOPIC = BASE_TOPIC + "dhw-temperature/get";
const String TEMP_DHW_SET_TOPIC = BASE_TOPIC + "dhw-temperature/set";
const String ACTUAL_TEMP_DHW_GET_TOPIC = BASE_TOPIC + "dhw-actual-temperature/get";

// domestic hot water enable/disable
const String STATE_DHW_GET_TOPIC = BASE_TOPIC + "dhw-state/get";
const String STATE_DHW_SET_TOPIC = BASE_TOPIC + "dhw-state/set";

// setpoint topic
const String SETPOINT_OVERRIDE_SET_TOPIC = BASE_TOPIC + "setpoint-override/set";

// logs topic
const String LOG_GET_TOPIC = BASE_TOPIC + "log";
