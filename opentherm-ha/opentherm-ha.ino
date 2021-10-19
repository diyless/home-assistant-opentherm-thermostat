/*************************************************************
  This example runs directly on ESP8266 chip.

  Please be sure to select the right ESP8266 module
  in the Tools -> Board -> WeMos D1 Mini

  Adjust settings in Config.h before run
 *************************************************************/

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <OpenTherm.h>
#include "config.h"

const unsigned long extTempTimeout_ms = 60 * 1000;
const unsigned long statusUpdateInterval_ms = 1000;
const unsigned long spOverrideTimeout_ms = 30 * 1000;

float  sp = 18, //set point
       t = 15, //current temperature
       t_last = 0, //prior temperature
       ierr = 25, //integral error
       dt = 0, //time between measurements
       op = 0; //PID controller output
float op_override;

unsigned long ts = 0, new_ts = 0; //timestamp
unsigned long lastUpdate = 0;
unsigned long lastTempSet = 0;

float dhwTarget = 48;

unsigned long lastSpSet = 0;

bool heatingEnabled = true;
bool enableHotWater = true;

#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

OneWire oneWire(ROOM_TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
OpenTherm ot(OT_IN_PIN, OT_OUT_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

// upper and lower bounds on heater level
float ophi = 63;
float oplo = 35;

const float noCommandSpOverride = 50;


void IRAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

float getTemp() {
  unsigned long now = millis();
  if (now - lastTempSet > extTempTimeout_ms)
    return sensors.getTempCByIndex(0);
  else
    return t;
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
  float KP = 10;
  float KI = 0.02;

  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;
  // calculate the measurement derivative
  //float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error; //proportional contribution
  float I = ierr; //integral contribution
  float op = P + I;
  // implement anti-reset windup
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I;

  Serial.println("sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I));

  return op;
}

// This function calculates temperature and sends data to MQTT every second.
void updateData()
{
  //Set/Get Boiler Status
  bool enableCooling = false;
  unsigned long response = ot.setBoilerStatus(heatingEnabled, enableHotWater, enableCooling);
  OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
  if (responseStatus != OpenThermResponseStatus::SUCCESS) {
    String msg = "Error: Invalid boiler response " + String(response, HEX);
    Serial.println(msg);
    client.publish(LOG_GET_TOPIC.c_str(), msg.c_str());
  }

  ot.setDHWSetpoint(dhwTarget);

  if (responseStatus == OpenThermResponseStatus::SUCCESS) {

    unsigned long now = millis();

    new_ts = millis();
    dt = (new_ts - ts) / 1000.0;
    ts = new_ts;
    op = pid(sp, t, t_last, ierr, dt);

    if (now - lastSpSet <= spOverrideTimeout_ms) {
      op = op_override;
    }

    ot.setBoilerTemperature(op);
  }
  t_last = t;

  sensors.requestTemperatures(); //async temperature request

  float level = ot.getModulation();
  float bt = ot.getBoilerTemperature();
  float dhwTemp = ot.getDHWTemperature();

  client.publish(TEMP_BOILER_TARGET_GET_TOPIC.c_str(), String(op).c_str());
  client.publish(CURRENT_TEMP_GET_TOPIC.c_str(), String(t).c_str());
  client.publish(TEMP_BOILER_GET_TOPIC.c_str(), String(bt).c_str());
  client.publish(TEMP_SETPOINT_GET_TOPIC.c_str(), String(sp).c_str());
  client.publish(INTEGRAL_ERROR_GET_TOPIC.c_str(), String(ierr).c_str());
  client.publish(MODE_GET_TOPIC.c_str(), heatingEnabled ? "heat" : "off");
  client.publish(FLAME_STATUS_GET_TOPIC.c_str(), ot.isFlameOn(response) ? "on" : "off");
  client.publish(FLAME_LEVEL_GET_TOPIC.c_str(), String(level).c_str());
  client.publish(STATE_DHW_GET_TOPIC.c_str(), enableHotWater ? "on" : "off");
  client.publish(TEMP_DHW_GET_TOPIC.c_str(), String(dhwTarget).c_str());
  client.publish(ACTUAL_TEMP_DHW_GET_TOPIC.c_str(), String(dhwTemp).c_str());

  Serial.print("Current temperature: " + String(t) + " °C ");
  String tempSource = (millis() - lastTempSet > extTempTimeout_ms)
                      ? "(internal sensor)"
                      : "(external sensor)";
  Serial.println(tempSource);
}

String convertPayloadToStr(byte* payload, unsigned int length) {
  char s[length + 1];
  s[length] = 0;
  for (int i = 0; i < length; ++i)
    s[i] = payload[i];
  String tempRequestStr(s);
  return tempRequestStr;
}

bool isValidNumber(String str) {
  bool valid = true;
  for (byte i = 0; i < str.length(); i++)
  {
    char ch = str.charAt(i);
    valid &= isDigit(ch) ||
             ch == '+' || ch == '-' || ch == ',' || ch == '.' ||
             ch == '\r' || ch == '\n';
  }
  return valid;
}

void callback(char* topic, byte* payload, unsigned int length) {
  const String topicStr(topic);

  String payloadStr = convertPayloadToStr(payload, length);
  payloadStr.trim();

  if (topicStr == TEMP_SETPOINT_SET_TOPIC) {
    Serial.println("Set target temperature: " + payloadStr);
    sp = payloadStr.toFloat();
    if (isnan(sp)) {
      Serial.println("Setpoint NaN, defaulting to 21");
      sp = 21;
    }
  }
  else if (topicStr == CURRENT_TEMP_SET_TOPIC) {
    float t1 = payloadStr.toFloat();
    if (isnan(t1) || !isValidNumber(payloadStr)) {
      Serial.println("Current temp set is not a valid number, ignoring...");
    }
    else {
      t = t1;
      lastTempSet = millis();
    }
  }
  else if (topicStr == MODE_SET_TOPIC) {
    Serial.println("Set mode: " + payloadStr);
    if (payloadStr == "heat")
      heatingEnabled = true;
    else if (payloadStr == "off")
      heatingEnabled = false;
    else
      Serial.println("Unknown mode " + payloadStr);
  }
  else if (topicStr == TEMP_DHW_SET_TOPIC) {
    dhwTarget = payloadStr.toFloat();
  }
  else if (topicStr == STATE_DHW_SET_TOPIC) {
    if (payloadStr == "on")
      enableHotWater = true;
    else if (payloadStr == "off")
      enableHotWater = false;
    else
      Serial.println("Unknown domestic hot water state " + payloadStr);
  }
  else if (topicStr == SETPOINT_OVERRIDE_SET_TOPIC) {
    lastSpSet = millis();
    op_override = payloadStr.toFloat();
    if (isnan(op_override)) {
      Serial.println("Setpoint override NaN, defaulting to 40");
      op_override = 40;
    }
  }
  else {
    Serial.printf("Unknown topic: %s\r\n", topic);
    return;
  }

  lastUpdate = 0;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    const char* clientId = "opentherm-thermostat-test";
    if (client.connect(clientId, mqtt_user, mqtt_password)) {
      Serial.println("ok");

      client.subscribe(TEMP_SETPOINT_SET_TOPIC.c_str());
      client.subscribe(MODE_SET_TOPIC.c_str());
      client.subscribe(CURRENT_TEMP_SET_TOPIC.c_str());
      client.subscribe(TEMP_DHW_SET_TOPIC.c_str());
      client.subscribe(STATE_DHW_SET_TOPIC.c_str());
      client.subscribe(SETPOINT_OVERRIDE_SET_TOPIC.c_str());
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  int deadCounter = 20;
  while (WiFi.status() != WL_CONNECTED && deadCounter-- > 0) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to " + String(ssid));
    while (true);
  }
  else {
    Serial.println("ok");
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ot.begin(handleInterrupt);

  //Init DS18B20 sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode
  t, t_last = sensors.getTempCByIndex(0);
  ts = millis();
  lastTempSet = -extTempTimeout_ms;
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastUpdate > statusUpdateInterval_ms) {
    lastUpdate = now;
    updateData();
  }
  if (now - lastTempSet > extTempTimeout_ms && now - lastSpSet > spOverrideTimeout_ms) {
    lastSpSet = millis();
    op_override = noCommandSpOverride;
  }
}
