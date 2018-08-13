// Base Arduino and ESP8266 Libraries
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

// Additional hardware libraries
#include <Sparkfun_APDS9301_Library.h>
#include <SparkFunBME280.h>

// Additional software
#include <PubSubClient.h>

#define MIN_LIGHT_THRESHOLD 20 

const char WiFiSSID[] = "fingerdoggy";
const char WiFiPSK[] = "nrCtpEwYsL115unx";
IPAddress MQTTServerIP = IPAddress(192, 168, 1, 31);
String SensorName = "brewroom_atmos";
String PubTopic = "sensor_data";

const int LED_PIN = 5;
const int LIGHT_INT_PIN = 0;

WiFiClient wifiClient;
BME280 mySensor;
APDS9301 lightSensor;
PubSubClient psClient(wifiClient);
Ticker ticker;

volatile uint32_t waitingTime = 900000; // 15 minutes
volatile bool send_data = true;

void setSendData() {
  send_data = true;
}

void mqttCallback(const MQTT::Publish& pub) {
  Serial.print("Message arrived [");
  Serial.print(pub.topic());
  Serial.print("] ");
  Serial.println(pub.payload_string());

  if (pub.topic().equals("sensor_ping") && pub.payload_string().equals(SensorName)) {
    setSendData();
  }
}

void setupMQTT() {
  psClient.set_server(MQTTServerIP, 1883);
  psClient.set_callback(mqttCallback);
}

void connectMQTT() {
  while (!psClient.connected()) {
    if (psClient.connect(SensorName)) {
      if (!psClient.subscribe("sensor_ping")) {
        Serial.println("Failed to subscribe to sensor_ping");
      }
      if (!psClient.subscribe("sensor_config")) {
        Serial.println("Failed to subscribe to sensor_config");
      }
      Serial.println("Connected to MQTT");
    } else {
      Serial.println("Failed to connect to MQTT, try again in 5 seconds.");
      delay(5000);
    }
  }
}

void pub_sensor_data() {
  String msg = String(SensorName);
  msg += " " + String(mySensor.readTempC());
  msg += " " + String(mySensor.readFloatPressure());
  msg += " " + String(mySensor.readFloatHumidity());
  msg += " " + String(lightSensor.readLuxLevel());

  Serial.print("Publish message: ");
  Serial.println(msg);

  int result = psClient.publish(String(PubTopic), msg);
  
  blinkLED();
  
  if (result == 0) {
    Serial.println("MQTT send failed!");
  }
}

void loop() {
  if (!ensureWiFi()) {
    return;
  }
  
  if (!psClient.connected()) {
    connectMQTT();
  } else {
    psClient.loop();
  }

  yield();
  
  if (send_data) {
    pub_sensor_data();
    lightSensor.clearIntFlag();
    send_data = false;
  }
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  Serial.println("Connecting to " + String(WiFiSSID));
  WiFi.begin(WiFiSSID, WiFiPSK);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    return false;
  }
  Serial.println("WiFi Connected");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());

  return true;
}


void initHardware() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(LIGHT_INT_PIN, INPUT_PULLUP);
  Wire.begin();
}

void gotLightInterrupt() {
  digitalWrite(LED_PIN, LOW);
  setSendData();
}

void initAPDS9301() {
  lightSensor.begin(0x39);
  lightSensor.setGain(APDS9301::HIGH_GAIN);
  lightSensor.setIntegrationTime(APDS9301::INT_TIME_13_7_MS);
  lightSensor.setLowThreshold(0);
  lightSensor.setHighThreshold(20);
  lightSensor.setCyclesForInterrupt(3);
  lightSensor.enableInterrupt(APDS9301::INT_ON);
  lightSensor.clearIntFlag();

  attachInterrupt(digitalPinToInterrupt(LIGHT_INT_PIN), gotLightInterrupt, FALLING);
}

void initBME280() {
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;

  mySensor.settings.runMode = 3;
  mySensor.settings.tStandby = 0;
  mySensor.settings.filter = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;

  delay(10);

  Serial.print("Starting BME280... result of .begin(): 0x");
  Serial.println(mySensor.begin(), HEX);
}

void blinkLED() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
  }
}

void setup() {
  initHardware();
  initAPDS9301();
  initBME280();
  setupMQTT();

  ticker.attach_ms(waitingTime, setSendData);
}
