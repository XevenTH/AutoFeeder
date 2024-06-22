#include <WiFi.h>
#include <MQTTPubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <DHT.h>

#define ID_DEVICE 575812

struct HCSRData {
  char name[20];
  float value;
};

const char* ssid = "Punyaku";
const char* pass = "yess1235";
char topic[50] = "xeventh/575812";

/* PIN OUT AND IN!! */
// const int buttonPin = 2;
int ultrasonicEchoPin = 13;
int ultrasonicTrigPin = 12;
int servoConPin = 2;
int DHTPin = 19;
float distance = 0;

/* INITIALIZE OBJECT!!! */
WiFiClient client;
MQTTPubSubClient mqtt;
Servo servo;
DHT dht(DHTPin, DHT11);

/* STATE!! */
int buttonState = 0;
int servoPos = 0;
float servoTime = 90;
int signalPublish = 0;

void connect() {
connect_to_wifi:
  Serial.print("connecting to wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");

connect_to_host:
  Serial.print("connecting to host...");
  client.stop();
  while (!client.connect("broker.emqx.io", 1883)) {
    Serial.print(".");
    delay(1000);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected");
      goto connect_to_wifi;
    }
  }
  Serial.println(" connected!");

  Serial.print("connecting to mqtt broker...");
  mqtt.disconnect();
  while (!mqtt.connect("esp", "public", "public")) {
    Serial.print(".");
    delay(1000);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected");
      goto connect_to_wifi;
    }
    if (client.connected() != 1) {
      Serial.println("WiFiClient disconnected");
      goto connect_to_host;
    }
  }
  Serial.println(" connected!");
}

void ServoControl(int time) {
  for (servoPos = 90; servoPos >= 55; servoPos -= 1) {
    servo.write(servoPos);
  }
  delay(time);
  for (servoPos = 55; servoPos <= 90; servoPos += 1) {
    servo.write(servoPos);
  }
}

float readDistanceCM() {
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  int duration = pulseIn(ultrasonicEchoPin, HIGH);
  int hasil = duration * 0.034 / 2;

  return hasil;
}

String JSONOutput() {
  // struct DHT11Data DHT11DataObj = DHT11Reader();

  // JsonObject data1 = array.createNestedObject();
  // data1["sensor"] = DHT11DataObj.name;
  // StaticJsonDocument<300> DHT11Value;
  // DHT11Value["temperature"] = DHT11DataObj.temperature;
  // DHT11Value["humidity"] = DHT11DataObj.humidity;
  // data1["data"] = DHT11Value;

  // JsonObject data2 = array.createNestedObject();
  StaticJsonDocument<300> HCSRValue;
  HCSRValue["id_device"] = ID_DEVICE;
  HCSRValue["sensor"] = "HCSR";
  HCSRValue["data"] = distance;

  char jsonBuffer[300];
  serializeJson(HCSRValue, jsonBuffer);
  Serial.print(jsonBuffer);

  return jsonBuffer;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  servo.attach(servoConPin);
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
  dht.begin();

  // initialize mqtt client
  mqtt.begin(client);

  // connect to wifi, host and mqtt broker
  connect();

  // subscribe callback which is called when every packet has come
  mqtt.subscribe([](const String& topic, const String& payload, const size_t size) {
    Serial.println("mqtt received: " + topic + " - " + payload);
  });

  char tempTopicData[50];
  strcpy(tempTopicData, topic);
  strcat(tempTopicData, "/data");
  // subscribe topic and callback which is called when /hello has come
  mqtt.subscribe(tempTopicData, [](const String& payload, const size_t size) {
    Serial.println(payload);
  });


  char tempTopicSignal[50];
  strcpy(tempTopicSignal, topic);
  strcat(tempTopicSignal, "/signal");
  mqtt.subscribe(tempTopicSignal, [](const String& payload, const size_t size) {
    if (isdigit(payload[0])) {
      signalPublish = payload.toInt();
    }
  });

  char tempTopicServo[50];
  strcpy(tempTopicServo, topic);
  strcat(tempTopicServo, "/servo");
  mqtt.subscribe(tempTopicServo, [](const String& payload, const size_t size) {
    if (isdigit(payload[0])) {
      servoTime = payload.toInt();
      if (servoTime != 0) {
        ServoControl(servoTime);
      }
    }
  });
}

void loop() {
  mqtt.update();  // should be called

  if (!mqtt.isConnected()) {
    connect();
  }

  distance = readDistanceCM();

  delay(1000);

  if (signalPublish == 1) {
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms > 3000) {
      prev_ms = millis();

      char tempTopicData[50];
      strcpy(tempTopicData, topic);
      strcat(tempTopicData, "/data");
      mqtt.publish(tempTopicData, JSONOutput());
    }
    signalPublish = 0;
  }

  // ServoControl(1000);

  Serial.print("Jarak: ");
  Serial.println(distance);
}
