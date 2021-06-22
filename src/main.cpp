#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include "config.h"

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define BUFFERSIZE (64)

#define PIN_BUTTON   (12)
#define PIN_RELAY (13)

constexpr uint32_t delayMS = 10000;

WiFiClient espClient;
PubSubClient client(espClient);  // lib required for mqtt
AsyncWebServer server(80);

constexpr uint16_t mqttPort = 1883;

const char* mqttServer = "io.adafruit.com";  // mqtt server
const char* hostName = "watervalve";

char buffer[BUFFERSIZE];

char topicValve[BUFFERSIZE];

char topicLog[BUFFERSIZE];

uint8_t buttonState = 0u;

void connectmqtt();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
static void cleanBuffer();
unsigned long myTime = 0;
void prepareStrings();

void prepareStrings() {
  snprintf(topicValve, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/watervalve");
  snprintf(topicLog, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/log");
}

void setup() {
  prepareStrings();
  delay(5000);     // wait 10s before we start anything
  Serial.begin(115200);
  Serial.println("Booting");
  cleanBuffer();

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_BUTTON, 0);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

static void cleanBuffer() { std::fill(buffer, buffer + BUFFERSIZE, 0); }

void loop() {
  AsyncElegantOTA.loop();
  buttonState = digitalRead(PIN_BUTTON);
  if(buttonState == 1) {
    digitalWrite(PIN_BUTTON, 1);
    delay(200);
  }
  if (millis() - myTime > delayMS) {
    myTime = millis();

    if (!client.connected()) {
      reconnect();
    }

    client.loop();
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  if ((char)payload[0] == 'O' && (char)payload[1] == 'N')  // on
  {
    // digitalWrite(LED, HIGH);
    Serial.println("on");
  } else if ((char)payload[0] == 'O' && (char)payload[1] == 'F' &&
             (char)payload[2] == 'F')  // off
  {
    // digitalWrite(LED, LOW);
    Serial.println(" off");
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(hostName, IO_USERNAME, IO_KEY)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topicLog, "Nodemcu connected to MQTT");
      // ... and resubscribe
      // client.subscribe("inTopic");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void connectmqtt() {
  client.connect(hostName, IO_USERNAME,
                 IO_KEY);  // ESP will connect to mqtt broker with clientID
  {
    Serial.println("Watervalve Connected to MQTT");
    // Once connected, publish an announcement...

    // ... and resubscribe
    // client.subscribe("inTopic"); //topic=Demo
    client.publish(topicLog, "Watervalve Connected to MQTT");

    if (!client.connected()) {
      reconnect();
    }
  }
}
