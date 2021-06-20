#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>

#include "config.h"

#define BUFFERSIZE (64)

constexpr uint32_t delayMS = 10000;

void setup();

WiFiClient espClient;
PubSubClient client(espClient);  // lib required for mqtt

constexpr uint16_t mqttPort = 1883;
constexpr uint16_t otaPort = 3232;

const char* mqttServer = "io.adafruit.com";  // mqtt server
const char* hostName = "watervalve";

char buffer[BUFFERSIZE];

char topicValve[BUFFERSIZE];

char topicLog[BUFFERSIZE];

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
  Serial.begin(115200);
  Serial.println("Booting");
  cleanBuffer();
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  ArduinoOTA.setPort(otaPort);

  ArduinoOTA.setHostname(hostName);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()

    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

static void cleanBuffer() { std::fill(buffer, buffer + BUFFERSIZE, 0); }

void loop() {
  ArduinoOTA.handle();

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
