#include <Arduino.h>
#include <AsyncElegantOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>

#include "config.h"

#define BUFFERSIZE (64)

#define PIN_BUTTON (12)
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

constexpr uint16_t debounceDelay = 50u;
uint8_t lastSteadyState = HIGH;
uint8_t lastFlickerableState = HIGH;
uint8_t currentState;
unsigned long lastDebounceTime = 0;

void connectmqtt();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
static void cleanBuffer();
unsigned long myTime = 0;

void prepareStrings() {
  snprintf(topicValve, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/don-pavlov.watervalve");
  snprintf(topicLog, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/don-pavlov.log");
}

void setup() {
  prepareStrings();
  delay(500);     // wait 500ms before we start anything
  Serial.begin(115200);
  Serial.println("Booting");
  cleanBuffer();

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);

  digitalWrite(PIN_RELAY, 1);  // Reset Relay
  delay(50);
  digitalWrite(PIN_RELAY, 0);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Meh! go to /update");
  });

  AsyncElegantOTA.begin(&server);  // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(topicValve);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

static void cleanBuffer() { std::fill(buffer, buffer + BUFFERSIZE, 0); }

void loop() {
  AsyncElegantOTA.loop();
  currentState = digitalRead(PIN_BUTTON);

  // Debounce
  if (currentState != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = currentState;
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (lastSteadyState == HIGH && currentState == LOW) {
      // Serial.println("The button is pressed");
      digitalWrite(PIN_RELAY, !digitalRead(PIN_RELAY));
    }

    else if (lastSteadyState == LOW && currentState == HIGH)
      // Serial.println("The button is released");
      // save the the last steady state
      lastSteadyState = currentState;
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
    Serial.print("\r\n");
    //Todo(donpavlov) add more payload with timing and convert the data to uints
    digitalWrite(PIN_RELAY, 1);

  } else if ((char)payload[0] == 'O' && (char)payload[1] == 'F' &&
             (char)payload[2] == 'F')  // off
  {
    // digitalWrite(LED, LOW);
    Serial.print("\r\n");
    digitalWrite(PIN_RELAY, 0);

  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(hostName, IO_USERNAME, IO_KEY)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topicLog, "Watervalve connected to MQTT");
      // ... and resubscribe
      client.subscribe(topicValve);

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
    client.subscribe(topicValve);
    client.publish(topicLog, "subscribed to watervalve");

    if (!client.connected()) {
      reconnect();
    }
  }
}
