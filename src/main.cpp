#include <Arduino.h>
#include <AsyncElegantOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include <DHT_U.h>
#include "config.h"

#define BUFFERSIZE (64)

#define PIN_BUTTON (12)
#define PIN_RELAY (13)
#define DHTPIN (14)
#define DHTTYPE DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);
constexpr uint32_t delayMS = 10000;

WiFiClient espClient;
PubSubClient client(espClient);  // lib required for mqtt
AsyncWebServer server(80);

constexpr uint16_t mqttPort = 1883;

const char* mqttServer = "io.adafruit.com";  // mqtt server
const char* hostName = "watervalve";

char buffer[BUFFERSIZE];

char topicValve[BUFFERSIZE];
char topicHumidity[BUFFERSIZE];
char topicTemp[BUFFERSIZE];

char topicLog[BUFFERSIZE];

char bufferValve[BUFFERSIZE];
char bufferHumidity[BUFFERSIZE];
char bufferTemp[BUFFERSIZE];

constexpr uint16_t debounceDelay = 50u;
uint8_t lastSteadyState = HIGH;
uint8_t lastFlickerableState = HIGH;
uint8_t currentState = 0u;
uint8_t currentRelay = 0u;
uint8_t oldRelay= 0u;
bool publishRelay = false;

unsigned long lastDebounceTime = 0u;

void connectmqtt();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
static void cleanBuffer();
unsigned long myTime = 0;

void prepareStrings() {
  snprintf(topicValve, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.watervalve");
  snprintf(topicLog, BUFFERSIZE, "%s%s", IO_USERNAME, "/f/don-pavlov.log");
  snprintf(topicHumidity, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.humidity");
  snprintf(topicTemp, BUFFERSIZE, "%s%s", IO_USERNAME,
           "/f/don-pavlov.temperature");
}

void setup() {
  prepareStrings();
  delay(500);  // wait 500ms before we start anything
  Serial.begin(115200);
  Serial.println("Booting");
  cleanBuffer();
  std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
  std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);

  digitalWrite(PIN_RELAY, 1);  // Reset Relay
  delay(50);
  digitalWrite(PIN_RELAY, 0);

  oldRelay = digitalRead(PIN_RELAY);
  publishRelay = true;
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

  dht.begin();
  sensor_t sensor;

  Serial.println(F("DHTxx Unified Sensor Example"));
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

static void cleanBuffer() { std::fill(buffer, buffer + BUFFERSIZE, 0); }

void loop() {
  currentState = digitalRead(PIN_BUTTON);
  currentRelay = digitalRead(PIN_RELAY);
  if (currentRelay != oldRelay) {
    oldRelay = currentRelay;
    publishRelay = true;
  }

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

    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      snprintf(bufferTemp, BUFFERSIZE, "Temperature: %.2f°C",
               event.temperature);
      Serial.println(bufferTemp);
      std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
      snprintf(bufferTemp, BUFFERSIZE, "%.2f", event.temperature);
      if (client.connected()) {
        client.publish(topicTemp, bufferTemp);
      }
      std::fill(bufferTemp, bufferTemp + BUFFERSIZE, 0);
    }

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    } else {
      snprintf(bufferHumidity, BUFFERSIZE, "Humidity: %.2f%%",
               event.relative_humidity);
      Serial.println(bufferHumidity);
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
      snprintf(bufferHumidity, BUFFERSIZE, "%.2f", event.relative_humidity);
      if (client.connected()) {
        client.publish(topicHumidity, bufferHumidity);
      }
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
    }

    if (publishRelay) {
      snprintf(bufferValve, BUFFERSIZE, "RelayState: %d", currentRelay);
      Serial.println(bufferValve);
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
      if (currentRelay == 0) {
        snprintf(bufferValve, BUFFERSIZE, "OFF");
      }
      else {
        snprintf(bufferValve, BUFFERSIZE, "ON");
      }
      if (client.connected()) {
        client.publish(topicValve, bufferValve);
      }
      std::fill(bufferHumidity, bufferHumidity + BUFFERSIZE, 0);
    }

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
  for (uint32_t i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  if ((char)payload[0] == 'O' && (char)payload[1] == 'N')  // on
  {
    Serial.print("\r\n");
    // Todo(donpavlov) add more payload with timing and convert the data to
    // uints
    if (digitalRead(PIN_RELAY) == 1) {
      publishRelay = false;
    }
    else {
      publishRelay = true;
    }
    digitalWrite(PIN_RELAY, 1);


  } else if ((char)payload[0] == 'O' && (char)payload[1] == 'F' &&
             (char)payload[2] == 'F')  // off
  {
    if (digitalRead(PIN_RELAY) == 0) {
      publishRelay = false;
    }
    else {
      publishRelay = true;
    }
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
