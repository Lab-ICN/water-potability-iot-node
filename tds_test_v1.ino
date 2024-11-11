#include <EEPROM.h>
#include "GravityTDS.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Define the TDS sensor pin
#define TdsSensorPin 2

// Initialize TDS sensor object
GravityTDS gravityTds;

float temperature = 25, tdsValue = 0;

// WiFi settings
const char* ssid = "Lab-ICN_v3";
const char* password = "labjarkomnomorsatu";

// MQTT Broker settings
const char* mqtt_server = "10.34.1.150";  // Replace with the IP of your Mosquitto broker
const int mqtt_port = 1883;  // Default MQTT port

// Initialize WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect to the MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // TDS Sensor setup
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  // Reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  // 1024 for 10bit ADC; 4096 for 12bit ADC
  gravityTds.begin();  // Initialization

  // Connect to WiFi
  setup_wifi();

  // Set the MQTT broker and port
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  // Ensure the MQTT client is connected
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Get TDS value
  gravityTds.setTemperature(temperature);  // Set the temperature and execute temperature compensation
  gravityTds.update();  // Sample and calculate
  tdsValue = gravityTds.getTdsValue();  // Then get the value
  
  // Print TDS value to Serial
  Serial.print("TDS Value: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  // Prepare TDS value as a string to publish over MQTT
  char tdsString[10];
  dtostrf(tdsValue, 1, 2, tdsString);  // Convert float to string with 2 decimal places

  // Publish the TDS value to the MQTT broker
  client.publish("node/TDS", tdsString);

  // Delay before next reading
  delay(10000);
}