#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>  
#include "DFRobot_PH.h"
#include "EEPROM.h"
#include "DFRobot_EC.h"

#define TdsSensorPin 34    
#define PH_PIN 36          
#define EEPROM_SIZE 512

char ssid[] = "Lab-ICN_v3";           
char pass[] = "labjarkomnomorsatu";     
char mqtt_server[] = "10.34.4.184";    
int mqtt_port = 1883;                 
char mqtt_user[] = "client_iot";     
char mqtt_pass[] = "F96labjarkomiot";     

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);  // Declare an instance of MqttClient

DFRobot_PH phSensor;
DFRobot_EC ecSensor;
float tdsAdc, phAdc, tdsVoltage, phVoltage, phValue, tdsValue, temperature = 25;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE); 

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("WiFi Connected!");

  mqttClient.setId("ESP32Client");
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);

  phSensor.begin();
  ecSensor.begin();
}

void loop() {
  // Reconnect if the MQTT client is not connected
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  mqttClient.poll();  // Process MQTT messages

  // Reading TDS sensor values
  tdsAdc = analogRead(TdsSensorPin);
  Serial.print("TdsSensor Adc: ");
  Serial.println(tdsAdc);
  tdsVoltage = tdsAdc / 4095.0 * 3300;  
  tdsValue = ecSensor.readEC(tdsVoltage, temperature);    
  Serial.print("TDS: ");
  Serial.println(tdsValue);

  // Reading pH sensor values
  phAdc = analogRead(PH_PIN);
  Serial.print("pHSensor Adc: ");
  Serial.println(phAdc);
  phVoltage = phAdc / 4095.0 * 3300;  
  phValue = phSensor.readPH(phVoltage, temperature);  
  Serial.print("pH: ");
  Serial.println(phValue);

  // Create JSON payload
  StaticJsonDocument<200> doc;
  doc["TDS"] = tdsValue;
  doc["pH"] = phValue;
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer); 

  // Publish data to MQTT broker
  mqttClient.beginMessage("wp");
  mqttClient.print(jsonBuffer);
  mqttClient.endMessage();

  delay(60000);  // Delay before reading again
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    // Attempt to connect to the MQTT broker
    if (mqttClient.connect(mqtt_server, mqtt_port)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect, error code=");
      Serial.println(mqttClient.connectError());
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}