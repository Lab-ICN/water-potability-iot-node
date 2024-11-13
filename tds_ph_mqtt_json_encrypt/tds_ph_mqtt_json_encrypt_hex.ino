#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "DFRobot_PH.h"
#include "EEPROM.h"
#include "DFRobot_EC.h"
#include <Crypto.h>
#include <AES.h>
#include <GCM.h>

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
MqttClient mqttClient(wifiClient);  

DFRobot_PH phSensor;
DFRobot_EC ecSensor;
float tdsAdc, phAdc, tdsVoltage, phVoltage, phValue, tdsValue, temperature = 25;

byte aesKey[16] = {0x50, 0x52, 0x30, 0x54, 0x30, 0x54, 0x31, 0x50, 0x33, 0x5F, 0x57, 0x34, 0x54, 0x45, 0x52, 0x5F};
byte iv[12] = {0x6C, 0x61, 0x62, 0x69, 0x63, 0x6E, 0x6C, 0x61, 0x62, 0x69, 0x63, 0x6E};

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
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  mqttClient.poll();

  tdsAdc = analogRead(TdsSensorPin);
  Serial.print("TdsSensor Adc: ");
  Serial.println(tdsAdc);
  tdsVoltage = tdsAdc / 4095.0 * 3300;  
  tdsValue = ecSensor.readEC(tdsVoltage, temperature);    
  Serial.print("TDS: ");
  Serial.println(tdsValue);

  phAdc = analogRead(PH_PIN);
  Serial.print("pHSensor Adc: ");
  Serial.println(phAdc);
  phVoltage = phAdc / 4095.0 * 3300;  
  phValue = phSensor.readPH(phVoltage, temperature);  
  Serial.print("pH: ");
  Serial.println(phValue);

  StaticJsonDocument<200> doc;
  doc["TDS"] = tdsValue;
  doc["pH"] = phValue;
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer); 

  String encryptedData = encryptAES128GCM((const byte*)jsonBuffer, strlen(jsonBuffer));

  // Publish encrypted data
  mqttClient.beginMessage("wp2");
  mqttClient.print(encryptedData);
  mqttClient.endMessage();

  delay(60000);
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(mqtt_server, mqtt_port)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect, error code=");
      Serial.println(mqttClient.connectError());
      delay(5000);
    }
  }
}

String encryptAES128GCM(const byte* plaintext, size_t len) {
    GCM<AES128> gcm;
    byte ciphertext[256];
    byte tag[16];
    char encodedMessage[512];

    gcm.clear();
    gcm.setKey(aesKey, sizeof(aesKey));
    gcm.setIV(iv, sizeof(iv));
    gcm.addAuthData(nullptr, 0);
    gcm.encrypt(ciphertext, plaintext, len);
    gcm.computeTag(tag, sizeof(tag));

    // Concatenate ciphertext and tag for transmission
    byte fullMessage[256 + 16];
    memcpy(fullMessage, ciphertext, len);
    memcpy(fullMessage + len, tag, 16);

    // Encode to Hexadecimal for MQTT transmission
    int encodedLength = encodeHex(fullMessage, len + 16, encodedMessage);
    encodedMessage[encodedLength] = '\0';

    return String(encodedMessage);
}

// Hexadecimal encoding function
int encodeHex(const byte* input, int length, char* output) {
    const char hexChars[] = "0123456789ABCDEF";
    int encLen = 0;

    for (int i = 0; i < length; i++) {
        output[encLen++] = hexChars[(input[i] >> 4) & 0x0F];
        output[encLen++] = hexChars[input[i] & 0x0F];
    }

    output[encLen] = '\0'; // Null-terminate the output
    return encLen;
}