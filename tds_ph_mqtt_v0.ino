#include <WiFi.h>
#include <PubSubClient.h>
#include "DFRobot_PH.h"
#include "EEPROM.h"
#include "DFRobot_EC.h"

#define TdsSensorPin 34    // Pin analog untuk sensor TDS
#define PH_PIN 36       // Pin analog untuk sensor pH
#define EEPROM_SIZE 512

char ssid[] = "Lab-ICN_v3";  // SSID WiFi
char pass[] = "[LAB PASSWORD]";  // Password WiFi
char mqtt_server[] = "10.34.1.150";  // Alamat MQTT broker
int mqtt_port = 1883;  // Port MQTT broker

WiFiClient espClient;
PubSubClient client(espClient);

DFRobot_PH phSensor;
DFRobot_EC ecSensor;
float tdsAdc, phAdc, tdsVoltage, phVoltage, phValue, tdsValue, temperature = 25;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE); // EEPROM untuk pH sensor

  // Setup koneksi WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi Connected!");

  client.setServer(mqtt_server, mqtt_port);  // Set broker MQTT

  // Inisialisasi sensor
  phSensor.begin();
  ecSensor.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();  // Reconnect jika MQTT disconnect
  }
  client.loop();

  // Membaca nilai dari sensor TDS
  tdsAdc = analogRead(TdsSensorPin);
  Serial.print("TdsSensor Adc: ");
  Serial.println(tdsAdc);
  tdsVoltage = tdsAdc / 4095.0 * 3300;  // Mengonversi nilai ADC ke tegangan (mV)
  tdsValue = ecSensor.readEC(tdsVoltage, temperature);    // Membaca nilai TDS
  Serial.print("TDS: ");
  Serial.println(tdsValue);

  // Membaca nilai dari sensor pH
  phAdc = analogRead(PH_PIN);
  Serial.print("pHSensor Adc: ");
  Serial.println(phAdc);
  phVoltage = phAdc / 4095.0 * 3300;  // Mengonversi nilai ADC ke tegangan (mV)
  phValue = phSensor.readPH(phVoltage, temperature);  // Membaca nilai pH
  Serial.print("pH: ");
  Serial.println(phValue);

  // Publish data ke MQTT broker
  char tdsString[8];
  dtostrf(tdsValue, 1, 2, tdsString);
  client.publish("node/TDS", tdsString);

  char phString[8];
  dtostrf(phValue, 1, 2, phString);
  client.publish("node/pH", phString);

  delay(2000);  // Delay 2 detik sebelum membaca lagi
}

// Fungsi untuk reconnect ke MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}