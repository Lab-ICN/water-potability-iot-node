#include "DFRobot_PH.h"
#include "GravityTDS.h"
#include <EEPROM.h>

#define PH_PIN A1         // pH sensor connected to analog pin A1
#define TDS_PIN A0        // TDS sensor connected to analog pin A0

float phVoltage, phValue;
float tdsVoltage, tdsValue;
float temperature = 25;   // Default temperature for compensation
DFRobot_PH ph;
GravityTDS gravityTds;

void setup()
{
    Serial.begin(115200);

    // Initialize pH sensor
    ph.begin();

    // Initialize TDS sensor
    gravityTds.setPin(TDS_PIN);
    gravityTds.setAref(5.0);  // Reference voltage on ADC (5.0V for Arduino UNO)
    gravityTds.setAdcRange(1024);  // 1024 for 10-bit ADC (Arduino UNO)
    gravityTds.begin();
}

void loop()
{
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) {  // Time interval: 1 second
        timepoint = millis();

        // pH Sensor Reading
        phVoltage = analogRead(PH_PIN) / 1024.0 * 5000;  // Convert analog reading to voltage (in millivolts)
        phValue = ph.readPH(phVoltage, temperature);     // Convert voltage to pH, applying temperature compensation

        // TDS Sensor Reading
        gravityTds.setTemperature(temperature);   // Set temperature for compensation
        gravityTds.update();                      // Sample and calculate the TDS value
        tdsValue = gravityTds.getTdsValue();      // Get the TDS value

        // Print out the sensor values
        Serial.print("Temperature: ");
        Serial.print(temperature, 1);
        Serial.print(" ^C  pH: ");
        Serial.print(phValue, 2);
        Serial.print("  TDS: ");
        Serial.print(tdsValue, 0);
        Serial.println(" ppm");
    }

    // Calibration of the pH sensor
    ph.calibration(phVoltage, temperature);  // Calibration process through Serial commands

    delay(1000);  // Wait 1 second before next reading
}