#include "WProgram.h"

#define c_BatteryVInPin 8 // analog input pin for the battery voltage divider
#define c_VoltageDividerRatio 2.921

float _BatteryVoltage;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  int rawValue = analogRead(c_BatteryVInPin);         // read the voltage on the divider
  float dividedBatteryVoltage = rawValue * 0.00488;   // A reading of 1 for the A/D = 5V / 1024 = 0.00488 V
  float _BatteryVoltage = dividedBatteryVoltage * c_VoltageDividerRatio; 

  Serial.print(_BatteryVoltage, 4);
  Serial.print("\n");

  delay(500);
}

