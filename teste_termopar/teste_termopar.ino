/**
   @file SerialThermometer.ino
   This is a simple serial port thermometer application using the Termopar Nanoshield.

   Copyright (c) 2015 Circuitar
   This software is released under the MIT license. See the attached LICENSE file for details.
*/
#include <SPI.h>
#include "Nanoshield_Termopar.h"

// Termopar Nanoshield on CS pin D8, type K thermocouple, no averaging
Nanoshield_Termopar tc(A3, TC_TYPE_T, TC_AVG_OFF);

void setup()
{
  Serial.begin(115200);
  Serial.println("-------------------------------");
  Serial.println(" Nanoshield Serial Thermometer");
  Serial.println("-------------------------------");
  Serial.println();

  tc.begin();
}

void printErrors() {
  if (tc.isOpen()) {
    Serial.print("Open circuit");
  } else if (tc.isOverUnderVoltage()) {
    Serial.print("Overvoltage/Undervoltage");
  } else if (tc.isInternalOutOfRange()) {
    Serial.print("Internal temperature (cold junction) out of range)");
  } else if (tc.isExternalOutOfRange()) {
    Serial.print("External temperature (hot junction) out of range");
  }
}

void loop()
{
  // Read thermocouple data
  tc.read();


  Serial.print("Internal: ");
  Serial.print(tc.getInternal());
  Serial.print("\t | External: ");
  Serial.print(tc.getExternal());
  Serial.print("\t");
  printErrors();
  Serial.println();


  // Wait for next reading
  delay(1000);
}
