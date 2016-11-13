#include <DHT.h>
DHT dhtI(5, DHT22);
DHT dhtE(8, DHT22);

#include <SPI.h>
#include "Nanoshield_Termopar.h"

// Termopar Nanoshield on CS pin D8, type K thermocouple, no averaging
Nanoshield_Termopar tc(A3, TC_TYPE_T, TC_AVG_OFF);

void setup() {
  // put your setup code here, to run once:
  dhtI.begin();
  dhtE.begin();
  tc.begin();
  Serial.begin(115200);
  Serial.println("TempInt,UmidInt,TempExt,UmidExt,Termopar");
}

void loop() {
  float tempI = dhtI.readTemperature();
  float umidI = dhtI.readHumidity();
  delay(100);
  float tempE = dhtE.readTemperature();
  float umidE = dhtE.readHumidity();
  delay(100);
  tc.read();
  float termopar = tc.getExternal();

  Serial.print(tempI); Serial.print(","); Serial.print(umidI); Serial.print(",");
  Serial.print(tempE); Serial.print(","); Serial.print(umidE); Serial.print(",");

  if (tc.hasError()) {printErrors();}
  else {Serial.println(termopar);}
  

  for (int i = 0; i < 30; i++) {
    delay(1000);
  }

}

void printErrors() {
  if (tc.isOpen()) {
    Serial.println("Open circuit");
  } else if (tc.isOverUnderVoltage()) {
    Serial.println("Overvoltage/Undervoltage");
  } else if (tc.isInternalOutOfRange()) {
    Serial.println("Internal temperature (cold junction) out of range)");
  } else if (tc.isExternalOutOfRange()) {
    Serial.println("External temperature (hot junction) out of range");
  }
}
