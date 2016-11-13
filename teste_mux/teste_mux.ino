#define nCabos 1
#define nSensores 6

#define PinMuxTerm0 6
#define PinMuxTerm1 7
#define PinMuxTerm2 8

#define PinMuxCabo0 A0
#define PinMuxCabo1 A1
#define PinMuxCabo2 A2

#include <SoftwareSerial.h>
#include <SPI.h>
#include "Nanoshield_Termopar.h"
Nanoshield_Termopar tc(A3, TC_TYPE_T, TC_AVG_OFF);  // Termopar Nanoshield on CS pin D8, type

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PinMuxTerm0, OUTPUT);
  pinMode(PinMuxTerm1, OUTPUT);
  pinMode(PinMuxTerm2, OUTPUT);

  pinMode(PinMuxCabo0, OUTPUT);
  pinMode(PinMuxCabo1, OUTPUT);
  pinMode(PinMuxCabo2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  TestaMux();
}

void TestaMux() {
  float temps[nCabos][nSensores];
  //  for (int cabo = 0; cabo < nCabos; cabo++)
  //  {

  //Seta controle para o MUX que seleciona o cabo
  digitalWrite(PinMuxCabo0, 1);
  digitalWrite(PinMuxCabo1, 0);
  digitalWrite(PinMuxCabo2, 0
  );

  for (int sensor = 0; sensor < nSensores; sensor++)
  {
    //Seta controle para o MUX que controla o termopar
    digitalWrite(PinMuxTerm0, bitRead(sensor, 0));
    digitalWrite(PinMuxTerm1, bitRead(sensor, 1));
    digitalWrite(PinMuxTerm2, bitRead(sensor, 2));

    delay(500);

    tc.read();
    temps[0][sensor] = tc.getExternal();

    if (tc.hasError()) Serial.println("erro");

    Serial.print("0"); Serial.print("-"); Serial.print(sensor); Serial.print("\t");
    Serial.println(temps[0][sensor]);

    delay(500);
  }
  //  }
}

