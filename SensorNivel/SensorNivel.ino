//BIBLIOTECAS
#include <TheThingsUno.h>
TheThingsUno ttu;

#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);
#define debugSerial Serial
#define loraSerial Serial1

#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }

//CONSTANTES E VARIAVEIS
const byte devAddr[4] = {0x02, 0x01, 0x55, 0xB0};
const byte nwkSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
const byte appSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

byte data[3];


void setup() {
  debugSerial.begin(115200);
  loraSerial.begin(57600);
  delay(1000);

  ttu.init(loraSerial, debugSerial);
  ttu.reset();
  ttu.personalize(devAddr, nwkSKey, appSKey);
  delay(6000);

  ttu.showStatus();
  debugPrintLn("Setup complete!");
  delay(1000);
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, 1);
  LeSensores();
  ttu.sendBytes(data, sizeof(data), 40, true);
  digitalWrite(13, 0);
  for (int i = 0; i < 30; i++)
  {
    for (int t = 0; t < 60; t++) {
      delay(1000);
    }
  }
}

void LeSensores() {
  digitalWrite(5, HIGH);
  delay(2500);
  int sensorValue = analogRead(A0);
  digitalWrite(5, LOW);
  
  // change the analog out value:
  float vout = (sensorValue * 5.0) / 1023;
  int tensao = round(vout * 10000);
  data[0] = highByte(tensao);
  data[1] = lowByte(tensao);

  float h = (vout - 0.21) / 2.793194;
  uint8_t cm = round(h * 100);
  data[2] = cm;

  Serial.print("Tensao = ");
  Serial.print(vout);
  Serial.print("V \t altura = ");
  Serial.print(cm);
  Serial.println(" cm");
}

