//BIBLIOTECAS-----------------------------------------------------------------------------------------------------------------//
#include <math.h>

#include <TheThingsUno.h>
TheThingsUno ttu;

#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);
SoftwareSerial RS485Serial(9, 10); // RX, TX
#define PinRS485 A4

#define debugSerial Serial
#define loraSerial Serial1

#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }

#include <DHT.h>
DHT dhtInt(4, DHT22);

#define DHTPIN 4
DHT dhtExt(5, DHT22);

#define PinMuxTerm0 6
#define PinMuxTerm1 7
#define PinMuxTerm2 8

#define PinMuxCabo0 A0
#define PinMuxCabo1 A1
#define PinMuxCabo2 A2

#include <SPI.h>
#include "Nanoshield_Termopar.h"

// Termopar Nanoshield on CS pin D8, type T thermocouple, no averaging
Nanoshield_Termopar tc(A3, TC_TYPE_T, TC_AVG_OFF);

//CONSTANTES E VARIAVEIS---------------------------------------------------------------------------------------------------//
const byte devAddr[4] = {0x02, 0x01, 0x55, 0xB0};
const byte nwkSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
const byte appSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

#define nCabos 5
#define nSensores 8

bool ligarVent;
float tempI, umidI, tempE, umidE, tempGrao, altura;

byte data[12]; //0: Bits controle / 1: Uso estimado / 2,3: temp grao / 4,5: temp int / 6,7: umid int / 8,9: temp ext / 10,11: umid ext
float tensaoRef;
uint8_t difUmid;

//ERROS POSSIVEIS---------------------------------------------------------------------------------------------------------//
bool erro_dht_int, erro_dht_ext, erro_termopar, erro_altura, erro_rs485;


void setup() {
  debugSerial.begin(115200);
  loraSerial.begin(57600);
  dhtInt.begin();
  dhtExt.begin();
  delay(1000);

  ttu.init(loraSerial, debugSerial);
  ttu.reset();
  ttu.personalize(devAddr, nwkSKey, appSKey);
  delay(6000);

  ttu.showStatus();
  debugPrintLn("LoRa setup complete!");
  delay(1000);

  pinMode(PinMuxTerm0, OUTPUT);
  pinMode(PinMuxTerm1, OUTPUT);
  pinMode(PinMuxTerm2, OUTPUT);

  pinMode(PinMuxCabo0, OUTPUT);
  pinMode(PinMuxCabo1, OUTPUT);
  pinMode(PinMuxCabo2, OUTPUT);

  pinMode(PinRS485, OUTPUT);
  digitalWrite(PinRS485, 0);
  RS485Serial.begin(4800);

  tc.begin();
}

//LOOP--------------------------------------------------------------------------------------------------------------------//
void loop()
{
  //Le DHT da parte interna e externa do silo
  LeDHT();

  //Le temperatura dos termopares e calcula altura
  LeTermopares();

  //Gera os bits de controle
  SetaBitsControle();

  //Envia os dados LoRa
  ttu.sendBytes(data, sizeof(data), 30, true);

  //Envia comando de ligar para o segundo arduino


  //Aguarda próxima transmissão

}

//FUNÇÕES-----------------------------------------------------------------------------------------------------------------//
void LeDHT()
{
  erro_dht_int = false;
  erro_dht_ext = false;

  tempI = dhtInt.readTemperature();
  umidI = dhtInt.readHumidity();
  if (isnan(tempI) || isnan(umidI))
  {
    erro_dht_int = true;
    for (int i = 4; i <= 7; i++)
      data[i] = 0xFF;
  }
  else
  {
    int tempInt = round(tempI * 10);
    data[4] = highByte(tempInt);
    data[5] = lowByte(tempInt);
    debugPrint("Temperatura Interna: "); debugPrintLn(tempInt / 10.0);

    int umidInt = round(umidI * 10);
    data[6] = highByte(umidInt);
    data[7] = lowByte(umidInt);
    debugPrint("Umidade Interna: "); debugPrintLn(umidInt / 10.0);
  }

  tempE = dhtExt.readTemperature();
  umidE = dhtExt.readHumidity();
  if (isnan(tempE) || isnan(umidE))
  {
    erro_dht_ext = true;
    for (int i = 8; i <= 11; i++)
      data[i] = 0xFF;
  }
  else
  {
    int tempExt = round(tempE * 10);
    data[8] = highByte(tempExt);
    data[9] = lowByte(tempExt);
    debugPrint("Temperatura Interna: "); debugPrintLn(tempExt / 10.0);

    int umidExt = round(umidE * 10);
    data[10] = highByte(umidExt);
    data[11] = lowByte(umidExt);
    debugPrint("Umidade Interna: "); debugPrintLn(umidExt / 10.0);
  }
}

void LeTermopares()
{
  erro_termopar = false;
  erro_altura = false;

  //MEDIÇÃO DAS TEMPERATURAS DOS TERMOPARES
  float temps[nCabos][nSensores];
  for (int cabo = 0; cabo < nCabos; cabo++)
  {
    //Seta controle para o MUX que seleciona o cabo
    digitalWrite(PinMuxCabo0, bitRead(cabo, 0));
    digitalWrite(PinMuxCabo1, bitRead(cabo, 1));
    digitalWrite(PinMuxCabo2, bitRead(cabo, 2));

    for (int sensor = 0; sensor < nSensores; sensor++)
    {
      //Seta controle para o MUX que controla o termopar
      digitalWrite(PinMuxTerm0, bitRead(sensor, 0));
      digitalWrite(PinMuxTerm1, bitRead(sensor, 1));
      digitalWrite(PinMuxTerm2, bitRead(sensor, 2));

      delay(100);

      tc.read();
      temps[cabo][sensor] = tc.getExternal();

      if (tc.hasError()) erro_termopar = true;

      delay(900);
    }
  }

  //CALCULO DA ALTURA E USO ESTIMADO DO SILO
  int altIndice[nCabos];
  float sum;
  for (int i = 0; i < nCabos; i++)
  {
    float maxDif = 2;
    for (int j = 0; j < nSensores - 1; j++)
    {
      float dif = temps[i][j] - temps[i][j + 1];
      if (abs(dif) > abs(maxDif))
      {
        altIndice[i] = j;
        maxDif = dif;
      }

      if (maxDif == 2) erro_altura = true;
    }
    sum += altIndice[i] + 0.5;
  }
  sum /= (float)nCabos;
  altura = (100.0 * sum) / (nSensores - 1.5);

  uint8_t alt = round(altura);
  if (alt > 100 || alt <= 0) erro_altura = true;
  data[1] = lowByte(alt);
  debugPrint("Altura: "); debugPrintLn(alt);

  //CALCULO DA TEMPERATURA DO GRÃO
  float sumCabo[nCabos];
  tempGrao = 0;
  for (int i = 0; i < nCabos; i++)
  {
    sumCabo[i] = 0;
    for (int j = 0; j < altIndice[i]; j++)
    {
      sumCabo[i] += temps[i][j];
    }
    sumCabo[i] /= (altIndice[i] + 1);

    if (sumCabo[i] > 60) erro_termopar = true;
    tempGrao += sumCabo[i];
  }
  tempGrao /= (float)nCabos;

  int grao = round(tempGrao * 10);
  debugPrint("Temperatura grão: "); debugPrintLn(grao / 10.0);
  data[2] = highByte(grao);
  data[3] = lowByte(grao);
}

void SetaBitsControle()
{
  bitWrite(data[0], 7, ligarVent ? 1 : 0);
  bitWrite(data[0], 6, erro_dht_int ? 1 : 0);
  bitWrite(data[0], 5, erro_dht_ext ? 1 : 0);
  bitWrite(data[0], 4, erro_termopar ? 1 : 0);
  bitWrite(data[0], 3, erro_altura ? 1 : 0);
  bitWrite(data[0], 2, erro_rs485 ? 1 : 0);
}

void EnviaRS485() {
  digitalWrite(PinRS485, 1);  // Enable RS485 Transmit
  RS485Serial.write(0xA1);    //Envia dado para ligar/desligar
  delay(10);
  digitalWrite(PinRS485, 0);

  byte byteReceived;
  bool received = false;
  for (int i = 0; i < 5 && !received; i++) {
    delay(100);
    if (RS485Serial.available()) {
      byteReceived = RS485Serial.read();    // Read received byte
      if (byteReceived == 0xB1) erro_rs485 = true;
      debugPrint("Arduino Ventilador: "); debugPrintLn(byteReceived == 0xB1 ? "ok" : "erro");
    }
  }
}

