//BIBLIOTECAS
#include <TheThingsUno.h>
TheThingsUno ttu;

#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3);
#define debugSerial Serial
#define loraSerial Serial1

#include <DHT.h>
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define PinMuxTerm0 5
#define PinMuxTerm1 6
#define PinMuxTerm2 7

#define PinMuxCabo0 8
#define PinMuxCabo1 9
#define PinMuxCabo2 10

#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }

//CONSTANTES E VARIAVEIS
const byte devAddr[4] = {0x02, 0x01, 0x55, 0xB0};
const byte nwkSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
const byte appSKey[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

const int nCabos = 5;
const float ganho = 1000.0;

byte data[nCabos * 8 + 9];
float tensaoRef;


void setup() {
  debugSerial.begin(115200);
  loraSerial.begin(57600);
  dht.begin();
  delay(1000);

  ttu.init(loraSerial, debugSerial);
  ttu.reset();
  ttu.personalize(devAddr, nwkSKey, appSKey);
  delay(6000);

  ttu.showStatus();
  debugPrintLn("Setup complete!");
  delay(1000);

  pinMode(PinMuxTerm0, OUTPUT);
  pinMode(PinMuxTerm1, OUTPUT);
  pinMode(PinMuxTerm2, OUTPUT);

  pinMode(PinMuxCabo0, OUTPUT);
  pinMode(PinMuxCabo1, OUTPUT);
  pinMode(PinMuxCabo2, OUTPUT);
}

void loop() {
  LeSensores();
  ttu.sendBytes(data, sizeof(data), 30, true);

  for (int t = 0; t < 15 * 60; t++) {
    delay(1000);
  }
}

void LeSensores() {
  int16_t temperature = dht.readTemperature() * 10;
  data[0] = highByte(temperature);
  data[1] = lowByte(temperature);
  debugPrint("Temperatura: "); debugPrintLn(temperature / 10.0);
  tensaoRef = TempParaTensao(temperature / 10.0);

  int16_t humidity = dht.readHumidity() * 10;
  data[2] = highByte(humidity);
  data[3] = lowByte(humidity);
  debugPrint("Umidade: "); debugPrintLn(humidity / 10.0);

  for (int cabo = 0; cabo < nCabos; cabo++)
  {
    //Seta controle para o MUX que seleciona o cabo
    digitalWrite(PinMuxCabo0, bitRead(cabo, 0));
    digitalWrite(PinMuxCabo1, bitRead(cabo, 1));
    digitalWrite(PinMuxCabo2, bitRead(cabo, 2));

    {
      //Seta controle para o MUX que controla o termopar
      digitalWrite(PinMuxTerm0, bitRead(sensor, 0));
      digitalWrite(PinMuxTerm1, bitRead(sensor, 1));
      digitalWrite(PinMuxTerm2, bitRead(sensor, 2));

      int pos = 9 + (cabo * 8) + sensor;
      int8_t temp = GetTemp();
      data[pos] = temp;
      debugPrint("Temp termopar "); debugPrint(cabo); debugPrint("/"); debugPrint(sensor); debugPrint(": "); debugPrintLn(temp);
    }
  }
}

//Tensão em mV e temp em ºC
int8_t GetTemp()
{
  int tensaoD = analogRead(A0);
  float tensaoMiliV = map(tensaoD, 0, 1023, 0.0, 5000.0);
  float temp = 24.308 * ((tensaoMiliV / ganho) + tensaoRef) + 0.557;
  return round(temp);
}

  return 0.0411 * _temp - 0.0226;
}


