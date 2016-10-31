//BIBLIOTECAS-----------------------------------------------------------------------------------------------------------------//
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <DHT.h>
#include <SmartLoRa.h>
#include "Nanoshield_Termopar.h"

SmartLoRa lora;

#define PinRS485EN A4
Nanoshield_Termopar tc(A3, TC_TYPE_T, TC_AVG_OFF);  // Termopar Nanoshield on CS pin D8, type T thermocouple, no averaging

#define debugSerial Serial
#define loraSerial Serial1
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }
SoftwareSerial Serial1(2, 3);
SoftwareSerial RS485Serial(9, 10); // RX, TX

DHT dhtInt(4, DHT22);
DHT dhtExt(5, DHT22);

#define PinMuxTerm0 6
#define PinMuxTerm1 7
#define PinMuxTerm2 8

#define PinMuxCabo0 A0
#define PinMuxCabo1 A1
#define PinMuxCabo2 A2


//CONSTANTES E VARIAVEIS---------------------------------------------------------------------------------------------------//
const byte devAddr[4] = { 0x02, 0x01, 0x55, 0xB0 };
const byte nwkSKey[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
const byte appSKey[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

//Constantes gráfico de aeração
const float k = 0.1;
const int mInf = 3, mSup = 1;

#define nCabos 5
#define nSensores 7

float tempI, umidI, tempE, umidE, tempGrao, altura;
byte data[12]; //0: Bits controle / 1: Uso estimado / 2,3: temp grao / 4,5: temp int / 6,7: umid int / 8,9: temp ext / 10,11: umid ext
byte downlink;
bool ligarVent;


//ERROS POSSIVEIS---------------------------------------------------------------------------------------------------------//
bool erro_dht_int, erro_dht_ext, erro_termopar, erro_altura, erro_rs485, erro_downlink;


void setup() {
	debugSerial.begin(115200);
	loraSerial.begin(57600);
	dhtInt.begin();
	dhtExt.begin();
	delay(1000);

	lora.init(loraSerial, debugSerial);
	lora.reset();
	lora.personalize(devAddr, nwkSKey, appSKey);
	delay(6000);

	lora.showStatus();
	debugPrintLn("LoRa setup complete!");
	delay(1000);

	pinMode(PinMuxTerm0, OUTPUT);
	pinMode(PinMuxTerm1, OUTPUT);
	pinMode(PinMuxTerm2, OUTPUT);

	pinMode(PinMuxCabo0, OUTPUT);
	pinMode(PinMuxCabo1, OUTPUT);
	pinMode(PinMuxCabo2, OUTPUT);

	pinMode(PinRS485EN, OUTPUT);
	digitalWrite(PinRS485EN, 0);
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
	downlink = lora.sendBytes(data, sizeof(data), 30, true);

	//Checa a resposta do servidor
	ChecaServidor();

	//Envia comando de ligar para o segundo arduino
	EnviaRS485();

	//Aguarda próxima transmissão em 1 hora
	for (int i = 0; i < 60; i++)
		for (int j = 0; j < 60; j++)
			delay(1000);
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
	debugPrint("Uso estimado: "); debugPrintLn(alt);

	//CALCULO DA TEMPERATURA DO GRÃO
	float sumCabo[nCabos];
	tempGrao = 0;
	for (int i = 0; i < nCabos; i++)
	{
		sumCabo[i] = 0;
		for (int j = 0; j < altIndice[i]; j++)
			sumCabo[i] += temps[i][j];

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
	bitWrite(data[0], 1, erro_downlink ? 1 : 0);
}

void ChecaServidor() 
{
	delay(1000);

	if (downlink > 0 || downlink != 0xA0 || downlink != 0xA1) {
		erro_downlink = false;
		debugPrintLn("Received " + String(downlink) + " bytes")
			// Print the received bytes
			for (int i = 0; i < downlink; i++)
				debugPrint(String(lora.downlink[i]) + " ");

		debugPrintLn();
	}
	else
	{
		downlink = 0x00;
		erro_downlink = true;
	}
}

void EnviaRS485() {
	erro_rs485 = false;

	byte byteReceived, byteEnviar;
	bool received = false;

	if (erro_downlink) //Não recebeu dados do servidor ou é um valor diferente do esperado => usar base para verificar ventilação
	{
		switch (ChecaVentiladores())
		{
		case -1:	//Erro de leitura dos termopares ou do sensor de umidade
			byteEnviar = 0xFF; break;
		case 0:		//Desligar o ventilador
			byteEnviar = 0xA0; break;
		case 1:		//Ligar o ventilador
			byteEnviar = 0xA1; break;
		default:
			byteEnviar = 0xFF; break;
		}
	}
	else
		byteEnviar = downlink;

	for (int i = 0; i < 5 && !received; i++)
	{
		digitalWrite(PinRS485EN, 1);  // Enable RS485 Transmit
		RS485Serial.write(byteEnviar);    //Envia dado recebido pelo servidor
		delay(10);
		digitalWrite(PinRS485EN, 0);


		delay(1000);
		if (RS485Serial.available())
		{
			byteReceived = RS485Serial.read();    // Read received byte
			received = true;
			if (byteReceived != 0xB0 || byteReceived != 0xB1) erro_rs485 = true;	//Não respondeu com o status do ventilador
			debugPrint("Arduino Ventilador: "); debugPrintLn(!erro_rs485 ? "ok" : "erro");
		}
	}
}

int ChecaVentiladores()
{
	if (erro_dht_ext || erro_termopar)
		return -1;

	float difTemp = tempGrao - tempE;

	if (difTemp <= umidE * k - mInf) //Zona vermelha - aeração sem interesse
		return 0;

	else if (difTemp > umidE * k - mInf && difTemp <= umidE * k - mSup)	//Zona verde / azul - aeração recomendada		
		return 1;

	else	//Zona amarela - Aeração possivel mas sem interesse	
		return 0;
}

