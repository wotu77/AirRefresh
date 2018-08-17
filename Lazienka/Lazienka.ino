/*
 Name:		Lazienka.ino
 Created:	07.10.2017 18:07:42
 Author:	wotu
*/
// Enable debug prints
//#include <MyConfig.h>

//#include <DHT_U.h>
//#include <MyConfig.h>

//#define MY_DEBUG
// Enable and select radio type attached 
#define MY_RADIO_NRF24
#define MY_NODE_ID 6



//#define MY_RADIO_RFM69
//#define MY_RS485

#include <SPI.h>
#include <MySensors.h>  

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <TimeLib.h>
#include "EEPROMextent.h"

//#include <EEPROM.h>

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 7
//#define DHTLIB_TIMEOUT  10000 
#define PUFF_PIN 8



#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0

#define SENSOR_TEMP_OFFSET 2.0
#define SENSOR_HUM_OFFSET 31.0

static const long UPDATE_INTERVAL = 30000;// 0;

static const uint8_t FORCE_UPDATE_N_READS = 6;

#define CHILD_ID_HUM 35
#define CHILD_ID_TEMP 31
//#define CHILD_ID_VAR 32
#define CHILD_ID_LIGHT 33
#define CHILD_ID_RELAY 34

bool jasnoscStatus = false;
bool jasnoscLastStatus = false;
unsigned long jasnoscLastChange = 0;


float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
uint8_t nNoUpdatesLight;
bool metric = true;

#define LUX_DATA_PIN 0

#define MAX_ADC_READING           1023
#define ADC_REF_VOLTAGE           5.0
#define REF_RESISTANCE           4600  // measure this for best results
#define LUX_CALC_SCALAR           12518931
#define LUX_CALC_EXPONENT         -1.405


int16_t lastLux=0;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
//MyMessage msgVar1(CHILD_ID_RELAY, V_VAR1);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgRelay(CHILD_ID_RELAY, V_STATUS);

DHT dht(DHT_DATA_PIN, DHT11);

time_t lastRefreshTime = 0;
unsigned long lastTempTime = 0;
unsigned long lastHumTime = 0;
unsigned long lastLevelTime = 0;
unsigned long lastGetTime = 0;
#define PIN_IRQ  3

bool timeReceived = false;
int refreshLastTmeAdress = 5;

void presentation()
{
	//noInterrupts();
	// Send the sketch version information to the gateway
	sendSketchInfo("RefreshAir", "0.7.5");

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_TEMP, S_TEMP);
	//present(CHILD_ID_VAR, S_CUSTOM);
	present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
	present(CHILD_ID_RELAY, S_BINARY);

	metric = getControllerConfig().isMetric;
	

	//interrupts();
}


void receiveTime(unsigned long time) {
	// Ok, set incoming time 
	//Serial.println("przyszedl czas");
	setTime(time);
	timeReceived = true;

}

// the setup function runs once when you press reset or power the board
void setup() {

	//dht.setup(DHT_DATA_PIN);
		
	dht.begin();

	// Sleep for the time of the minimum sampling period to give the sensor time to power up
	// (otherwise, timeout errors might occure for the first reading)
	
	//lastRefreshTime = loadState(0);
	//lastRefreshTime = EEPROMReadlong(0);
	//lastRefreshTime = now();

	pinMode(PUFF_PIN, OUTPUT);
	pinMode(PIN_IRQ, INPUT);

	
	char output[10];
	EEPROMextent.readString(refreshLastTmeAdress, output, 10);
	
	lastRefreshTime = strtoul(output, NULL, 10);



	//attachInterrupt(digitalPinToInterrupt(PIN_IRQ), irqSwiatlo, FALLING);
	send(msgRelay.set(RELAY_OFF));
	
	//requestTime();

}


// the loop function runs over and over again until power down or reset
void loop() {
	

	/*Serial.println("------------------------------------");

	Serial.print(hour());

	Serial.print(":");
	Serial.print(minute());

	Serial.print(":");
	Serial.print(second());

	Serial.print(" ");

	Serial.print(day());

	Serial.print(" ");

	Serial.print(month());

	Serial.print(" ");

	Serial.print(year());

	Serial.println();

	Serial.println("------------------------------------");*/

	sleep(5);
	jasnoscStatus = digitalRead(PIN_IRQ);
	if (jasnoscStatus != jasnoscLastStatus) {
		jasnoscLastStatus = jasnoscStatus;
		jasnoscLastChange = millis();

		int16_t jasnosc = (1023 - analogRead(LUX_DATA_PIN)) / 10.23;
		lastLux = jasnosc;
		lastLevelTime = millis();
		send(msgLight.set(jasnosc, 1));
		nNoUpdatesLight = 0;

	}

	sendJasnosc();
	
	

	//smartSleep(2000); // przerwa na pomiar
	sendTemperature();
	sendHumidity();


	
	if (!timeReceived) {
		//Serial.println("potrzebuje czas");
		
		requestTime();
		smartSleep(2000);
		//wait(3000);
		//delay(3000);
	}
	else {

		sendZapach();

		if ((jasnoscStatus) && (jasnoscLastChange + 600UL) < millis()) { // po zgaszeniu swiatla jeszcze przez 10 minut czujnik jest aktywny
			//Serial.println("zasypiam");
			timeReceived = false;
			sleep(digitalPinToInterrupt(PIN_IRQ), CHANGE, 1500UL);
		}
		else {
			//Serial.println("czuwam");
			wait(60000);
			
		}
	}
	
	
	
}

void receive(const MyMessage &message)
{
	Serial.print(message.type);
	Serial.println(" Jest wiadomosc");
	if (message.isAck()) {
		Serial.println("This is an ack from gateway");
	}
	if (message.type == V_VAR1) {
		Serial.print("V_VAR1: ");
		Serial.println(message.getString());
	}
	if (message.type == V_STATUS) {
		// Change relay state
		//digitalWrite(PUFF_PIN, message.getBool() ? RELAY_ON : RELAY_OFF);
		if (message.getBool() == RELAY_ON) {
			lastRefreshTime = now();
			Serial.println("----------------last----------------");
			Serial.println(lastRefreshTime);
			Serial.print(hour(lastRefreshTime));

			Serial.print(":");
			Serial.print(minute(lastRefreshTime));

			Serial.print(":");
			Serial.print(second(lastRefreshTime));

			Serial.print(" ");

			Serial.print(day(lastRefreshTime));

			Serial.print(" ");

			Serial.print(month(lastRefreshTime));

			Serial.print(" ");

			Serial.print(year(lastRefreshTime));

			Serial.println();
			Serial.println("------------------------------------");
			//EEPROMWritelong(0, lastRefreshTime);
			//EEPROM_writelong(0, lastRefreshTime);
			//EEPROM.writeLong(0, lastRefreshTime);

			char buf[10];
			ltoa(lastRefreshTime, buf, 10);
			EEPROMextent.writeString(refreshLastTmeAdress, buf);

			//send(msgRelay.set(RELAY_ON));
			digitalWrite(PUFF_PIN, RELAY_ON);

			delay(2000);

			digitalWrite(PUFF_PIN, RELAY_OFF);
			send(msgRelay.set(RELAY_OFF));
		}
		if (message.getBool() == RELAY_OFF) {
		
			digitalWrite(PUFF_PIN, RELAY_OFF);
		}
		// Store state in eeprom
		//saveState(1, message.getBool());
		// Write some debug info

	}
	
	
}
//
//float LightSensorLDR() {
//
//	int ADC;
//	float RLDR;
//	double Vout;
//	int Lux;
//
//	ADC = analogRead(LUX_DATA_PIN);
//
//	int16_t lightLevel = ADC / 10.23;
//
//
//	Vout = (ADC * 0.0048828125);    // Vout = Output voltage from potential Divider. [Vout = ADC * (Vin / 1024)] 
//
//	RLDR = (10000.0 * (5 - Vout));     // Equation to calculate Resistance of LDR, [R-LDR =(R1 (Vin - Vout))/ Vout]
//											  // R1 = 10,000 Ohms , Vin = 5.0 Vdc.
//
//	Lux = (2500 / Vout - 500) / 10;
//	//Lux = 2500.0 / (1.0 * ((5.0 - Vout) / Vout));
//
//
//	int   ldrRawData;
//	float resistorVoltage, ldrVoltage;
//	float ldrResistance;
//	float ldrLux;
//
//	// Perform the analog to digital conversion  
//	ldrRawData = ADC;
//
//	// RESISTOR VOLTAGE_CONVERSION
//	// Convert the raw digital data back to the voltage that was measured on the analog pin
//	resistorVoltage = (float)ldrRawData / MAX_ADC_READING * ADC_REF_VOLTAGE;
//
//	// voltage across the LDR is the 5V supply minus the 5k resistor voltage
//	ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;
//
//	// LDR_RESISTANCE_CONVERSION
//	// resistance that the LDR would have for that voltage  
//	ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
//
//	// LDR_LUX
//	// Change the code below to the proper conversion from ldrResistance to
//	// ldrLux
//	ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);
//
//
//	//return Lux;
//	return lightLevel;
//}




void sendTemperature() {
	//delay(dht.getMinimumSamplingPeriod());
	float temperature = dht.readTemperature();
	//Serial.print("bez update temp: ");
	//Serial.println(nNoUpdatesTemp);
	if (isnan(temperature)) {
		Serial.print("Failed reading temperature from DHT! - ");
	}
	else if (((temperature != lastTemp ) || (nNoUpdatesTemp >= FORCE_UPDATE_N_READS)) && (lastTempTime + 300000 < millis())) {
		// Only send temperature if it changed since the last measurement or if we didn't send an update for n times
		
		lastTempTime = millis();
		lastTemp = temperature;
		if (!metric) {
			temperature = dht.convertCtoF(temperature);
		}
		// Reset no updates counter
		nNoUpdatesTemp = 0;
		temperature += SENSOR_TEMP_OFFSET;
		send(msgTemp.set(temperature, 1));

	}
	else {
		// Increase no update counter if the temperature stayed the same
		nNoUpdatesTemp++;
	}
}

void sendHumidity() {
	//delay(dht.getMinimumSamplingPeriod());
	float humidity = dht.readHumidity();

	if (isnan(humidity)) {
		Serial.print("Failed reading humidity from DHT - ");

	}
	else if (((humidity != lastHum )||( nNoUpdatesHum >= FORCE_UPDATE_N_READS)) & (lastHumTime + 300000 < millis())) {
		// Only send humidity if it changed since the last measurement or if we didn't send an update for n times
		lastHum = humidity;
		lastHumTime = millis();
		// Reset no updates counter
		nNoUpdatesHum = 0;
		humidity += SENSOR_HUM_OFFSET;
		send(msgHum.set(humidity, 1));

	}
	else {
		// Increase no update counter if the humidity stayed the same
		nNoUpdatesHum++;
	}
}

void sendJasnosc() {
	

	//delay(5);
	int16_t jasnosc = (1023 - analogRead(LUX_DATA_PIN)) / 10.23;

	Serial.print("Send jasnosc: ");
	Serial.println(jasnosc);
	Serial.print("last lux: ");
	Serial.println(lastLux);
	Serial.print("nNoUpdatesLight: ");
	Serial.println(nNoUpdatesLight);
	Serial.print("lastLevelTime: ");
	Serial.println(lastLevelTime);

	if (((lastLux != jasnosc) || (nNoUpdatesLight >= FORCE_UPDATE_N_READS)) && (lastLevelTime + 30000< millis())) {
		Serial.println("Send jasnosc OK ");
		
		lastLux = jasnosc;
		lastLevelTime = millis();
		send(msgLight.set(jasnosc, 1));
		nNoUpdatesLight = 0;
	}
	else {
		Serial.println("Send jasnosc NO OK ");
		
		nNoUpdatesLight++;
	}
}

void sendZapach() {
	if (!jasnoscStatus && timeReceived && (lastRefreshTime + 1800UL < now())) {
		/*Serial.println("zapach k1");*/
		
			lastRefreshTime = now();
			/*Serial.println("----------------last----------------");
			Serial.println(lastRefreshTime);
			Serial.print(hour(lastRefreshTime));

			Serial.print(":");
			Serial.print(minute(lastRefreshTime));

			Serial.print(":");
			Serial.print(second(lastRefreshTime));

			Serial.print(" ");

			Serial.print(day(lastRefreshTime));

			Serial.print(" ");

			Serial.print(month(lastRefreshTime));

			Serial.print(" ");

			Serial.print(year(lastRefreshTime));

			Serial.println();
			Serial.println("------------------------------------");*/

			char buf[10];
			ltoa(lastRefreshTime, buf, 10);
			/*Serial.print("buf: ");
			Serial.println(buf);*/
			
			EEPROMextent.writeString(refreshLastTmeAdress, buf);
			//EEPROM.writeBlock<char>(0, buf, 10);

			send(msgRelay.set(RELAY_ON));
			digitalWrite(PUFF_PIN, RELAY_ON);
			
			delay(2000);
			
			digitalWrite(PUFF_PIN, RELAY_OFF);
			send(msgRelay.set(RELAY_OFF));
			
		
	}
	/*else {
		Serial.println("zapach k2");
		Serial.println("----------oczekuje jeszcze-------------");
		Serial.print(minute(lastRefreshTime + 1800UL -now()));
		Serial.println("----------ostatnio bylo-------------");
		Serial.println(lastRefreshTime);
		Serial.print(hour(lastRefreshTime));

		Serial.print(":");
		Serial.print(minute(lastRefreshTime));

		Serial.print(":");
		Serial.print(second(lastRefreshTime));

		Serial.print(" ");

		Serial.print(day(lastRefreshTime));

		Serial.print(" ");

		Serial.print(month(lastRefreshTime));

		Serial.print(" ");

		Serial.print(year(lastRefreshTime));

		Serial.println();
		
		Serial.println("----------bedzie-------------");
		
		Serial.println(lastRefreshTime);
		Serial.print(hour(lastRefreshTime+ 1800UL));

		Serial.print(":");
		Serial.print(minute(lastRefreshTime + 1800UL));

		Serial.print(":");
		Serial.print(second(lastRefreshTime + 1800UL));

		Serial.print(" ");

		Serial.print(day(lastRefreshTime + 1800UL));

		Serial.print(" ");

		Serial.print(month(lastRefreshTime + 1800UL));

		Serial.print(" ");

		Serial.print(year(lastRefreshTime + 1800UL));

		Serial.println();
		Serial.println("------------------------------------");
		
	}*/
}

void irqSwiatlo() {
	noInterrupts();
#ifdef MY_DEBUG_1
	Serial.println("-------------Irq-----------------");
	//digitalClockDisplay();
#endif
	checkRefresh();
	//sendJasnosc();
	delay(30000);
#ifdef MY_DEBUG_1
	Serial.println("-------------Irq end-------------");
#endif
	interrupts();
}



void checkRefresh() {
#ifdef MY_DEBUG_1
	Serial.println("-------------CHECK REFRESH-------");
#endif
	if (digitalRead(PIN_IRQ)) {
#ifdef MY_DEBUG_1
		Serial.println("Poziom jasnosci za niski");
		float jasnosc = LightSensorLDR();

	Serial.print("J: ");
	Serial.println(jasnosc);
#endif

	}else {
#ifdef MY_DEBUG_1
		Serial.println("Poziom jasnosci OK");
		float jasnosc = LightSensorLDR();

		Serial.print("J: ");
		Serial.println(jasnosc);
		
		
#endif
		if (lastRefreshTime + 1800000 < millis()) {
#ifdef  MY_DEBUG_1
			
			Serial.println(" akcja refresh");
#endif //  MY_DEBUG_1

			refreshAir(false);
		}
		else {
#ifdef MY_DEBUG_1
			Serial.println("--NIE MOGE WYKONAC REFRESH bo nie minal czas -----");
			Serial.print((lastRefreshTime + 1800000 - millis())/1000/60);
			
			
			Serial.print(" minut");
			Serial.println();
			Serial.println("---------------------------------- -----");
#endif
		}
	}
#ifdef MY_DEBUG_1
	Serial.println("-------------CHECK REFRESH-end---");
#endif
}

void refreshAir(bool noMessage) {
	//noInterrupts

	lastRefreshTime = millis();
#ifdef MY_DEBUG_1
	Serial.println("------------Refresh wykonany teraz----------------");
	Serial.print(lastRefreshTime/1000/60);
	Serial.println(" minut od uruchomienia");
	Serial.println("--------------------------------------------------");

#endif
	//saveState(0, lastRefreshTime);
	//EEPROMWritelong(0, lastRefreshTime);

	if (!noMessage) {
#ifdef MY_DEBUG_1
		Serial.println("-- SEND ON ---");
#endif
		send(msgRelay.set(RELAY_ON));
	}
	
	digitalWrite(PUFF_PIN, RELAY_ON);
	//Serial.println(millis());
	delay(2000);
	//Serial.println(millis());

	//digitalWrite(PUFF_PIN, HIGH);
	digitalWrite(PUFF_PIN, RELAY_OFF);
	if (!noMessage) {
#ifdef MY_DEBUG_1
		Serial.println("-- SEND OFF ---");
#endif
		send(msgRelay.set(RELAY_OFF));
	}

#ifdef MY_DEBUG_1


	//Serial.print(" last time refresh ");
	//printTime(lastRefreshTime);

	//Serial.print("  a bedzie o " );
	//printTime(lastRefreshTime+1800);
	//Serial.print("  to jest ");
	//Serial.print(minute((now()+ 1800) - lastRefreshTime));
	//Serial.println(" minut ");
	//Serial.println(minute(1800));
	Serial.println("");
	Serial.println("------------Koniec refresh -----------------------");
#endif
	//interrupts();

}







