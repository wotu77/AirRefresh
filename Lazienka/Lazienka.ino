/*
 Name:		Lazienka.ino
 Created:	07.10.2017 18:07:42
 Author:	wotu
*/
// Enable debug prints
//#include <MyConfig.h>
//#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
#define MY_NODE_ID 14

//#define MY_RADIO_RFM69
//#define MY_RS485

#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>
#include <TimeLib.h>
#include <EEPROM.h>

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 7
//#define DHTLIB_TIMEOUT  10000 
#define PUFF_PIN 8

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0

#define SENSOR_TEMP_OFFSET 0

static const long UPDATE_INTERVAL = 30000;

static const uint8_t FORCE_UPDATE_N_READS = 6;

#define CHILD_ID_HUM 30
#define CHILD_ID_TEMP 31
#define CHILD_ID_VAR 32
#define CHILD_ID_LIGHT 33
#define CHILD_ID_RELAY 34


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


float lastLux;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
//MyMessage msgVar1(CHILD_ID_HUM, V_VAR1);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgRelay(CHILD_ID_RELAY, V_STATUS);
DHT dht;

bool getTime = false;
unsigned long lastRefreshTime = 0;
unsigned long lastGetTime = 0;
#define PIN_IRQ  3

void presentation()
{
	// Send the sketch version information to the gateway
	sendSketchInfo("Czujnik-lazienka", "0.5.1");

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_TEMP, S_TEMP);
	//present(CHILD_ID_VAR, S_HUM);
	present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
	present(CHILD_ID_RELAY, S_BINARY);

	metric = getControllerConfig().isMetric;
}

// the setup function runs once when you press reset or power the board
void setup() {
	//request(CHILD_ID_HUM, V_VAR1);
	
	dht.setup(DHT_DATA_PIN);
	if (UPDATE_INTERVAL <= (uint32_t)dht.getMinimumSamplingPeriod()) {

		Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
	}
	
	// Sleep for the time of the minimum sampling period to give the sensor time to power up
	// (otherwise, timeout errors might occure for the first reading)
	wait(dht.getMinimumSamplingPeriod());
	//lastRefreshTime = loadState(0);
	lastRefreshTime = EEPROMReadlong(0);
	Serial.print(" last time refresh from loadstate");
	printTime(lastRefreshTime);
	Serial.println("");
	pinMode(PUFF_PIN, OUTPUT);
	pinMode(PIN_IRQ,INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_IRQ), irqSwiatlo, RISING);
	
	

}

// the loop function runs over and over again until power down or reset
void loop() {

	if (lastGetTime==0 || now()>lastGetTime+ 86400)  // co 24 godziny
	{
		printTime();
		Serial.println("Wysy³am zadanie czasu");
		lastGetTime = now();
		requestTime();
	}


	//request(CHILD_ID_HUM, V_VAR1);
	//delay(dht.getMinimumSamplingPeriod());
	//sleep(dht.getMinimumSamplingPeriod());

	sendTemperature();
	
	sendHumidity();

	sendJasnosc();
	
#ifdef MY_DEBUG
	//Serial.println("---------------maly kroczek-------------- ");

#endif

	//send(msgVar1.set(77, 1));
	
	checkRefresh();
	//send(msgVar1.set(t, 1));
	// jak jest sleep nie odbierane sa  wiadomosci
	Serial.print("Teraz jest:               ");
	printTime();
	Serial.println();
	Serial.print("Last time ze zmiennej:    ");
	printTime(lastRefreshTime);
	Serial.println();
	Serial.print("Last time z eeprom:       ");
	printTime(EEPROMReadlong(0));
	Serial.println();

	Serial.print("Data aktualizowana o:     ");
	printTime(lastGetTime);
	Serial.println();

	
	wait(UPDATE_INTERVAL);

}

void receive(const MyMessage &message)
{
	//Serial.println("Jest wiadomosc: ");
	if (message.type == V_VAR1) {

		Serial.print("V_VAR1: ");
		Serial.println(message.getString());

	}
	if (message.type == V_STATUS) {
		// Change relay state
		//digitalWrite(PUFF_PIN, message.getBool() ? RELAY_ON : RELAY_OFF);
		refreshAir();
		// Store state in eeprom
		//saveState(1, message.getBool());
		// Write some debug info
		Serial.print("Incoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print(", New status: ");
		Serial.println(message.getBool());
	}

}

float LightSensorLDR() {

	int ADC;
	float RLDR;
	double Vout;
	int Lux;

	ADC = analogRead(LUX_DATA_PIN);

	int16_t lightLevel = ADC / 10.23;

	Serial.print("Jasnosc (%): ");
	Serial.println(lightLevel);

	Vout = (ADC * 0.0048828125);    // Vout = Output voltage from potential Divider. [Vout = ADC * (Vin / 1024)] 

	RLDR = (10000.0 * (5 - Vout));     // Equation to calculate Resistance of LDR, [R-LDR =(R1 (Vin - Vout))/ Vout]
											  // R1 = 10,000 Ohms , Vin = 5.0 Vdc.



	Lux = (2500 / Vout - 500) / 10;
	//Lux = 2500.0 / (1.0 * ((5.0 - Vout) / Vout));


	int   ldrRawData;
	float resistorVoltage, ldrVoltage;
	float ldrResistance;
	float ldrLux;

	// Perform the analog to digital conversion  
	ldrRawData = ADC;

	// RESISTOR VOLTAGE_CONVERSION
	// Convert the raw digital data back to the voltage that was measured on the analog pin
	resistorVoltage = (float)ldrRawData / MAX_ADC_READING * ADC_REF_VOLTAGE;

	// voltage across the LDR is the 5V supply minus the 5k resistor voltage
	ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;

	// LDR_RESISTANCE_CONVERSION
	// resistance that the LDR would have for that voltage  
	ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;

	// LDR_LUX
	// Change the code below to the proper conversion from ldrResistance to
	// ldrLux
	ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);



	Serial.print("Jasnosc (V-LUX): ");
	Serial.print(ADC);
	Serial.print("\t");
	Serial.println(Lux);

	//return Lux;
	return lightLevel;
}


void receiveTime(unsigned long controllerTime) {
	getTime = true;
	setTime(controllerTime);
	lastGetTime = now();;
	// Ok, set incoming time 
	//deltaTime = controllerTime - millis();
	Serial.print("Time value received: ");

	printTime(controllerTime);
	Serial.println("");
	
	checkRefresh();
	

}

void sendTemperature() {
	delay(dht.getMinimumSamplingPeriod());
	float temperature = dht.getTemperature();
	if (isnan(temperature)) {

		Serial.print("Failed reading temperature from DHT! - ");
		Serial.println(dht.getStatusString());
	}
	else if (temperature != lastTemp || nNoUpdatesTemp >= FORCE_UPDATE_N_READS) {
		// Only send temperature if it changed since the last measurement or if we didn't send an update for n times
		lastTemp = temperature;
		if (!metric) {
			temperature = dht.toFahrenheit(temperature);
		}
		// Reset no updates counter
		nNoUpdatesTemp = 0;
		temperature += SENSOR_TEMP_OFFSET;
		send(msgTemp.set(temperature, 1));

#ifdef MY_DEBUG
		Serial.print("T: ");
		Serial.println(temperature);
#endif
	}
	else {
		// Increase no update counter if the temperature stayed the same
		nNoUpdatesTemp++;
	}
}

void sendHumidity() {
	delay(dht.getMinimumSamplingPeriod());
	float humidity = dht.getHumidity();

	//if (dht.getStatusString() == "TIMEOUT") {
	//	delay(dht.getMinimumSamplingPeriod());
	//	temperature = dht.getTemperature();
	//}


	// Get humidity from DHT library

	if (isnan(humidity)) {
		Serial.print("Failed reading humidity from DHT - ");
		Serial.println(dht.getStatusString());
	}
	else if (humidity != lastHum || nNoUpdatesHum >= FORCE_UPDATE_N_READS) {
		// Only send humidity if it changed since the last measurement or if we didn't send an update for n times
		lastHum = humidity;
		// Reset no updates counter
		nNoUpdatesHum = 0;
		send(msgHum.set(humidity, 1));

#ifdef MY_DEBUG
		Serial.print("H: ");
		Serial.println(humidity);
#endif
	}
	else {
		// Increase no update counter if the humidity stayed the same
		nNoUpdatesHum++;
	}
}

void sendJasnosc() {
	float jasnosc = LightSensorLDR();
#ifdef MY_DEBUG
	Serial.print("J: ");
	Serial.println(jasnosc);
#endif
	if (lastLux != jasnosc || nNoUpdatesLight >= FORCE_UPDATE_N_READS) {
		lastLux = jasnosc;
		send(msgLight.set(jasnosc, 1));
		nNoUpdatesLight = 0;
	}
	else {
		nNoUpdatesLight++;
	}
}

void irqSwiatlo() {
	sendJasnosc();
	refreshAir();

}

void checkRefresh() {
	if (digitalRead(PIN_IRQ)) {
		Serial.println("IRQ jest wysoki");
		
	}
	else {
		Serial.println("IRQ jest niski");
		printTime();
		Serial.println(" akcja refresh");
		if ((getTime) & (lastRefreshTime + 1800 < now())) {
			refreshAir();
		}
		else {
			Serial.println("!!!!!!!!  NIE MOGE WYKONAC REFRESH !!!!!!!!!!!!");
		}
	}
}

void refreshAir() {
	
		Serial.println("Refresh wykonany teraz:");
		lastRefreshTime = now();
		//saveState(0, lastRefreshTime);
		EEPROMWritelong(0, lastRefreshTime);
		send(msgRelay.set(RELAY_ON));
		digitalWrite(PUFF_PIN, RELAY_ON);
		delay(700);
		digitalWrite(PUFF_PIN, RELAY_OFF);
		send(msgRelay.set(RELAY_OFF));
	
	
	printTime();
	Serial.print(" last time refresh ");
	printTime(lastRefreshTime);
	
	Serial.print("  a bedzie o " );
	printTime(lastRefreshTime+1800);
	Serial.print("  to jest ");
	Serial.print(minute((now()+ 1800) - lastRefreshTime));
	Serial.println(" minut ");
	Serial.println(minute(1800));


	
}

void printTime() {
	Serial.print(now());
	Serial.print("-");
	Serial.print(year());
	Serial.print("-");
	Serial.print(month());
	Serial.print("-");
	Serial.print(day());
	Serial.print(" ");
	Serial.print(hour());
	Serial.print(":");
	Serial.print(minute());
	Serial.print(":");
	Serial.print(second());

}

void printTime(unsigned long data) {
	Serial.print(data);
	Serial.print("-");
	Serial.print(year(data));
	Serial.print("-");
	Serial.print(month(data));
	Serial.print("-");
	Serial.print(day(data));
	Serial.print(" ");
	Serial.print(hour(data));
	Serial.print(":");
	Serial.print(minute(data));
	Serial.print(":");
	Serial.print(second(data));

}

void EEPROMWritelong(int address, long value)
{
	//Decomposition from a long to 4 bytes by using bitshift.
	//One = Most significant -> Four = Least significant byte
	byte five = (value & 0xFF);
	byte four = ((value>>8) & 0xFF);
	byte three = ((value >> 16) & 0xFF);
	byte two = ((value >> 24) & 0xFF);
	byte one = ((value >> 32) & 0xFF);

	//Write the 4 bytes into the eeprom memory.
	EEPROM.write(address, five);
	EEPROM.write(address + 1, four);
	EEPROM.write(address + 2, three);
	EEPROM.write(address + 3, two);
	EEPROM.write(address + 4, one);
}

//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.
long EEPROMReadlong(long address)
{
	//Read the 4 bytes from the eeprom memory.
	long five =  EEPROM.read(address);
	long four =  EEPROM.read(address + 1);
	long three = EEPROM.read(address + 2);
	long two =   EEPROM.read(address + 3);
	long one =   EEPROM.read(address + 4);

	//Return the recomposed long by using bitshift.
	return ((five << 0) & 0xFF) + ((four << 8) & 0xFFFF) + ((three << 16) & 0xFFFFFF) + ((two << 24) & 0xFFFFFFFF) + ((one << 32) & 0xFFFFFFFFFF);
}




