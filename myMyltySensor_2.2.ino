#define MY_RADIO_NRF24 // Enable and select radio type attached 
#define MY_RF24_PA_LEVEL RF24_PA_LOW
//#define MY_RF24_PA_LEVEL RF24_PA_MIN

//#define MY_DEBUG // Enable debug prints
//#define PRE_CONFIG // Enable load sensor number & scale_constant to EEPROM


#include <MySensors.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>




#define NUM_READS 5				//Размер буфера измерений напряжения
#define BATTERY_SENSE_PIN A6          //Battary Voltage pin 
#define HUM_SENSOR_ANALOG_PIN A7      //Humidity Analog pin
#define ONE_WIRE_BUS 3                //DS18B20
#define PIN_HUM_VCC  8                //VCC for analog humidity sensor pin 
#define HUMIDITY_SENSOR_DIGITAL_PIN 4 //DHT22 Data pin
#define PIN_DHT_VCC 5                 //VCC for DHT22 sensor pin 
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define CHILD_ID_TEMP1 0
#define CHILD_ID_HUM1 1
#define CHILD_ID_TEMP2 10
#define CHILD_ID_HUM2 11
#define CHILD_ID_VOLTS 5

long scale_constant = 1125300L;
float buffer[NUM_READS];				//Буфер измерений напряжения
int index=0;						//Счетчик буфера измерений напряжения
float Volts;
int sensorValue;
float batteryV;
int8_t myNodeId;
//unsigned long SLEEP_TIME = 300000; // Sleep time between reads (in milliseconds)
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)
int newsleep;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
DHT dht(HUMIDITY_SENSOR_DIGITAL_PIN, DHT22);
boolean receivedConfig = false;
// Initialize temperature & hum messages
MyMessage msgTemp1(CHILD_ID_TEMP1, V_TEMP);
MyMessage msgHum1(CHILD_ID_HUM1, V_HUM);
MyMessage msgTemp2(CHILD_ID_TEMP2, V_TEMP);
MyMessage msgHum2(CHILD_ID_HUM2, V_HUM);
MyMessage msgVolts(CHILD_ID_VOLTS, V_VOLTAGE);
MyMessage msgDiagnostic(254, V_CUSTOM);

float temperature1;
long int humidity1;
float temperature2;
float humidity2;
float temp1cor; //EEPROM - 5
float temp2cor; //EEPROM - 6

int oldBatteryPcnt = 0;
int batteryPcnt;

int mySensorNumber;
char SketchInfo[] = "MyMultiSensor # ";
char Group1Name[] = "Ground # ";
char Group2Name[] = "Air # ";

bool metric = true;
bool sendVolts = true; //EEPROM - 7

void before() {
	#ifdef PRE_CONFIG
	uint8_t multisensorNumber = 5;
	long scale_constant = 1125300L;
	//saveState(0, (byte)(scale_constant & 0xFF));
	//saveState(1, (byte)((scale_constant >> 8) & 0xFF));
	//saveState(2, (byte)((scale_constant >> 16) & 0xFF));
	//saveState(4, multisensorNumber);
	//saveState(5, 128);
	//saveState(6, 128);
	saveState(7, 3);
	Serial.println(loadState(0));
	Serial.println(loadState(1));
	Serial.println(loadState(2));
	Serial.println(loadState(3));
	Serial.println(loadState(4));
	Serial.println(loadState(5));
	Serial.println(loadState(6));
	Serial.println(loadState(7));
	#endif
	
	SLEEP_TIME = (unsigned long)(loadState(7)) * 10000;

	mySensorNumber = loadState(4);

	myNodeId = mySensorNumber+10;
	mySensorNumber = mySensorNumber+48; // int->char
	transportAssignNodeID(myNodeId);

	SketchInfo[15] = mySensorNumber;
	Group1Name[8] = mySensorNumber;
	Group2Name[5] = mySensorNumber;
}

void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo(SketchInfo, "1.3");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_TEMP1, S_TEMP, Group1Name);
  present(CHILD_ID_HUM1, S_HUM, Group1Name);
  present(CHILD_ID_TEMP2, S_TEMP, Group2Name);
  present(CHILD_ID_HUM2, S_HUM, Group2Name);
  present(CHILD_ID_VOLTS, S_MULTIMETER,"Battery");
  present(254, S_CUSTOM, "Diagnostic");
  metric = getControllerConfig().isMetric;
}

void setup(){ 
  pinMode(PIN_HUM_VCC,  OUTPUT); //питание Hum
  pinMode(PIN_DHT_VCC,  OUTPUT); //питание DHT
  
  digitalWrite(PIN_DHT_VCC,HIGH);

  temp1cor = (float)(loadState(5)-128)/10;
  temp2cor = (float)(loadState(6)-128)/10;
  scale_constant = ((long)(loadState(2) * 256L)+ loadState(1))*256 + loadState(0);
  sendVolts = (loadState(8)) ? true : false;

  send(msgTemp1.set(1, 1));
  send(msgHum1.set(2, 1));
  send(msgTemp2.set(3, 1));
  send(msgHum2.set(4, 1));

  //send(msgDiagnostic.set(temp1cor,2));
  //send(msgDiagnostic.set(temp2cor, 2));
  //send(msgDiagnostic.set((uint32_t)SLEEP_TIME));


  #ifdef MY_DEBUG
	Serial.println("---------------------");
	Serial.print("myNodeId: ");
	Serial.println(myNodeId);
    Serial.print("Temp correction: ");
    Serial.print(temp1cor);
	Serial.print("/");
	Serial.println(temp2cor);
	Serial.print("Scale_constant: ");
	Serial.println(scale_constant);
	Serial.print("metric: ");
	Serial.println(metric);
	Serial.print("SLEEP_TIME: ");
	Serial.println(SLEEP_TIME);
	Serial.print("sendVolts: ");
	Serial.println(sendVolts);
	Serial.println("---------------------");
  #endif
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  // Initialize DHT library
  dht.begin();
  
  for (int i = 0; i < 5; i++) {
	  digitalWrite(PIN_HUM_VCC, LOW);
	  wait(1000);
	  digitalWrite(PIN_HUM_VCC, HIGH);
	  wait(1000);
  }
  wait(10000);
  for (int i = 0; i < 5; i++) {
	  digitalWrite(PIN_HUM_VCC, LOW);
	  wait(1000);
	  digitalWrite(PIN_HUM_VCC, HIGH);
	  wait(1000);
  }

}


void loop()     
{
  digitalWrite(PIN_DHT_VCC,HIGH);
  
//Analog Humidity read
  digitalWrite(PIN_HUM_VCC,HIGH);
  delay(1000);
  humidity1 = (1023-analogRead(HUM_SENSOR_ANALOG_PIN))/10.23;
  digitalWrite(PIN_HUM_VCC,LOW);

//Dallas temperature read
  sensors.requestTemperatures(); // Fetch temperatures from Dallas sensors
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution()); // query conversion time and sleep until conversion completed
  sleep(conversionTime);  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  temperature1 = (static_cast<float>(static_cast<int>((metric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.) + temp1cor;


//DHT22 read
	temperature2 = dht.readTemperature() + temp2cor;
	humidity2 = dht.readHumidity();
	digitalWrite(PIN_DHT_VCC, LOW); 
  

  send(msgTemp1.set(temperature1,1));
  send(msgHum1.set(humidity1));

  
#ifdef MY_DEBUG
  Serial.println("---------------------");
  Serial.print("Humidity Analog: ");
  Serial.print(humidity1);
  Serial.print("%, TempDallas: ");
  Serial.println(temperature1);
  Serial.print("Humidity DHT: ");
  Serial.print(humidity2);
  Serial.print("%, Temp DHT: ");
  Serial.println(temperature2);
  Serial.println("---------------------");
#endif
  
  //sleep(SLEEP_TIME, true);
  sleep(SLEEP_TIME);
  Volts = readVcc();
  fakeRead(BATTERY_SENSE_PIN);
  sensorValue = analogRead(BATTERY_SENSE_PIN);
  addReading(float(sensorValue) * Volts / 1023000);

  batteryV = average();
  if (sendVolts) {
	send(msgVolts.set(batteryV,2));
  }
  batteryV = min(batteryV, 3);
  batteryPcnt= (batteryV-0.8)*100/2.2; //0,8V - мин напряжение,3,0V - макс напряжение, 2,2V -дельта
  
  if (oldBatteryPcnt != batteryPcnt) {
	sendBatteryLevel(batteryPcnt);
	oldBatteryPcnt = batteryPcnt;
  }

  send(msgTemp2.set(temperature2, 1));
  send(msgHum2.set(humidity2, 1));
  

#ifdef MY_DEBUG
  Serial.println("---------------------");
  Serial.print("SensorValue: ");
  Serial.println(sensorValue);
  Serial.print("Volts: ");
  Serial.println(Volts);
  Serial.print("scale_constant: ");
  Serial.println(scale_constant);
  Serial.print(", ");
  Serial.print(batteryPcnt);
  Serial.println("%");
  Serial.println("---------------------");
#endif

  //sleep(SLEEP_TIME,true);
  sleep(SLEEP_TIME);
}

void receive(const MyMessage &message) {
	// V_VAR1 - Voltage, V_VAR2 - temp1 (ground), V_VAR3 - temp2 (air), V_VAR4 *10000 - SLEEP_TIME, mls , V_VAR5 - sendVolts
	//Serial.println("++++++++++++++++++++++++++++++");

	float myCor = message.getFloat();
	//if (message.sensor == 254) {
		switch (message.type) {
		case V_VAR1: //Voltage
			scale_constant = (long)(scale_constant*myCor);
			saveState(0, (byte)(scale_constant & 0xFF));
			saveState(1, (byte)((scale_constant >> 8) & 0xFF));
			saveState(2, (byte)((scale_constant >> 16) & 0xFF));
			send(msgDiagnostic.set(scale_constant));
			//Serial.print("Correct voltage ");
			//Serial.println(scale_constant);
			break;
		case V_VAR2: //temp1 
			temp1cor=temp1cor + myCor;
			saveState(5, (int)(temp1cor * 10 - 128));
			send(msgDiagnostic.set(temp1cor,2));
			//Serial.print("Correct temp1 for ");
			//Serial.println(temp1cor);
			break;
		case V_VAR3: //temp2
			temp2cor=temp2cor + myCor;
			saveState(6, (int)(temp2cor * 10 - 128));
			send(msgDiagnostic.set(temp2cor, 2));
			//Serial.print("Correct temp2 for ");
			//Serial.println(temp2cor);
			break;
		case V_VAR4: //V_VAR3 * 10000 - SLEEP_TIME, mls
			newsleep = message.getInt();
			SLEEP_TIME = (unsigned long)newsleep * 1000;
			saveState(7, (int)(newsleep/10));
			send(msgDiagnostic.set((uint32_t)SLEEP_TIME));
			//Serial.print("SLEEP_TIME ");
			//Serial.println(SLEEP_TIME);
			break;
		case V_VAR5: //sendVolts
			sendVolts = message.getBool();
			saveState(8, (int)(sendVolts));
			send(msgDiagnostic.set((int)(sendVolts)));
			Serial.print("sendVolts ");
			Serial.println(sendVolts);
			break;
		default:
			break;
		}
	//}
#ifdef MY_DEBUG
	Serial.println("++++++++++++++++++++++++++++++");
	Serial.print("message.type ");
	Serial.println(message.type);
	Serial.print("message.sensor ");
	Serial.println(message.sensor);

	Serial.print("message.getFloat() ");
	Serial.println(message.getFloat());
	Serial.println("++++++++++++++++++++++++++++++");
#endif


}

void addReading(float batteryV)
{
	buffer[index] = batteryV;
	index++;
	if (index >= NUM_READS) {
		index = 0;
	}
}


void fakeRead(int pin) {
	for (int counter = 0; counter < 2; counter++) {
		analogRead(pin);
		wait(500);
	}
}

float average() {
	float sum = 0;
	for (int i = 0; i < NUM_READS; i++) {
		sum += buffer[i];
	}
	return (float)(sum / NUM_READS);
}


long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = scale_constant / result; // Calculate Vcc (in mV); 
  return result; // Vcc in millivolts
}








