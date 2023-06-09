#include <Arduino.h>
#include "DHT.h"   
#include <SoftwareSerial.h>
#include <DS1603L.h>
#include <TinyGPSPlus.h>
#define FREQUENCY_868
#include "LoRa_E32.h"       

#define DEBUG 1
#define DHTPIN 4          // DHT Pin
#define DHTTYPE DHT22
#define SWSERIAL_TX 26
#define SWSERIAL_RX 27
#define DS1603L_TX 23
#define DS1603L_RX 22
#define GPS_TX 14
#define GPS_RX 12
#define MOSFET_PUMPE 33 
#define FLOW 39
#define FLOW_ON_OFF 32
#define SWSERIAL_BAUD 9600
#define HWSERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
unsigned long int time_old = 0;
int t_dht = 1000; //millis

bool readDHT22(void);

void flow_handler(void);
void readSMT100(void);
void readFlow(void);
void readDS1603L(void);
void readGPS(void);

const double mls_per_count = 3.0382;
volatile unsigned long flow_counter = 0;
unsigned long old_counter = 0;

struct smt100
{
   float permittivity;
   float volwater;
   float temp;
   float voltage;
};
smt100 smt100_;

DHT dht(DHTPIN, DHTTYPE);

EspSoftwareSerial::UART sdiSerial;
EspSoftwareSerial::UART ds1603LSerial;

DS1603L ds1603(ds1603LSerial);
LoRa_E32 e32ttl100(16,17,&Serial2,-1,-1,-1,UART_BPS_RATE_9600,SERIAL_8N1); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX

TinyGPSPlus gps;
void printLORAParameters(struct Configuration configuration);
void printLORAModuleInformation(struct ModuleInformation moduleInformation);
void initLORA(void);

void setup() {
  // Hardwareserials
  Serial.begin(HWSERIAL_BAUD);
  Serial1.begin(9600, SERIAL_8N1, 12, 14); // funktioniert
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // funktioniert

  sdiSerial.begin(SWSERIAL_BAUD, SWSERIAL_8N1, SWSERIAL_RX, SWSERIAL_TX, false);
  if (!sdiSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }

  ds1603LSerial.begin(SWSERIAL_BAUD, SWSERIAL_8N1, DS1603L_RX, DS1603L_TX, false);
  if (!ds1603LSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }

  pinMode(MOSFET_PUMPE, OUTPUT);
  pinMode(FLOW, INPUT);
  pinMode(FLOW_ON_OFF, OUTPUT);

  digitalWrite(FLOW_ON_OFF, HIGH);
  
  // Die Funktion flowb_handler() als Interrupthandler für steigende Flanken des Durchflusssensors festlegen
  attachInterrupt(digitalPinToInterrupt(FLOW), flow_handler, FALLING);
  
  ds1603.begin();
  
  dht.begin();

  //init of LoRA Modul
	initLORA();

  delay(500);  // allow things to settle
  while (!Serial) // Auf alle Serials warten?
    ;
}

void loop() {
  unsigned long loopend = millis() + 10000;
  readDHT22();
  readSMT100();
  readFlow();
  readDS1603L();
  //digitalWrite(MOSFET_PUMPE, !digitalRead(MOSFET_PUMPE));
  //digitalWrite(FLOW_ON_OFF, HIGH);

  
  while(millis() < loopend) {
    while(Serial1.available() > 0)
      gps.encode(Serial1.read());
  }
  Serial.println();

  readGPS();
}

bool readDHT22(void) {
  bool read = false;
  if ((millis()-time_old)>t_dht){
    float h = dht.readHumidity();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h
    float t = dht.readTemperature(); // Lesen der Temperatur in °C und speichern in die Variable t
    if (DEBUG){
      Serial.print("Luftfeuchtigkeit:");
      Serial.print(h);                  // Ausgeben der Luftfeuchtigkeit
      Serial.print("%\t");              // Tabulator
      Serial.print("Temperatur: ");
      Serial.print(t);                  // Ausgeben der Temperatur
      Serial.write("°");                // Schreiben des ° Zeichen
      Serial.println("C");
    }
    time_old = millis();
    /*********************( Überprüfen ob alles richtig Ausgelesen wurde )*********************/ 
    if (isnan(h) || isnan(t)) {       
      Serial.println("Fehler beim auslesen des Sensors!");
      read = false;
    }else read = true;
  }
  return read;
}

void flow_handler() {
  flow_counter++;
}

void readFlow(void) {
  unsigned long current_counter = flow_counter;
  Serial.println((String) "Flow Counter: " + current_counter);
  Serial.println((String) "Delta: " + (current_counter - old_counter));
  old_counter = current_counter;
  Serial.println((String) (flow_counter * mls_per_count) + " ml");
}

void readDS1603L(void) {
  Serial.println(F("Starting reading."));
  unsigned int reading = ds1603.readSensor();       // Call this as often or as little as you want - the sensor transmits every 1-2 seconds.
  byte sensorStatus = ds1603.getStatus();           // Check the status of the sensor (not detected; checksum failed; reading success).
  switch (sensorStatus) {                           // For possible values see DS1603L.h
    case DS1603L_NO_SENSOR_DETECTED:                // No sensor detected: no valid transmission received for >10 seconds.
      Serial.println(F("No sensor detected (yet). If no sensor after 1 second, check whether your connections are good."));
      break;

    case DS1603L_READING_CHECKSUM_FAIL:             // Checksum of the latest transmission failed.
      Serial.print(F("Data received; checksum failed. Latest level reading: "));
      Serial.print(reading);
      Serial.println(F(" mm."));
      break;

    case DS1603L_READING_SUCCESS:                   // Latest reading was valid and received successfully.
      Serial.print(F("Reading success. Water level: "));
      Serial.print(reading);
      Serial.println(F(" mm."));
      break;
  }
}


void readSMT100(void){
  String sdiSensorInfo = "<?M!>";
  String sdiMeasure = "<?D0!>";
  char c = ' ';
  const int BUFFER_SIZE = 50;
  char buf[BUFFER_SIZE];
  int j = 0;
  bool marker = false;
  String stm100_calibrated_permittivity_s;
  String stm100temp_s;
  String stm100moisture_s;
  String stm100voltage_s;
  sdiSerial.print(sdiSensorInfo);
  delay(300);                    // wait a while for a response
  while (sdiSerial.available()) {  // write the response to the screen
    c = sdiSerial.read();
  }
  delay(2000);  // print again in three seconds
  sdiSerial.print(sdiMeasure);
  delay(300);                    // wait a while for a response
  while (sdiSerial.available()) {  // write the response to the screen
    // read the incoming bytes:
    int rlen = sdiSerial.readBytes(buf, BUFFER_SIZE);
    // prints the received data   
    for (int i = 0; i < rlen; i++) {
      if (buf[i] == '+') j++;
      else if (j == 2) stm100_calibrated_permittivity_s += (String)buf[i];
      else if (j == 3) stm100moisture_s += (String)buf[i];
      else if (j == 4) stm100temp_s += (String)buf[i];
      else if (j == 5) stm100voltage_s += (String)buf[i];    
    }
  }
  if (DEBUG){
    Serial.println("-----SMT100 Data-----");
    Serial.print("Calibrated permittivity: ");Serial.println(stm100_calibrated_permittivity_s); 
    Serial.print("Calibrated volumetric water content in percent: ");Serial.println(stm100moisture_s);
    Serial.print("Temperature: ");Serial.println(stm100temp_s);
    Serial.print("Voltage: ");Serial.println(stm100voltage_s);
  }
  smt100_.permittivity = stm100_calibrated_permittivity_s.toFloat();
  smt100_.volwater = stm100moisture_s.toFloat();
  smt100_.temp = stm100temp_s.toFloat();
  smt100_.voltage = stm100voltage_s.toFloat();
}

void readGPS()
{
  String gpsLocation = "";

  if (gps.date.isValid() && (gps.date.age() <= 1500))
  {
    if (gps.time.hour() < 10)
      gpsLocation += "0";
    gpsLocation += String (gps.time.hour());
    gpsLocation += (":");
    if (gps.time.minute() < 10)
      gpsLocation += "0";
    gpsLocation += String (gps.time.minute());
    gpsLocation += (":");
    if (gps.time.second() < 10)
      gpsLocation += "0";
    gpsLocation += String (gps.time.second());
    gpsLocation += (",");
  }

  else
    gpsLocation += "NO_VALID_TIMESTAMP,";

  if (gps.location.isValid())
  {
    gpsLocation += String (gps.speed.kmph());
    gpsLocation += (",");
    gpsLocation += String (gps.location.lat(), 6);
    gpsLocation += (",");
    gpsLocation += String (gps.location.lng(), 6);
    gpsLocation += (",");
  }

  else
  {
    gpsLocation += "NO_VALID_SPEED,NO_VALID_LAT,NOV_VALID_LONG,";

  }

  if (gps.course.isValid())
    gpsLocation += String (gps.course.deg());
  else
    gpsLocation += "NO_VALID_COURSE";


  Serial.println(gpsLocation);
}

void initLORA(void){
  // Startup all pins and UART
	e32ttl100.begin();

	ResponseStructContainer c;
	c = e32ttl100.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printLORAParameters(configuration);
  configuration.ADDL = 0x0;
	configuration.ADDH = 0x1;
	configuration.CHAN = 0x19;

	configuration.OPTION.fec = FEC_0_OFF;
	configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
	configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
	configuration.OPTION.transmissionPower = POWER_17;
	configuration.OPTION.wirelessWakeupTime = WAKE_UP_1250;

	configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;
	configuration.SPED.uartBaudRate = UART_BPS_115200;
	configuration.SPED.uartParity = MODE_00_8N1;

	// Set configuration changed and set to not hold the configuration
	ResponseStatus rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
	Serial.println(rs.getResponseDescription());
	Serial.println(rs.code);
	printLORAParameters(configuration);

	ResponseStructContainer cMi;
	cMi = e32ttl100.getModuleInformation();
	// It's important get information pointer before all other operation
	ModuleInformation mi = *(ModuleInformation*)cMi.data;

	Serial.println(cMi.status.getResponseDescription());
	Serial.println(cMi.status.code);

	printLORAModuleInformation(mi);

	c.close();
	cMi.close();

  rs = e32ttl100.sendMessage("Hello, world?");
  // Check If there is some problem of succesfully send
  Serial.println(rs.getResponseDescription());
}

void printLORAParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD BIN: "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH BIN: "));  Serial.println(configuration.ADDH, BIN);
	Serial.print(F("AddL BIN: "));  Serial.println(configuration.ADDL, BIN);
	Serial.print(F("Chan BIN: "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit BIN    : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDataRate BIN : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate BIN  : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans BIN       : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup BIN      : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup BIN      : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC BIN         : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower BIN       : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

}
void printLORAModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD BIN: "));  Serial.print(moduleInformation.HEAD, BIN);Serial.print(" ");Serial.print(moduleInformation.HEAD, DEC);Serial.print(" ");Serial.println(moduleInformation.HEAD, HEX);

	Serial.print(F("Freq.: "));  Serial.println(moduleInformation.frequency, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");

}