#include <Arduino.h>
#include "DHT.h"   
#include <SoftwareSerial.h>
#include <DS1603L.h>
#include <TinyGPSPlus.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>      

#define DEBUG 1
#define DHTPIN 27          // DHT Pin
#define DHTTYPE DHT22
#define NANO_SWSERIAL_TX 26
#define NANO_SWSERIAL_RX 35
//#define DS1603L_TX 23
//#define DS1603L_RX 22
#define GPS_TX 14
#define GPS_RX 12
#define MOSFET_PUMPE 33
#define MOSFET_NANO_SMT_WATERMARK 4
#define MOSFET_GPS 13
#define MOSFET_DS1603 25
#define FLOW 39
#define FLOW_DHT_ON_OFF 32
#define SWSERIAL_BAUD 9600
#define HWSERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
#define LORA_NSS 22
#define LORA_RST 21
#define LORA_DIO0 5
#define LORA_DIO1 2
#define LORA_DIO2 15
#define WATERMARKPIN 34
#define TIME_TO_DEEPSLEEP 10000000        // Time ESP32 will go to sleep (in µseconds) 3600000000 µS = 1h for debug 10s (10000000)
RTC_DATA_ATTR int bootCount = 0;          // save bootCounter in non volatile memory
RTC_DATA_ATTR int dailyWaterOutput = 0;   // save dailyWaterOutput in non volatile memory

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 2000

DHT dht(DHTPIN, DHTTYPE);

EspSoftwareSerial::UART smtSerial;

//EspSoftwareSerial::UART ds1603LSerial;
//DS1603L ds1603(ds1603LSerial);
DS1603L ds1603(Serial2);

TinyGPSPlus gps;

const double mls_per_count = 3.0382;
volatile unsigned long flow_counter = 0;
unsigned long old_counter = 0;

void flow_handler(void);
void readFlow(void);
bool readDHT22(void);
void readSMT100(void);
void readDS1603L(void);
void readGPS(void);
void readWatermark(void);

//--------------------LoRaWAN-------------------
void do_send(osjob_t* j);
void onEvent (ev_t ev);

static uint8_t lora_data[10];
static osjob_t sendjob;

static const u1_t PROGMEM APPEUI[8]={0x00, 0x12, 0x25, 0xFF, 0xFF, 0x41, 0x40, 0xA8}; //a84041ffff251200 Gateway id
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0xCF, 0xF5, 0x81, 0x20, 0xE1, 0x37, 0x45, 0x96 }; //964537e12081f5cf Devui vom ESP
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
 
static const u1_t PROGMEM APPKEY[16] = { 0x70, 0xBE, 0x53, 0x6E, 0x52, 0xC3, 0xBE, 0x45, 0x0C, 0x0F, 0x88, 0xF5, 0x12, 0x0F, 0x50, 0x8B}; 
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};
//---------------------------------------------
struct smt100
{
   float permittivity;
   float volwater;
   float temp;
   float voltage;
};
smt100 smt100_;
struct dht22
{
   float temp;
   float humidity;
};
dht22 dht22_;
struct ds1603L
{
   uint16_t waterlvl;
};
ds1603L ds1603L_;

struct flowsens
{
   uint16_t waterflow;
};
flowsens flowsens_;

struct watermark
{
   uint16_t adc;
   uint16_t soilwatertension;
};
watermark watermark_;

void setup() {

  //-----------------------
  //------PIN inits-------
  //-----------------------
  //assing MOSFET gate pins
  pinMode(MOSFET_GPS, OUTPUT);
  pinMode(MOSFET_NANO_SMT_WATERMARK, OUTPUT);
  pinMode(MOSFET_PUMPE, OUTPUT);
  pinMode(MOSFET_DS1603, OUTPUT);
  pinMode(FLOW, INPUT);
  //Die Funktion flow_handler() als Interrupthandler für steigende Flanken des Durchflusssensors festlegen
  attachInterrupt(digitalPinToInterrupt(FLOW), flow_handler, FALLING);

  pinMode(DHTPIN, INPUT_PULLUP); //needs to be a pullup to readout the sensor
  pinMode(FLOW, INPUT);
  pinMode(FLOW_DHT_ON_OFF, OUTPUT);

  //write sensor on/off
  digitalWrite(MOSFET_GPS, LOW);
  // if (bootCount%10==0) digitalWrite(MOSFET_GPS, HIGH);
  digitalWrite(MOSFET_NANO_SMT_WATERMARK, HIGH);
  delay(15000);
  digitalWrite(MOSFET_PUMPE, LOW);  
  digitalWrite(MOSFET_DS1603, HIGH);
  digitalWrite(FLOW_DHT_ON_OFF, HIGH);
  
  //-----------------------
  //-----Serial inits------
  //-----------------------
  // Hardwareserials
  Serial.begin(HWSERIAL_BAUD);
  Serial1.begin(9600, SERIAL_8N1, 12, 14); // funktioniert
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // funktioniert

  //Softwareserial
  smtSerial.begin(SWSERIAL_BAUD, SWSERIAL_8N1, NANO_SWSERIAL_RX,NANO_SWSERIAL_TX, false);
  if (!smtSerial) { // If the object did not initialize, then its configuration is invalid
    if(DEBUG)Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
  /*ds1603LSerial.begin(SWSERIAL_BAUD, SWSERIAL_8N1, DS1603L_RX, DS1603L_TX, false);
  if (!ds1603LSerial) { // If the object did not initialize, then its configuration is invalid
    if(DEBUG)Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }*/

  //------------------------
  //----DeepSleep Setup-----
  //------------------------
  //Increment boot number
  ++bootCount;
  //configure the wake up source to timer, set ESP32 to wake up every #TIME_TO_DEEPSLEEP in µs
  esp_sleep_enable_timer_wakeup(TIME_TO_DEEPSLEEP);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  if(DEBUG){
    Serial.println("Boot number: " + String(bootCount));
    Serial.println("Setup ESP32 to sleep for " + String(TIME_TO_DEEPSLEEP/1000000) + " Seconds");
  }

  //--------------------------
  //------LoRaWAN setup-------
  //--------------------------
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC specific parameters
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  delay(500);  // allow things to settle
  
  //-------------------------
  //------Read Sensors-------
  //-------------------------
  //DHT22
  dht.begin();
  delay(100);
  readDHT22();
  lora_data[0] = (uint8_t)dht22_.humidity;
  lora_data[1] = (uint8_t)dht22_.temp;
  //Truebner SMT100
  readSMT100();
  lora_data[2] = (uint8_t)smt100_.volwater;
  lora_data[3] = (uint8_t)(smt100_.voltage*10);
  //Irrometer Watermark
  readWatermark();
  watermark_.adc = analogRead(WATERMARKPIN);
  watermark_.soilwatertension = watermark_.adc/4095;
  lora_data[4] = highByte(watermark_.adc);
  lora_data[5] = lowByte(watermark_.adc);
  //Ultrasonic waterlevel
  digitalWrite(MOSFET_DS1603, HIGH);
  delay(100);
  ds1603.begin();
  delay(100);
  readDS1603L();
  digitalWrite(MOSFET_DS1603, LOW);
  lora_data[6] = highByte(ds1603L_.waterlvl);
  lora_data[7] = lowByte(ds1603L_.waterlvl);

  //--------------WIP---------------
  //------irrigation algorithm-------
  //---------------------------------
  if ((ds1603L_.waterlvl>=50) && ((watermark_.soilwatertension<=2500 && dailyWaterOutput<=20000)||(bootCount%24 == 0))){
    

    // digitalWrite(MOSFET_PUMPE, !digitalRead(MOSFET_PUMPE)); //pump on
    // while ((ds1603L_.waterlvl>=50) && (flowsens_.waterflow<=5000))
    // {
    //   readFlow();
    // }
    // digitalWrite(MOSFET_PUMPE, !digitalRead(MOSFET_PUMPE)); //pump off

    // lora_data[8] = highByte(flowsens_.waterflow);
    // lora_data[9] = lowByte(flowsens_.waterflow);
    // dailyWaterOutput = dailyWaterOutput + flowsens_.waterflow;
    // flowsens_.waterflow = 0;
    // digitalWrite(FLOW_ON_OFF, LOW);
    if (DEBUG)Serial.println("Gießen!!!");
  }

  //once in a day
  if (bootCount%24==0) {
    //ToDo GPS
    // while(Serial1.available() > 0)
    //   gps.encode(Serial1.read());
    // Serial.println();
    // readGPS();

    dailyWaterOutput = 0;
  }

  unsigned long current_counter = flow_counter;

  unsigned long lastprint = millis();
  double flow = 0.0;
  digitalWrite(MOSFET_PUMPE, HIGH);
  while (flow < 0)
  {
    flow = (flow_counter - current_counter) * mls_per_count;
    if (millis() - lastprint >= 1000)
    {
      Serial.println("Flow counter: " + (String) flow_counter);
      Serial.println("Flow: " + (String) flow);
      lastprint = millis();
    }
  }
  digitalWrite(MOSFET_PUMPE, LOW);
  lora_data[8] = flow/1000;

  //-----------------------------------
  //------send LoRaWAN Dataframe-------
  //-----------------------------------
  os_runloop_once();
  do_send(&sendjob);

  //----------------------------
  //------enter DeepSleep-------
  //----------------------------
  //pull down all output pins
  digitalWrite(MOSFET_GPS, LOW);
  delay(5000);
  digitalWrite(MOSFET_NANO_SMT_WATERMARK, LOW);
  digitalWrite(MOSFET_PUMPE, LOW);  
  digitalWrite(MOSFET_DS1603, LOW);
  digitalWrite(FLOW_DHT_ON_OFF, LOW);
  if(DEBUG)Serial.println("Going to sleep now");
  Serial.flush();
  Serial1.flush();
  Serial2.flush();
  //ds1603LSerial.flush();
  smtSerial.flush();
  delay(200);
  esp_deep_sleep_start();
}

void loop() {
}

bool readDHT22(void) {
  bool read = false;
  dht22_.humidity = dht.readHumidity();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h
  dht22_.temp = dht.readTemperature(); // Lesen der Temperatur in °C und speichern in die Variable t
  if (DEBUG){
    Serial.print("Luftfeuchtigkeit:");
    Serial.print(dht22_.humidity);                  // Ausgeben der Luftfeuchtigkeit
    Serial.print("%\t");              // Tabulator
    Serial.print("Temperatur: ");
    Serial.print(dht22_.temp);                  // Ausgeben der Temperatur
    Serial.write("°");                // Schreiben des ° Zeichen
    Serial.println("C");
  }
  /*********************( Überprüfen ob alles richtig Ausgelesen wurde )*********************/ 
  if (isnan(dht22_.humidity) || isnan(dht22_.temp)) {       
    Serial.println("Fehler beim auslesen des Sensors!");
    read = false;
  }else read = true;
  
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
  flowsens_.waterflow = flow_counter * mls_per_count;
  Serial.println((String) (flowsens_.waterflow) + " ml");
}

void readDS1603L(void) {
  Serial.println(F("Starting reading."));
  ds1603L_.waterlvl = ds1603.readSensor();       // Call this as often or as little as you want - the sensor transmits every 1-2 seconds.
  byte sensorStatus = ds1603.getStatus();           // Check the status of the sensor (not detected; checksum failed; reading success).
  switch (sensorStatus) {                           // For possible values see DS1603L.h
    case DS1603L_NO_SENSOR_DETECTED:                // No sensor detected: no valid transmission received for >10 seconds.
      Serial.println(F("No sensor detected (yet). If no sensor after 1 second, check whether your connections are good."));
      break;

    case DS1603L_READING_CHECKSUM_FAIL:             // Checksum of the latest transmission failed.
      Serial.print(F("Data received; checksum failed. Latest level reading: "));
      Serial.print(ds1603L_.waterlvl);
      Serial.println(F(" mm."));
      break;

    case DS1603L_READING_SUCCESS:                   // Latest reading was valid and received successfully.
      Serial.print(F("Reading success. Water level: "));
      Serial.print(ds1603L_.waterlvl);
      Serial.println(F(" mm."));
      break;
  }
}

void readSMT100(void){
  String SensorInfo = "<?M!>";
  String Measure = "<?D0!>";
  char c = ' ';
  const int BUFFER_SIZE = 50;
  char buf[BUFFER_SIZE];
  int j = 0;
  bool marker = false;
  String stm100_calibrated_permittivity_s;
  String stm100temp_s;
  String stm100moisture_s;
  String stm100voltage_s;
  smtSerial.print(SensorInfo);
  delay(300);                    // wait a while for a response
  Serial.print("reading smt");
  while (smtSerial.available()) {  // write the response to the screen
    c = smtSerial.read();
    Serial.print(".");
  }
  Serial.println();
  delay(2000);  // print again in three seconds
  smtSerial.print(Measure);
  delay(300);                    // wait a while for a response
  while (smtSerial.available()) {  // write the response to the screen
    // read the incoming bytes:
    int rlen = smtSerial.readBytes(buf, BUFFER_SIZE);
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

void readGPS(){
  String gpsLocation = "";
  if (gps.date.isValid() && (gps.date.age() <= 1500)){
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
  }else
    gpsLocation += "NO_VALID_TIMESTAMP,";
  if (gps.location.isValid()){
    gpsLocation += String (gps.speed.kmph());
    gpsLocation += (",");
    gpsLocation += String (gps.location.lat(), 6);
    gpsLocation += (",");
    gpsLocation += String (gps.location.lng(), 6);
    gpsLocation += (",");
  }else{
    gpsLocation += "NO_VALID_SPEED,NO_VALID_LAT,NOV_VALID_LONG,";
  }
  if (gps.course.isValid())
    gpsLocation += String (gps.course.deg());
  else
    gpsLocation += "NO_VALID_COURSE";
  Serial.println(gpsLocation);
}

void readWatermark() {
  int raw = analogRead(WATERMARKPIN);
  float kPa = (raw * 3.3 * 239) / (4095 * 2.8);
  Serial.println ("Volt: " + (String) (raw * 3.3/ 4095));
  Serial.println("Bodenwasserspannung: " + (String) kPa + " kPa");
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
 
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
 
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lora_data, sizeof(lora_data)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}