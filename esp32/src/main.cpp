// Arduino-Bibliothek importieren
#include <Arduino.h>

// Headerdatei mit Definitionen importieren
#include "treeirrigation.h"

// DHT-Objektinstanz für den Temperatur-Luftfeuchtigkeitssensor erstellen
DHT dht(DHTPIN, DHTTYPE);

// Softwareserial-Objektinstanz für die UART-Verbindung zum SMT100-Bodenfeuchtigkeitssensors erstellen
EspSoftwareSerial::UART smtSerial;

// DS1603L-Objektinstanz für den Ultraschall-Tankfüllstandssensor erstellen
DS1603L ds1603(Serial2);

// TinyGPSPlus-Objektinstanz für das GPS-Modul erstellen
TinyGPSPlus gps;

// Zählervariable für die Umdrehungen des Durchflusssensor-Schaufelrades definieren
volatile unsigned long flow_counter = 0;

//Zeit die die Bewässerung in annspruch nimmt
unsigned long irrigation_time = 0;

// Deklaration der Interrupthandler-Funktion für das Durchflusssensorsignal
void flow_handler(void);

// Deklaration der Funktion für das Auslesen des Durchflusssensors
double readFlow(void);

// Deklaration der Funktion für das Auslesen des Temperatur-Luftfeuchtigkeitssensors
void readDHT22(void);

// Deklaration der Funktion für das Auslesen des volumetrischen Bodenfeuchtigkeitssensors
void readSMT100(void);

// Deklaration der Funktion für das Auslesen des Ultraschall-Tankfüllstandssensors
void readDS1603L(void);

// Deklaration der Funktion für das Auslesen des GPS-Moduls
void readGPS(void);

// Deklaration der Funktion für das Auslesen des Bodenwasserspannungssensors
void readWatermark(void);

// Setup-Funktion
void setup() {
  // Alle 24 Stunden wird die tägliche Bewässerungsmenge sowie der Bootvorgangszähler auf 0 zurückgesetzt 
  if ((bootCount * TIME_TO_DEEPSLEEP) >= 86400) {
    dailyWaterOutput = 0;
    bootCount = 0;
  }

  // MOSFET-Gate-Pins als Output-Pins konfigurieren
  pinMode(MOSFET_GPS, OUTPUT);
  pinMode(MOSFET_NANO_SMT_WATERMARK, OUTPUT);
  pinMode(MOSFET_PUMPE, OUTPUT);
  pinMode(MOSFET_DS1603, OUTPUT);

  // Durchflusssensor-Signalanschlusspin als Input Pin konfigurieren
  pinMode(FLOW, INPUT);

  // ADC Pin für Voltmeter als Input Pin konfigurieren
  pinMode(WATERMARKPIN, INPUT);

  // Die Funktion flow_handler() als Interrupthandler für steigende Flanken des Durchflusssensors festlegen
  attachInterrupt(digitalPinToInterrupt(FLOW), flow_handler, FALLING);
  
  // Stromversorgungspin für Durchfluss- und Temperatur-Feuchtigkeitssensor als Output-Pin konfigurieren
  pinMode(FLOW_DHT_ON_OFF, OUTPUT);

  // Hardwareserials konfigurieren
  Serial.begin(HWSERIAL_BAUD);
  Serial1.begin(SERIAL_BAUD, SERIAL_8N1, 12, 14); 
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, 16, 17); 

  //Softwareserial für SMT100-Bodenfeuchtigkeitssensor konfigurieren
  smtSerial.begin(SERIAL_BAUD, SWSERIAL_8N1, NANO_SWSERIAL_RX,NANO_SWSERIAL_TX, false);
  if (!smtSerial) { // If the object did not initialize, then its configuration is invalid
    if(DEBUG)Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }

  // Zähler für Bootvorgänge um 1 erhöhen
  ++bootCount;

  // LoRaWAN-Setup
  os_init();
  LMIC_reset();
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    
  //DHT22 auslesen
  digitalWrite(FLOW_DHT_ON_OFF, HIGH);
  dht.begin();
  delay(100);
  readDHT22();
  digitalWrite(FLOW_DHT_ON_OFF, LOW);
  lora_data[0] = (uint8_t)dht22_.humidity;
  int16_t temp_float = dht22_.temp*100;
  lora_data[1] = highByte(temp_float);
  lora_data[2] = lowByte(temp_float);
  
  // Stromversorgung von Arduino und Bodenfeuchtigkeitssensoren einschalten
  digitalWrite(MOSFET_NANO_SMT_WATERMARK, HIGH);  
  
  //Truebner SMT100-Bodenfeuchtigkeitssensor auslesen
  readSMT100();
  uint16_t volwater_float = smt100_.volwater*100;
  lora_data[3] = highByte(volwater_float);
  lora_data[4] = lowByte(volwater_float);
  lora_data[5] = (uint8_t)(smt100_.voltage*10);
  temp_float = smt100_.temp*100;
  lora_data[6] = highByte(temp_float);
  lora_data[7] = lowByte(temp_float);
  
  //Irrometer Watermark Bodenwasserspannungssensor auslesen
  readWatermark();
  digitalWrite(MOSFET_NANO_SMT_WATERMARK, LOW);
  lora_data[8] = highByte(watermark_.soilwatertension);
  lora_data[9] = lowByte(watermark_.soilwatertension);
  
  // Ultraschall-Tankfüllstandssensor auslesen
  digitalWrite(MOSFET_DS1603, HIGH);
  delay(100);
  ds1603.begin();
  delay(100);
  readDS1603L();
  digitalWrite(MOSFET_DS1603, LOW);
  if (DEBUG) Serial.println("Tankinhalt: " + String(ds1603L_.tank_content) + " L - " + String(ds1603L_.tank_content_percentage) + " %");
  uint16_t tank_content_int = ds1603L_.tank_content*100;
  lora_data[10] = highByte(tank_content_int);
  lora_data[11] = lowByte(tank_content_int);
  lora_data[12] = ds1603L_.tank_content_percentage;

  // Deklaration der Variable für die auszubringende Wassermenge
  int irrigation = 0; 

  // Die Ausbringung der festgelegten Wassermenge soll genau dann erfolgen, wenn der Tankfüllstand höher als die Minimal messbaren 50 mm ist, die maximale
  // tägliche Bewässserungsmenge noch nicht erreicht wurde und der prozentuale volumetrische Wassergehalt der Erde unter dem festgelegten Grenzwert liegt
  if ((ds1603L_.waterlvl >= 52) && (dailyWaterOutput <= max_daily_irrigation) && ((smt100_.volwater < irrigation_threshold_vol)) ){
    irrigation = irrigation_volume * 1000;
    if (DEBUG)Serial.println("Gießen!!!");
  }

  // Timestamp für die letzte Debug-Ausgabe der Bewässerungsmenge
  unsigned long lastprint = millis();
  unsigned long lastflowcheck = millis();

  // Stromversorgung des Durchflusssensors aktivieren
  digitalWrite(FLOW_DHT_ON_OFF, HIGH);

  // Pumpe einschalten
  digitalWrite(MOSFET_PUMPE, HIGH);

  // Variable für die ausgebrachte Wassermenge deklarieren
  double flow = 0.0;

  int flow_old = 0;
  irrigation_time = millis();
  bool irrigationtimeout = false;
  // Schleife läuft, solange die ausgebrachte Wassermenge kleiner als die Zielmenge ist
  while ((flow < irrigation) && !irrigationtimeout)
  {
    // Bisher ausgebrachte Wassermenge auslesen
    flow = readFlow();

    // Alle 1000 ms Debug-Message ausgeben, falls DEBUG = 1
    if ((millis() - lastprint >= 1000) && DEBUG){
      Serial.println("Flow counter: " + (String) flow_counter);
      Serial.println("Flow: " + (String) flow);
      lastprint = millis();
    }

    if ((millis() - lastflowcheck >= 5000)){
      if ((flow-flow_old)<=20){
        if (DEBUG) Serial.println("Gießtimeout. Gießvorgang wird abgebrochen");
        irrigationtimeout = true;
      }
      lastflowcheck = millis();
      flow_old = flow;
    }    
  }
  irrigation_time = millis()-irrigation_time;
  // Pumpe ausschalten 
  digitalWrite(MOSFET_PUMPE, LOW);

  // Stromversorgung des Durchflusssensors deaktivieren
  digitalWrite(FLOW_DHT_ON_OFF, LOW);
  
  // Ausgebrachte Wassermenge zum Zähler für die tägliche Bewässerungsmenge hinzuaddieren
  dailyWaterOutput = dailyWaterOutput + flow;
  
  // Im aktuellen Bewässerungsgang ausgebrachte Wassermenge in den LoRa-Buffer schreiben
  uint16_t irrigation_volume_int = flowsens_.waterflow/10;
  lora_data[13] = highByte(irrigation_volume_int);
  lora_data[14] = lowByte(irrigation_volume_int);

  // Inhalt des LoRa-Buffers übertragen
  do_send(&sendjob);
  delay(200);
}

// Variable, die auf true gesetzt wird, wenn alle Messungs-, Gieß- und Datenübertragungsvorgänge abgeschlossen wurden
bool GOTO_DEEPSLEEP_TIMEOUT = false;
bool GOTO_DEEPSLEEP_TRANS_SUCCESS = false;

void loop() {

  // Überprüfen, ob Daten zur LoRa-Übertragung zur Verfügung stehen und Sendevorgang initiieren, wenn möglich
  os_runloop_once();
  
  // Überprüfen, ob im Zeitraum des nächsten Deepsleeps Daten gesendet werden sollen
  // if(!os_queryTimeCriticalJobs(ms2osticksRound( (TIME_TO_DEEPSLEEP*1000) ))) {
    
  //   // Wenn in dem Zeitraum keine Daten gesendet werden sollen und alle aktuellen Aufgaben abgearbeitet wurden Deepsleep initiieren
  //   GOTO_DEEPSLEEP_TRANS_SUCCESS = true;
  // }

  // Wenn die Übertragung der gemessenen Daten nicht innerhalb des definierten Zeitintervalls erfolgen konnte, von einem
  // Verbindungsverlust ausgehen und ESP wieder in den Deepsleep versetzen
  if (millis() > (LORA_TIMEOUT+irrigation_time)) {
    if (DEBUG)Serial.println("No LoRaWAN connection");
    GOTO_DEEPSLEEP_TIMEOUT = true;
  }

  if(GOTO_DEEPSLEEP_TIMEOUT || GOTO_DEEPSLEEP_TRANS_SUCCESS){
      Serial.flush();
      Serial1.flush();
      Serial2.flush();
      smtSerial.flush();
      if(DEBUG){
        Serial.println("Setup ESP32 to sleep for " + String(TIME_TO_DEEPSLEEP) + " Seconds");
        Serial.println("Going to sleep now");
      }
      
      // LoRaWAN-Modul in den Standby versetzen
      LMIC_shutdown();

      // Deepsleep bis zum nächsten geplanten Aufweckzeitpunkt initiieren
      esp_sleep_enable_timer_wakeup(TIME_TO_DEEPSLEEP * 1000000 - (millis()/1000));
      esp_deep_sleep_start();
    }
}

// Funktion für das Auslesen des Temperatur-Luftfeuchtigkeitssensors
void readDHT22(void) {
  float humidity = NAN;
  float temperature = NAN;
  if (DEBUG)
    Serial.print("Reading DHT");
  while (isnan(humidity) || isnan( temperature)) {
    delay(3000);
    humidity = dht.readHumidity(); // Lesen der Luftfeuchtigkeit und speichern in die Variable h
    temperature = dht.readTemperature(); // Lesen der Temperatur in °C und speichern in die Variable t
    if (DEBUG)
      Serial.print(".");
  }
    
  dht22_.humidity = humidity;   
  dht22_.temp = temperature; 
  if (DEBUG){
    Serial.print("Luftfeuchtigkeit:");
    Serial.print(dht22_.humidity);                  // Ausgeben der Luftfeuchtigkeit
    Serial.print("%\t");              // Tabulator
    Serial.print("Temperatur: ");
    Serial.print(dht22_.temp);                  // Ausgeben der Temperatur
    Serial.write("°");                // Schreiben des ° Zeichen
    Serial.println("C");
  }
  // Überprüfen ob alles richtig ausgelesen wurde
  if (isnan((dht22_.humidity) || isnan(dht22_.temp)) && DEBUG)      
    Serial.println("Fehler beim auslesen des Sensors!");
}

// Funktion, die den Zähler des Durchflussensors um 1 erhöht, wenn der Hall-Sensor des Schaufelrades eine Umdrehung registriert
void flow_handler(void) {
  flow_counter++;
}

// Funktion, die den aktuellen Stand des Durchflusszählers ausliest und im Struct des Sensors speichert und zurückgibt
double readFlow(void) {
  flowsens_.waterflow = flow_counter * mls_per_count;
  return flowsens_.waterflow;
}

// Funktion, die den Ultraschall-Tankfüllstandssensor ausliest
void readDS1603L(void) {
  Serial.println(F("Starting reading."));
  ds1603L_.waterlvl = ds1603.readSensor();       
  byte sensorStatus = ds1603.getStatus();
  while (sensorStatus == DS1603L_NO_SENSOR_DETECTED) {      
    switch (sensorStatus) {                        
      case DS1603L_NO_SENSOR_DETECTED:             
        Serial.println(F("No sensor detected (yet). If no sensor after 1 second, check whether your connections are good."));
        break;

      case DS1603L_READING_CHECKSUM_FAIL:            
        Serial.print(F("Data received; checksum failed. Latest level reading: "));
        Serial.print(ds1603L_.waterlvl);
        Serial.println(F(" mm."));
        break;

      case DS1603L_READING_SUCCESS:
        Serial.print(F("Reading success. Water level: "));
        Serial.print(ds1603L_.waterlvl);
        Serial.println(F(" mm."));
        break;
    }
    delay(2000);
    ds1603L_.waterlvl = ds1603.readSensor();
    sensorStatus = ds1603.getStatus();
  } 
  // Berechnung des Tankfüllstands in Litern (abzüglich der 8 mm Wandstärke des Tankbodens)
  ds1603L_.tank_content = tank_length * tank_width * (ds1603L_.waterlvl - 8) / 1000000.0;

  // Berechung des Prozentualen Tankfüllstands
  ds1603L_.tank_content_percentage = round((ds1603L_.tank_content/tank_volume)*100);
}

// Funktion für das Auslesen des Sensors für die volumetrische Bodenfeuchtigkeit
void readSMT100(void){
  String start_measurement = "<?M!>";
  String request_measurements = "<?D0!>";
  char c = ' ';
  const int BUFFER_SIZE = 50;
  char buf[BUFFER_SIZE];
  int j = 0;
  bool marker = false;
  String stm100_calibrated_permittivity_s;
  String stm100temp_s;
  String stm100moisture_s;
  String stm100voltage_s;
  
  // Neue Messung auslösen
  smtSerial.print(start_measurement);
  
  //delay(300);###################################################################################                    
  
  if(DEBUG)
    Serial.print("reading smt");
  
  while (smtSerial.available()) {  
    c = smtSerial.read();
    
    if(DEBUG) 
      Serial.print(".");
  }
  if (DEBUG)
    Serial.println();
  
  // Auf Fertigstellung der aktuellen Messung warten
  delay(2000);
  
  // Messergebnisse abrufen
  smtSerial.print(request_measurements);

  // delay(300); ########################################################################################
  
  // Dekodieren der empfangenen Antwort
  while (smtSerial.available()) {  
    int rlen = smtSerial.readBytes(buf, BUFFER_SIZE);
    for (int i = 0; i < rlen; i++) {
      if (buf[i] == '+') j++;
      else if (j == 2) stm100_calibrated_permittivity_s += (String)buf[i];
      else if (j == 3) stm100moisture_s += (String)buf[i];
      else if (j == 4) stm100temp_s += (String)buf[i];
      else if (j == 5) stm100voltage_s += (String)buf[i];    
    }
  }

  // Testausgabe
  if (DEBUG){
    Serial.println("-----SMT100 Data-----");
    Serial.print("Calibrated permittivity: ");Serial.println(stm100_calibrated_permittivity_s); 
    Serial.print("Calibrated volumetric water content in percent: ");Serial.println(stm100moisture_s);
    Serial.print("Temperature: ");Serial.println(stm100temp_s);
    Serial.print("Voltage: ");Serial.println(stm100voltage_s);
  }

  // Abspeichern der Messergebnisse
  smt100_.permittivity = stm100_calibrated_permittivity_s.toFloat();
  smt100_.volwater = stm100moisture_s.toFloat();
  smt100_.temp = stm100temp_s.toFloat();
  smt100_.voltage = stm100voltage_s.toFloat();
}


// Funktion zum Auslesen des GPS-Moduls
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

// Funktion zum Auslesen des Bodenwasserspannungssensors
void readWatermark() {
  int raw = analogRead(WATERMARKPIN);
  float kPa = (raw * 3.3 * 239) / (4095 * 2.8);
  if(DEBUG){
    Serial.println("Raw: " + (String) raw);
    Serial.println ("Volt: " + (String) (raw * 3.3/ 4095));
    Serial.println("Bodenwasserspannung: " + (String) kPa + " kPa");
  }
  watermark_.soilwatertension = kPa;
}

// Funktion zum Dekodieren von LoRaWAN-Ereignissen
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
            GOTO_DEEPSLEEP_TRANS_SUCCESS = true;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
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

// Funktion für das Starten von LoRaWAN Sendeaufträgen
void do_send(osjob_t* j){
    // Überprüfen, ob aktuell eine Übertragung läuft
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Wenn keine Übertragung läuft, Daten für die Übertragung zum nächstmöglichen Zeitpunt vorbereiten
        LMIC_setTxData2(1, lora_data, sizeof(lora_data)-1, 0);
        Serial.println(F("Packet queued"));
    }
}