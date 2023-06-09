#include <Arduino.h>
#include "DHT.h"   
#include <SoftwareSerial.h>             

#define DEBUG 1
#define DHTPIN 4          // DHT Pin
#define DHTTYPE DHT22
#define SWSERIAL_TX 26
#define SWSERIAL_RX 27
#define SWSERIAL_BAUD 9600
#define HWSERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
unsigned long int time_old = 0;
int t_dht = 1000; //millis

bool readDHT22(void);

void readSMT100(void);
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


void setup() {
  Serial.begin(HWSERIAL_BAUD);
  sdiSerial.begin(SWSERIAL_BAUD, SWSERIAL_8N1, SWSERIAL_RX, SWSERIAL_TX, false);
  if (!sdiSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
  while (!Serial)
    ;

  dht.begin();
  delay(500);  // allow things to settle
}

void loop() {
  readDHT22();
  //readSMT100();
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