#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DS1603L.h>

#define DEBUG 1
#define SWSERIAL_TX 26
#define SWSERIAL_RX 27
#define DS1603L_TX 23
#define DS1603L_RX 22
#define MOSFET_PUMPE 33 
#define FLOW 39
#define FLOW_ON_OFF 32
#define SWSERIAL_BAUD 9600
#define HWSERIAL_BAUD 115200 /*!< The baud rate for the output serial port */

void flow_handler(void);
void readSMT100(void);
void readFlow(void);
void readDS1603L(void);

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

EspSoftwareSerial::UART sdiSerial;
EspSoftwareSerial::UART ds1603LSerial;

DS1603L ds1603(ds1603LSerial);

void setup() {
  // Hardwareserials
  Serial.begin(HWSERIAL_BAUD);
  Serial1.begin(9600, SERIAL_8N1, 9, 10); // funktioniert
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

  delay(500);  // allow things to settle
  while (!Serial)
    ;
}

void loop() {
  //readSMT100();
  readFlow();
  readDS1603L();
  //digitalWrite(MOSFET_PUMPE, !digitalRead(MOSFET_PUMPE));
  //digitalWrite(FLOW_ON_OFF, HIGH);

  
  delay(2000);
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