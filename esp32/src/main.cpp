#include <Arduino.h>
#include <SoftwareSerial.h>

#define SWSERIAL_TX 26
#define SWSERIAL_RX 27
#define SWSERIAL_BAUD 9600
#define HWSERIAL_BAUD 115200 /*!< The baud rate for the output serial port */

EspSoftwareSerial::UART sdiSerial;

String myCommand = "<?M!>";
String myCommand2 = "<?D0!>";

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
  delay(500);  // allow things to settle
}

void loop() {
  sdiSerial.print(myCommand);
  delay(300);                    // wait a while for a response
  while (sdiSerial.available()) {  // write the response to the screen
    Serial.write(sdiSerial.read());
  }
  delay(3000);  // print again in three seconds
}