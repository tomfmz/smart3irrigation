#include <Arduino.h>

/**
 * @file d_simple_logger.ino
 * @copyright (c) 2013-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 * @author Kevin M.Smith <SDI12@ethosengineering.org>
 * @date August 2013
 *
 * @brief Example D: Check all Addresses for Active Sensors and Log Data
 *
 * This is a simple demonstration of the SDI-12 library for Arduino.
 *
 * It discovers the address of all sensors active on a single bus and takes continuous
 * measurements from them.
 */

#include <SDI12.h>

#define SERIAL_BAUD 9600 /*!< The baud rate for the output serial port */
#define DATA_PIN 12         /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1       /*!< The sensor power pin (or -1 if not switching power) */

/** Define the SDI-12 bus */
SDI12 mySDI12(DATA_PIN);

/**
  '?' is a wildcard character which asks any and all sensors to respond
  'I' indicates that the command wants information about the sensor
  '!' finishes the command
*/
String myCommand = "?M!";
String myCommand2 = "?D0!";

void setup() {
  pinMode(DATA_PIN,INPUT);
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;

  Serial.println("Opening SDI-12 bus...");
  mySDI12.begin();
  delay(500);  // allow things to settle

  // Power the sensors;
  if (POWER_PIN > 0) {
    Serial.println("Powering up sensors...");
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    delay(200);
  }
}

void loop() {
  mySDI12.sendCommand(myCommand);
  Serial.println("SDI-12 Command sent...");
  delay(300);                    // wait a while for a response
  while (mySDI12.available()) {  // write the response to the screen
    Serial.write(mySDI12.read());
    // Serial.println("SDI-12 Command received...");
  }
  delay(3000);
  mySDI12.sendCommand(myCommand2);
  Serial.println("SDI-12 Command2 sent...");
  delay(300);                    // wait a while for a response
  while (mySDI12.available()) {  // write the response to the screen
    Serial.write(mySDI12.read());
    // Serial.println("SDI-12 Command received...");
  }
  delay(5000);  // print again in three seconds
}