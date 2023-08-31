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

void recvWithStartEndMarkers(void);
String convertToString(char* a, int size);

/**
  '?' is a wildcard character which asks any and all sensors to respond
  'I' indicates that the command wants information about the sensor
  '!' finishes the command
*/
String myCommand = "?M!";

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

void setup() {
  pinMode(DATA_PIN,INPUT);
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;
  mySDI12.begin();
  delay(500);  // allow things to settle
}

void loop() {
  //wait for the ESP to sent a request
  recvWithStartEndMarkers();
  myCommand = convertToString(receivedChars, numChars);
  

  //handle sent SDI-12 request
  if (newData){
    mySDI12.sendCommand(myCommand);
    delay(300);                    // wait a while for a response
    while (mySDI12.available()) {  // write the response to serial
      Serial.write(mySDI12.read());
    }
    //Serial.write("1+42+1337+42+1337+42");
    newData = false;
  }
}

String convertToString(char* a, int size){
    int i;
    String s = "";
    for (i = 0; i < size; i++){
      if(a[i] == '\0') break;
      s = s + a[i];
    } 
    return s;
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}