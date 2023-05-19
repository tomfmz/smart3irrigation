#include <Arduino.h>
#include "DHT.h"                
#define DHTPIN 2          // DHT Pin
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

unsigned long int time_old = 0;
int t_dht = 2000; //millis
bool debug = false; // set to true for debugging messages

void setup() {
  Serial.begin(9600);
  Serial.println("DHT22 Init");
  dht.begin();
}

void loop() {
  if ((millis()-time_old)>t_dht){
    float h = dht.readHumidity();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h
    float t = dht.readTemperature(); // Lesen der Temperatur in °C und speichern in die Variable t
    
    /*********************( Überprüfen ob alles richtig Ausgelesen wurde )*********************/ 
    if (isnan(h) || isnan(t)) {       
      Serial.println("Fehler beim auslesen des Sensors!");
      return;
    }
    if (debug){
      Serial.print("Luftfeuchtigkeit:");
      Serial.print(h);                  // Ausgeben der Luftfeuchtigkeit
      Serial.print("%\t");              // Tabulator
      Serial.print("Temperatur: ");
      Serial.print(t);                  // Ausgeben der Temperatur
      Serial.write("°");                // Schreiben des ° Zeichen
      Serial.println("C");
    }
    time_old = millis();
  }
}