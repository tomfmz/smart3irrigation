// Bibliothek für Luftfeuchtigkeitssensor importieren
#include "DHT.h"   

// Bibliothek für Softwareserial importieren
#include <SoftwareSerial.h>

// Bibliothek für Ultraschall-Tankfüllstandssensor importieren
#include <DS1603L.h>

// Bibliothek für GPS-Modul importieren
#include <TinyGPSPlus.h>

// Bibliotheken für LoRaWAN-Modul importieren
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>      

// 1 für Aktivierung von Debug-Messages, 0 für Deaktivierung
#define DEBUG 1

// ESP-Pin, an den der Luftfeuchtigkeitssensor angeschlossen ist
#define DHTPIN 27

// Luftfeuchtigkeitssensortyp
#define DHTTYPE DHT22

// UART-Pins für die Kommunikation mit dem Arduino Nano
#define NANO_SWSERIAL_TX 26
#define NANO_SWSERIAL_RX 35

// UART-Pins für die Kommunikation mit dem Tankfüllstandssensor
#define DS1603L_TX 23
#define DS1603L_RX 22

// UART-Pins für die Kommunikation mit dem GPS-Modul
#define GPS_TX 14
#define GPS_RX 12

// ESP-Pin, an den der MOSFET für die Pumpenaktivierung angeschlossen ist
#define MOSFET_PUMPE 33

// ESP-Pin, an den der MOSFET für den Arduino Nano sowie die Bodenfeuchtigkeitssensoren angeschlossen ist
#define MOSFET_NANO_SMT_WATERMARK 4

// ESP-Pin, an den der MOSFET für das GPS-Modul angeschlossen ist
#define MOSFET_GPS 13

// ESP-Pin, an den der MOSFET für den Tankfüllstandssensor angeschlossen ist
#define MOSFET_DS1603 25

// ESP-Pin für das Signal des Durchflusssensors
#define FLOW 39

// ESP-Pin, über den Durchflussensor und Luftfeuchtigkeitssensor mit Strom versorgt werden
#define FLOW_DHT_ON_OFF 32

// Baudrate für die Serial-Verbindungen mit den Sensoren
#define SERIAL_BAUD 9600

// Baudrate für den Serial-Monitor
#define HWSERIAL_BAUD 115200 

// LoRa-Anschlusspins am ESP
#define LORA_NSS 22
#define LORA_RST 21
#define LORA_DIO0 5
#define LORA_DIO1 2
#define LORA_DIO2 15

// LoRa-Timeoutintervall [ms]
#define LORA_TIMEOUT 60000

// ESP-Pin, an den das Signal des Watermark angeschlossen ist
#define WATERMARKPIN 34

// Intervall, in dem der ESP aus dem Deepsleep aufgeweckt wird und Messungen durchführt [s]
#define TIME_TO_DEEPSLEEP 30

// Umrechnungsfaktor für die ml pro Umdrehung des Durchflussensorschaufelrades definieren (experimentell ermittelt)
const double mls_per_count = 3.0382;

// Wasserausbringsmenge für einen Bewässerungsgang [L]
float irrigation_volume = 0.0;

// Maximale tägliche Bewässerungsmenge [L]
float max_daily_irrigation = 20.0;

// Volumetrischer Grenzwert für die Bewässerung [%]
float irrigation_threshold_vol = 20.0;

// Tankinnenmaße [mm]
const int tank_length = 734;
const int tank_width = 284;

// Tankkapazität [L]
const int tank_volume = 105;

// Minimaler Tankfüllstand, der durch Ultraschallsensor detektierbar ist
const float min_tank_content = (tank_length * tank_width * 42) / 1000000.0;

// Zähler für die Anzahl der Bootvorgänge (nicht flüchtig)
RTC_DATA_ATTR int bootCount = 0;          

// Variable für die tägliche Bewässerungsmenge (nicht flüchtig)
RTC_DATA_ATTR int dailyWaterOutput = 0;   

// Deklaration der Funktion, die die Übertragung des aktuellen LoRaWAN-Buffers auslöst
void do_send(osjob_t* j);

// LoRaWAN Eventdecoder
void onEvent (ev_t ev);

// LoRaWAN-Buffer
static uint8_t lora_data[15];

// Objekt für den aktuellen LoRA-Sendevorgang
static osjob_t sendjob;

// LoRa-Gateway ID (a84041ffff251200)
static const u1_t PROGMEM APPEUI[8]={0x00, 0x12, 0x25, 0xFF, 0xFF, 0x41, 0x40, 0xA8};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DEVUI des ESP (964537e12081f5cf)
static const u1_t PROGMEM DEVEUI[8]={ 0xCF, 0xF5, 0x81, 0x20, 0xE1, 0x37, 0x45, 0x96 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
 
// LoRa-Appkey
static const u1_t PROGMEM APPKEY[16] = { 0x70, 0xBE, 0x53, 0x6E, 0x52, 0xC3, 0xBE, 0x45, 0x0C, 0x0F, 0x88, 0xF5, 0x12, 0x0F, 0x50, 0x8B}; 
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Pin Mapping des LoRa-Moduls
const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

// Struct für die Daten des SMT100-Bodenfeuchtigkeitssensors
struct smt100
{
   float permittivity;
   float volwater;
   float temp;
   float voltage;
};
smt100 smt100_;

// Struct für die Daten des Temperatur-Luftfeuchtigkeitssensors
struct dht22
{
   float temp;
   float humidity;
};
dht22 dht22_;

// Struct für die Daten des Ultraschall-Tankfülsstandssensors
struct ds1603L
{
   uint16_t waterlvl;
   float tank_content;
   uint8_t tank_content_percentage;
};
ds1603L ds1603L_;

// Struct für die Daten des Durchflusssensors
struct flowsens
{
   uint16_t waterflow;
};
flowsens flowsens_;

// Struct für die Daten des Watermark-Bodenwasserspannungssensors
struct watermark
{
   uint16_t adc;
   uint16_t soilwatertension;
};
watermark watermark_;