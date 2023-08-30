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
#define DS1603L_TX 23
#define DS1603L_RX 22
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
#define TIME_TO_DEEPSLEEP 30        // Time ESP32 will go to sleep [s] 
RTC_DATA_ATTR int bootCount = 0;          // save bootCounter in non volatile memory
RTC_DATA_ATTR int dailyWaterOutput = 0;   // save dailyWaterOutput in non volatile memory

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 2000

//--------------------LoRaWAN-------------------
void do_send(osjob_t* j);
void onEvent (ev_t ev);

static uint8_t lora_data[15];
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

