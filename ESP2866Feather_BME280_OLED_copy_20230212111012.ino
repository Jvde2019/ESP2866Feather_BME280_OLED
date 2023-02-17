/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
//#include <SPI.h>
//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Sensor.h>
//BME
#include <Adafruit_BME280.h>
//FRAM
#include "FRAM.h"


Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
#define BUTTON_A 0
#define BUTTON_B 16
#define BUTTON_C 2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
#elif defined(TEENSYDUINO)
#define BUTTON_A 4
#define BUTTON_B 3
#define BUTTON_C 8
#elif defined(ARDUINO_NRF52832_FEATHER)
#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#else  // 32u4, M0, M4, nrf52840, esp32-s2 and 328p
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#endif

//BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

//FRAM

FRAM fram;

uint32_t start;
uint32_t stop;

uint32_t sizeInBytes = 0;
uint16_t r;
float x[10];

float feld[4];
float temp;
// String Items[4] =  {"Menuitem 1","Menuitem 2","Menuitem 3","Menuitem 4"};
// int*  ptr;
// ptr = &Items;

volatile bool Menu = true;
volatile bool debug = false;
volatile int action = 0;
uint32_t delay_time = 60000;
uint32_t act_time = 0;
uint32_t old_time =0;

// Checks if motion was detected, sets LED HIGH and starts a timer
IRAM_ATTR void activate_menu() {
  Menu = !Menu;
    if (Menu){      
    Serial.println("Menü aktiviert");
  }
  else{
    Serial.println("Menü deaktiviert");
  } 
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
//******************OLED
  Serial.println("128x64 OLED FeatherWing test");
  delay(250);                 // wait for the OLED to power up
  display.begin(0x3C, true);  // Address 0x3C default
  Serial.println("OLED begun");
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);  

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  display.setRotation(1);
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(F("BME280 test"));
  
//******************BME280
  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.println("-- Default Test --");
  Serial.println();

//******************  FRAM
  Wire.begin();
  Serial.println(__FILE__);
  Serial.print("FRAM_LIB_VERSION: ");
  Serial.println(FRAM_LIB_VERSION);

  int rv = fram.begin(0x50);
  if (rv != 0) {
    Serial.print("INIT ERROR: ");
    Serial.println(rv);
  }

// Interrupt Menu aktivieren deaktivieren
  attachInterrupt(digitalPinToInterrupt(BUTTON_C), activate_menu, RISING);  
}

void loop() {
  while (Menu){
  display.clearDisplay();
  display.display();     
  menuehandling(); 
  }  
  //Serial.println(action);
  switch (action) {
  case 8:
    measure();
    break;
  case 16:
    readback();
    menuehandling();
    break;
  case 24:
    clearFRAM();        
    menuehandling(); 
   break;
  default:
    // statements
    break;
  }
}

void measure (void) {
    //if(!digitalRead(BUTTON_A)) Serial.print("A");  
//****************** nonblocking delay
  act_time = millis();
  if (act_time - old_time >= delay_time){
    old_time = act_time;
    Serial.print(act_time);  
    getvalues(feld);     // Values holen
    printValues(feld);   // Values ausgeben
    FRAM_storage(feld);  // Values speichern  
  }
}

void menuehandling(void) {
  int  i = 8;
  int j=0;    
  String Items[5] =  {"  Messung Starten","  FRAM readback","  FRAM loeschen","  Menuitem 3","  Menuitem 4"};
  while (Menu){
    display.setRotation(1);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("select with Button B")); 
    for (j = 0;j <= 4 ;j++){
      display.println(Items[j]);
      }
    display.display();
    while (!digitalRead(BUTTON_B)){
      delay(200);
      i=i+8;      
      if (i >= 48) i = 8 ;  
      Serial.println(i);   
      display.clearDisplay();
      display.display(); 
      display.setCursor(0,8);
      for (j = 0;j <= 4 ;j++){
         display.println(Items[j]);
         }
      display.setCursor(0,i);
      display.print("> ");
      //display.print(i);      
      display.display();           
    }  
    while(!digitalRead(BUTTON_A)){
      delay(200);
      action = i;      
    }  
  }    
}





//read back written Values
void readback (void) {
  uint16_t address = fram.read16(100);
  Serial.println(address);
  if (address == 0) {
    Serial.println("nothing to print...");
  } 
  else {
    r = 100;
    while (r < address) {            
      Serial.print(r);
      Serial.print(" ");      
      for (int i = 0; i < 4; i++) {
        r = fram.readObject(r, feld[i]);
        Serial.print(feld[i]);
        Serial.print(";");
      }
      Serial.println();
    }
  }
}

void clearFRAM (void) {
  uint16_t address = fram.read16(100);
  Serial.println(address);
  if (address == 0) {
    Serial.println("nothing to delete...");
  } 
  else {
    for (int i = 0; i < 4; i++) {  
        feld[i] = 0;
    }               
    r = 100;
    while (r < address) {            
      for (int i = 0; i < 4; i++) {
        address = fram.writeObject(address, feld[i]);
      }
    }
    fram.write16(100, 0);
   Serial.println("FRAM gelöscht") ;
  }
}

void getvalues(float feld[]) {
  temp = bme.readTemperature();
  temp = bme.readPressure() / 100.0F;
  temp = bme.readAltitude(SEALEVELPRESSURE_HPA);
  temp = bme.readHumidity();  
  feld[0] = bme.readTemperature();
  feld[1] = bme.readPressure() / 100.0F;
  feld[2] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  feld[3] = bme.readHumidity();
}

void printValues(float feld[]) {
//Serial Monitor
  Serial.print("Temperature = ");
  Serial.print(feld[0]);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(feld[1]);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(feld[2]);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(feld[3]);
  Serial.println(" %");
  Serial.println();

//OLED
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.print("Temperatur = ");
  display.print(feld[0]);
  display.println("*C");

  display.print("Druck      = ");
  display.print(feld[1]);
  display.println(" hPa");

  display.print("Hoehe      = ");
  display.print(feld[2]);
  display.println(" m");

  display.print("rel.Feuchte= ");
  display.print(feld[3]);
  display.println(" %");

  display.display();
}

void FRAM_storage(float feld[]) {
  //
  //    FILE: FRAM_writeObject.ino
  //  AUTHOR: Rob Tillaart
  // PURPOSE: demo writing reading objects
  //     URL: https://github.com/RobTillaart/FRAM_I2C
  //
  // experimental
  
  //  FILL AND WRITE 10 FLOATS
  
  uint16_t address = fram.read16(100);
  if (debug) Serial.println(address);

  if (address == 0) address = 100;
  if (debug) Serial.println(address);

  for (int i = 0; i < 4; i++) {
    if (debug) Serial.println(feld[i], 5);
    address = fram.writeObject(address, feld[i]);
    if (debug) Serial.println(address);
  }
  Serial.println(address);
  fram.write16(100, address);

}

