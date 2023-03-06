/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 
#include <TinyGPS++.h>
#include <WiFi.h>


#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    915E6

TinyGPSPlus gps;                            
SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet ;           

struct DataX {
  float latx; //4
  float lonx; //4
  float altx; //4 
  float velx; //4
  float ax;   //4
  float ay;   //4
  float az;   //4 
  float gx;   //4
  float gy;   //4
  float gz;   //4 
  float mx;   //4
  float my;   //4
  float mz;   //4 
  float air;  //4
  int id;     //2
} datos;      //58

void setup()
{
  datos={0.0,0.0,0.0,0,0,0,0,};
  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(915E6)) {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0 , 0 , "Starting LoRa Failed, \nReset");
    display.display();
    while (1);
  }
  delay(1500);
  datos.mac="78-21-84-88-72-20";//WiFi.macAddress();
}

void loop()
{

  smartDelay(2000);                                      

  if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS data received: check wiring"));
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);
      display.drawString(0 , 0 , "Searching for satellites");
      //display.drawStringMaxWidth(0 , 26 , 128, packet);
      display.display();
    }
    else
    {
      datos.latx=gps.location.lat();
      datos.lonx=gps.location.lng();
      datos.satx=gps.satellites.value();
      datos.altx=gps.altitude.feet()/ 3.2808;
      datos.hrx=gps.time.hour();
      datos.minx=gps.time.minute();
      datos.secx=gps.time.second();
      Serial.print("Latitude  : ");
      Serial.println(datos.latx, 8);
      Serial.print("Longitude : ");
      Serial.println(datos.lonx, 8);
      Serial.print("Satellites: ");
      Serial.println(datos.satx);
      Serial.print("Altitude  : ");
      Serial.print(datos.altx);
      Serial.println("M");
      Serial.print("Time      : ");
      Serial.print(datos.hrx);
      Serial.print(":");
      Serial.print(datos.minx);
      Serial.print(":");
      Serial.println(datos.secx);
      Serial.print("ID: ");
      Serial.println(datos.mac);
      //Serial.println(sizeof(datos));
      Serial.println("**********************");
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);
      display.drawString(0 , 0 , "Lat: "+ String(datos.latx, 8));
      display.drawString(0 , 10 , "Lon: "+ String(datos.lonx, 8));
      display.drawString(0 , 20 , "Alt: "+ String(datos.altx, 2)+" m");
      display.drawString(0 , 30 , "T: "+ String(datos.hrx) + ":"+String(datos.minx)+":"+String(datos.secx));
      display.drawString(0 , 40 , "Sat: "+ String(datos.satx));
      display.drawString(0 , 50 , "MAC: "+ datos.mac);
      display.display();
      LoRa.beginPacket();
      
      LoRa.write((uint8_t *)&datos,sizeof(datos));
      LoRa.endPacket();
      
      }
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
