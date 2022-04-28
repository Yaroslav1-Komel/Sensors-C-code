//wifi
#include <WiFi.h>
//const char* ssid = "ITEK 2nd"; //Else IP address from raspberypi4
//const char* password = "Four_Sprints_F21v";   // password for the network
 const char* ssid = "NETGEAR98";                                                      
 const char* password = "quickink024"; 
WiFiClient espClient;

//mqtt
#include <PubSubClient.h>
//const char* mqtt_server = "10.120.0.220"; // raspberi pi  or another mqtt server
const char* mqtt_server = "test.mosquitto.org";
const char* inTopic = "eaaa/e21a/test/Project3";
PubSubClient client(espClient);

             // GPS start const
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 15, TXPin = 2; 
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
             // GPS finish const
             //temperatura start
             const float hot = 25; //hot parameter
const float cold = 15; //cold parameter
int count =10;// dont caunt first 10 mesurements 
              // temperatura finish
              // AIR sensor start
//may be wrong...
//wire library
#include <Wire.h>

//air quality sensor library
#include <DFRobot_SGP40.h>

//the new pins
int const I2C_SDA = 33;
int const I2C_SCL = 32;

TwoWire I2CBME = TwoWire(0);

DFRobot_SGP40 air_sensor;
               // AIR sensor finish.
               //Dust sensor start
#include <Wire.h>
#define dust_sensor 12
              //Dust sensor finish
              
             
void setup()
{              // GPS setup start
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.println(F("NEO-6M GPS MODULE QUICK TEST"));
  Serial.print(F("Testing with TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(); // GPS setup finish
  
                    // Temperatura setup start
  pinMode(12, INPUT );
pinMode(13, OUTPUT);
// Begin serial communication at 9600 baud rate
  Serial.begin(9600);
                        //Temperatura setup finish
                       //AIR sensor setup start
 Serial.begin(115200);
   I2CBME.begin(I2C_SCL, I2C_SDA);//last param is clock freq
   air_sensor.begin(10000);
                     //AIR sensor setup finish       
                     // Dust sensor setup start
                      Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,dust_sensor,-1);
                     // Dust sensor setup finish
  
}
void loop()
{                     //GPS setup start
   while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS Module Found! Check Hardware!!"));
    while(true);
  }
}
void displayInfo()
{
  Serial.print(F("Loc in DK: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();           //Gps loop finish
                   // Temperatura loop start 
float sensor = analogRead(14);
float temp =  (sensor / 4095.0)* 3.3;
// Convert the voltage into the temperature in Celsius
  float temperatureC = temp * 100;

if (temperatureC < cold && count == 0){
  
digitalWrite(13, LOW);
}
else if (temperatureC >= hot && count == 0){ 
  digitalWrite(13, HIGH);
} else {
  count--;
}

delay(1000);
// подключаем термометр и лампу
 // Print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print("\xC2\xB0"); // shows degree symbol
  Serial.print("C  |  ");
  
  // Print the temperature in Fahrenheit
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.print(temperatureF);
  Serial.print("\xC2\xB0"); // shows degree symbol
  Serial.println("F");

  delay(1000); // wait a second between readings

           // AIR sensor loop start
           uint16_t index = air_sensor.getVoclndex();

  Serial.print("vocIndex = ");
  Serial.println(index);
  delay(1000);  
            // AIR sensor loop finish.           
            // Dust sensor loop start
byte dust_byte[30];
  int dust_index = 0;
  int dust_data[3];
  int l = 0;
  while(!Serial2.available()) { 
  }
  if(Serial2.read() == 0x42 && Serial2.read() == 0x4D) {
    //Serial.println("header done");
    for(int i = 0; i < 14; i++) {
       if(Serial2.available()) {
        
         //if(i%2 == 0) {
           //Serial.print("HIGH ");
         //} else {
           //Serial.print("LOW ");
         //}
         dust_byte[dust_index] =  Serial2.read();
         //Serial.println(dust_byte[dust_index]);
         dust_index++;
       }
    }
    for(int i = 14; i < 30; i++) { //flush the no-data bytes[
      if(Serial2.available()) {
        Serial2.read();
      }
    }
  }
 //int h_length = dust_byte[0];
 //int l_length = dust_byte[1];
 //l = h_length + l_length + 2;
 //Serial.print("length: ");
 //Serial.println(l);
  for(int i = 0; i < 3; i++) {
    int value = (dust_byte[2+i*2]<<4) + dust_byte[2+1+(i*2)];
    dust_data[i] = value;
  }
  Serial.print("PM1:");
  Serial.print(dust_data[0]);
  Serial.print(",\t");
  Serial.print("PM2.5:");
  Serial.print(dust_data[1]);
  Serial.print(",\t");
  Serial.print("PM10:");
  Serial.print(dust_data[2]);
  Serial.println("");
              // Dust loop finish.
            
}


               
