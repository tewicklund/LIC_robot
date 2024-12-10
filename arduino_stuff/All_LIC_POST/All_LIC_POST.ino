#include <WiFiS3.h>             // Library for WiFi on Arduino Uno Rev4
#include <WiFiClient.h>         // Library for Arduino to act as a WiFi client
#include <ArduinoHttpClient.h>  // Library for Arduino to act as an HTTP client
#include <WiFiUdp.h>            // UDP library for network communication
#include <Wire.h>               // I2C library
#include "Adafruit_TCS34725.h"  // Library for color sensor

// Number to specify UUT, THIS WILL NEED TO BE ADJUSTED WHEN CHANGING UUT
const int SENSOR_ID = 0;

// Information about the WiFi network the Arduino needs to join
const char* SSID = "Ynet";
const char* PASSWORD = "CLTC1234";

// Information about the HTTP server that will receive and store LIC trigger events
const char* SERVER = "192.168.4.28";  // e.g., "example.com"
const int PORT = 8080;               // Change to 443 for HTTPS
const char* PATH = "/LIC_triggers";

// Information about the ntp server used to get an epoch timestamp UTC
const char* ntpServer = "pool.ntp.org";
const int ntpPort = 123;                          // NTP uses port 123
const unsigned long seventyYears = 2208988800UL;  // UNIX epoch starts from 1970, NTP from 1900
const int timeZoneOffset = 0;                     // Offset in seconds, adjust based on your timezone

// Digital pin assignment
const int buttonPin = 2;    // connect test button to digital pin 2
const int presencePin = 3;  // connect TTL output from LIC to digital pin 3 (Keilton only)
const int noWifiPin=4;
const int yesWifiPin=5;
const int triggerIndicatorPin=6;

// WiFi setup stuff
WiFiClient wifiClient;
HttpClient client = HttpClient(wifiClient, SERVER, PORT);
WiFiUDP udp;

// Color sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp;

//analog read setup
int analogReading=0;

void setup() {
  // Serial comms setup
  Serial.begin(115200);

  

  // Input pin setup
  pinMode(buttonPin, INPUT);
  pinMode(presencePin, INPUT);
  pinMode(noWifiPin,OUTPUT);
  pinMode(yesWifiPin,OUTPUT);
  pinMode(triggerIndicatorPin,OUTPUT);

  //WiFi setup
  if (WiFi.begin(SSID, PASSWORD) == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    digitalWrite(yesWifiPin, HIGH);
    digitalWrite(noWifiPin, LOW);
  } else {
    Serial.println("Failed to connect to WiFi");
    digitalWrite(yesWifiPin, LOW);
    digitalWrite(noWifiPin, HIGH);
    while (true)
      ;
  }
  udp.begin(ntpPort);

  // Color sensor setup, only applicable for 2 LICs
  if (SENSOR_ID == 3 || SENSOR_ID == 4) {
    if (tcs.begin()) {
      Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1)
        ;
    }
  }
}

void loop() {
  switch (SENSOR_ID) {

    // test mode, use button to trigger POST request
    case 0:
      if (digitalRead(buttonPin)) {
        Serial.println("Detected button press, sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next button press...");
      }
      break;

    // UUT = Keilton LIC
    case 1:
      if (digitalRead(presencePin)) {
        Serial.println("Detected presence from Keilton LIC, sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next LIC trigger...");
      }
      break;

    // UUT = Acuity LIC
    case 2:
      analogReading=analogRead(A0);
      if (analogReading < 512) {
        Serial.print("Detected presence from Acuity LIC (analogRead = ");
        Serial.print(analogReading);
        Serial.println("), sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next LIC trigger...");
      }
      break;

    // UUT = Cooper LIC
    case 3:
      tcs.getRawData(&r, &g, &b, &c);
      if (b > 160) {
        Serial.print("Detected presence from Cooper LIC (Blue = ");
        Serial.print(b);
        Serial.println("), sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next LIC trigger...");
      }
      break;

    //UUT = EasySense LIC, the one that can't be taken out of its luminaire
    case 4:
      tcs.getRawData(&r, &g, &b, &c);
      if (r > 40) {
        Serial.print("Detected presence from EasySense LIC (Red = ");
        Serial.print(r);
        Serial.println("),sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next LIC trigger...");
      }
      break;

    //UUT = WIZ20 LIC
    case 5:
      if (digitalRead(presencePin)) {
        Serial.println("Detected presence from WIZ20 LIC, sending post request");
        sendPostRequest();
        Serial.println("Sent request successfully! Waiting 5 seconds...");
        digitalWrite(triggerIndicatorPin,HIGH);
        delay(5000);
        digitalWrite(triggerIndicatorPin,LOW);
        Serial.println("Waiting for next LIC trigger...");
      }
      break;
  }
}
