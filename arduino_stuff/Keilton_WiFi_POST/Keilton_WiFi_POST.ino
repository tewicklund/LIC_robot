#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoHttpClient.h>
#include <Arduino.h>

const char* SSID = "Ynet";
const char* PASSWORD = "CLTC1234";
const char* SERVER = "192.168.4.6";  // e.g., "example.com"
const int PORT = 8080;  // Change to 443 for HTTPS
const char* PATH = "/LIC_triggers";

const int SENSOR_ID = 1;  // Replace with your sensor's ID number
const int inputPin = 2;

WiFiClient wifiClient;
HttpClient client = HttpClient(wifiClient, SERVER, PORT);

volatile bool sendPost = false;

void setup() {
  Serial.begin(115200);
  pinMode(inputPin, INPUT);

  if (WiFi.begin(SSID, PASSWORD) == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
  } else {
    Serial.println("Failed to connect to WiFi");
    while (true);
  }

  attachInterrupt(digitalPinToInterrupt(inputPin), onRisingEdge, RISING);
}

void loop() {
  if (sendPost) {
    sendPostRequest();
    sendPost = false;
  }
}

void onRisingEdge() {
  sendPost = true;
}

void sendPostRequest() {
  unsigned long timestamp = millis();
  String payload = "{\"sensor_id\":" + String(SENSOR_ID) + ", \"timestamp\":" + String(timestamp) + "}";

  client.beginRequest();
  client.post(PATH);
  client.sendHeader("Content-Type", "application/json");
  client.sendHeader("Content-Length", payload.length());
  client.beginBody();
  client.print(payload);
  client.endRequest();

  int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
}
