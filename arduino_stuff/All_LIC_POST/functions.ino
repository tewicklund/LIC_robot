unsigned long getEpochTime() {
  // Send an NTP packet to the time server
  sendNTPpacket(ntpServer);
  
  // Wait for a response for up to 1 second
  delay(1000);
  if (udp.parsePacket()) {
    byte buffer[48];
    udp.read(buffer, 48);

    // Extract time in seconds since Jan 1, 1900 (NTP epoch)
    unsigned long highWord = word(buffer[40], buffer[41]);
    unsigned long lowWord = word(buffer[42], buffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // Convert NTP time to UNIX epoch
    unsigned long epochTime = secsSince1900 - seventyYears + timeZoneOffset;
    return epochTime;
  }
  return 0; // Return 0 if no response
}

void sendNTPpacket(const char* address) {
  // Clear the buffer and prepare the request packet
  byte packetBuffer[48] = { 0 };
  packetBuffer[0] = 0b11100011; // NTP request header
  
  // Send the packet
  udp.beginPacket(address, ntpPort);
  udp.write(packetBuffer, 48);
  udp.endPacket();
}

void sendPostRequest(double startSeconds) {
  double timestamp = double(millis())/double(1000)+startSeconds;
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