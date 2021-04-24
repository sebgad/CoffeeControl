/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <WiFi.h>
#include "WifiAccess.h"
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#define FORMAT_SPIFFS_IF_FAILED true

long iTemp = 0;
long iPressure = 0;
long check_wifi = millis();

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String readTemperature() {
  iTemp++;
  return String(iTemp);
}

String readPressure() {
  iPressure++;
  return String(iPressure);
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  delay(500);
  
  // Initialize SPIFFS
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }
   
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  delay(500);
  // Connect to Wi-Fi
  WiFi.begin(charSsid, charPassword);
  delay(500);


  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readTemperature().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readPressure().c_str());
  });

  // Start server
  server.begin();
}
 
void loop(){

}
