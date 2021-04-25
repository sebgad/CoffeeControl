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
#include <ESPmDNS.h>
#include <SPIFFS.h>

#define FORMAT_SPIFFS_IF_FAILED true

long iTemp = 0;
long iPressure = 0;
bool bEspOnline = false;
bool bEspMdns = false;

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

bool connectWiFi(const int i_total_fail = 3, const int i_timout_attemp = 1000){
  /**
   * Try to connect to WiFi Accesspoint based on the given information in the header file WiFiAccess.h.
   * A defined number of connection is performed.
   * @param 
   *    i_timout_attemp:     Total amount of connection attemps
   *    i_waiting_time:   Waiting time between connection attemps
   * 
   * @return 
   *    b_status:         true if connection is successfull
   */
  
  WiFi.disconnect(true);
  delay(500);

  Serial.print("Device ");
  Serial.print(WiFi.macAddress());
  Serial.print(" try connecting to ");
  Serial.println(charSsid);
  delay(500);

  int i_run_cnt_fail = 0;
  int i_wifi_status = WL_IDLE_STATUS;
  bool b_successful = false;

  WiFi.mode(WIFI_STA);

  // Connect to WPA/WPA2 network:
  WiFi.begin(charSsid, charPassword);

  while ((i_wifi_status != WL_CONNECTED) && (i_run_cnt_fail<i_total_fail)) {
    // wait for connection establish
    delay(i_timout_attemp);
    i_run_cnt_fail++;
    i_wifi_status = WiFi.status();
  }

  if (i_wifi_status == WL_CONNECTED) {
      // Print ESP32 Local IP Address
      Serial.print("Connection successful. Local IP: ");
      Serial.println(WiFi.localIP());
      b_successful = true;
  } else {
    Serial.print("Connection unsuccessful. WiFi status: ");
    Serial.println(i_wifi_status);
    b_successful = false;
  }

  return b_successful;
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

  bEspOnline = connectWiFi();

  if bEspOnline{
    // ESP has wifi connection

    // register mDNS. ESP is available under http://coffee.local
    if (!MDNS.begin("coffee")) {
          Serial.println("Error setting up MDNS responder!");
    }
    // add service to standart http connection
    MDNS.addService("http", "tcp", 80);

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
}
 
void loop(){

}
