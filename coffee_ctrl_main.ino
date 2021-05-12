/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
 * TODOS:
 *  - Create different Tasks for operating the PID regulator and other tasks
 *      - xms task for analog / digital read out of the sensor values
 *      - 100ms task for PID controler
 *      - 300ms task for adding measurement value to array
 *      - 900ms task for serializing stream to JSON file
 *      - how to implement it? how many ISR?
 *  - Implement PID regulator
 *      - temperatur as input, SSR control as output
 *      - 2 basic regulation ideas: 1) PWM with short period. 2) PWM with relatively long period.
 *      - Implement both ideas and switching regulation over web interface, e.g. config page?
 *  - Implement JSON data import on webserver
 *      - define update interval seperately for the gauges, status table and chart graph
 *      - write JSON import function
 *  - Add measurement values and integrate them in JSON request
 *      - add status measurement values and state machines? e.g. heating is "on" depending on a duty cycle threshold? Water pump is on, etc.
 *  - Implement config web page and define parameters which can be set dynamically (pid control parameters, etc.)
 *  - Make it look nice, maybe?
*********/

#include <WiFi.h>
#include "WifiAccess.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "time.h"


#define FORMAT_SPIFFS_IF_FAILED true

volatile long iTemp = 0;
volatile long iPressure = 0;
bool bEspOnline = false;
bool bEspMdns = false;

// configure NTP Client
const char* charNtpServerUrl = "europe.pool.ntp.org";
const long  iGmtOffsetSec = 60000;
const int   iDayLightOffsetSec = 3600;

// Data Handling with JSON file. Containing nested arrays
// Serialization --> Transforming data to byte stream and dump it to json file. 
//                   Example: {timetamp:[245324, 245536, 253466], temperature:[20,30,40,50], pressure:[7.0, 8.1, 8.3]}
// Deserialization --> Reverse process to read in the data. The webserver is doing that via google graph library
StaticJsonDocument<10000> ObjJsonDocument;
JsonArray objDataTime = ObjJsonDocument.createNestedArray("timestamp");
JsonArray objDataTemp = ObjJsonDocument.createNestedArray("temperature");
JsonArray objDataPressure = ObjJsonDocument.createNestedArray("pressure");
// Maximum Data points of each array. If exceeded, first data point will be deleted
const int iMaxArrSize = 10000;

// Create Timerobject
hw_timer_t * objTimer = NULL;
portMUX_TYPE objTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void IRAM_ATTR onTimer(){
  /** Interrupt Service Routine
   *  - PID-Regulator
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
  portENTER_CRITICAL_ISR(&objTimerMux);
  //TODO Do some stuff here
  portEXIT_CRITICAL_ISR(&objTimerMux);
}

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

  if (bEspOnline == true) {
    // ESP has wifi connection

    // register mDNS. ESP is available under http://coffee.local
    if (!MDNS.begin("coffee")) {
          Serial.println("Error setting up MDNS responder!");
    }
    // add service to standart http connection
    MDNS.addService("http", "tcp", 80);

    // TODO: Suggestion to change to google charts since it's totally free and easy to use. Also suggest to use JSON for storage Data in SPIFFS.
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/index.html");
    });
  
    server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/graphs.html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/style.css");
    });

    server.on("/table.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/table.js");
    });
    
    //server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    //  request->send_P(200, "text/plain", readTemperature().c_str());
    //});
    //server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    //  request->send_P(200, "text/plain", readPressure().c_str());
    //});

    // Start server
    server.begin();

    // initialize NTP client
    configTime(iGmtOffsetSec, iDayLightOffsetSec, charNtpServerUrl);

  // Initialize Timer 
  // Prescaler: 80 --> 1 step per microsecond (80Mhz base frequency)
  // true: increasing counter
  objTimer = timerBegin(0, 80, true);
  // Attach ISR function to timer
  timerAttachInterrupt(objTimer, &onTimer, true);
  // Define timer alarm
  // factor is 100000, equals 100ms when prescaler is 80
  // true: Alarm will be reseted automatically
  timerAlarmWrite(objTimer, 100000, true);
  timerAlarmEnable(objTimer);
  }
}

unsigned long getEpochTime() {
  /** Get verified epoche unix time stamp
   * Returns: unix time stamp: seconds after 01.01.1970 as long integer
   */
  time_t obj_timestamp;
  
  // verify whether timestamp is valid
  struct tm struct_timestamp;
  if (!getLocalTime(&struct_timestamp)) {
    // return 0 if timestamp is not valid
    return(0);
  }

  time(&obj_timestamp);
  return obj_timestamp;   
}

void addPointsToJsonStream(){
  /** add data points to defined JSON Array
   * 
  */
  long int i_epoch_time = getEpochTime();
  objDataTime.add(i_epoch_time);
  objDataTemp.add(iTemp);
  objDataPressure.add(iPressure);

  if (objDataTime.size() > iMaxArrSize) {
    // release first entry if max size is exceeded
    objDataTime.remove(0);
    objDataTemp.remove(0);
    objDataPressure.remove(0);
  }
}


 
void loop(){

}