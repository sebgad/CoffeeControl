/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
 * TODOS:
 *  - Create different Tasks for operating the PID regulator and other tasks
 *      -  92ms for sensor measurement aquisition
 *      - 100ms task for PID controler
 *      - 500ms task for storing data to csv file
 *  - Implement PID regulator
 *      - temperatur as input, SSR control as output
 *      - 2 basic regulation ideas: 1) PWM with short period. 2) PWM with relatively long period.
 *      - Implement both ideas and switching regulation over web interface, e.g. config page?
 *  - Implement config web page and define parameters which can be set dynamically (pid control parameters, etc.)
 *  - Make it look nice, maybe?
 *  - Make coffe.local address also available for wifi clients
*********/

#include <WiFi.h>
#include "WifiAccess.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include "time.h"


// File system definitions
#define FORMAT_SPIFFS_IF_FAILED true
const char* strMeasFilePath = "/data.csv";
const int iMaxBytes = 1000000; // bytes

// Sensor Variable
volatile int iTemp = 0;
volatile int iPressure = 0;
volatile int iPumpStatus = 0; // 0: off, 1: on
volatile int iHeatingStatus = 0; // 0: off, 1:on
volatile int bStoreData = 0;

// PID controler output
volatile int iPidOut = 0;

bool bEspOnline = false;
bool bEspMdns = false;

// configure NTP Client
const char* charNtpServerUrl = "europe.pool.ntp.org";
const long  iGmtOffsetSec = 60000;
const int   iDayLightOffsetSec = 3600;

// Create Timerobject
hw_timer_t * objTimerSensor = NULL;
hw_timer_t * objTimerControler = NULL;
hw_timer_t * objTimerFileStream = NULL;

// Definition for critical section
portMUX_TYPE objTimerSensorMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE objTimerFileStreamMux = portMUX_INITIALIZER_UNLOCKED;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void IRAM_ATTR onTimerSensor(){
  /** Interrupt Service Routine
   *  - Sensor read out
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
  portENTER_CRITICAL_ISR(&objTimerSensorMux);
    iTemp = iTemp + 1;
    iPressure = iPressure + 1;
    iHeatingStatus = 0;
    iPumpStatus = 1;
    bStoreData++;
  portEXIT_CRITICAL_ISR(&objTimerSensorMux);
}

void IRAM_ATTR onTimerControler(){
  /** Interrupt Service Routine
   *  - PID-Regulator
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
  portENTER_CRITICAL_ISR(&objTimerSensorMux);
    //TODO Do some stuff here
    iPidOut = iTemp + iPressure + iPumpStatus + iHeatingStatus;
  portEXIT_CRITICAL_ISR(&objTimerSensorMux);
}

void IRAM_ATTR onTimerFilestream(){
  /** Interrupt Service Routine
   *  - Store to measurement file
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
  portENTER_CRITICAL_ISR(&objTimerFileStreamMux);
    bStoreData = 1;
  portEXIT_CRITICAL_ISR(&objTimerFileStreamMux);
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
      // Initialization of SPIFFS failed
      Serial.println("SPIFFS Mount Failed");
      return;
  } else {
    // Initialization successfull, create csv file
    unsigned int i_total_bytes = SPIFFS.totalBytes();
    unsigned int i_used_bytes = SPIFFS.usedBytes();

    Serial.println("File system info:");
    Serial.print("Total space on SPIFFS: ");
    Serial.print(i_total_bytes);
    Serial.println("byte");
    
    Serial.print("Total space used on SPIFFS: ");
    Serial.print(i_used_bytes);
    Serial.println("byte");

    Serial.print("Create File ");
    Serial.println(strMeasFilePath);
    File obj_meas_file = SPIFFS.open(strMeasFilePath, "w");

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

      server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, strMeasFilePath, "text/plain");
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
    
      obj_meas_file.print("Measurement File created on ");
      struct tm obj_timeinfo;
      if(!getLocalTime(&obj_timeinfo)){
        Serial.println("Failed to obtain time stamp online");
        obj_meas_file.println("n.a.");
      } else{
        obj_meas_file.println(&obj_timeinfo, "%d.%m.%Y %H:%M:%S");
      }  
    } else {
      // Offline Mode
        obj_meas_file.println("n.a.");
    }

    obj_meas_file.println("");
    // generate header in file
    obj_meas_file.println("Time,Temperature,Pressure,HeatOn,PumpOn");
    obj_meas_file.close();
  }

  // Initialize Timer 
  // Prescaler: 80 --> 1 step per microsecond (80Mhz base frequency)
  // true: increasing counter
  objTimerSensor = timerBegin(0, 80, true);
  objTimerControler = timerBegin(1, 80, true);
  objTimerFileStream = timerBegin(2, 80, true);
  // Attach ISR function to timer
  timerAttachInterrupt(objTimerSensor, &onTimerSensor, true);
  timerAttachInterrupt(objTimerControler, &onTimerControler, true);
  timerAttachInterrupt(objTimerFileStream, &onTimerFilestream, true);
  
  // Define timer alarm
  // factor is 100000, equals 100ms when prescaler is 80
  // true: Alarm will be reseted automatically
  timerAlarmWrite(objTimerSensor, 92000, true);
  timerAlarmWrite(objTimerControler, 100000, true);
  timerAlarmWrite(objTimerFileStream, 5000000, true);
  timerAlarmEnable(objTimerSensor);
  delay(100);
  timerAlarmEnable(objTimerControler);
  delay(100);
  timerAlarmEnable(objTimerFileStream);
}


void loop(){
    if (bStoreData == 1){
      Serial.print("Enter ");
      Serial.println(millis());
      portENTER_CRITICAL_ISR(&objTimerSensorMux);
        int f_pressure_local = iPressure;
        int i_temp_local = iTemp;
        int i_pump_status_local = iPumpStatus;
        int f_heating_status_local = iHeatingStatus;
      portEXIT_CRITICAL_ISR(&objTimerSensorMux);
    
      portENTER_CRITICAL_ISR(&objTimerFileStreamMux);
        bStoreData = 0;
      portEXIT_CRITICAL_ISR(&objTimerFileStreamMux);
      
      File obj_meas_file = SPIFFS.open(strMeasFilePath, "a");
        if (obj_meas_file.size() < iMaxBytes) {
          unsigned long i_time = millis();
          obj_meas_file.print(i_time);
          obj_meas_file.print(",");
          obj_meas_file.print(f_pressure_local);
          obj_meas_file.print(",");
          obj_meas_file.print(i_temp_local);
          obj_meas_file.print(",");
          obj_meas_file.print(i_pump_status_local);
          obj_meas_file.print(",");
          obj_meas_file.println(f_heating_status_local);
          obj_meas_file.close();
        } else {
          obj_meas_file.close();
          SPIFFS.remove(strMeasFilePath);
        }
    }
}
