/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
 * TODOS:
 *  - Create different interrupts for operating the PID regulator and other tasks
 *      -  90ms for sensor measurement aquisition
 *      - 100ms task for PID controler
 *      - 450ms task for storing data to csv file
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

// Counter for function call timing, used in ISR
volatile int iCntInterruptSensor = 0;
volatile int iCntInterruptPid = 0;
volatile int iCntInterruptStore = 0;

// Sensor variables
float fTemp = 0;
float fPressure = 0;
int iPumpStatus = 0; // 0: off, 1: on
int iHeatingStatus = 0; // 0: off, 1:on

// PID controler output variable
float fPidOut = 0;

// bit variable to indicate whether ESP32 has a online connection
bool bEspOnline = false;
bool bEspMdns = false;

// configure NTP client
const char* charNtpServerUrl = "europe.pool.ntp.org";
const long  iGmtOffsetSec = 3600; // UTC for germany +1h = 3600s
const int   iDayLightOffsetSec = 3600; //s Time change in germany 1h = 3600s

// timer object for ISR
hw_timer_t * objTimer = NULL;

// Definition for critical section port
portMUX_TYPE objTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Task intervals in microseconds
const unsigned long iInterruptIntervalMicros = 10000; //microseconds

// Interrupt counter limits for function sensor read, pid control and measurement file store
const int iCntTotalInterruptSensor = 9; // Number of interrupts until sensor read function is called
const int iCntTotalInterruptPid = 10; // Number of interrupts until pid regulator function is called
const int iCntTotalInterruptStore = 45; // Number of interrupts until sensor value is stored in measurement file

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void IRAM_ATTR onTimer(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      iCntInterruptSensor++;
      iCntInterruptPid++;
      iCntInterruptStore++;
    portEXIT_CRITICAL_ISR(&objTimerMux);
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

      // Measurement file, available under http://coffee.local/data.csv
      server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, strMeasFilePath, "text/plain");
      });

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
        obj_meas_file.println(&obj_timeinfo, "%c");
      }  
    } else {
      // Offline mode
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
  objTimer = timerBegin(0, 80, true);

  // Attach ISR function to timer
  timerAttachInterrupt(objTimer, &onTimer, true);
  
  // Define timer alarm
  // factor is 100000, equals 100ms when prescaler is 80
  // true: Alarm will be reseted automatically
  timerAlarmWrite(objTimer, iInterruptIntervalMicros, true);
  timerAlarmEnable(objTimer);
}

void readSensors(){
  // Read out Sensor values
    fTemp = fTemp + 0.1;
    fPressure = fPressure + 0.1;
    iHeatingStatus = 1;
    iPumpStatus = 1;
}

void controlHeating(){
  /** PID regulator function for controling heating device
   *
   */
  portENTER_CRITICAL_ISR(&objTimerMux);
    fPidOut = 99;
  portEXIT_CRITICAL_ISR(&objTimerMux);
}

void writeMeasFile(){
  /** Function to store the data in measurement file
   *
   */

  portENTER_CRITICAL_ISR(&objTimerMux);
    float f_pressure_local = fPressure;
    float f_temp_local = fTemp;
    int i_pump_status_local = iPumpStatus;
    int f_heating_status_local = iHeatingStatus;
  portEXIT_CRITICAL_ISR(&objTimerMux);

  File obj_meas_file = SPIFFS.open(strMeasFilePath, "a");
  if (obj_meas_file.size() < iMaxBytes) {
    unsigned long i_time = millis();
    obj_meas_file.print(i_time);
    obj_meas_file.print(",");
    obj_meas_file.print(f_pressure_local);
    obj_meas_file.print(",");
    obj_meas_file.print(f_temp_local);
    obj_meas_file.print(",");
    obj_meas_file.print(i_pump_status_local);
    obj_meas_file.print(",");
    obj_meas_file.println(f_heating_status_local);
    obj_meas_file.close();
  } else {
    Serial.println("Data file size exceeded, delete it.");
    obj_meas_file.close();
    SPIFFS.remove(strMeasFilePath);
  }
}

void loop(){
    if (iCntInterruptSensor >= iCntTotalInterruptSensor){
        // Read sensor values if counter reaches limit
        portENTER_CRITICAL_ISR(&objTimerMux);
          iCntInterruptSensor = 0;
        portEXIT_CRITICAL_ISR(&objTimerMux);
        readSensors();
    } else if (iCntInterruptPid >= iCntTotalInterruptPid) {
        // Call pid regulator function if counter reaches limit
        portENTER_CRITICAL_ISR(&objTimerMux);
          iCntInterruptPid = 0;
        portEXIT_CRITICAL_ISR(&objTimerMux);
        controlHeating();
    } else if (iCntInterruptStore >= iCntTotalInterruptStore) {
        // Call store function if counter reaches limit 
        portENTER_CRITICAL_ISR(&objTimerMux);
          iCntInterruptStore = 0;
        portEXIT_CRITICAL_ISR(&objTimerMux);
        writeMeasFile();
    }
}
