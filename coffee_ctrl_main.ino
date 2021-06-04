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
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <time.h>
#include "WifiAccess.h"
#include "ADS1115.h"
#include <Wire.h>

// Hardware definitions for i2c
// I2C pins
#define SDA_0 23
#define SCL_0 22
#define CONV_RDY_PIN 13

// File system definitions
#define FORMAT_SPIFFS_IF_FAILED true

const char* strMeasFilePath = "/data.csv";
const int iMaxBytes = 1000000;

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

// Initialize ADS1115 I2C connection
TwoWire objI2cBus = TwoWire(0);
ADS1115 objAds1115(&objI2cBus);

// timer object for ISR
// Short for Sensor Read Out and PID control
// Long for Measurement Store to csv
hw_timer_t * objTimerShort = NULL;
hw_timer_t * objTimerLong = NULL;

// Status for function call timing, used in ISR
volatile uint8_t iStatusMeas = 0; // 0:init/idle, 1:store value running, 2: store value finished
volatile uint8_t iStatusCtrl = 0; // 0:init/idle, 1:measurement running, 2:measurement finished, 3:control

// enums for the status
enum eStatusCtrl {
  IDLE,
  MEASURE,
  CONTROL,
};

enum eStatusMeas {
  IDLE_MEAS,
  STORE_MEAS
};

// Definition for critical section port
portMUX_TYPE objTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Task intervals in microseconds
const unsigned long iInterruptShortIntervalMicros = 100000; //microseconds
const unsigned long iInterruptLongIntervalMicros = 450000; //microseconds

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void IRAM_ATTR onTimerShort(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      if (iStatusCtrl == IDLE) {
        // Only change Status when idle to measurement running
        iStatusCtrl = MEASURE;
      }
    portEXIT_CRITICAL_ISR(&objTimerMux);
}

void IRAM_ATTR onTimerLong(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      if (iStatusMeas == IDLE_MEAS) {
        // Only change Status when idle to measurement running
        iStatusMeas = STORE_MEAS;
      }
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
  
  // Initialize I2c on defined pins with default adress
  if (!objAds1115.begin(SDA_0, SCL_0, ADS1115_I2CADD_DEFAULT)){
    Serial.println("Failed to initialize I2C sensor connection, stop working.");
    // TODO: diagnose
    while(1);
  } 

  // set differential voltage: A0-A1
  objAds1115.setMux(ADS1115_MUX_AIN0_AIN1);

  // set data rate (samples per second)
  objAds1115.setRate(ADS1115_RATE_8);

  // set to continues conversion method
  objAds1115.setOpMode(ADS1115_MODE_CONTINUOUS);

  // set gain amplifier
  objAds1115.setPGA(ADS1115_PGA_4P096);

  // set latching mode
  objAds1115.setCompLatchingMode(ADS1115_CMP_LAT_ACTIVE);

  // assert after one conversion
  objAds1115.setPinRdyMode(true, ADS1115_CMP_QUE_ASSERT_1_CONV);

  // Initialize SPIFFS
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      // Initialization of SPIFFS failed, restart it
      Serial.println("SPIFFS mount Failed, restart ESP");
      delay(1000);
      ESP.restart();
  } else{
    Serial.println("SPIFFS mount successfully.");
  }

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
  
  // Connect to wifi
  bEspOnline = connectWiFi();
  char char_timestamp[50];

  if (bEspOnline == true) {
      // ESP has wifi connection

      // register mDNS. ESP is available under http://coffee.local
      if (!MDNS.begin("coffee")) {
        Serial.println("Error setting up MDNS responder!");
      } else {
        // add service to standart http connection
        MDNS.addService("http", "tcp", 80);
      }

      // Webserver Settings
      // Route for root / web page
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html");
      });
    
      // Route for graphs web page
      server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/graphs.html");
      });

      // Route for stylesheets.css
      server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/style.css");
      });

      // Route for java script status table
      server.on("/table.js", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/table.js");
      });

      // Measurement file, available under http://coffee.local/data.csv
      server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, strMeasFilePath, "text/plain");
      });

      // Start web server
      server.begin();

      // initialize NTP client
      configTime(iGmtOffsetSec, iDayLightOffsetSec, charNtpServerUrl);

      // get local time
      struct tm obj_timeinfo;
      if(!getLocalTime(&obj_timeinfo)){
        Serial.println("Failed to obtain time stamp online");
      } else {
        // write time stamp into variable
        strftime(char_timestamp, sizeof(char_timestamp), "%c", &obj_timeinfo);
      }
    }

  // generate header in file
  Serial.print("Create File ");
  Serial.println(strMeasFilePath);
  
  File obj_meas_file = SPIFFS.open(strMeasFilePath, "w");

  obj_meas_file.print("Measurement File created on ");
  obj_meas_file.println(char_timestamp);
  obj_meas_file.println("");
  obj_meas_file.println("Time,Temperature,Pressure,HeatOn,PumpOn");
  obj_meas_file.close();

  // Initialize Timer 
  // Prescaler: 80 --> 1 step per microsecond (80Mhz base frequency)
  // true: increasing counter
  objTimerShort = timerBegin(0, 80, true);
  objTimerLong = timerBegin(1, 80, true);

  // Attach ISR function to timer
  // timerAttachInterrupt(objTimerShort, &onTimerShort, true);
  timerAttachInterrupt(objTimerLong, &onTimerLong, true);
  pinMode(CONV_RDY_PIN, INPUT);
  attachInterrupt(CONV_RDY_PIN, &onTimerShort, HIGH);
  
  // Define timer alarm
  // factor is 100000, equals 100ms when prescaler is 80
  // true: Alarm will be reseted automatically
  // timerAlarmWrite(objTimerShort, iInterruptShortIntervalMicros, true);
  timerAlarmWrite(objTimerLong, iInterruptLongIntervalMicros, true);
  // timerAlarmEnable(objTimerShort);
  timerAlarmEnable(objTimerLong);
}

boolean readSensors(){
  // Read out Sensor values
    bool b_result = false;

    // read out temperature sensor from ADS_1115

    
    fTemp = objAds1115.readVoltage();
    fPressure = fPressure + 0.1;
    iHeatingStatus = 1;
    iPumpStatus = 1;
    b_result = true;

    return b_result;
}

bool controlHeating(){
  /** PID regulator function for controling heating device
   *
   */
  bool b_result = false;
  
  fPidOut = 99;
  b_result = true;
  
  return b_result;
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
  switch (iStatusCtrl) {
    // Call sensor read out function or control function
    case MEASURE:
      bool b_result_sensor;
      b_result_sensor = readSensors();

      if (b_result_sensor == true) {
        portENTER_CRITICAL_ISR(&objTimerMux);
          iStatusCtrl = CONTROL;
        portEXIT_CRITICAL_ISR(&objTimerMux);
      }
      break;
    
    case CONTROL:
      bool b_result_ctrl_heating;
      b_result_ctrl_heating = controlHeating();
      if (b_result_ctrl_heating == true) {
        portENTER_CRITICAL_ISR(&objTimerMux);
          iStatusCtrl = IDLE;
        portEXIT_CRITICAL_ISR(&objTimerMux);
      }
      break;
  }
 
  switch (iStatusMeas) {
    case STORE_MEAS:
      writeMeasFile();
      portENTER_CRITICAL_ISR(&objTimerMux);
        iStatusMeas = IDLE_MEAS;
      portEXIT_CRITICAL_ISR(&objTimerMux);
  }
    
}
