/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
 * TODOS:
 *  - Implement PID regulator
 *      - temperatur as input, SSR control as output
 *      - 2 basic regulation ideas: 1) PWM with short period. 2) PWM with relatively long period.
 *      - Implement both ideas and switching regulation over web interface, e.g. config page?
 *      - make Tuning Parameters editeable via webserver
 *  - Implement config web page and define parameters which can be set dynamically (pid control parameters, etc.)
 *  - Make it look nice, maybe?
 *  - Make coffe.local address also available for wifi clients
*********/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <time.h>
#include <string>
#include "WifiAccess.h"
#include "Pt1000.h"
#include "ADS1115.h"
#include "PidCtrl.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include "AsyncJson.h"


// Hardware definitions for i2c
// I2C pins
#define SDA_0 23
#define SCL_0 22
#define CONV_RDY_PIN 14

// File system definitions
#define FORMAT_SPIFFS_IF_FAILED true

// PWM defines
#define P_SSR_PWM 21

struct PID {
  float Target;
  float Kp;
  float Ki;
  float Kd;
};

const char* strMeasFilePath = "/data.csv";
const char* strParamFilePath = "/params.json";

const int iMaxBytes = 1000000;
unsigned long iTimeStart = 0;

const uint32_t iSsrFreq = 200; // Hz - PWM frequency
const uint32_t iPwmSsrChannel = 0; //  PWM channel. There are 16 channels from 0 to 15. Channel 0 is now SSR-Controll
const uint32_t iPwmSsrResolution = 8; //  resulution of the DC; 0 => 0%; 255 = (2**8) => 100%. -> required by PWM lib
float fTarPwm;
PID objPidCoeff;

// Sensor variables
float fTemp = 0; // TODO -  need to use double for PID lib
float fPressure = 0;
int iPumpStatus = 0; // 0: off, 1: on
int iHeatingStatus = 0; // 0: off, 1:on

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
// Long for Measurement Store to csv
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

// Initialisation of PID controler
PidCtrl objPid;

// Definition for critical section port
portMUX_TYPE objTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Task intervals in microseconds
const unsigned long iInterruptLongIntervalMicros = 450000; //microseconds

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// timers ==============================================================================================================
void IRAM_ATTR onAlertRdy(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      if (iStatusCtrl == IDLE) {
        // Only change Status when idle to measurement running
        iStatusCtrl = MEASURE;
        // read out temperature sensor from ADS_1115
        // fTemp = (double) objAds1115.getPhysVal();
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
} // connectWiFi

void loadConfiguration(){
  /**
   * Load configuration from configuration file 
   */
  
  File obj_param_file = SPIFFS.open(strParamFilePath, "r");
  StaticJsonDocument<200> json_doc;
  DeserializationError error = deserializeJson(json_doc, obj_param_file);

  if ((!obj_param_file) || (error)){
    Serial.println(F("Failed to read file, using default configuration"));
    initConfiguration();
  } else {
    objPidCoeff.Kd = json_doc["Kp"];
    objPidCoeff.Ki = json_doc["Ki"];
    objPidCoeff.Kd = json_doc["Kd"];
    objPidCoeff.Target = json_doc["Target"];
  }

  obj_param_file.close();
}

void saveConfiguration(){
  /**
   * Save configuration to configuration file 
   */
  
  StaticJsonDocument<200> json_doc;

  json_doc["Kp"] = objPidCoeff.Kp;
  json_doc["Ki"] = objPidCoeff.Ki;
  json_doc["Kd"] = objPidCoeff.Kd;
  json_doc["Target"] = objPidCoeff.Target;

  File obj_param_file = SPIFFS.open(strParamFilePath, "w");
  
  if (serializeJson(json_doc, obj_param_file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  obj_param_file.close();
}

void initConfiguration(){
  /**
   * init configuration to configuration file 
   */
  
  objPidCoeff.Kp = 0.01;
  objPidCoeff.Ki = 0.01;
  objPidCoeff.Kd = 0.01;
  objPidCoeff.Target = 92.5;
  saveConfiguration();
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting setup.");
  
  // Setup ADS1115 =========================================================================================================
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
  objAds1115.setPGA(ADS1115_PGA_1P024);

  // set latching mode
  objAds1115.setCompLatchingMode(ADS1115_CMP_LAT_ACTIVE);

  // assert after one conversion
  objAds1115.setPinRdyMode(ADS1115_CONV_READY_ACTIVE, ADS1115_CMP_QUE_ASSERT_1_CONV);

  switch (Pt1000_CONV_METHOD)
  {
  case Pt1000_CONV_LINEAR:
    objAds1115.setPhysConv(fPt1000LinCoeffX1, fPt1000LinCoeffX0);
    Serial.print("Applying linear regression function for Pt1000 conversion: ");
    Serial.print(fPt1000LinCoeffX1, 4);
    Serial.print(" * Umess + ");
    Serial.println(fPt1000LinCoeffX0, 4);
    break;
  case Pt1000_CONV_SQUARE:
    objAds1115.setPhysConv(fPt1000SquareCoeffX2, fPt1000SquareCoeffX1, fPt1000SquareCoeffX0);
    Serial.print("Applying square regression function for Pt1000 conversion: ");
    Serial.print(fPt1000SquareCoeffX2, 4);
    Serial.print(" * Umess² + ");
    Serial.print(fPt1000SquareCoeffX1, 4);
    Serial.print(" * Umess + ");
    Serial.println(fPt1000SquareCoeffX0);
    break;
  case Pt1000_CONV_LOOK_UP_TABLE:
    const size_t size_1d_map = sizeof(arrPt1000LookUpTbl) / sizeof(arrPt1000LookUpTbl[0]);
    objAds1115.setPhysConv(arrPt1000LookUpTbl, size_1d_map);
    Serial.println("Applying Lookuptable for Pt1000 conversion:");
    for(int i_row=0; i_row<size_1d_map; i_row++){
      Serial.print(arrPt1000LookUpTbl[i_row][0], 4);
      Serial.print("   ");
      Serial.println(arrPt1000LookUpTbl[i_row][1], 4);
    }
    break;
  }

  objAds1115.printConfigReg();

  // Setup Webserver =========================================================================================================

  // Initialize SPIFFS
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      // Initialization of SPIFFS failed, restart it
      Serial.println("SPIFFS mount Failed, restart ESP");
      delay(1000);
      ESP.restart();
  } else{
    Serial.println("SPIFFS mount successfully.");
    loadConfiguration();
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

      // Route for graphs web page
      server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/settings.html");
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

      // Parameter file, available under http://coffee.local/params.json
      server.on("/params.json", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, strParamFilePath, "text/plain");
      });

      AsyncCallbackJsonWebHandler* obj_handler = new AsyncCallbackJsonWebHandler("/paramUpdate", [](AsyncWebServerRequest *request, JsonVariant &json) {
        StaticJsonDocument<200> obj_json;
        obj_json = json.as<JsonObject>();
        
        objPidCoeff.Kp = obj_json["Kp"];
        objPidCoeff.Ki = obj_json["Ki"];
        objPidCoeff.Kd = obj_json["Kd"];
        objPidCoeff.Target = obj_json["Target"];
        
        saveConfiguration();
        request->send(200, "text/plain", "Parameters are updated");
      });

      server.addHandler(obj_handler);
      
      server.on("/paramReset", HTTP_GET, [](AsyncWebServerRequest *request){
        // initialization of a new parameter file
        initConfiguration();
        saveConfiguration();
        request->send(200, "text/plain", "Parameters are set back to default values");
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
  Serial.print(WiFi.localIP());
  Serial.println(strMeasFilePath);
  
  File obj_meas_file = SPIFFS.open(strMeasFilePath, "w");

  obj_meas_file.print("Measurement File created on ");
  obj_meas_file.println(char_timestamp);
  obj_meas_file.println("");
  obj_meas_file.println("Time,Temperature,AdReadVoltage,Pressure,HeatOn,PumpOn,TarPwm");
  obj_meas_file.close();

  // Initialize Timer 
  // Prescaler: 80 --> 1 step per microsecond (80Mhz base frequency)
  // true: increasing counter
  objTimerLong = timerBegin(1, 80, true);

  // Attach ISR function to timer
  // timerAttachInterrupt(objTimerShort, &onAlertRdy, true);
  timerAttachInterrupt(objTimerLong, &onTimerLong, true);
  pinMode(CONV_RDY_PIN, INPUT);
  attachInterrupt(CONV_RDY_PIN, &onAlertRdy, RISING);
  
  // Define timer alarm
  // factor is 100000, equals 100ms when prescaler is 80
  // true: Alarm will be reseted automatically
  timerAlarmWrite(objTimerLong, iInterruptLongIntervalMicros, true);
  timerAlarmEnable(objTimerLong);
  iTimeStart = millis();

  // Setup PID =========================================================================================================
  objPid.begin(&fTemp, &fTarPwm);
  objPid.addOutputLimits(0, (1<<iPwmSsrResolution)-1);
  objPid.changeTargetValue(objPidCoeff.Target);

  objPid.changePidCoeffs(objPidCoeff.Kp, objPidCoeff.Ki, objPidCoeff.Kd);

  // configure PWM functionalitites
  ledcSetup(iPwmSsrChannel, iSsrFreq, iPwmSsrResolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(P_SSR_PWM, iPwmSsrChannel);

}// Setup

boolean readSensors(){
  // Read out Sensor values
  bool b_result = false;
  // read conversion register over i2c
  objAds1115.readConversionRegister();
  // get physical value of sensor
  fTemp = objAds1115.getPhysVal();
  // get voltage level of sensor
  fPressure = random(0,10);
  
  // TODO S.: Logic for Heating Status indicator needs to be added
  if (fTarPwm >= 120) {
    iHeatingStatus = 1;
  } else {
    iHeatingStatus = 0;
  }

  iPumpStatus = 0;
  b_result = true;
  return b_result;
} //readSensors

bool controlHeating(){
  /** PID regulator function for controling heating device
   *
   */

  bool b_success = true;
  
  objPid.compute();
  ledcWrite(iPwmSsrChannel, (int)fTarPwm);

  return b_success;

} // controlHeating

void writeMeasFile(){
  /** Function to store the data in measurement file
   *
   */

  portENTER_CRITICAL_ISR(&objTimerMux);
    float f_pressure = fPressure;
    float f_temp_local = fTemp;
    int i_pump_status_local = iPumpStatus;
    int f_heating_status_local = iHeatingStatus;
    float f_tar_pwm = fTarPwm;
  portEXIT_CRITICAL_ISR(&objTimerMux);
  
  File obj_meas_file = SPIFFS.open(strMeasFilePath, "a");
  if (obj_meas_file.size() < iMaxBytes) {
    float f_time = (float)(millis() - iTimeStart) / 1000.0;
    obj_meas_file.print(f_time, 4);
    obj_meas_file.print(",");
    obj_meas_file.print(f_temp_local);
    obj_meas_file.print(",");
    obj_meas_file.print(f_pressure);
    obj_meas_file.print(",");
    obj_meas_file.print(f_heating_status_local);
    obj_meas_file.print(",");
    obj_meas_file.print(i_pump_status_local);
    obj_meas_file.print(",");
    obj_meas_file.println(f_tar_pwm);
    obj_meas_file.close();
  } else {
    Serial.println("Data file size exceeded, delete it.");
    obj_meas_file.close();
    SPIFFS.remove(strMeasFilePath);
  }
}// writeMeasFile

void loop(){
  if (iStatusCtrl == MEASURE) {
    // Call sensor read out function
    bool b_result_sensor;
    b_result_sensor = readSensors();

    if (b_result_sensor == true) {
      portENTER_CRITICAL_ISR(&objTimerMux);
        iStatusCtrl = CONTROL;
      portEXIT_CRITICAL_ISR(&objTimerMux);
    }
  }
    
  if (iStatusCtrl == CONTROL) {
    // Call heating control function
    bool b_result_ctrl_heating;
    b_result_ctrl_heating = controlHeating();
    
    if (b_result_ctrl_heating == true) {
      portENTER_CRITICAL_ISR(&objTimerMux);
        iStatusCtrl = IDLE;
      portEXIT_CRITICAL_ISR(&objTimerMux);
      }
  }
 
  if (iStatusMeas==STORE_MEAS) {
      writeMeasFile();
      portENTER_CRITICAL_ISR(&objTimerMux);
        iStatusMeas = IDLE_MEAS;
      portEXIT_CRITICAL_ISR(&objTimerMux);
  }
}// loop
