/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
*********/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <time.h>
#include <string>
#include "Pt1000.h"
#include "ADS1115.h"
#include "PidCtrl.h"
#include "WifiAccess.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include "AsyncJson.h"
#include <Update.h>

// PIN definitions

// I2C pins
#define SDA_0 23
#define SCL_0 22
#define CONV_RDY_PIN 14

// PWM defines
#define P_SSR_PWM 21
#define P_RED_LED_PWM 13
#define P_GRN_LED_PWM 27 // TODO used to be 12
#define P_BLU_LED_PWM 12 // TODO used to be 27
#define P_STAT_LED 33 // green status LED

// File system definitions
#define FORMAT_SPIFFS_IF_FAILED true
#define JSON_MEMORY 1000

// define timer related channels for PWM signals
#define RwmRedChannel 13 //  PWM channel. There are 16 channels from 0 to 15. Channel 13 is now Red-LED
#define RwmGrnChannel 14 //  PWM channel. There are 16 channels from 0 to 15. Channel 14 is now Green-LED
#define RwmBluChannel 15 //  PWM channel. There are 16 channels from 0 to 15. Channel 15 is now Blue-LED
#define PwmSsrChannel 0  //  PWM channel. There are 16 channels from 0 to 15. Channel 0 is now SSR-Controll

// config structure for online calibration
struct config {
  String wifiSSID;
  String wifiPassword;
  float CtrlTarget;
  bool CtrlTimeFactor;
  bool CtrlPropActivate;
  float CtrlPropFactor;
  bool CtrlIntActivate;
  float CtrlIntFactor;
  bool CtrlDifActivate;
  float CtrlDifFactor;
  bool LowThresholdActivate;
  float LowThresholdValue;
  bool HighThresholdActivate;
  float HighTresholdValue;
  float LowLimitManipulation;
  float HighLimitManipulation;
  uint32_t SsrFreq = 200; // Hz - PWM frequency
  uint32_t PwmSsrResolution = 8; //  resulution of the DC; 0 => 0%; 255 = (2**8) => 100%. -> required by PWM lib
  // RGB PWM config
  uint32_t RwmRgbFreq = 500; // Hz - PWM frequency
  uint32_t RwmRgbResolution = 8; //  resulution of the DC; 0 => 0%; 255 = (2**8) => 100%. 
  bool SigFilterActive;
  int SigFilterSize;
};


// File paths for measurement and calibration file
const char* strMeasFilePath = "/data.csv";
bool bMeasFileLocked = false;
const int iMaxMeasurements = 10000; // Maximum allowed measurement lines
const char* strParamFilePath = "/params.json";
bool bParamFileLocked = false;

// start time for measurement
unsigned long iTimeStart = 0;

// define target PWM
float fTarPwm;

// define configuration struct
config objConfig;

// Sensor variables
float fTemp = 0; // TODO -  need to use double for PID lib

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
hw_timer_t * objTimerLong = NULL;

// Status for function call timing, used in ISR
volatile uint8_t iStatusMeas = 0; // 0:init/idle, 1:store value running, 2: store value finished
volatile uint8_t iStatusCtrl = 0; // 0:init/idle, 1:measurement running, 2:measurement finished, 3:control
volatile uint8_t iStatusLED = 0; // 0:init/idle, 1: set new LED value

// define Counter for interrupt handling
volatile unsigned int iInterruptCntLong = 0;

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

enum eStatusLED {
  LED_IDLE,
  LED_SET
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
      }
    portEXIT_CRITICAL_ISR(&objTimerMux);
}

void IRAM_ATTR onTimerLong(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      // Interrupt counter
      iInterruptCntLong++;

      if (iStatusMeas == IDLE_MEAS) {
        // Only change Status when idle to measurement running
        iStatusMeas = STORE_MEAS;
      }

      if (iInterruptCntLong % 5 == 0) {
        // Only write LED value when interrupt function is called 5 times
        iStatusLED = LED_SET;
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
  
  bool b_successful = false;
  
  WiFi.disconnect(true);
  delay(100);

  Serial.print("Device ");
  Serial.print(WiFi.macAddress());

  Serial.print(" try connecting to ");
  Serial.println(objConfig.wifiSSID);
  delay(100);

  int i_run_cnt_fail = 0;
  int i_wifi_status = WL_IDLE_STATUS;

  WiFi.mode(WIFI_STA);

  // Connect to WPA/WPA2 network:
  WiFi.begin(objConfig.wifiSSID.c_str(), objConfig.wifiPassword.c_str());

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
      // Signal strength and approximate conversion to percentage
      int i_dBm = WiFi.RSSI();
      int i_dBm_percentage = 0;
      if (i_dBm>=-50) {
        i_dBm_percentage = 100;
      } else if (i_dBm<=-100) {
        i_dBm_percentage = 0;
      } else {
        i_dBm_percentage = 2*(i_dBm+100);
      }
      Serial.print("Signal Strength: ");
      Serial.print(i_dBm);
      Serial.print(" dB -> ");
      Serial.print(i_dBm_percentage);
      Serial.println(" %");
      b_successful = true;
  } else {
    Serial.print("Connection unsuccessful. WiFi status: ");
    Serial.println(i_wifi_status);
  }
  
  return b_successful;
} // connectWiFi

void reconnectWiFi(WiFiEvent_t event, WiFiEventInfo_t info){
  /**
   * Try to reconnect to WiFi when disconnected from network
   */
    
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  connectWiFi(3, 1000);
} // reconnectWiFi

bool loadConfiguration(){
  /**
   * Load configuration from configuration file 
   */
  
  bool b_success = false;

  if (!bParamFileLocked){
    // file is not locked by another process ->  save to read or write
    bParamFileLocked = true;
    File obj_param_file = SPIFFS.open(strParamFilePath, "r");
    StaticJsonDocument<JSON_MEMORY> json_doc;
    DeserializationError error = deserializeJson(json_doc, obj_param_file);

    if ((!obj_param_file) || (error)){
      Serial.println(F("Failed to read file, using default configuration"));
      
      if (!obj_param_file) {
        Serial.println(F("File could not be opened"));
      }
    
      if (error) {
        Serial.print(F("json deserializion error: "));
        Serial.println(error.c_str());
      }
    
      obj_param_file.close();
      bParamFileLocked = false;
      resetConfiguration(true);
    } else {
      // file could be read without issues and json document could be interpreted
      bParamFileLocked = false;
      // reset configuration struct before writing to get default values
      resetConfiguration(false);
      bool b_set_default_values = false; 
      
      // Assign Values from json-File to configuration struct. If Field does not exist in JSON-File write default value to JSON file.
      (json_doc["Wifi"]["wifiSSID"])?objConfig.wifiSSID = json_doc["Wifi"]["wifiSSID"].as<String>():b_set_default_values = true; // issue #118 in ArduinoJson
      (json_doc["Wifi"]["wifiPassword"])?objConfig.wifiPassword = json_doc["Wifi"]["wifiPassword"].as<String>():b_set_default_values = true; // issue #118 in ArduinoJson
      (json_doc["PID"]["CtrlTimeFactor"])?objConfig.CtrlTimeFactor = json_doc["PID"]["CtrlTimeFactor"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlPropActivate"])?objConfig.CtrlPropActivate = json_doc["PID"]["CtrlPropActivate"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlPropFactor"])?objConfig.CtrlPropFactor = json_doc["PID"]["CtrlPropFactor"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlIntActivate"])?objConfig.CtrlIntActivate = json_doc["PID"]["CtrlIntActivate"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlIntFactor"])?objConfig.CtrlIntFactor = json_doc["PID"]["CtrlIntFactor"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlDifActivate"])?objConfig.CtrlDifActivate = json_doc["PID"]["CtrlDifActivate"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlDifFactor"])?objConfig.CtrlDifFactor = json_doc["PID"]["CtrlDifFactor"]:b_set_default_values = true;
      (json_doc["PID"]["CtrlTarget"])?objConfig.CtrlTarget = json_doc["PID"]["CtrlTarget"]:b_set_default_values = true;
      (json_doc["PID"]["LowThresholdActivate"])?objConfig.LowThresholdActivate = json_doc["PID"]["LowThresholdActivate"]:b_set_default_values = true;
      (json_doc["PID"]["LowThresholdValue"])?objConfig.LowThresholdValue = json_doc["PID"]["LowThresholdValue"]:b_set_default_values = true;
      (json_doc["PID"]["HighThresholdActivate"])?objConfig.HighThresholdActivate = json_doc["PID"]["HighThresholdActivate"]:b_set_default_values = true;
      (json_doc["PID"]["HighTresholdValue"])?objConfig.HighTresholdValue = json_doc["PID"]["HighTresholdValue"]:b_set_default_values = true;
      (json_doc["PID"]["LowLimitManipulation"])?objConfig.LowLimitManipulation = json_doc["PID"]["LowLimitManipulation"]:b_set_default_values = true;
      (json_doc["PID"]["HighLimitManipulation"])?objConfig.HighLimitManipulation = json_doc["PID"]["HighLimitManipulation"]:b_set_default_values = true;
      (json_doc["SSR"]["SsrFreq"])?objConfig.SsrFreq = json_doc["SSR"]["SsrFreq"]:b_set_default_values = true;
      (json_doc["SSR"]["PwmSsrResolution"])?objConfig.PwmSsrResolution = json_doc["SSR"]["PwmSsrResolution"]:b_set_default_values = true;
      (json_doc["LED"]["RwmRgbFreq"])?objConfig.RwmRgbFreq = json_doc["LED"]["RwmRgbFreq"]:b_set_default_values = true;
      (json_doc["LED"]["RwmRgbResolution"])?objConfig.RwmRgbResolution = json_doc["LED"]["RwmRgbResolution"]:b_set_default_values = true;
      (json_doc["Signal"]["SigFilterActive"])?objConfig.SigFilterActive = json_doc["Signal"]["SigFilterActive"]:b_set_default_values = true;

      if (b_set_default_values){
        // default values are set to Json object -> write it back to file.
        if (!saveConfiguration()){
          Serial.println("Cannot write back to JSON file");
        }
      }
  }
    b_success = true;
  } else {
    b_success = false;
  }
  return b_success;
}

bool saveConfiguration(){
  /**
   * Save configuration to configuration file 
   */
  
  bool b_success = false;
  StaticJsonDocument<JSON_MEMORY> json_doc;

  json_doc["Wifi"]["wifiSSID"] = objConfig.wifiSSID;
  json_doc["Wifi"]["wifiPassword"] = objConfig.wifiPassword;
  json_doc["PID"]["CtrlTimeFactor"] = objConfig.CtrlTimeFactor;
  json_doc["PID"]["CtrlPropActivate"] = objConfig.CtrlPropActivate;
  json_doc["PID"]["CtrlPropFactor"] = objConfig.CtrlPropFactor;
  json_doc["PID"]["CtrlIntActivate"] = objConfig.CtrlIntActivate;
  json_doc["PID"]["CtrlIntFactor"] = objConfig.CtrlIntFactor;
  json_doc["PID"]["CtrlDifActivate"] = objConfig.CtrlDifActivate;
  json_doc["PID"]["CtrlDifFactor"] = objConfig.CtrlDifFactor;
  json_doc["PID"]["CtrlTarget"] = objConfig.CtrlTarget;
  json_doc["PID"]["LowThresholdActivate"] = objConfig.LowThresholdActivate;
  json_doc["PID"]["LowThresholdValue"] = objConfig.LowThresholdValue;
  json_doc["PID"]["HighThresholdActivate"] = objConfig.HighThresholdActivate;
  json_doc["PID"]["HighTresholdValue"] = objConfig.HighTresholdValue;
  json_doc["PID"]["LowLimitManipulation"] = objConfig.LowLimitManipulation;
  json_doc["PID"]["HighLimitManipulation"] = objConfig.HighLimitManipulation;
  json_doc["SSR"]["SsrFreq"]  = objConfig.SsrFreq;
  json_doc["SSR"]["PwmSsrResolution"]  = objConfig.PwmSsrResolution;
  json_doc["LED"]["RwmRgbFreq"] = objConfig.RwmRgbFreq;
  json_doc["LED"]["RwmRgbResolution"] = objConfig.RwmRgbResolution;
  json_doc["Signal"]["SigFilterActive"] = objConfig.SigFilterActive;

  if (!bParamFileLocked){
    bParamFileLocked = true;
    File obj_param_file = SPIFFS.open(strParamFilePath, "w");
  
    if (serializeJsonPretty(json_doc, obj_param_file) == 0) {
      Serial.println(F("Failed to write configuration to file"));
    }
    else{
      Serial.println("configuration file updated successfully");
      b_success = true;
    }
    obj_param_file.close();
    bParamFileLocked = false;
  } else {
    b_success = false;
  }
  return b_success;
}

void resetConfiguration(boolean b_safe_to_json){
  /**
   * init configuration to configuration file 
   * 
   * @param b_safe_to_json: Safe initial configuration to json file
   */
  
  objConfig.wifiSSID = strWifiSsidFactory;
  objConfig.wifiPassword = strWifiPwFactory;
  objConfig.CtrlTimeFactor = true;
  objConfig.CtrlPropActivate = true;
  objConfig.CtrlPropFactor = 10.0;
  objConfig.CtrlIntActivate = true;
  objConfig.CtrlIntFactor = 350.0;
  objConfig.CtrlDifActivate = false;
  objConfig.CtrlDifFactor = 0.0;
  objConfig.CtrlTarget = 91.0;
  objConfig.LowThresholdActivate = false;
  objConfig.LowThresholdValue = 0.0;
  objConfig.HighThresholdActivate = false;
  objConfig.HighTresholdValue = 0.0;
  objConfig.LowLimitManipulation = 0;
  objConfig.HighLimitManipulation = 255;
  objConfig.SsrFreq = 15;
  objConfig.PwmSsrResolution = 8;
  objConfig.RwmRgbFreq = 500; // Hz - PWM frequency
  objConfig.RwmRgbResolution = 8; //  resulution of the DC; 0 => 0%; 255 = (2**8) => 100%.
  objConfig.SigFilterActive = true;

  if (b_safe_to_json){
    saveConfiguration();
  }
}

void configLED(){
  /**
   * @brief Method to configure LED functionality
   * 
   */
  ledcSetup(RwmRedChannel, objConfig.RwmRgbFreq, objConfig.RwmRgbResolution);
  ledcSetup(RwmGrnChannel, objConfig.RwmRgbFreq, objConfig.RwmRgbResolution);
  ledcSetup(RwmBluChannel, objConfig.RwmRgbFreq, objConfig.RwmRgbResolution);

  ledcAttachPin(P_RED_LED_PWM, RwmRedChannel);
  ledcAttachPin(P_GRN_LED_PWM, RwmGrnChannel);
  ledcAttachPin(P_BLU_LED_PWM, RwmBluChannel);
}

void configPID(){
  /**
   * Configurate the PID controller
   */
  
  objPid.begin(&fTemp, &fTarPwm);
  objPid.addOutputLimits(objConfig.LowLimitManipulation, objConfig.HighLimitManipulation);
  objPid.changeTargetValue(objConfig.CtrlTarget);
  objPid.changePidCoeffs(objConfig.CtrlPropFactor, objConfig.CtrlIntFactor, objConfig.CtrlDifFactor, objConfig.CtrlTimeFactor);

  if (objConfig.LowThresholdActivate) {
    objPid.setOnThres(objConfig.LowThresholdValue);
  }

  objPid.activate(objConfig.CtrlPropActivate, objConfig.CtrlIntActivate, objConfig.CtrlDifActivate);

  // configure PWM functionalitites
  ledcSetup(PwmSsrChannel, objConfig.SsrFreq, objConfig.PwmSsrResolution);
}

void configWebserver(){
  /**
   * Configure and start asynchronous webserver
   */

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });

  // Route for graphs web page
  server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/graphs.html");
  });

  // Route for settings web page
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/settings.html");
  });
  
  // Route for ota web page
  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ota.html");
  });

  // Route for stylesheets.css
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css");
  });

  // Route for java script navigation menu
  server.on("/menubar.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/menubar.js");
  });


  // Measurement file, available under http://coffee.local/data.csv
  server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!bMeasFileLocked){
      bMeasFileLocked = true;
      request->send(SPIFFS, strMeasFilePath, "text/plain");
      bMeasFileLocked = false;
    }
  });

  server.on("/lastvalues.json", HTTP_GET, [](AsyncWebServerRequest *request){
      StaticJsonDocument<500> obj_json_values;
      // send TargetPWM, Temperature, PID-values
      portENTER_CRITICAL_ISR(&objTimerMux);
        obj_json_values["Temperature"] = fTemp;
        
        obj_json_values["PID"]["TargetPWM"] = fTarPwm;
        obj_json_values["PID"]["ErrorIntegrator"] = objPid.getErrorIntegrator();
        obj_json_values["PID"]["TargetValue"] = objPid.getTargetValue();
        obj_json_values["PID"]["ErrorDiff"] = objPid.getErrorDiff();
      portEXIT_CRITICAL_ISR(&objTimerMux);

      String str_response;
      serializeJson(obj_json_values, str_response);
      request->send(200, "application/json", str_response);
    
  });

  // Parameter file, available under http://coffee.local/params.json
  server.on("/params.json", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!bParamFileLocked){
      bParamFileLocked = true;
      request->send(SPIFFS, strParamFilePath, "text/plain");
      bParamFileLocked = false;
    }
  });


  AsyncCallbackJsonWebHandler* obj_handler = new AsyncCallbackJsonWebHandler("/paramUpdate", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<JSON_MEMORY> obj_json;
    obj_json = json.as<JsonObject>();
    
    objConfig.wifiSSID = obj_json["Wifi"]["wifiSSID"].as<String>(); // issue #118 in ArduinoJson
    objConfig.wifiPassword = obj_json["Wifi"]["wifiPassword"].as<String>(); // issue #118 in ArduinoJson
    objConfig.CtrlTimeFactor = obj_json["PID"]["CtrlTimeFactor"];
    objConfig.CtrlPropActivate = obj_json["PID"]["CtrlPropActivate"];
    objConfig.CtrlPropFactor = obj_json["PID"]["CtrlPropFactor"];
    objConfig.CtrlIntActivate = obj_json["PID"]["CtrlIntActivate"];
    objConfig.CtrlIntFactor = obj_json["PID"]["CtrlIntFactor"];
    objConfig.CtrlDifActivate = obj_json["PID"]["CtrlDifActivate"];
    objConfig.CtrlDifFactor = obj_json["PID"]["CtrlDifFactor"];
    objConfig.CtrlTarget = obj_json["PID"]["CtrlTarget"];
    objConfig.LowThresholdActivate = obj_json["PID"]["LowThresholdActivate"];
    objConfig.LowThresholdValue = obj_json["PID"]["LowThresholdValue"];
    objConfig.HighThresholdActivate = obj_json["PID"]["HighThresholdActivate"];
    objConfig.HighTresholdValue = obj_json["PID"]["HighTresholdValue"];
    objConfig.HighLimitManipulation = obj_json["PID"]["HighLimitManipulation"];
    objConfig.LowLimitManipulation = obj_json["PID"]["LowLimitManipulation"];
    objConfig.SsrFreq = obj_json["SSR"]["SsrFreq"];
    objConfig.PwmSsrResolution = obj_json["SSR"]["PwmSsrResolution"];
    objConfig.RwmRgbFreq = obj_json["LED"]["RwmRgbFreq"];
    objConfig.RwmRgbResolution = obj_json["LED"]["RwmRgbResolution"];
    objConfig.SigFilterActive = obj_json["Signal"]["SigFilterActive"];

    if (saveConfiguration()){
      configPID();
      configLED();
      
      if(objConfig.SigFilterActive){
        objAds1115.activateFilter();
      } else {
        objAds1115.deactivateFilter();
      }
      
      request->send(200, "text/plain", "Parameters are updated and changes applied.");
    } else {
      request->send(200, "text/plain", "Parameters are not updated due to write lock");
    }
  });

  server.addHandler(obj_handler);
  
  server.on("/paramReset", HTTP_GET, [](AsyncWebServerRequest *request){
    // initialization of a new parameter file
    resetConfiguration(true);
    configPID();
    configLED();
    request->send(200, "text/plain", "Parameters are set back to default values");
  });

  // OTA Update functionality
  server.on("/ota_firmware", HTTP_POST, [&](AsyncWebServerRequest *request) {
    // definition of response when request is finished (file upload is complete)
    AsyncWebServerResponse *response = request->beginResponse((Update.hasError())?500:200, "text/plain", (Update.hasError())?"Firmware flash FAIL":"Firmware flash OK");
    response->addHeader("Connection", "close");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response); // request->send is called when upload from client is finished
    delay(1000);
    request->redirect("/index.html");
    delay(1000);
    ESP.restart();

    }, [&](AsyncWebServerRequest *ptr_request, String str_filename, size_t i_index, uint8_t *ptr_data, size_t i_len, bool b_final){
          /**
            * @brief (anonymous function) file is devided into chunks/parts and request is called repeatedly
            * 
            * @param ptr_request    pointer object of the request
            * @param str_filename   filename of the file to upload
            * @param i_index        index of the current data chunk
            * @param ptr_data       actual data of the chunk
            * @param i_len          length of the actual data
            * @param b_final        fileupload is completed
            */
        
            if (!i_index){
              // precheck before upload the filestream (index not given)
              if(!ptr_request->hasParam("MD5", true)) {
                // request has no parameter MD5 (Hash value) --> aborting file upload
                return ptr_request->send(400, "text/plain", "MD5 parameter missing");
              } 
              
              if(!Update.setMD5(ptr_request->getParam("MD5", true)->value().c_str())) {
                // MD5 parameter is not a valid format --> aborting file upload
                return ptr_request->send(400, "text/plain", "MD5 parameter invalid");
              } 
              
              // MD5 precheck done, creating temp file on spiffs file system
              if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)){
                // Update cannot be started for reasons
                Update.printError(Serial);
                return ptr_request->send(400, "text/plain", "OTA could not begin");
              }
            }

            if (i_len) {
              if (Update.write(ptr_data, i_len) != i_len){
                // data chunks cannot be written for defined length
                return ptr_request->send(400, "text/plain", "OTA aborted.");
              }
            }

            if (b_final){
              // file download finished, last data chunk is reached
              if (!Update.end(true)) {
                Update.printError(Serial);
                return ptr_request->send(400, "text/plain", "Could not end OTA");
              } else {
                Serial.println("Firmware flash successful. Restart ESP.");
              }
            } else {
              // everything runs smooth in this iteration -> return none
              return;
            }
     });


  server.on("/ota_spiffs", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Parameters are updated to file and applied to system");
    }, [&](AsyncWebServerRequest *ptr_request, String str_filename, size_t i_index, uint8_t *ptr_data, size_t i_len, bool b_final){
          /**
          * @brief (anonymous function) file is devided into chunks/parts and request is called repeatedly
          * 
          * @param ptr_request    pointer object of the request
          * @param str_filename   filename of the file to upload
          * @param i_index        index of the current data chunk
          * @param ptr_data       actual data of the chunk
          * @param i_len          length of the actual data
          * @param b_final        fileupload is completed
          */
          if (!i_index) {
            // open the file on first call and store the file handle in the request object
            ptr_request->_tempFile = SPIFFS.open("/" + str_filename, "w");
            Serial.print("Start uploading file: ");
            Serial.println(str_filename);
          }

          if (i_len) {
            // stream the incoming chunk to the opened file
            ptr_request->_tempFile.write(ptr_data, i_len);
          }

          if (b_final) {
            // close the file handle as the upload is now done
            ptr_request->_tempFile.close();
            ptr_request->redirect("/ota");
            Serial.println("File upload finished.");
          }
    }
    );

  server.on("/restartesp", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ESP is going to restart");
    delay(200);
    ESP.restart();
  });

  // Start web server
  server.begin();
}


bool configADS1115(){
  /**
   * Configure Analog digital converter ADS1115
   */
  
  // Initialize I2c on defined pins with default adress
  if (!objAds1115.begin(SDA_0, SCL_0, ADS1115_I2CADD_DEFAULT)){
    Serial.println("Failed to initialize I2C sensor connection, stop working.");
    return false;
  }

  // Set Signal Filter Status
  if(objConfig.SigFilterActive){
    objAds1115.activateFilter();
  }

  // set differential voltage: A0-A1
  objAds1115.setMux(ADS1115_MUX_AIN0_AIN1);

  // set data rate (samples per second)
  objAds1115.setRate(ADS1115_RATE_8);

  // set to continues conversion method
  objAds1115.setOpMode(ADS1115_MODE_CONTINUOUS);

  // set gain amplifier
  objAds1115.setPGA(ADS1115_PGA_0P256);

  // set latching mode
  objAds1115.setCompLatchingMode(ADS1115_CMP_LAT_ACTIVE);

  // assert after one conversion
  objAds1115.setPinRdyMode(ADS1115_CONV_READY_ACTIVE, ADS1115_CMP_QUE_ASSERT_1_CONV);
  #ifdef Pt1000_CONV_LINEAR
    objAds1115.setPhysConv(fPt1000LinCoeffX1, fPt1000LinCoeffX0);
    Serial.print("Applying linear regression function for Pt1000 conversion: ");
    Serial.print(fPt1000LinCoeffX1, 4);
    Serial.print(" * Umess + ");
    Serial.println(fPt1000LinCoeffX0, 4);
  #endif
  #ifdef Pt1000_CONV_SQUARE
    objAds1115.setPhysConv(fPt1000SquareCoeffX2, fPt1000SquareCoeffX1, fPt1000SquareCoeffX0);
    Serial.print("Applying square regression function for Pt1000 conversion: ");
    Serial.print(fPt1000SquareCoeffX2, 4);
    Serial.print(" * Umess² + ");
    Serial.print(fPt1000SquareCoeffX1, 4);
    Serial.print(" * Umess + ");
    Serial.println(fPt1000SquareCoeffX0);
  #endif
  #ifdef Pt1000_CONV_LOOK_UP_TABLE
    const size_t size_1d_map = sizeof(arrPt1000LookUpTbl) / sizeof(arrPt1000LookUpTbl[0]);
    objAds1115.setPhysConv(arrPt1000LookUpTbl, size_1d_map);
    Serial.println("Applying Lookuptable for Pt1000 conversion:");
    for(int i_row=0; i_row<size_1d_map; i_row++){
      Serial.print(arrPt1000LookUpTbl[i_row][0], 4);
      Serial.print("   ");
      Serial.println(arrPt1000LookUpTbl[i_row][1], 4);
    }
  #endif

  objAds1115.printConfigReg();
  return true;
}

void setup(){
  // Initialize Serial port for debugging purposes
  Serial.begin(115200);
  delay(50);
  Serial.println("Starting setup.");

  // turn green status LED on
  pinMode(P_STAT_LED, OUTPUT);
  digitalWrite(P_STAT_LED, HIGH);

  
  // configure RGB-LED PWM output (done early so error codes can be outputted via LED)
  configLED();
  setColor(100, 100, 100); // White

  // initialize SPIFFs and load configuration files
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      // Initialization of SPIFFS failed, restart it
      Serial.println("SPIFFS mount Failed, restart ESP");
      delay(1000);
      ESP.restart();
  } else {
    Serial.println("SPIFFS mount successfully.");
    // initialize configuration before load json file
    resetConfiguration(false);
    
    // load configuration from file in eeprom
    if (!loadConfiguration()) {
      Serial.println("Parameter file is locked on startup. Please reset to factory settings.");
    }
  }

  // Initialization successfull, create csv file
  unsigned int i_total_bytes = SPIFFS.totalBytes();
  unsigned int i_used_bytes = SPIFFS.usedBytes();

  Serial.println("File system info:");
  Serial.print("Total space on SPIFFS: ");
  Serial.print(i_total_bytes);
  Serial.println(" bytes");

  Serial.print("Total space used on SPIFFS: ");
  Serial.print(i_used_bytes);
  Serial.println(" bytes");
  Serial.println("");

  // Connect to wifi and create time stamp if device is Online
  bEspOnline = connectWiFi();
  char char_timestamp[50];

  if (bEspOnline == true) {
    // ESP has wifi connection

    // Define reconnect action when disconnecting from Wifi
    WiFi.onEvent(reconnectWiFi, SYSTEM_EVENT_STA_DISCONNECTED);

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

    // print location to measurement file
    Serial.print("Create File ");
    Serial.print(WiFi.localIP());
    Serial.println(strMeasFilePath);
  
  } else {
    // No wifi connection possible start SoftAP
    Serial.print("Connection to SSID '");
    Serial.print(objConfig.wifiSSID);
    Serial.println("' not possible. Making Soft-AP with SSID 'SilviaCoffeeCtrl'");
    WiFi.softAP("SilviaCoffeeCtrl");
    
    // print location to measurement file
    Serial.print("Create File ");
    Serial.print(WiFi.softAPIP());
    Serial.println(strMeasFilePath);

    // set RGB-LED to purple to user knows whats up
    setColor(170, 0, 255);   // Purple 
  }

  // configure and start webserver
  configWebserver();

  // register mDNS. ESP is available under http://coffee.local
  if (!MDNS.begin("coffee")) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    // add service to standart http connection
    MDNS.addService("http", "tcp", 80);
  }

  // Write Measurement file header
  if (!bMeasFileLocked){
    bMeasFileLocked = true;
    File obj_meas_file = SPIFFS.open(strMeasFilePath, "w");

    obj_meas_file.print("Measurement File created on ");
    obj_meas_file.println(char_timestamp);
    obj_meas_file.println("");
    obj_meas_file.println("Time,Temperature,TargetPWM,FilterStatus");
    obj_meas_file.close();
    bMeasFileLocked = false;
  } else {
    Serial.println("Could not create measurement file due to active Lock");
  }

  // configure ADS1115
  if(!configADS1115()) {
    // TODO add diagnosis when ADS1115 is not connected
    Serial.println("ADS1115 configuration not successful. System halt.");
  }

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

  // Configure PID library
  configPID();
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(P_SSR_PWM, PwmSsrChannel);

}// Setup

boolean readSensors(){
  // Read out Sensor values
  bool b_result = false;
  // get physical value of sensor
  fTemp = objAds1115.getPhysVal();
  // get voltage level of sensor

  b_result = true;
  return b_result;
} //readSensors


bool controlHeating(){
  /** PID regulator function for controling heating device
   *
   */

  bool b_success = false;

  objPid.compute();
  ledcWrite(PwmSsrChannel, (int)fTarPwm);

  b_success = true;
  return b_success;

} // controlHeating

bool writeMeasFile(){
  /** Function to store the data in measurement file
   *
   */

  bool b_success = false;
  portENTER_CRITICAL_ISR(&objTimerMux);
    float f_temp_local = fTemp;
    float f_tar_pwm = fTarPwm;
    bool b_filter_status = objAds1115.getFilterStatus();
  portEXIT_CRITICAL_ISR(&objTimerMux);
  
  if (!bMeasFileLocked){
    bMeasFileLocked = true;
    File obj_meas_file = SPIFFS.open(strMeasFilePath, "a");
    float f_time = (float)(millis() - iTimeStart) / 1000.0;
    obj_meas_file.print(f_time, 4);
    obj_meas_file.print(",");
    obj_meas_file.print(f_temp_local);
    obj_meas_file.print(",");
    obj_meas_file.print(f_tar_pwm);
    obj_meas_file.print(",");
    obj_meas_file.println(b_filter_status);
    obj_meas_file.close();
    b_success = true;
    bMeasFileLocked = false;
  } else {
    b_success = false;
  }
  return b_success;
}// writeMeasFile


void setColor(int i_red_value, int i_green_value, int i_blue_value) {
  /** Function to output a RGB value to the LED
   * examples: (from https://www.rapidtables.com/web/color/RGB_Color.html)
   * setColor(255, 0, 0);     // Red 
   * setColor(0, 255, 0);     // Green 
   * setColor(0, 0, 255);     // Blue 
   * setColor(255, 255, 255); // White 
   * setColor(170, 0, 255);   // Purple 
   * setColor(255,10,0);     // Orange 
   * setColor(255,20,0);     // Yellow 
   *
   * @param i_red_value    [0, 255] Red RGB value
   * @param i_green_value  [0, 255] Green RGB value
   * @param i_blue_value   [0, 255] Blue RGB value
   */
  ledcWrite(RwmRedChannel, i_red_value);
  ledcWrite(RwmGrnChannel, i_green_value);
  ledcWrite(RwmBluChannel, i_blue_value);

}// setColor

void loop(){
  if (iStatusCtrl == MEASURE) {
    // Call sensor read out function
    bool b_result_sensor;
    b_result_sensor = readSensors();

    if (b_result_sensor == true) {
      portENTER_CRITICAL_ISR(&objTimerMux);
        if(objAds1115.getConnectionStatus()){
          // only control if connection to ADS1115 is successful
          iStatusCtrl = CONTROL;
        }
      portEXIT_CRITICAL_ISR(&objTimerMux);
    }
  }

  if (iStatusLED == LED_SET) {

    if(objAds1115.getConnectionStatus()){
          // only check for frozen values if connection to ADS1115 is successful
          if(objAds1115.isValueFrozen()){
            setColor(170, 0, 255);
            Serial.println("ADS1115 Sensor value frozen");
            // configure ADS1115 again
            //configADS1115(); // error occured reconfigure ADC?

          }
          else{
            // sensor is OK-> displya heating status

            if (fTemp < objConfig.CtrlTarget - 1.0) {
              // Heat up signal
              setColor(255,15,0);     // orange
            } 
            else if (fTemp > objConfig.CtrlTarget + 1.0){
              // Cool down signal
              setColor(0,0,5);     // blue
            }
            else {
              // temperature in range signal
              setColor(0,5,0);     // green 
            }
          }
        }// if objAds1115.getConnectionStatus()

    portENTER_CRITICAL_ISR(&objTimerMux);
      iStatusLED = LED_IDLE;
    portEXIT_CRITICAL_ISR(&objTimerMux);
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
      bool b_result_meas_write;
      b_result_meas_write = writeMeasFile();
      portENTER_CRITICAL_ISR(&objTimerMux);
        if (b_result_meas_write){
          iStatusMeas = IDLE_MEAS;
        }
      portEXIT_CRITICAL_ISR(&objTimerMux);
  }
}// loop

