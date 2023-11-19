/*********
 * 
 * coffee_ctrl_main
 * A script to control an espresso machine and display measurement values on a webserver of an ESP32
 * 
*********/
#define LOG_LEVEL ESP_LOG_VERBOSE

// PIN definitions

// I2C pins
#define SDA_0 23
#define SCL_0 22
#define CONV_RDY_PIN 14

// PWM defines
#define P_SSR_PWM 21
#define P_RED_LED_PWM 13
#define P_GRN_LED_PWM 27
#define P_BLU_LED_PWM 12
#define P_STAT_LED 33 // green status LED

// File system definitions
#define FORMAT_SPIFFS_IF_FAILED true
#define JSON_MEMORY 1600


// define timer related channels for PWM signals
#define RwmRedChannel 13 //  PWM channel. There are 16 channels from 0 to 15. Channel 13 is now Red-LED
#define RwmGrnChannel 14 //  PWM channel. There are 16 channels from 0 to 15. Channel 14 is now Green-LED
#define RwmBluChannel 15 //  PWM channel. There are 16 channels from 0 to 15. Channel 15 is now Blue-LED
#define PwmSsrChannel 0  //  PWM channel. There are 16 channels from 0 to 15. Channel 0 is now SSR-Controll

#define WDT_Timeout 75 // WatchDog Timeout in seconds

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include "FS.h"
#include <LittleFS.h>
#include <time.h>
#include <string>
#include "Pt1000.h"
#include "ADS1115.h"
#include "PidCtrl.h"
#include "WifiAccess.h"
#include <ArduinoJson.h>
#include "AsyncJson.h"
#include <Update.h>
#include <esp_task_wdt.h>
#include "ota.h"
#include "esp32-hal-log.h"


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
  uint32_t SsrFreq;
  uint32_t PwmSsrResolution;
  uint32_t RwmRgbFreq;
  uint32_t RwmRgbResolution; 
  float RwmRgbGainFactorRed;
  float RwmRgbGainFactorGreen;
  float RwmRgbGainFactorBlue;
  float RwmRgbColorRedFactor;
  float RwmRgbColorGreenFactor;
  float RwmRgbColorBlueFactor;
  float RwmRgbColorOrangeFactor;
  float RwmRgbColorPurpleFactor;
  float RwmRgbColorWhiteFactor;
  bool SigFilterActive;
  uint32_t TimeToStandby;
};


// File paths for measurement and calibration file
const char* strMeasFilePath = "/data.csv";
bool bMeasFileLocked = false;
const char* strParamFilePath = "/params.json";
bool bParamFileLocked = false;
const char* strRecentLogFilePath = "/logfile_recent.txt";
const char* strLastLogFilePath = "/logfile_last.txt";
static char bufPrintLog[512];
const char* strUserLogLabel = "USER";

// start time for measurement
unsigned long iTimeStart = 0;

// define target PWM
float fTarPwm;

// define configuration struct
config objConfig;

// global variabel for WiFi strength
int iDbmPercentage = 0;

// Sensor variables
float fTime = 0.F;
float fTemp = 0.F;

// bit variable to indicate whether ESP32 has a online connection
bool bEspOnline = false;
bool bEspMdns = false;

// configure NTP client
const char* charNtpServerUrl = "europe.pool.ntp.org";
const long  iGmtOffsetSec = 3600; // UTC for germany +1h = 3600s
const int   iDayLightOffsetSec = 3600; //s Time change in germany 1h = 3600s

// Initialize ADS1115 I2C connection
ADS1115 *objADS1115 = new ADS1115;

// timer object for ISR
hw_timer_t * objTimerLong = NULL;

// define Counter for interrupt handling
volatile unsigned long iInterruptCntLong = 0;
volatile unsigned long iInterruptCntAlert = 0;
volatile unsigned long iInterruptCntAlertCatch = 0;

// soft Sleep mode
bool bSleepMode = false; 

enum eState{
  IDLE      = (1u << 0),
  MEASURE   = (1u << 1),
  PID_CTRL  = (1u << 2),
  STORE     = (1u << 3),
  DIAG      = (1u << 4),
  LED_CTRL  = (1u << 5)
};

enum eError{
  NO_ERROR          = (1u << 0),
  TEMP_OUT_RANGE    = (1u << 1),
  TEMP_FROZEN       = (1u << 2),
  MEAS_DEV_RESET    = (1u << 3),
  WIFI_DISCONNECT   = (1u << 4),
  };

enum eLEDColor{
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
  LED_COLOR_ORANGE,
  LED_COLOR_PURPLE,
  LED_COLOR_WHITE,
  LED_COLOR_SLEEP
};

// Initialisation of PID controler
PidCtrl objPid;

// Definition for critical section port
portMUX_TYPE objTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Task intervals in microseconds
const unsigned long iInterruptLongIntervalMicros = 450000; //microseconds

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Status for function call timing, used in ISR
volatile unsigned int iState = IDLE;

int iErrorId = NO_ERROR;

// timers ==============================================================================================================
void IRAM_ATTR onAlertRdy(){
  /** Interrupt Service Routine for sensor read outs
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
    // Perform Measurement on interrupt call
    iState |= MEASURE;

    if (iInterruptCntAlert % 2 == 0) {
        // Only apply control on every second interrupt
        iState |= PID_CTRL;
    }
    iInterruptCntAlert++;
    portEXIT_CRITICAL_ISR(&objTimerMux);
}

void IRAM_ATTR onTimerLong(){
  /** ISR triggered by timer alarm. Used for Store Measurement file, LED-Set and DIAG-Function
   *  Info: IRAM_ATTR stores function in RAM instead of flash memory, faster.
  **/

  // Define Critical Code section, also needs to be called in Main-Loop
    portENTER_CRITICAL_ISR(&objTimerMux);
      // Only change Status when idle to measurement running
      if (iInterruptCntLong % 1 == 0) {
        iState |= STORE;
      }

      if (iInterruptCntLong % 3 == 0) {
        // Only write LED value when interrupt function is called 5 times
        iState |= LED_CTRL;
      }

      if (iInterruptCntLong % 2 == 0){
        iState |= DIAG;
      }
    portEXIT_CRITICAL_ISR(&objTimerMux);

    // Interrupt counter
    iInterruptCntLong++;
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
  
  //WiFi.disconnect(true);
  //delay(100);

  esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Device %s try connecting to %s\n", WiFi.macAddress().c_str(), objConfig.wifiSSID.c_str());

  int i_run_cnt_fail = 0;
  int i_wifi_status;

  WiFi.mode(WIFI_STA);

  // Connect to WPA/WPA2 network:
  WiFi.begin(objConfig.wifiSSID.c_str(), objConfig.wifiPassword.c_str());
  i_wifi_status = WiFi.status();

  while ((i_wifi_status != WL_CONNECTED) && (i_run_cnt_fail<i_total_fail)) {
    // wait for connection establish
    vTaskDelay(i_timout_attemp/portTICK_PERIOD_MS);
    i_run_cnt_fail++;
    i_wifi_status = WiFi.status();
    esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Connection Attemp: %d\n", i_run_cnt_fail);
  }

  if (i_wifi_status == WL_CONNECTED) {
      // Print ESP32 Local IP Address
      esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Connection successful. Local IP: %s\n", WiFi.localIP().toString().c_str());
      // Signal strength and approximate conversion to percentage
      int i_dBm = WiFi.RSSI();
      calcWifiStrength(i_dBm);

      esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Signal strength: %d dB -> %d %%\n", i_dBm, iDbmPercentage);
      b_successful = true;
      WiFi.setAutoReconnect(true);
  } else {
    esp_log_write(ESP_LOG_WARN, strUserLogLabel,  "Connection unsuccessful. WiFi status: %d\n", i_wifi_status);
  }
  
  return b_successful;
} // connectWiFi

void calcWifiStrength(int i_dBm){
  /**
   * calculate the wifi strength from dB to % 
   */
  iDbmPercentage = 0;
  if (i_dBm>=-50) {
    iDbmPercentage = 100;
  } else if (i_dBm<=-100) {
    iDbmPercentage = 0;
  } else {
    iDbmPercentage = 2*(i_dBm+100);
  }
}//getWifiStreng

bool loadConfiguration(){
  /**
   * Load configuration from configuration file 
   */
  
  bool b_success = false;

  if (!bParamFileLocked){
    // file is not locked by another process ->  save to read or write
    bParamFileLocked = true;
    File obj_param_file = LittleFS.open(strParamFilePath, "r");
    StaticJsonDocument<JSON_MEMORY> json_doc;
    DeserializationError error = deserializeJson(json_doc, obj_param_file);

    if ((!obj_param_file) || (error)){
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Failed to read file, using default configuration.\n");
      
      if (!obj_param_file) {
        esp_log_write(ESP_LOG_INFO, strUserLogLabel, "File could not be opened.\n");
      }
    
      if (error) {
        //esp_log_write(ESP_LOG_ERROR, strUserLogLabel, "JSON deserializion error: %*c\n", error.c_str());
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
      (json_doc["LED"]["GainFactorRed"])?objConfig.RwmRgbGainFactorRed = json_doc["LED"]["GainFactorRed"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorGreen"])?objConfig.RwmRgbGainFactorGreen = json_doc["LED"]["GainFactorGreen"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorBlue"])?objConfig.RwmRgbGainFactorBlue = json_doc["LED"]["GainFactorBlue"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorRed"])?objConfig.RwmRgbColorRedFactor = json_doc["LED"]["GainFactorColorRed"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorGreen"])?objConfig.RwmRgbColorGreenFactor = json_doc["LED"]["GainFactorColorGreen"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorBlue"])?objConfig.RwmRgbColorBlueFactor = json_doc["LED"]["GainFactorColorBlue"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorOrange"])?objConfig.RwmRgbColorOrangeFactor = json_doc["LED"]["GainFactorColorOrange"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorPurple"])?objConfig.RwmRgbColorPurpleFactor = json_doc["LED"]["GainFactorColorPurple"]:b_set_default_values = true;
      (json_doc["LED"]["GainFactorColorWhite"])?objConfig.RwmRgbColorWhiteFactor = json_doc["LED"]["GainFactorColorWhite"]:b_set_default_values = true;
      (json_doc["Signal"]["SigFilterActive"])?objConfig.SigFilterActive = json_doc["Signal"]["SigFilterActive"]:b_set_default_values = true;
      (json_doc["System"]["TimeToStandby"])?objConfig.TimeToStandby = json_doc["System"]["TimeToStandby"]:b_set_default_values = true;

      if (b_set_default_values){
        // default values are set to Json object -> write it back to file.
        if (!saveConfiguration()){
          esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Cannot write back to JSON file.\n");
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
  json_doc["LED"]["GainFactorRed"] = objConfig.RwmRgbGainFactorRed;
  json_doc["LED"]["GainFactorGreen"] = objConfig.RwmRgbGainFactorGreen;
  json_doc["LED"]["GainFactorBlue"] = objConfig.RwmRgbGainFactorBlue;
  json_doc["LED"]["GainFactorColorRed"] = objConfig.RwmRgbColorRedFactor;
  json_doc["LED"]["GainFactorColorGreen"] = objConfig.RwmRgbColorGreenFactor;
  json_doc["LED"]["GainFactorColorBlue"] = objConfig.RwmRgbColorBlueFactor;
  json_doc["LED"]["GainFactorColorOrange"] = objConfig.RwmRgbColorOrangeFactor;
  json_doc["LED"]["GainFactorColorPurple"] = objConfig.RwmRgbColorPurpleFactor;
  json_doc["LED"]["GainFactorColorWhite"] = objConfig.RwmRgbColorWhiteFactor;
  json_doc["Signal"]["SigFilterActive"] = objConfig.SigFilterActive;
  json_doc["System"]["TimeToStandby"] = objConfig.TimeToStandby;

  if (!bParamFileLocked){
    bParamFileLocked = true;
    File obj_param_file = LittleFS.open(strParamFilePath, "w");
  
    if (serializeJsonPretty(json_doc, obj_param_file) == 0) {
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Failed to write configuration to file\n");
    }
    else{
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Configuration file updated successfully\n");
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
  objConfig.CtrlTarget = 85.0;
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
  objConfig.RwmRgbGainFactorRed = 1.0;
  objConfig.RwmRgbGainFactorGreen = 1.0;
  objConfig.RwmRgbGainFactorBlue = 1.0;
  objConfig.RwmRgbColorRedFactor = 1.0;
  objConfig.RwmRgbColorGreenFactor = 1.0;
  objConfig.RwmRgbColorBlueFactor = 1.0;
  objConfig.RwmRgbColorOrangeFactor = 1.0;
  objConfig.RwmRgbColorPurpleFactor = 1.0;
  objConfig.RwmRgbColorWhiteFactor = 1.0;
  objConfig.SigFilterActive = true;
  objConfig.TimeToStandby =  3600; //s

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

  if (objConfig.HighThresholdActivate) {
    objPid.setOffThres(objConfig.HighTresholdValue);
  }

  objPid.activate(objConfig.CtrlPropActivate, objConfig.CtrlIntActivate, objConfig.CtrlDifActivate);

  // configure PWM functionalitites
  ledcSetup(PwmSsrChannel, objConfig.SsrFreq, objConfig.PwmSsrResolution);
}

void configWebserver(){
  /**
   * Configure and start asynchronous webserver
   */

  // favicons in LitleFs have to be included in the server 
  //TODO maybe not static?
  server.serveStatic("/favicon-32x32.png", LittleFS, "favicon-32x32.png");
  server.serveStatic("/apple-touch-icon.png", LittleFS, "apple-touch-icon.png");

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html");
  });

  // Route for graphs web page
  server.on("/graphs.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/graphs.html");
  });

  // Route for settings web page
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/settings.html");
  });
  
  // Route for ota web page
  server.on("/ota.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/ota.html");
  });

  // Route for ota web page
  server.on("/log.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/log.html");
  });

  // Route for ota web page
  server.on("/failsafe", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", ota_html_gz, ota_html_gz_len);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  // Route for stylesheets.css
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/style.css");
  });

  // Measurement file, available under http://coffee.local/data.csv
  server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, strMeasFilePath, "text/plain");
  });

  // Current log file
  server.on("/recentlogfile.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, strRecentLogFilePath, "text/plain");
  });

  // Log file from previous session
  server.on("/lastlogfile.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, strLastLogFilePath, "text/plain");
  });

  server.on("/lastvalues.json", HTTP_GET, [](AsyncWebServerRequest *request){
      StaticJsonDocument<200> obj_json_values;
      // send TargetPWM, Temperature, PID-values
      portENTER_CRITICAL_ISR(&objTimerMux);
        obj_json_values["Time"] = fTime;
        obj_json_values["Temperature"] = fTemp;
        
        obj_json_values["PID"]["TargetValue"] = objPid.getTargetValue();
        obj_json_values["PID"]["TargetPWM"] = fTarPwm;
        obj_json_values["PID"]["ErrorIntegrator"] = objPid.getErrorIntegrator();
        obj_json_values["PID"]["ErrorDiff"] = objPid.getErrorDiff();

        obj_json_values["WiFi"]["SignalStrength in %"] = iDbmPercentage;

      portEXIT_CRITICAL_ISR(&objTimerMux);

      String str_response;
      serializeJson(obj_json_values, str_response);
      request->send(200, "application/json", str_response);
    
  });

  // Parameter file, available under http://coffee.local/params.json
  server.on("/params.json", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, strParamFilePath, "text/plain");
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
    objConfig.RwmRgbGainFactorRed = obj_json["LED"]["GainFactorRed"];
    objConfig.RwmRgbGainFactorGreen = obj_json["LED"]["GainFactorGreen"];
    objConfig.RwmRgbGainFactorBlue = obj_json["LED"]["GainFactorBlue"];
    objConfig.RwmRgbColorRedFactor = obj_json["LED"]["GainFactorColorRed"];
    objConfig.RwmRgbColorGreenFactor = obj_json["LED"]["GainFactorColorGreen"];
    objConfig.RwmRgbColorBlueFactor = obj_json["LED"]["GainFactorColorBlue"];
    objConfig.RwmRgbColorOrangeFactor = obj_json["LED"]["GainFactorColorOrange"];
    objConfig.RwmRgbColorPurpleFactor = obj_json["LED"]["GainFactorColorPurple"];
    objConfig.RwmRgbColorWhiteFactor = obj_json["LED"]["GainFactorColorWhite"];
    objConfig.SigFilterActive = obj_json["Signal"]["SigFilterActive"];
    objConfig.TimeToStandby = obj_json["System"]["TimeToStandby"];

    if (saveConfiguration()){
      configPID();
      configLED();
      
      if(objConfig.SigFilterActive){
        objADS1115->activateFilter();
      } else {
        objADS1115->deactivateFilter();
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
              
              // MD5 precheck done, creating temp file on LittleFS file system
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
                esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Firmware flash successful. Restart ESP.\n");
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
            ptr_request->_tempFile = LittleFS.open("/" + str_filename, "w");
            esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Start uploading file: %s\n", str_filename);
          }

          if (i_len) {
            // stream the incoming chunk to the opened file
            ptr_request->_tempFile.write(ptr_data, i_len);
          }

          if (b_final) {
            // close the file handle as the upload is now done
            ptr_request->_tempFile.close();
            ptr_request->redirect("/ota.html");
            esp_log_write(ESP_LOG_INFO, strUserLogLabel, "File upload finished.\n"); 
          }
    }
    );

  server.on("/restartesp", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ESP is going to restart");
    delay(2000);
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
  if (!objADS1115->begin(SDA_0, SCL_0, ADS1115_I2CADD_DEFAULT)){
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Failed to initialize I2C sensor connection, stop working.\n");
    return false;
  }

  // Set Signal Filter Status
  if(objConfig.SigFilterActive){
    objADS1115->activateFilter();
  }

  // set Comparator Polarity to active high
  objADS1115->setCompPolarity(ADS1115_CMP_POL_ACTIVE_HIGH);

  // set differential voltage: A0-A1
  objADS1115->setMux(ADS1115_MUX_AIN0_AIN1);

  // set data rate (samples per second)
  objADS1115->setRate(ADS1115_RATE_8);


  #ifdef Pt1000_CONV_LINEAR
    objADS1115->setPhysConv(fPt1000LinCoeffX1, fPt1000LinCoeffX0);
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Applying linear regression function for Pt1000 conversion: %.4f * Umess + %.4f\n", fPt1000LinCoeffX1, fPt1000LinCoeffX0);
  #endif
  #ifdef Pt1000_CONV_SQUARE
    objADS1115->setPhysConv(fPt1000SquareCoeffX2, fPt1000SquareCoeffX1, fPt1000SquareCoeffX0);
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Applying square regression function for Pt1000 conversion: \
                               %.4f * Umess^2 + %.4f * Umess + %.4f\n", fPt1000SquareCoeffX2, fPt1000SquareCoeffX1, fPt1000SquareCoeffX0);
  #endif
  #ifdef Pt1000_CONV_LOOK_UP_TABLE
    const size_t size_1d_map = sizeof(arrPt1000LookUpTbl) / sizeof(arrPt1000LookUpTbl[0]);
    objADS1115->setPhysConv(arrPt1000LookUpTbl, size_1d_map);
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Applying lookup table for Pt1000 conversion:\n");
    for(int i_row=0; i_row<size_1d_map; i_row++){
      esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "%.4f    %.4f\n", arrPt1000LookUpTbl[i_row][0], arrPt1000LookUpTbl[i_row][1]);
    }
  #endif

  // set gain amplifier
  objADS1115->setPGA(ADS1115_PGA_0P512);

  // set latching mode
  objADS1115->setCompLatchingMode(ADS1115_CMP_LAT_ACTIVE);

  // assert after one conversion
  objADS1115->setPinRdyMode(ADS1115_CONV_READY_ACTIVE, ADS1115_CMP_QUE_ASSERT_1_CONV);

  // set to continues conversion method
  objADS1115->setOpMode(ADS1115_MODE_CONTINUOUS);
  
  objADS1115->printConfigReg();
  return true;
}


int vprintf_into_FS(const char* szFormat, va_list args) {
	//write evaluated format string into buffer
	int i_ret = vsnprintf (bufPrintLog, sizeof(bufPrintLog), szFormat, args);

	//output is now in buffer. write to file.
	if(i_ret >= 0) {
    if(!LittleFS.exists(strRecentLogFilePath)) {
      // Create logfile if it does not exist.
      File writeLog = LittleFS.open(strRecentLogFilePath, FILE_WRITE);
      if(!writeLog) Serial.println("Couldn't open log file"); 
      delay(50);
      writeLog.close();
    }
    
		File LogFile = LittleFS.open(strRecentLogFilePath, FILE_APPEND);
		//debug output
		LogFile.write((uint8_t*) bufPrintLog, (size_t) i_ret);
		//flush print log, to make sure message is written to file.
		LogFile.flush();
		LogFile.close();
	}
	return i_ret;
}


void setup(){
  // Initialize Serial port for debugging purposes
  Serial.begin(115200);
  delay(50);

  // initialize LittleFS and load configuration files
  if(!LittleFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    // Initialization of LittleFS failed, restart it
    Serial.println("LittleFS mount Failed, restart ESP.");
    delay(1000);
    ESP.restart();
  } else {
    // Move last log file
    if (LittleFS.exists(strLastLogFilePath)){
      LittleFS.remove(strLastLogFilePath);
    }
    
    if (LittleFS.exists(strRecentLogFilePath)){
      LittleFS.rename(strRecentLogFilePath, strLastLogFilePath);
    }

    // Link logging output to function
    esp_log_set_vprintf(&vprintf_into_FS);
    // Verbose output level
    esp_log_level_set("*", LOG_LEVEL);
    esp_log_level_set("wifi", LOG_LEVEL);
    esp_log_level_set(strUserLogLabel, ESP_LOG_VERBOSE);
    
    esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "\n\n-----------------------------------Starting Logging.\n");
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "LittleFS mount successfully.\n");
    unsigned int i_reset_reason = esp_reset_reason();
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Last reset reason: %d\n", i_reset_reason);
    // initialize configuration before load json file
    resetConfiguration(false);
    
    // load configuration from file in eeprom
    if (!loadConfiguration()) {
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Parameter file is locked on startup. Please reset to factory settings.\n");
    }
  }

  // Initialization successfull, create csv file
  unsigned int i_total_bytes = LittleFS.totalBytes();
  unsigned int i_used_bytes = LittleFS.usedBytes();

  esp_log_write(ESP_LOG_INFO, strUserLogLabel, "File system info:\n");
  esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Total space on LittleFS: %d bytes\n", i_total_bytes);
  esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Total space used on LittleFS: %d bytes\n", i_used_bytes);

  // turn green status LED on
  pinMode(P_STAT_LED, OUTPUT);
  digitalWrite(P_STAT_LED, HIGH);
  
  // configure RGB-LED PWM output (done early so error codes can be outputted via LED)
  configLED();
  setColor(LED_COLOR_WHITE, true); // White

  // Connect to wifi and create time stamp if device is Online
  bEspOnline = connectWiFi(3, 6000);
  char char_timestamp[50];

  if (bEspOnline == true) {
    // ESP has wifi connection

    // initialize NTP client
    configTime(iGmtOffsetSec, iDayLightOffsetSec, charNtpServerUrl);

    // get local time
    struct tm obj_timeinfo;
    if(!getLocalTime(&obj_timeinfo)){
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Failed to obtain time stamp online\n");
    } else {
      // write time stamp into variable
      strftime(char_timestamp, sizeof(char_timestamp), "%s", &obj_timeinfo);
    }
    // print location to measurement file
    esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Create File %s%s\n", WiFi.localIP().toString().c_str(), strMeasFilePath);
  } else {
    // No wifi connection possible start SoftAP

    esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "Connection to SSID '%s' not possible. Making Soft-AP with SSID 'SilviaCoffeeCtrl'\n", objConfig.wifiSSID.c_str());
    WiFi.softAP("SilviaCoffeeCtrl");
    
    // print location to measurement file
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Create file %s%s\n", WiFi.softAPIP().toString().c_str(), strMeasFilePath);

    // set RGB-LED to purple to user knows whats up
    setColor(LED_COLOR_PURPLE, false); 
  }

  // configure and start webserver
  configWebserver();

  // register mDNS. ESP is available under http://coffee.local
  if (!MDNS.begin("coffee")) {
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "Error setting up MDNS responder!\n");
  } else {
    // add service to standart http connection
    MDNS.addService("http", "tcp", 80);
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "MDNS responder successfully initialized.\n");
  }

  // configure ADS1115
  if(!configADS1115()) {
    // TODO add diagnosis when ADS1115 is not connected
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "ADS1115 configuration not successful.\n");
    iErrorId |= MEAS_DEV_RESET;
  } else {
    esp_log_write(ESP_LOG_INFO, strUserLogLabel, "ADS initialized successfully.\n");
  }

  // Write Measurement file header
  File obj_meas_file = LittleFS.open(strMeasFilePath, "w");
  uint16_t i_config_reg = objADS1115->getRegisterValue(ADS1115_CONFIG_REG);
  uint16_t i_low_reg = objADS1115->getRegisterValue(ADS1115_LOW_THRESH_REG);
  uint16_t i_high_reg = objADS1115->getRegisterValue(ADS1115_HIGH_THRESH_REG);
  obj_meas_file.print("Measurement File created on ");
  obj_meas_file.println(char_timestamp);
  obj_meas_file.println("ADS1115 Register Settings");
  obj_meas_file.print("Config Register: 0b");
  obj_meas_file.println(i_config_reg, BIN);
  obj_meas_file.print("Low Threshold Register: 0b");
  obj_meas_file.println(i_low_reg, BIN);
  obj_meas_file.print("High Threshold Register: 0b");
  obj_meas_file.println(i_high_reg, BIN);
  
  obj_meas_file.println("");
  obj_meas_file.println("Time,Temperature,TargetPWM,Buffer,InterruptCountAlertReady");
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

  // Enable Hardware Watchdog with Kernel panic reaction
  esp_task_wdt_init(WDT_Timeout, true);
  esp_task_wdt_add(NULL); // add current thread to watchdog

  // Configure PID library
  configPID();
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(P_SSR_PWM, PwmSsrChannel);
  esp_log_write(ESP_LOG_INFO, strUserLogLabel, "JUMP to main loop.\n");

}// Setup

bool controlHeating(){
  /** PID regulator function for controling heating device
   *
   */

  bool b_success = false;
  if ((!bSleepMode) && (iErrorId == NO_ERROR)) {
    objPid.compute();
    b_success = true;
  } else if (bSleepMode){
    fTarPwm = 0.F;
    b_success = true;
  } else {
    fTarPwm = 0.F;
    b_success = false;
  }
  ledcWrite(PwmSsrChannel, (int)fTarPwm);

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
  float f_time = fTime;
  portEXIT_CRITICAL_ISR(&objTimerMux);
  
  File obj_meas_file = LittleFS.open(strMeasFilePath, "a");
  obj_meas_file.print(f_time, 3);
  obj_meas_file.print(",");
  obj_meas_file.print(f_temp_local);
  obj_meas_file.print(",");
  obj_meas_file.println(f_tar_pwm);
  obj_meas_file.close();
  b_success = true;
  return b_success;
}// writeMeasFile


void setColor(int i_color, bool b_gain_active) {
  /** Function to output a RGB value to the LED
   * examples: (from https://www.rapidtables.com/web/color/RGB_Color.html)
   * setColor(255, 0, 0);     // Red 
   * setColor(0, 255, 0);     // Green 
   * setColor(0, 0, 255);     // Blue 
   * setColor(255, 255, 255); // White 
   * setColor(170, 0, 255);   // Purple 
   * setColor(255,10,0);      // Orange 
   * setColor(255,20,0);      // Yellow 
   *
   * @param i_color        color to set for the RGB LED
   * @param b_gain_active  gain factor is used for displaying LED
   */

  float f_red_value = 0.0F;
  float f_green_value = 0.0F;
  float f_blue_value = 0.0F;
  float f_max_resolution = (float)(1<<objConfig.RwmRgbResolution)-1.0F;

  if (i_color == LED_COLOR_RED){
    f_red_value = 255.F * objConfig.RwmRgbColorRedFactor;
  } else if (i_color == LED_COLOR_BLUE) {
    f_blue_value = 255.F * objConfig.RwmRgbColorBlueFactor;
  } else if (i_color == LED_COLOR_GREEN) {
    f_green_value = 255.F * objConfig.RwmRgbColorGreenFactor;
  } else if (i_color == LED_COLOR_ORANGE) {
    f_red_value = 255.F * objConfig.RwmRgbColorOrangeFactor;
    f_green_value = 10.F * objConfig.RwmRgbColorOrangeFactor;
  } else if (i_color == LED_COLOR_PURPLE) {
    f_red_value = 170.F * objConfig.RwmRgbColorPurpleFactor;
    f_blue_value = 255.F * objConfig.RwmRgbColorPurpleFactor;
  } else if (i_color == LED_COLOR_WHITE) {
    f_red_value = 100.F * objConfig.RwmRgbColorWhiteFactor;
    f_green_value = 100.F * objConfig.RwmRgbColorWhiteFactor;
    f_blue_value = 100.F * objConfig.RwmRgbColorWhiteFactor;
  }else if (i_color == LED_COLOR_SLEEP) {
    f_red_value = 10.F * objConfig.RwmRgbColorWhiteFactor;
  }

  if (b_gain_active){
    // gain is active
    f_red_value *= objConfig.RwmRgbGainFactorRed;
    f_green_value *= objConfig.RwmRgbGainFactorGreen;
    f_blue_value *= objConfig.RwmRgbGainFactorBlue;
  }

  // Value saturation check
  f_red_value = (f_red_value>f_max_resolution) ? f_max_resolution : f_red_value;
  f_red_value = (f_red_value<0.F) ? 0.F: f_red_value;

  f_green_value = (f_green_value>f_max_resolution) ? f_max_resolution : f_green_value;
  f_green_value = (f_green_value<0.F) ? 0.F : f_green_value;

  f_blue_value = (f_blue_value>f_max_resolution) ? f_max_resolution : f_blue_value;
  f_blue_value = (f_blue_value<0.F) ? 0.F : f_blue_value;

  ledcWrite(RwmRedChannel, (int)f_red_value);
  ledcWrite(RwmGrnChannel, (int)f_green_value);
  ledcWrite(RwmBluChannel, (int)f_blue_value);

}// setColor

void setToSleep(){
  /** set the coffemachine to sleep
   *
   */

  bSleepMode = true;
  esp_log_write(ESP_LOG_WARN, strUserLogLabel, "Timeout reached -> machine going into sleep\n");
  objPid.setPidToSleep();

} //setToSleep


void stateCtrlLed(){
  /** what is done in the LED state (without state switching)
   *
   */
  if(bSleepMode){
    //sleep mode
    setColor(LED_COLOR_SLEEP, false);
    return;
  }
  if(iErrorId > NO_ERROR){
      setColor(LED_COLOR_PURPLE, false); 
    } else if (fTemp < objConfig.CtrlTarget - 1.0) {
      // Heat up signal
      setColor(LED_COLOR_ORANGE, true);
    } else if (fTemp > objConfig.CtrlTarget + 1.0){
      // Cool down signal
      setColor(LED_COLOR_BLUE, true);
    } else {
      // temperature in range signal
      setColor(LED_COLOR_GREEN, true); 
    }

}//stateCtrlLed


void stateCtrlDiag(){
  /** what is done in the DIAG state (without state switching)
   *
   */

  // Error: Temperature Range unplausible
    if (fTemp < 10.F){
      // Sensor range is not valid
      iErrorId |= TEMP_OUT_RANGE;
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "ERROR: Temperature range unplausible. Measured Value: %.2f C.\n", fTemp);
    } else {
      iErrorId &= ~TEMP_OUT_RANGE;
    }

    // Error: Value frozen
    // dont trigger if in sleep
    if ((objADS1115->isValueFrozen()) && (!bSleepMode)){
      iErrorId |= TEMP_FROZEN;
      esp_log_write(ESP_LOG_INFO, strUserLogLabel,  "ERROR: ADS1115 Sensor values frozen.\n");
    } else {
      iErrorId &= ~TEMP_FROZEN;
    }

    // Error: Internal reset of ADS
    if (objADS1115->getOpMode()==ADS1115_MODE_SINGLESHOT){
      iErrorId |= MEAS_DEV_RESET;
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "ERROR: Conversion mode changed to single shot (default) during runtime.\n");
    } else {
      iErrorId &= ~MEAS_DEV_RESET;
    }

    if ((WiFi.status() != WL_CONNECTED) && (WiFi.getMode() == WIFI_MODE_STA)){
      iErrorId |= WIFI_DISCONNECT;
      esp_log_write(ESP_LOG_INFO, strUserLogLabel, "ERROR: Wifi disconnected.\n");
    } else {
      iErrorId &= ~WIFI_DISCONNECT;
    }

    // try error handling
    if (iErrorId > NO_ERROR){
      if (iErrorId <= (TEMP_OUT_RANGE + TEMP_FROZEN + MEAS_DEV_RESET)){
        // disable powerstages
        fTarPwm = 0.F;
        ledcWrite(PwmSsrChannel, 0);
        // configure ADS1115 again
        objADS1115->stop();
        configADS1115();
      } else if (iErrorId == WIFI_DISCONNECT) {
        server.end();
        server.begin();
      }
    }

}


void loop(){
  if ((iState & MEASURE) == MEASURE) {
    fTime = (float)(millis() - iTimeStart) / 1000.0;
    // get physical value of sensor
    fTemp = objADS1115->getPhysVal();
    
    portENTER_CRITICAL_ISR(&objTimerMux);
      iState &= ~MEASURE;
    portEXIT_CRITICAL_ISR(&objTimerMux);

    // reset watchdog timer every time a sensor value is read
    esp_task_wdt_reset();
  }

  if ((iState & PID_CTRL) == PID_CTRL) {
    // Call heating control function
    bool b_result_ctrl_heating;
    b_result_ctrl_heating = controlHeating();
    
    portENTER_CRITICAL_ISR(&objTimerMux);
      iState &= ~PID_CTRL;
    portEXIT_CRITICAL_ISR(&objTimerMux);
  }

  if ((iState & LED_CTRL) == LED_CTRL) {
    stateCtrlLed()  
    
    portENTER_CRITICAL_ISR(&objTimerMux);
      iState &= ~LED_CTRL;
    portEXIT_CRITICAL_ISR(&objTimerMux);
  }

  if ((iState & STORE) == STORE) {
    // Write values to measurement file if not in sleep mode
    bool b_result_meas_write;
    if (bSleepMode){
      b_result_meas_write =true
    } else {
      b_result_meas_write = writeMeasFile();
    }
    
    if (b_result_meas_write){
      portENTER_CRITICAL_ISR(&objTimerMux);
        iState &= ~STORE;
      portEXIT_CRITICAL_ISR(&objTimerMux);
    }
  }

  // TODO: add a dynamic timeout -> (start-now)>TimeToStandby; timeout can then be reset via a button in webinterface
  if ((millis() >= objConfig.TimeToStandby * 1000) && (!bSleepMode)) {
    // check whether timeout is reached, PWM will be deactivated.
    setToSleep():
  }
  
  // Diagnosis functionality
  if ((iState & DIAG) == DIAG){
    stateCtrlDiag()    
    
    portENTER_CRITICAL_ISR(&objTimerMux);
      iState &= ~DIAG;
    portEXIT_CRITICAL_ISR(&objTimerMux);
  }
}// loop
