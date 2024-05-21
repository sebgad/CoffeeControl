/**
 * All Types for coffee_ctrl_main are saved here
*/

#define LOG_LEVEL ESP_LOG_VERBOSE

/* PIN definitions */
#define P_PUMP_RELAY 15

/* I2C pins */
#define SDA_0 23
#define SCL_0 22
#define CONV_RDY_PIN 14

/* PWM defines */
#define P_SSR_PWM 21
#define P_RED_LED_PWM 13
#define P_GRN_LED_PWM 27
#define P_BLU_LED_PWM 12
#define P_STAT_LED 33 /* green status LED */

/* File system definitions */
#define FORMAT_SPIFFS_IF_FAILED true

/* define timer related channels for PWM signals */
#define RwmRedChannel 13 /*  PWM channel. There are 16 channels from 0 to 15. Channel 13 is now Red-LED */
#define RwmGrnChannel 14 /*  PWM channel. There are 16 channels from 0 to 15. Channel 14 is now Green-LED */
#define RwmBluChannel 15 /*  PWM channel. There are 16 channels from 0 to 15. Channel 15 is now Blue-LED */
#define PwmSsrChannel 0  /*  PWM channel. There are 16 channels from 0 to 15. Channel 0 is now SSR-Controll */

#define WDT_Timeout 75 /* WatchDog Timeout in seconds */

/* config structure for online calibration */
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

enum eState{
  IDLE      = (1u << 0),
  MEASURE   = (1u << 1),
  PID_CTRL  = (1u << 2),
  STORE     = (1u << 3),
  DIAG      = (1u << 4),
  LED_CTRL  = (1u << 5),
  BREWING   = (1u << 6)
};

enum eError{
  NO_ERROR          = (1u << 0),
  TEMP_OUT_RANGE    = (1u << 1),
  MEAS_DEV_RESET    = (1u << 2),
  WIFI_DISCONNECT   = (1u << 3),
  };

enum eLEDColor{
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  LED_COLOR_BLUE,
  LED_COLOR_ORANGE,
  LED_COLOR_PURPLE,
  LED_COLOR_WHITE
};