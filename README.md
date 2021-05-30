# CoffeeControl

This is primarily a temperatur controller for a Rancilio Silvia Espresso machine.
The project is based on an eps32 via the arduino IDE.

required aduino libaries:
- for PID control [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)
- for Webserver ESPAsyncWebserver (https://github.com/me-no-dev/ESPAsyncWebServer)
- for Async TCP transfer (https://github.com/me-no-dev/AsyncTCP)


In addition there is a file "WifiAccess.h" required for storing the credentials for your network:

```c++
#ifndef WifiAccess_h
#define WifiAccess_h
#include "Arduino.h"
static const char charSsid[] = "XXX";
static const char charPassword[] = "XXX";
#endif
```
