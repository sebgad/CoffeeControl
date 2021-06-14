# CoffeeControl :coffee:

This is primarily a temperatur controller for a Rancilio Silvia Espresso machine.
The project is based on an eps32 via the arduino IDE.

## code

### required aduino libaries:
- for PID control [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)
- for Webserver [ESPAsyncWebserver](https://github.com/me-no-dev/ESPAsyncWebServer)
- for Async TCP transfer [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)


In addition there is a file "WifiAccess.h" required for storing the credentials for your network:

```c++
#ifndef WifiAccess_h
#define WifiAccess_h
#include "Arduino.h"
static const char charSsid[] = "XXX";
static const char charPassword[] = "XXX";
#endif
```

## PCB
- The PCB was created via KiCAD [KiCAD](https://www.kicad.org/)
- Rouing was done via the autorouter from freerouting [freerouting](https://github.com/freerouting/freerouting)
- The sorce fiels of the PCB are in *tools/PCB-KiCAD/EspressoV1*
Note:
- The autorouter cannot create a 100% working PCB but is a good start 
- Do not sace the preferenences of the outorouter. Otherweise the routing cannot be redone. See [here](https://github.com/freerouting/freerouting/issues/26#issuecomment-814837891)
- The Design uses an own libary. KiCAD might not be able to find the file initially. 
	- re-Open KiCAD: Do Fixes in "Symbol Editor" and "Footprint Editor":
	- Preferences -> Symbol Libaries|Manage Footprint Libaries : update the path of the esp32 libary
	- close and open KiCAD

