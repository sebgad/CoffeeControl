# CoffeeControl :coffee:

This is primarily a temperatur controller for a Rancilio Silvia Espresso machine.
The project is based on an eps32 via the arduino IDE.

## code

### required aduino libaries:
- for PID control [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)
- for Webserver [ESPAsyncWebserver](https://github.com/me-no-dev/ESPAsyncWebServer)
- for Async TCP transfer [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)


In addition there are two files required for individual setup:

"WifiAccess.h" required for storing the credentials for your network:

```c++
#ifndef WifiAccess_h
#define WifiAccess_h
#include "Arduino.h"
static const char charSsid[] = "XXX";
static const char charPassword[] = "XXX";
#endif
```

"Pt1000.h" for defining individual characteristic of wheatstone bridge:
```c++
#ifndef Pt1000_h
#define Pt1000_h
#include "Arduino.h"
// define characteristic of wheatstone bridge, columns: U-Voltage, Temperature
const float arrPt1000Conv[][2] = {
                                {-0.219045697568849, 0},
                                {-0.187945450343167, 10},
                                {-0.157941178967306, 20},
                                {-0.143266864702809, 25},
                                {-0.128976235258584, 30},
                                {-0.100997810214978, 40},
                                {-0.073956614203555, 50},
                                {-0.047873480802489, 60},
                                {-0.022634468010329, 70},
                                {0.001800502812121, 80},
                                {0.02546901145367, 90},
                                {0.048406330607007, 100},
                                {0.070645600232249, 110},
                                {0.092161695908939, 120},
                                {0.113043254024256, 130},
                                {0.133371073099347, 140},
                                {0.153010996231061, 150}
                                 };
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

