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

#define Pt1000_CONV_LINEAR 0b001
#define Pt1000_CONV_SQUARE 0b010
#define Pt1000_CONV_LOOK_UP_TABLE 0b011

const int Pt1000_CONV_METHOD = Pt1000_CONV_SQUARE;

// Define coefficients for linear regression function
const float fPt1000LinCoeffX1 = 433.6520668;
const float fPt1000LinCoeffX0 = 82.01975366;

// Define coefficients for quadratic regression function
const float fPt1000SquareCoeffX2 = 295.16714369;
const float fPt1000SquareCoeffX1 = 417.84868016;
const float fPt1000SquareCoeffX0 = 76.91861221;

// Define lookup table for temperature calculation
const float arrPt1000LookUpTbl[][2] = {
                                        {-0.21396866840731069, 0.0},
                                        {-0.19826086393703643, 5.0},
                                        {-0.18283911631429636, 10.0},
                                        {-0.16770344877338753, 15.0},
                                        {-0.15282330361877822, 20.0},
                                        {-0.13821466591521075, 25.0},
                                        {-0.12386291635089636, 30.0},
                                        {-0.10976138828633408, 35.0},
                                        {-0.09589652769003139, 40.0},
                                        {-0.08227645893153232, 45.0},
                                        {-0.06888104591949339, 50.0},
                                        {-0.05571158923387528, 55.0},
                                        {-0.0427624663099406, 60.0},
                                        {-0.0300282397238783, 65.0},
                                        {-0.017503649635036495, 70.0},
                                        {-0.00518360659538545, 75.0},
                                        {0.0069431178724726535, 80.0},
                                        {0.01886859632220499, 85.0},
                                        {0.03060996495366849, 90.0},
                                        {0.04216540043842523, 95.0},
                                        {0.05353924250669009, 100.0},
                                        {0.06472982975573649, 105.0},
                                        {0.07575883980844787, 110.0},
                                        {0.08661262418699037, 115.0},
                                        {0.09730088303101485, 120.0},
                                        {0.10782733455505109, 125.0},
                                        {0.11820106425587913, 130.0},
                                        {0.12841994955774214, 135.0},
                                        {0.13848205967873975, 140.0},
                                        {0.1484014359031887, 145.0},
                                        {0.15817588158467566, 150.0},
                                        {0.16780851306788105, 155.0},
                                        {0.17730235788402426, 160.0},
                                        {0.18666535708795248, 165.0},
                                        {0.19589030865119572, 170.0},
                                        {0.20498993006709912, 175.0},
                                        {0.21395711383340363, 180.0},
                                        {0.22280417956398454, 185.0},
                                        {0.23152881217857227, 190.0},
                                        {0.2401335018563869, 195.0},
                                        {0.24862067160467974, 200.0}
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

