# REP LOTI.05.032 Robin Wersich

This project aims to build a robot that can [move and balance on a basketball](https://www.youtube.com/watch?v=eqhnZmMAU6M).
The main concepts in the project explored are:

- use of omniwheels
- bluetooth remote control
- balancing control logic
- 3D printing
- hardware selection and wiring

## Hardware List

- [x] 1 [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/)
- [x] 3 [motors with encoders](https://gopigo.io/gopigo3-motor-replacement-kit/) (but similar motors would work, too)
- [x] 2 [Dual DC motor drivers](https://www.tme.eu/ee/en/details/df-dri0044/motor-control-modules/dfrobot/dri0044/)
- [x] 1 12V battery pack
- [x] 1 [IMU](https://www.pololu.com/product/2863)
- [x] parts for 3 omniwheels
  - [x] 3 \* 16 small rubber coated wheels similar to [these](https://www.aliexpress.com/item/2054027643.html) (+ axis)
  - [x] 3 \* 3 M2x8 screws + nuts
- [x] [ProtoBoard](https://www.dfrobot.com/product-660.html) + wires for connecting electronic components
- [x] 6 M3x25 screws (head < 2mm) + nuts for mounting the motors
- [ ] M30 FF PCB spacer
- [ ] 8 M3x6 screws for mounting pcb and spacers
- [ ] Velcro for attaching battery
- [x] metal housing (could also be 3D printed or omitted)
- [x] XBox or other bluetooth controller

## 3D printed parts

The following parts are 3D-printed:

- omniwheels (partly)
- robot framework

A 3D model with all printed and non-preinted parts can be found [here](https://cad.onshape.com/documents/aef269c9421a5698f51293a2/w/28d1817e881a7e140ad68222/e/52373abffadf50390ef456f1).
In addition, exported _STEP_ files are available in the `CAD` folder.

## Wiring

The wiring diagram can be found in the `wiring` folder.

## Project Code

The Raspberry Pi Pico W is programmed in C++ using the official SDK. As the latter uses [CMake](https://cmake.org/), this project does as well.

### Setup

The following describes the setup using [Visual Studio Code](https://code.visualstudio.com/) but [CLion](https://www.jetbrains.com/de-de/clion/), being CMake-based, should also work well.

1. Set up the Pico SDK using the _Getting Started_ Instructions on the [official website](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html).
1. Set up the [Bluepad32 library](https://github.com/ricardoquesada/bluepad32) (steps 2 and 3 of this [guide](https://bluepad32.readthedocs.io/en/latest/plat_picow/))
1. Open the repository folder in VS Code. Install the C/C++ Extension and the CMake extension.
1. Set the `PICO_SDK_PATH` and `BLUEPAD32_ROOT` either as global environment variable or in the settings of the CMake extension under _configure environment_.
1. Configure CMake by pressing `Ctrl`+`Shift`+`P` and search for _CMake: configure_. Choose the GCC arm-non-eabi which was installed in Step 1 as compiler. CMake should then automatically be run and create a `build` folder.
   If you run CMake manually, make sure to pass the `-DPICO_BOARD=pico_w` flag.
1. Navigate to the `build` folder and execute `make` to build all project files. Use `-j4` to use 4 threads for a faster build.
1. Connect the Pico while pressing the bootsel button, which should mount is as mass storage device. Then copy the `*.uf2` file to this drive.
