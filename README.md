# REP LOTI.05.032 Robin Wersich

This project aims to build a robot that can [move and balance on a basketball](https://www.youtube.com/watch?v=eqhnZmMAU6M).
The main concepts in the project explored are:

- use of omniwheels
- bluetooth remote control
- balancing control logic
- 3D printing
- hardware selection and wiring

## Hardware BOM

- 3 [motors with encoders](https://gopigo.io/gopigo3-motor-replacement-kit/)
- 3 motor connector cables: 6-pin (0.2mm spacing) to 6-pin (0.254mm spacing), ~8cm length
- 1 [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/)
- 2 [Dual DC motor drivers](https://www.tme.eu/ee/en/details/df-dri0044/motor-control-modules/dfrobot/dri0044/)
- 1 [IMU](https://www.pololu.com/product/2863)
- 1 12V battery pack
- parts for 3 omniwheels
  - 3 \* 16 small rubber coated wheels similar to [these](https://www.aliexpress.com/item/2054027643.html) (+ axis)
  - 3 \* 3 M2x8  bolts (socket head cap or other non-countersunk) + nuts
- control board
  - [ProtoBoard](https://www.dfrobot.com/product-660.html)
  - 6 10-33nF capacitors
  - 2 screw terminals (2 pins each)
  - MF headers: 2x 20-pin, 2x 14-pin, 1x 5-pin, 3x 6-pin
  - MM headers: 2x 3-pin
  - 2 M2 threaded heat inserts
  - 2 M2x3 bolts (socket head cap or other non-countersunk) for securing the IMU
  - 0.28mm² insulated wire (20cm black, 25cm red, 50cm green, 40cm white)
  - 0.75mm² insulated wire (5cm black, 5cm red)
- battery connector: 9x5x5 barrel jack to 2x2 pin headers, 15cm length, 0.75mm²
- USB-A to Micro-USB cable (ca. 15cm length)
- 6 M3x25 (countersunk or low-profile non-countersunk) and 3 M2.5x15 bolts (socket head cap or other non-countersunk) + nuts for mounting the motors
- 4 M30 FF PCB spacer
- 4 M3 threaded heat inserts
- 10 M3x6 bolts and 1 M3x10 (socket head cap or other non-countersunk) headless bolt for mounting case, pcb and spacers
- 2cm Velcro for attaching battery
- metal case (150x150x90)
- bluetooth gamepad
- size 7 basketball

## 3D printed parts

The following parts are 3D-printed:

- 3x omniwheels (partly)
- robot framework
- IMU mount

A 3D model with all printed and non-printed parts can be found [here](https://cad.onshape.com/documents/aef269c9421a5698f51293a2/w/28d1817e881a7e140ad68222/e/52373abffadf50390ef456f1).
In addition, exported _STEP_ files are available in the `CAD` folder.

## Wiring

The wiring diagram can be found in the `wiring` folder.

## Project Code

The Raspberry Pi Pico W is programmed in C++ using the official SDK. As the latter uses [CMake](https://cmake.org/), this project does as well.

### Setup

The following describes the setup using [Visual Studio Code](https://code.visualstudio.com/) but [CLion](https://www.jetbrains.com/de-de/clion/), being CMake-based, should also work well.

1. Set up the Pico SDK using the _Getting Started_ Instructions on the [official website](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html).
1. Set up the [Bluepad32 library](https://github.com/ricardoquesada/bluepad32) (steps 2 and 3 of this [guide](https://bluepad32.readthedocs.io/en/latest/plat_picow/)).
1. Install the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.
1. Open the repository folder in VS Code. Install the C/C++ Extension and the CMake extension.
1. Set the `PICO_SDK_PATH`, `BLUEPAD32_ROOT` and `Eigen3_DIR` either as global environment variable or in the settings of the CMake extension under _configure environment_.
1. Configure CMake by pressing `Ctrl`+`Shift`+`P` and search for _CMake: configure_. Choose the GCC arm-non-eabi which was installed in Step 1 as compiler. CMake should then automatically be run and create a `build` folder.
   If you run CMake manually, make sure to pass the `-DPICO_BOARD=pico_w` flag.
1. Navigate to the `build` folder and execute `make` to build all project files. Use `-j4` to use 4 threads for a faster build.
1. Connect the Pico while pressing the bootsel button, which should mount is as mass storage device. Then copy the `*.uf2` file to this drive.

### IMU Calibration
The IMU needs to be calibrated for good results:
1. Flash the `playground/imu_calibration` software on the pico.
1. Use the `scripts/imu_calibration.py` script to retrieve the magnetometer and gyro bias, as well as accelerometer measurements.
1. Use [Magneto](https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html) or a similar tool to get the bias and correction matrix for the accelerometer.
1. Adjust the values in `imu_calibration_values.h` correspondingly.
