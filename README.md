# Embedded Game Controller library

A static library for embedded devices to interact with Game Controllers over USB (and, in the future, bluetooth). When Embedded Game Controller is too long to write, you are welcome to use the EGC acronym.

## Features

### Supported USB game controllers

| Device Name              | Vendor Name | Vendor ID | Product ID |
|:------------------------:|:-----------:|:---------:|:----------:|
| PlayStation 3 Controller | Sony Corp.  | 054c      | 0268       |
| DualShock 4 [CUH-ZCT1x]  | Sony Corp.  | 054c      | 05c4       |
| DualShock 4 [CUH-ZCT2x]  | Sony Corp.  | 054c      | 09cc       |
| Generic PC controller    | Dragonrise  | 0079      | 0006       |

- DS3 and DS4 support includes LEDs, rumble, and the accelerometer
- DS4's touchpad is used to emulate the Wiimote IR Camera pointer
- Both controllers emulate a Wiimote with the Nunchuk and Classic Controller extensions connected. Press L1+L3 to switch between them
- Three IR pointer emulation modes: direct (touchpad, only for DS4), analog axis relative (move the pointer with the right analog) and analog axis absolute (the pointer is moved proportionally to the right analog starting from the center). Press R1+R3 to switch between them

### Supported target platforms

EGC can be built on the following platforms:

- Nintendo Wii
- Nintendo Wii's Starlet processor (cIOS)
- Linux with libusb

## Compilation

### Nintendo Wii

##### 1) Install `devkitPPC`

- Download and install [devkitPPC](https://devkitpro.org/wiki/Getting_Started)
- Make sure to install the `devkitppc-cmake` package when using `pacman`

##### 3) Build `libembedded-game-controller.a`

1. `mkdir build && cd build`
2. Configure it with CMake:
  &ensp; `cmake CMAKE_TOOLCHAIN_FILE="$DEVKITPRO/cmake/Wii.cmake" -DBUILD_EXAMPLE=ON ..`
3. `make` (or `ninja` if configured with `-G Ninja`)
4. `libembedded-game-controller.a` will be generated

I recommend passing `-DCMAKE_COLOR_DIAGNOSTICS:BOOL=TRUE`, especially when using Ninja.

### For the Wii's Starlet processor

##### 1) Install `devkitARM`

- Download and install [devkitARM](https://devkitpro.org/wiki/Getting_Started)
- Make sure to install the `devkitarm-cmake` package when using `pacman`

##### 3) Build `libembedded-game-controller.a`

1. `mkdir build && cd build`
2. Configure it with CMake. Two options:\
  &ensp;a. `arm-none-eabi-cmake ..`\
  &ensp;b. `cmake -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$DEVKITPRO/cmake/devkitARM.cmake ..`
3. `make` (or `ninja` if configured with `-G Ninja`)
4. `libembedded-game-controller.a` will be generated

I recommend passing `-DCMAKE_COLOR_DIAGNOSTICS:BOOL=TRUE`, especially when using Ninja.

## Credits
- [xerpi's Fakemote](https://github.com/xerpi/fakemote), where this project came out from
- [Dolphin emulator](https://dolphin-emu.org/) developers
- [Wiibrew](https://wiibrew.org/) contributors
- [d2x cIOS](https://github.com/davebaol/d2x-cios) developers
- [Aurelio Mannara](https://twitter.com/AurelioMannara/)
- _neimod_, for their [Custom IOS Module Toolkit](http://wiibrew.org/wiki/Custom_IOS_Module_Toolkit)
