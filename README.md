     ██████╗ ██████╗  ██████╗ ██╗██████╗
    ██╔════╝██╔═══██╗██╔════╝ ██║██╔══██╗
    ██║     ██║   ██║██║  ███╗██║██████╔╝
    ██║     ██║   ██║██║   ██║██║██╔═══╝
    ╚██████╗╚██████╔╝╚██████╔╝██║██║
     ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝╚═╝

MCU firmware
============

This repository contains firmware source code of Cogip robot for Eurobot, French robotic contest.
Its firmware intends to run on stm32f4xx MCU family and rely on
[RIOT-OS](https://riot-os.org/).

As this firmware and RIOT-OS are both evolving, two git repositories lives next to each other, and
both are required to generate the firwmare binaries.

## Build status
[![Build Status](https://travis-ci.org/cogip/mcu-firmware.svg?branch=master)](https://travis-ci.org/cogip/mcu-firmware)


# Cloning repositories

```bash
$ git clone https://github.com/RIOT-OS/RIOT.git -b 2019.07-branch
$ git clone https://github.com/cogip/mcu-firmware.git
```

# Requirements

To install toolchain and development on ubuntu 18.04:

```bash
$ sudo apt install build-essential gcc-8 gcc-8-multilib gcc-multilib openocd python-serial
```
Minimal gcc version: 8.1

To manually install arm-none-eabi toolchain:
```bash
$ mkdir ~/toolchain/
$ cd ~/toolchain/
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
$ tar xf gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
```

Edit ~/.bashrc file and add $HOME/toolchain/gcc-arm-none-eabi-8-2018-q4-major/bin/ to $PATH variable:
```bash
PATH=${PATH}:$HOME/toolchain/gcc-arm-none-eabi-8-2018-q4-major/bin/
```

## Build, deploy and connect on target

# Simulation

Assuming the platform is cogip2019-cortex-simulation

## Build and launch

```bash
$ cd platforms/cogip2019-cortex-simulation/
$ make -j$(nproc)
$ bin/cogip2019-cortex-native/cortex-simulation.elf
```

## Build and simulate in FreeCAD

Add robot and area designs:

```bash
$ cp <robot_design_path.iges> ~/.FreeCAD/Macro/cogip/simulation/Robot.iges
$ cp <table_design_path.iges> ~/.FreeCAD/Macro/cogip/simulation/Table.iges
```

Next build and copy simulation python script and binaries:

```bash
$ mkdir -p ~/.FreeCAD/Macro
$ cp simulation/robot_simulation_freecad.py ~/.FreeCAD/Macro/
$ cd platforms/cogip2019-cortex-simulation/
$ make -j$(nproc)
$ cp bin/ ~/.FreeCAD/Macro/ -Rf
```

Make sure '~/.FreeCAD/Macro/robot_simulation_freecad.py' has the correct paths
at the beginning of the script, according to your own files.

Now launch FreeCAD.

In 'Macro->Macros' launch 'robot_simulation_freecad.py' macro.

# Embedded target

Assuming the platform is cogip2019-cortex

## Build and flash

Make sure JTAG programmer is plugged on target board.

```bash
$ cd platforms/cogip2019-cortex/
$ make -j$(nproc) flash
```
