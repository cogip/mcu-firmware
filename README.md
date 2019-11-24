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

# Prepare python environment

Some tool rely on python to run. Minimum required version is python 3.6.
A possible preparation is proposed below:

```bash
$ python3 -m venv simulation/venv
$ source simulation/venv/bin/activate
$ pip install -r simulation/requirements.txt
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

## Build and observe control loop

A python script is available to observe the data of the control loop.
Script can be launched using following:

```bash
$ source simulation/venv/bin/activate
$ python simulation/robot_calibration.py
```

To get relevant visualization, please ensure following files have ENABLE_DEBUG
set:

- controllers/ctrl.c
- controllers/quadpid/quadpid.c
- platforms/common/cortex/platform-common.c

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

# General build targets

## Build all platforms

```bash
$ make
```

## Clean all platforms

```bash
$ make clean
```

## Distclean all platforms

```bash
$ make distclean
```

## Generate doc in various formats

```bash
$ make doc docman doclatex
```

## Clean doc

```bash
$ make docclean
```

## Start ddd on a gdbserver's target

```bash
$ make DBG=ddd DBG_FLAGS='--debugger "${GDB} ${DBG_DEFAULT_FLAGS}"' debug
```

Then, inside DDD application, in gdb prompt, type
```
target remote localhost:3333
```

To debug specific function, type
```
list <functionname>
```
