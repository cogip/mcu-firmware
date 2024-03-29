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
![Build Status](https://github.com/cogip/mcu-firmware/workflows/Compilation%20Workflow/badge.svg)

# Environment setup

## Cloning repositories

```bash
$ git clone https://github.com/cogip/RIOT.git -b cogip_master
$ git clone https://github.com/cogip/mcu-firmware.git
```

## Requirements

### Toolchain

To install toolchain and development on ubuntu 20.04:

```bash
$ sudo apt install build-essential gcc-multilib g++-multilib openocd
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

### Quilt

Quilt is a tool to manage large sets of patches by keeping track of the changes each patch makes.
It is used to apply patches on RIOT-OS

```bash
$ sudo apt install quilt
```

### cqfd (for native architecture only)

Cqfd wraps commands to run them inside the Docker container using your host
current user.
It can be configured with some pre defined commands called flavors and it can
produce release artifacts.

To install cqfd:

```bash
git clone git@github.com:savoirfairelinux/cqfd.git
cd cqfd/
sudo make install
```

For more information: [cqfd repository](https://github.com/savoirfairelinux/cqfd)

### Python Virtual Environment

```bash
$ sudo apt install  python3-pip python3-venv
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r mcu-firmware/requirements.txt
```

# Apply RIOT-OS patches

Prior to any build, apply RIOT-OS patches

```bash
make riot-patches
```

# Simulation target (x86_64 architecture)

Assuming the platform is pf_test and the application is app_test

## Build

### Build one application with default board

```bash
$ make -j$(nproc) -C applications/<application_name>
```

### Build one application with an other board

```bash
$ make -j$(nproc) BOARD=<board_name> -C applications/<application_name>
```

### Using cqfd

#### Init cqfd docker image

This command has to be done once per project:

```bash
$ cqfd init
```

#### Launch the build using a flavor

```bash
$ cqfd -b app_test-native
```

## Build and launch in debugger (only for native cpu architecture)

```bash
$ make -j$(nproc) BOARD=<board_name> -C applications/<application_name> all-debug
$ ddd applications/<application_name>/bin/<board_name>/<binary>.elf
```

## Build and flash (only for real board, not native based)

Make sure JTAG programmer is plugged on target board.

```bash
$ make -j$(nproc) BOARD=<board_name> -C applications/<application_name> flash
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

# General build targets

## Build all applications on all boards

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

## Run uncrustify to check coding rules

```bash
$ make check-codingrules
```
