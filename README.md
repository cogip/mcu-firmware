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
$ git clone https://github.com/cogip/RIOT.git -b firmware-dependencies
$ git clone https://github.com/cogip/mcu-firmware.git
```

# Simulation

## Build and launch

```bash
$ BOARD=native make -j4
$ bin/native/cortex.elf

```

# Embedded target

## Requirements

To install toolchain and development on debian/ubuntu:

```bash
$ sudo apt install gcc-arm-none-eabi openocd python-serial
```

## Build, deploy and connect on target

```bash
$ CFLAGS=-DCOGIP2018_CPU=F446 BOARD=cogip2018-f4xx make -j4
$ CFLAGS=-DCOGIP2018_CPU=F446 BOARD=cogip2018-f4xx make flash
$ CFLAGS=-DCOGIP2018_CPU=F446 BOARD=cogip2018-f4xx make term
```

