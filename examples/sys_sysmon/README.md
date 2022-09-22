# Overview

This example illustrates the usage of sysmon module to monitor heap memory and threads stack memory.
The second UART is used to communicate with another device using protobuf messages the memory status.

In the Cogip context, the STM32 embedded in the robot will communicate with another device
also embedded in the robot, like a Raspberry Pi or an Odroid (we will call it RPi).

For testing, the RPi can be replaced by our development PC connected to the STM32
using a USB-Serial adapter. In native mode, RPi is also replaced by the development PC.

The software running on the RPi is written in Python.

On the STM32 side, the example provides two commands:
* `heap_status` that display heap status.
* `threads_status` that displays al threads stack status.

Every second, the STM32 sends heap and threads status through Protobuf messages to the RPi.

`heap` and `ps` RIOT commands are available to compare and validate results of `heap_status` and `threads_status`.

On the RPi side, the Python software will read sysmon status messages and displays received status.

# Setup

## Debian Requirements on Development PC

In addition to RIOT development environment, we need the following package:

```sh
$ sudo apt install protobuf-compiler
```

## Python Requirements on RPi

Create a venv and install required packages inside:

```sh
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r requirements.txt
```

# Compiling and Running Example on Real Hardware

Use the `cogip-nucleo-f446re` board which defines a second UART on UART4 (RX on PA1 and TX on PA0).

## Compilation and Flash STM32

```sh
$ make -j4 BOARD=cogip-nucleo-f446re flash
```

## Run Software on RPi

```sh
$ python sysmon-client.py
```

It uses default serial port and baudrate if not specified on command line, see help for more info:

```
$ python sysmon-client.py --help
Usage: sysmon-client.py [OPTIONS] [PORT] [BAUD]

Arguments:
  [PORT]  Serial port connected to STM32 device  [default: /dev/ttyUSB0]
  [BAUD]  Baud rate  [default: 230400]

Options:
  --install-completion [bash|zsh|fish|powershell|pwsh]
                                  Install completion for the specified shell.
  --show-completion [bash|zsh|fish|powershell|pwsh]
                                  Show completion for the specified shell, to
                                  copy it or customize the installation.

  --help                          Show this message and exit.
```

# Compiling and Running Example on Native Board

Use the `cogip-native` board which allows using a second UART.

## Create Virtual Serial Ports

Make sure `socat`is installed:

```sh
$ sudo apt install socat
```

Create the virtual ports in a different terminal:

```sh
$ socat -d -d pty,raw,echo=0,link=/tmp/ttySTM32 pty,raw,echo=0,link=/tmp/ttyRPI
```

## Compilation and Run the Native Firmware

```sh
$ make -j4 BOARD=cogip-native
$ make BOARD=cogip-native PORT="-c /tmp -c /tmp/ttySTM32" term
```

Note: the first `-c` option is not used since the first UART is used for stdout.

## Run Python Software

In a different terminal:

```sh
$ python sysmon-client.py /tmp/ttyRPI
```
