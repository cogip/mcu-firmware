# About

This application is intended for testing LX servo serial bus.

# Usage

To have the list of available commands:
```
help
```

You will need to initialize one UART at 115200 baudrate (if the servomotor is in factory configuration):
```
init 2 115200
```

To ping the servomotor:
```
ping 1
```

Be careful!
    - If 2 servomotors with the same ID are connected on the same bus, you will have no response or corrupted one.
    - There is no acknowledgement of write commands

Factory configuration ID is 1, you need to change this to connect an other servo.

To scan every connected servomotors (IDs from 0 to 253):
```
scan
```

To move a servo to a given position at full speed:
```
m 1 2000
```

To get the status of a servo:
```
status 1
```
