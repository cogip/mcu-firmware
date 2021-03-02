# shmem example

This example demonstrates how to use shmem module with a shared memory segment created in Python.


## On Python side

Requires Python 3.7+.
Install Python packages listed in `requirements.txt`.
Run `shmem.py`.

This will display data set in the shared memory.
It also prints the key to used with the example binary.

```bash
$ python3 shmem.py
Shared memory initialized with:
data.even = [0, 2, 4, 6, 8, 10, 12, 14]
data.odd  = [1, 3, 5, 7, 9, 11, 13, 15]
Shm key = 18222
Press ENTER to exit...
```

## On binary side

From the current directory, compile the example and execute it:

```bash
$ make
$ make term
```

Execute the command `set_shmem_key` with the key given by the Python program as argument.

Execute the command `data` to print the data from the shared memory.

```bash
$ make term
.../mcu-firmware/examples/shmem/bin/native/shmem.elf /dev/ttyACM0
RIOT native interrupts/signals initialized.
LED_RED_OFF
LED_GREEN_ON
RIOT native board initialized.
RIOT native hardware initialization complete.

main(): This is RIOT! (Version: 2021.01)

== shmem example ==
Enter shell menu: Main menu
> help
help
Command              Description
---------------------------------------
exit                 Exit current menu
_help_json           Display available commands in JSON format
set_shmem_key        Set shared memory key to communicate with simulator
data                 Print data from the shared memory
reboot               Reboot the node
version              Prints current RIOT_VERSION
pm                   interact with layered PM subsystem
> set_shmem_key 18222
set_shmem_key 18222
> data
data
Data in shared memory:
  - data.even = 0 2 4 6 8 10 12 14
  - data.odd  = 1 3 5 7 9 11 13 15
>
```
