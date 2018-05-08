#!/usr/bin/python
# -*- coding: utf-8 -*-

import subprocess
import os, os.path, sys, time

CORTEX_BIN_PATH = "/bin/native/cortex.elf"

storing = False

def myrun(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stdout = []
    while True:
        global storing
        l = p.stdout.readline()
        if l == '' and p.poll() != None:
            break
        fifo.write(l)
        fifo.flush()
        stdout.append(l)
        print l,

    return ''.join(stdout)

# Path to be created
path = "/tmp/fifo_fc"
try:
    os.remove(path)
except OSError:
    pass
os.mkfifo(path, 0644)
fifo = open(path, "w")
time.sleep(2)
#myrun('/home/gdo/Developpement/Informatique/Personnel/mcu-firmware/bin/native/cortex.elf'
run_command = os.path.abspath(os.pardir) + CORTEX_BIN_PATH
myrun(run_command)
