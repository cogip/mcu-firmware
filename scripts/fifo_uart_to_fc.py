#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import select
import subprocess
import os, sys, time

# default serial port
port = '/dev/ttyACM0'
baud = 115200

serial = serial.Serial(
    port,
    baudrate=baud,
    parity='N',
    stopbits=1,
    bytesize=8,
    xonxoff=False,
    rtscts=False,
    timeout=0 ) # 5 seconds timeout

serial.close()

try:
    serial.open()
except Exception, e:
    print "error open serial port: " + str(e)
    exit()

l=''
def myrun():
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:
                serial.write(line.replace('\r','').replace('\n',''))
            else: # an empty line means stdin has been closed
              print('eof')
              exit(0)
        l = serial.readline()
        if l != '':
            fifo.write(l)
            fifo.flush()
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
myrun()
