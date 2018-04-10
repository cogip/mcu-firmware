#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import select
import pty,sys,os

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

csvfile = None
storing = False

def parse(l):
    global storing
    global csvfile
    if l.startswith('<<<< '):
        filename = l.split(' ')[1].rstrip('\r').rstrip('\n')
        csvfile = open(filename, 'wb')
        storing = True
    elif l.startswith('>>>>'):
        csvfile.close()
        storing = False
    elif storing:
        csvfile.write(l)
        l = '\033[92m' + l + '\033[0m'.lstrip(' ')
    return l


serial.flush()
l = ''
while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if line:
            serial.write(line.replace('\r','').replace('\n',''))
        else: # an empty line means stdin has been closed
          print('eof')
          exit(0)
    c = serial.read(1)
    l +=c
    if c == '\n':
        l = parse(l)
        print l,
        l = ''
exit()


