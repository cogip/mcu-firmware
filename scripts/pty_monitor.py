#!/usr/bin/python
# -*- coding: utf-8 -*-

import pty,sys,os

if len(sys.argv)<2:
    print "Need a command to run"
    sys.exit(-1)

csvfile = None

storing = False

def read(fd):
    data = os.read(fd, 1024)

    lines = data.split('\n')

    for i in range(0,len(lines)):
        l = lines[i]
        global storing
        global csvfile
        if l.startswith('<<<< '):
            filename = l.split(' ')[1].rstrip('\r')
            csvfile = open(filename, 'wb')
            storing = True
        elif l.startswith('>>>>'):
            csvfile.close()
            storing = False
        elif storing:
            # add some color
            lines[i] = '\033[92m' + lines[i] + '\033[0m'
            if i != len(lines)-1:
                csvfile.write(l + '\n')
            else:
                csvfile.write(l)
    return '\n'.join(lines)

pty.spawn(sys.argv[1:], read)
