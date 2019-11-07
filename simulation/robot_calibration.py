#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import fileinput
import getpass
import os
import psutil
os.environ['PYQTGRAPH_QT_LIB'] = 'PyQt5'
import pyqtgraph as pg
import pyqtgraph.exporters
import numpy as np
import re
from serial import *
import subprocess
import signal
from threading import Thread
from threading import Lock
from threading import RLock

BASE_PATH = "platforms/cogip2019-cortex-simulation/"
BIN_NAME = "cortex-simulation.elf"
BIN_PATH = BASE_PATH + "bin/cogip2019-cortex-native/" + BIN_NAME


# Lock for datas
lock = Lock()

# Create plots window
win = pg.GraphicsWindow()
win.setWindowTitle("Speeds")
# Create 2 plots
# * One for linear speed
# * One for angular speed
p_speed = win.addPlot()
p_pose = win.addPlot()
# Create associated curves
plt_speed_linear = p_speed.plot()
plt_speed_angular = p_speed.plot()
plt_pose_linear = p_pose.plot()
plt_pose_angular = p_pose.plot()
# Display grid on all plots
p_speed.showGrid(x=True,y=True)
p_pose.showGrid(x=True,y=True)


def update_plot():
    pass

class Parser(Thread):
    END_PATTERN = '>>>>'
    RESET_PATTERN = '<<<< RESET >>>>'
    STOP_PATTERN = '<<<< STOP >>>>'
    ROBOT_OBJECT_PATTERN = '@robot@'
    POSE_ORDER_PATTERN = '@pose_order@'
    POSE_CURRENT_PATTERN = '@pose_current@'
    POSE_ERROR_PATTERN = '@pose_error@'
    SPEED_CURRENT_PATTERN = '@speed_current@'

    def _decode(self, line):

        # Remove trailing spaces or line return
        line = line.rstrip().rstrip(',')
        if re.match(r'@.*@,.*', line) is not None:
            # Split parameters
            params = line.split(',')

            # Extract objects parameters
            obj_type = params[0]
            obj_param = params[1]
            obj_id = params[2]


    def parse(self, flow):
        while True:
            line = flow.readline().decode('utf-8')
            output = sys.stdout
            print('\x1b[0;35;40m' + line + '\033[0m', file=output, end='')
            output.flush()
            self._decode(line)
        print('Parsing finished')


class NativeParser(Parser):

    def parse(self, p):
        super(NativeParser, self).parse(p.stdout)

    def send(self, p):
        for line in fileinput.input():
            p.stdin.write(line.encode('utf-8'))
            p.stdin.flush()

class SerialParser(Parser):
    def __init__(self, serial_port):
        self.ser = Serial(port=serial_port, baudrate=115200, timeout=1, writeTimeout=1)

    def parse(self):
        if self.ser.isOpen():
            while True:
                super(SerialParser, self).parse(self.ser)

    def send(self, p):
        for line in fileinput.input():
            if self.ser.isOpen():
                self.ser.write(bytes(line, 'utf8'))

if __name__ == "__main__":
    # Kill all remaining bin instance
    process = filter(lambda p: BIN_NAME in p.name(), psutil.process_iter())
    for i in process:
        os.kill(i.pid, signal.SIGKILL)
        try:
            os.waitpid(i.pid, 0)
        except OSError:
            pass

    p = subprocess.Popen(BIN_PATH, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    parser = NativeParser()
    thread = Thread(target=parser.parse, args=(p,))
    #parser = SerialParser("/dev/ttyUSB0")
    #thread = Thread(target=parser.parse)
    thread.daemon = True
    thread.start()

    thread = Thread(target=parser.send, args=(p,))
    thread.daemon = True
    thread.start()

    # Timer used to refresh plots
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update_plot)
    timer.start(10)

    if sys.flags.interactive != 1 or not hasattr(pg.QtCore, 'PYQT_VERSION'):
        pg.QtGui.QApplication.exec_()

# vim: tabstop=4 expandtab shiftwidth=4 softtabstop=4
