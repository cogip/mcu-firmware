#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

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
import mutex

BASE_PATH = "platforms/cogip2019-cortex-simulation/"
BIN_NAME = "cortex-simulation.elf"
BIN_PATH = BASE_PATH + "bin/cogip2019-cortex-native/" + BIN_NAME

# Speeds array
speed_linear_values = []
speed_angular_values = []
speed_linear_times = []
speed_angular_times = []
# Pose array
pose_linear_values = []
pose_angular_values = []
pose_linear_times = []
pose_angular_times = []
# Motion control periods counters
speed_linear_time = 0
speed_angular_time = 0
pose_linear_time = 0
pose_angular_time = 0

# Lock for datas
lock = Lock()

# Flag to stop refreshing plots when no datas are available
stop_timer = 0

# Create plots window
win = pg.GraphicsWindow()
win.setWindowTitle("Speeds")
# Create 2 plots
# * One for linear speed
# * One for angular speed
p_speed = win.addPlot()
p_pose = win.addPlot()
# Create associated curves
plt_speed_linear = p_speed.plot(speed_linear_times, speed_linear_values, title="Speed linear")
plt_speed_angular = p_speed.plot(speed_angular_times, speed_angular_values, title="Speed angular")
plt_pose_linear = p_pose.plot(pose_linear_times, pose_linear_values, title="Pose linear")
plt_pose_angular = p_pose.plot(pose_angular_times, pose_angular_values, title="Pose angular")
# Display grid on all plots
p_speed.showGrid(x=True,y=True)
p_pose.showGrid(x=True,y=True)

last_len_linear = 0

def reset_arrays():
    global speed_linear_values
    global speed_angular_values
    global speed_linear_times
    global speed_angular_times
    global speed_linear_time
    global speed_angular_time
    global pose_linear_values
    global pose_angular_values
    global pose_linear_times
    global pose_angular_times
    global pose_linear_time
    global pose_angular_time

    lock.acquire()
    # Speeds array
    speed_linear_values = []
    speed_angular_values = []
    speed_linear_times = []
    speed_angular_times = []
    # Pose array
    pose_linear_values = []
    pose_angular_values = []
    pose_linear_times = []
    pose_angular_times = []

    plt_speed_linear.setData(speed_linear_times, speed_linear_values, clear=True, pen='r')
    plt_speed_angular.setData(speed_angular_times, speed_angular_values, clear=True, pen='b')
    plt_pose_linear.setData(pose_linear_times, pose_linear_values, clear=True, pen='r')
    plt_pose_angular.setData(pose_angular_times, pose_angular_values, clear=True, pen='b')
    lock.release()

def update_plot():
    global last_len_linear
    lock.acquire()
    if len(speed_linear_times) != last_len_linear:
        plt_speed_linear.setData(speed_linear_times, speed_linear_values, clear=True, pen='r')
        plt_speed_angular.setData(speed_angular_times, speed_angular_values, clear=False, pen='b')
        plt_pose_linear.setData(pose_linear_times, pose_linear_values, clear=True, pen='r')
        plt_pose_angular.setData(pose_angular_times, pose_angular_values, clear=False, pen='b')
        last_len_linear = len(speed_linear_times)
    lock.release()

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
        global speed_linear_values
        global speed_angular_values
        global speed_linear_times
        global speed_angular_times
        global speed_linear_time
        global speed_angular_time
        global pose_linear_values
        global pose_angular_values
        global pose_linear_times
        global pose_angular_times
        global pose_linear_time
        global pose_angular_time
        global stop_timer

        # Remove trailing spaces or line return
        line = line.rstrip().rstrip(',')
        if re.match(r'@.*@,.*', line) is not None:
            # Split parameters
            params = line.split(',')

            # Extract objects parameters
            obj_type = params[0]
            obj_param = params[1]
            obj_id = params[2]

            lock.acquire()
            if obj_type == self.ROBOT_OBJECT_PATTERN:
            #    if obj_param == self.POSE_ORDER_PATTERN:
            #    elif obj_param == self.POSE_CURRENT_PATTERN:
                if obj_param == self.SPEED_CURRENT_PATTERN:
                    stop_timer = 0
                    speed_linear_times.append(int(speed_linear_time))
                    speed_linear_values.append(float(params[3]))
                    speed_angular_times.append(int(speed_angular_time))
                    speed_angular_values.append(float(params[4]))
                    speed_linear_time = speed_linear_time + 1
                    speed_angular_time = speed_linear_time + 1
                elif obj_param == self.POSE_ERROR_PATTERN:
                    stop_timer = 0
                    pose_linear_times.append(int(pose_linear_time))
                    pose_linear_values.append(float(params[3]))
                    pose_angular_times.append(int(pose_angular_time))
                    pose_angular_values.append(float(params[4]))
                    pose_linear_time = pose_linear_time + 1
                    pose_angular_time = pose_linear_time + 1
                else:
                    output = sys.stderr
            lock.release()

    def parse(self, flow):
        global stop_timer
        while True:
            line = flow.readline()
            output = sys.stdout
            print(line, file=output, end='')
            if line.startswith(self.STOP_PATTERN):
                stop_timer = 1
            elif line.startswith(self.RESET_PATTERN):
                reset_arrays()
                stop_timer = 0
            elif line.startswith(self.END_PATTERN):
                stop_timer = 1
                break
            else:
                self._decode(line)
        print('Parsing finished')


class NativeParser(Parser):

    def parse(self, p):
        super(NativeParser, self).parse(p.stdout)

    def send(self, p):
        for line in fileinput.input():
            p.stdin.write(line)

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
                    self.ser.write(line)

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
