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
win.setWindowTitle("PID loop monitor")
win.resize(1080, 720)

class DataItemSameTime(pg.PlotDataItem):

    def __init__(self, *args, **kargs):
        pg.PlotDataItem.__init__(self, *args, **kargs)

        self.y_values =  np.array([])
        self.setData(y = self.y_values) # yvalues only

    def addPoint(self, value):
        self.y_values = np.append(self.y_values, [value])
        self.setData(y = self.y_values)


class CurveOverTime(pg.PlotItem):
    """
    This object manages two set of points over time series.

    One set represents commands ("Cmd") curve, the second set represents
    the measurements ("Meas").
    """

    def __init__(self, title=None, *args, **kargs):
        pg.PlotItem.__init__(self, title=title, *args, **kargs)

        self.setLabel('bottom', 'Time[20 ms steps]')
        self.setLabel('left', 'm.s-1 ?')
        self.showGrid(x = True, y = True)

        self._addDataItems()

        self.legend = self.addLegend()
        self.legend.addItem(self.dataItemCmd, 'Cmd')
        self.legend.addItem(self.dataItemMeas, 'Meas')


    def _addDataItems(self):
        # Green pen with few tranparency
        self.dataItemCmd = DataItemSameTime(pen = pg.mkPen(80, 255, 80, 200))
        self.addItem(self.dataItemCmd)
        # Red pen with few tranparency
        self.dataItemMeas = DataItemSameTime(pen = pg.mkPen(255, 80, 80, 200))
        self.addItem(self.dataItemMeas)


    def appendCommand(self, value):
        self.dataItemCmd.addPoint(value)


    def appendMeas(self, value):
        self.dataItemMeas.addPoint(value)


    def setLegends(self, cmd, meas):
        # FIXME: Following works once and this is ugly.
        self.legend.removeItem('Cmd')
        self.legend.removeItem('Meas')
        self.legend.addItem(self.dataItemCmd, cmd)
        self.legend.addItem(self.dataItemMeas, meas)


    def clear(self):
        pg.PlotItem.clear(self)
        self._addDataItems()



# Create plots

lw = CurveOverTime('Left wheel')
lw.setLegends('motor_cmd', 'qdec_speed')
win.addItem(lw, row=0, col=1)

rw = CurveOverTime('Right wheel')
rw.setLegends('motor_cmd', 'qdec_speed')
win.addItem(rw, row=1, col=1)

class RobotTelemetry():
    """
    Aggregates multi-lines measurements

    Class methods manages a thread safe FIFO of instances
    """

    telemetry_points = []
    lock = Lock()

    @classmethod
    def push(cls, point):
        """
        Store a telemetry record in FIFO.
        Intended to be called by Parser threads
        """
        cls.lock.acquire()
        cls.telemetry_points.append(point)
        cls.lock.release()

    @classmethod
    def pop(cls):
        """
        Load a telemetry record from FIFO.
        Intended to be called by Qt event loop, from refresh timer callback.
        """
        point = None

        cls.lock.acquire()
        if len(cls.telemetry_points):
            point = cls.telemetry_points.pop(0)
        cls.lock.release()

        return point

    def __init__(self, current_cycle):
        self.current_cycle = current_cycle # might not be needed...
        self.speed_order = (0., 0.)
        self.speed_current = (0., 0.)
        self.qdec_speed = (0, 0)
        self.motor_cmd = (0, 0)

        self.pose_order = (0., 0., 0.)
        self.pose_current = (0., 0., 0.)

    def set_speed_order(self, speed_lin, speed_order_ang):
        self.speed_order = (speed_lin, speed_order_ang)

    def set_speed_current(self, speed_lin, speed_ang):
        self.speed_current = (speed_lin, speed_ang)

    def set_qdec_speed(self, qdec_speed_left, qdec_speed_right):
        self.qdec_speed = (qdec_speed_left, qdec_speed_right)

    def set_motor_cmd(self, motor_cmd_left, motor_cmd_right):
        self.motor_cmd = (motor_cmd_left, motor_cmd_right)

    def set_pose_order(self, x, y, theta):
        self.pose_order = (x, y, theta)

    def set_pose_current(self, x, y, theta):
        self.pose_current = (x, y, theta)

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
