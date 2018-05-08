#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import time
import FreeCAD
import subprocess
from threading import Thread
from PySide import QtCore

FREECAD_PROJECT_PATH = "/home/khep/Documents/Robot/simulation.fcstd"
FIFO_PATH = "/tmp/fifo_fc"

class Reader(QtCore.QObject):
    instance = None

    def __new__(cls):
        if cls.instance is None:
            cls.instance = QtCore.QObject.__new__(cls)
            cls.instance.__initialized = False
        print ("Reader created !")
        return cls.instance

    def __init__(self):
        if (self.__initialized):
            return
        self.__initialized = True

        self.doc = None
        self.field = None
        self.robot = None
        self.thread_reader = None
        self.timer = None

        self.obstacles_list = list()
        self.current_obstacle_id = 0

        print ("Reader initialized !")

    def myrun(self):
        if self.doc is None:
            try:
                App.loadFile(FREECAD_PROJECT_PATH)
            except:
                print "File already opened."
        self.doc = App.getDocument("simulation")
        App.ActiveDocument = self.doc
        Gui.ActiveDocument = self.doc
        self.view = Gui.activeDocument().activeView()

        self.robot = App.ActiveDocument.getObjectsByLabel("Robot")[0]

        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle0")[0])
        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle1")[0])
        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle2")[0])
        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle3")[0])
        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle4")[0])
        self.obstacles_list.append(App.ActiveDocument.getObjectsByLabel("Obstacle5")[0])

        self.doc.recompute()
        Gui.activeDocument().activeView().viewAxonometric()

        for obstacle in self.obstacles_list:
            obstacle.ViewObject.Visibility = True

        if self.thread_reader is None:
            self.thread_reader = Thread(target=self._run)
            self.thread_reader.setDaemon(True)
        else:
            self.thread_reader.stop()
        self.thread_reader.start()

        self.x = self.robot.Placement.Base.x
        self.y = self.robot.Placement.Base.y
        self.O = self.robot.Placement.Rotation.Angle

        # Sets the obstacle value to the first one
        self.obstacle_x = self.obstacles_list[0].Placement.Base.x
        self.obstacle_y = self.obstacles_list[0].Placement.Base.y
        self.obstacle_O = self.obstacles_list[0].Placement.Rotation.Angle

        if self.timer is None:
            self.timer = QtCore.QTimer()
            QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self._parser)
        else:
            self.timer.stop()

        self.timer.start(20)

    def _parser(self):
        self.robot.Placement.Base = FreeCAD.Vector(self.x, self.y, 10)
        self.robot.Placement.Rotation = FreeCAD.Rotation(self.O, 0, 0)

        # Update obstacle list
        self.obstacles_list[self.current_obstacle_id].Placement.Base = FreeCAD.Vector(self.obstacle_x, self.obstacle_y, 10)
        self.obstacles_list[self.current_obstacle_id].Placement.Rotation = FreeCAD.Rotation(self.obstacle_O, 0, 0)

        if self.current_obstacle_id < len(self.obstacles_list):
            self.current_obstacle_id = self.current_obstacle_id + 1
        else:
            self.current_obstacle_id = 0

    def _run(self):
        try:
            os.remove(FIFO_PATH)
        except OSError:
            pass

        fifo = None
        while not os.path.exists(FIFO_PATH):
            pass
        fifo = open(FIFO_PATH, "r")

        l=''
        while True:
            c = fifo.read(1)
            l +=c
            if c == '\n':
                print l,
                if l == '' :
                    break
                if l.startswith("@c@"):
                    self.x=int(l.split(',')[4])
                    self.y=int(l.split(',')[5])
                    self.O=int(l.split(',')[6])
                elif l.startswith("@o@"):
                    self.obstacle.ViewObject.Visibility = True
                    self.obstacle_x=(int(l.split(',')[1]) + int(l.split(',')[5])) /2
                    self.obstacle_y=(int(l.split(',')[2]) + int(l.split(',')[6])) /2
                    self.obstacle_O=int(l.split(',')[9])
                    # print self.obstacle_x
                    # print self.obstacle_y
                l = ''
            if c == '':
                break
        self.timer.stop()
        fifo.close()
        os.remove(FIFO_PATH)
        print "That's all folks !'"

    def stop(self):
        self.timer.stop()

if __name__ == '__main__':
    if os.path.exists(FREECAD_PROJECT_PATH):
        reader = Reader()
        reader.myrun()
        sys.exit(0)
    else:
        print("Freecad project not found, please update the FREECAD_PROJECT_PATH to access your simulation.fcstd")
        sys.exit(1)
