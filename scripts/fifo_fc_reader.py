#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import time
import FreeCAD
import subprocess
from threading import Thread
from PySide import QtCore

path = "/tmp/fifo_fc"
stop = 0
storing = False

class Reader(QtCore.QObject):
    instance = None

    def __new__(cls):
        if cls.instance is None:
            cls.instance = QtCore.QObject.__new__(cls)
            cls.instance.__initialized = False
        print ("Reader created !")
        return cls.instance

    def __init__(self):
        if(self.__initialized): return
        self.__initialized = True

        self.doc = None
        self.field = None
        self.box = None
        self.obs = None
        self.thread_reader = None
        self.timer = None

        print ("Reader initialized !")

    def myrun(self):
        if self.doc is None:
            try:
                App.loadFile(u"$HOME/Bureau/test.fcstd")
            except:
                print "File already opened."
        self.doc = App.getDocument("simulation")
        App.ActiveDocument = self.doc
        Gui.ActiveDocument = self.doc
        self.view = Gui.activeDocument().activeView()

        self.box = App.ActiveDocument.getObjectsByLabel("Robot")[0]
        self.obs = App.ActiveDocument.getObjectsByLabel("Obstacle")[0]

        self.doc.recompute()
        Gui.activeDocument().activeView().viewAxonometric()

        self.obs.ViewObject.Visibility = True

        if self.thread_reader is None:
            self.thread_reader = Thread(target=self._run)
            self.thread_reader.setDaemon(True)
        else:
            self.thread_reader.stop()
        self.thread_reader.start()

        self.x = self.box.Placement.Base.x
        self.y = self.box.Placement.Base.y
        self.O = self.box.Placement.Rotation.Angle

        self.obs_x = self.obs.Placement.Base.x
        self.obs_y = self.obs.Placement.Base.y
        self.obs_O = self.obs.Placement.Rotation.Angle

        if self.timer is None:
            self.timer = QtCore.QTimer()
            QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self._parser)
        else:
            self.timer.stop()

        self.timer.start(20)

    def _parser(self):
        self.box.Placement.Base = FreeCAD.Vector(self.x,self.y,10)
        self.box.Placement.Rotation = FreeCAD.Rotation(self.O, 0, 0)
        self.obs.Placement.Base = FreeCAD.Vector(self.obs_x,self.obs_y,10)
        self.obs.Placement.Rotation = FreeCAD.Rotation(self.obs_O, 0, 0)

    def _run(self):
        global path
        try:
            os.remove(path)
        except OSError:
            pass

        fifo = None
        while not os.path.exists(path):
            pass
        fifo = open(path, "r")

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
                    self.obs.ViewObject.Visibility = True
                    self.obs_x=(int(l.split(',')[1]) + int(l.split(',')[5])) /2
                    self.obs_y=(int(l.split(',')[2]) + int(l.split(',')[6])) /2
                    self.obs_O=int(l.split(',')[9])
                    print self.obs_x
                    print self.obs_y
                l = ''
            if c == '':
                break
        self.timer.stop()
        fifo.close()
        os.remove(path)
        print "It's over !"

    def stop(self):
        self.timer.stop()
try:
	reader
except NameError:
	reader = None
reader = Reader()
reader.myrun()
