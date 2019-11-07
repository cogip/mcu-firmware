#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import FreeCAD
from FreeCAD import Part
import FreeCADGui
import os
import psutil
import re
from serial import *
import serial
import subprocess
import signal
import sys
from threading import Thread

BASE_PATH = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Macro").GetString("MacroPath") + "/cogip/simulation/"
BIN_NAME = "cortex-simulation.elf"
BIN_PATH = BASE_PATH + "bin/cogip2019-cortex-native/" + BIN_NAME
ROBOT_DESIGN_PATH = BASE_PATH + "Robot.step"
AREA_DESIGN_PATH = BASE_PATH + "Table.iges"
OBSTACLE_DESIGN_PATH = BASE_PATH + "Robot.step"
FCD_DOC_NAME = "RobotSimulation"
FCD_ROBOT_NAME = "Robot_{}_pose_{}"
FCD_OBSTACLE_NAME = "Obstacle_{}_pose_{}"


class Simulator:
    @classmethod
    def init_fcd_doc(cls):
        # Create document if not already opened
        try:
            cls.fcd_doc = FreeCAD.getDocument(FCD_DOC_NAME)
        except NameError:
            cls.fcd_doc = FreeCAD.newDocument(FCD_DOC_NAME)

        FreeCAD.Gui.setActiveDocument(cls.fcd_doc)
        # Clean FreeCAD document
        for obj in cls.fcd_doc.Objects:
            # Clean robots objects
            if obj.Label.startswith('Robot_'):
                Robot.do_init_shape = False
            if obj.Label.startswith('Area'):
                Area.do_init_shape = False

            cls.fcd_doc.getObject(obj.Label).ViewObject.Visibility = False

        Robot.init_shape()
        Obstacle.init_shape()
        Area.init_shape()


class FcdObject:
    __fcd_objects = {}
    do_init_shape = True

    @classmethod
    def import_shape_from_file(cls):
        # Get shape from file path
        cls.fcd_shape = Part.Shape()
        cls.fcd_shape.read(cls._design_path)

    @classmethod
    def create_default_shape(cls):
        # Create default shape, a simple box
        cls.fcd_shape = Part.makeBox(cls.default_length, cls.default_width, cls.default_height)

    @classmethod
    def init_shape(cls):
        if cls.do_init_shape:
            try:
                cls.import_shape_from_file()
            except (FreeCAD.Base.FreeCADError, AttributeError, FileNotFoundError):
                cls.create_default_shape()

            # Prepare the first object and hide it
            cls.get_fcd_object(0)

    @classmethod
    def get_fcd_object(cls, fcd_object_id):
        if fcd_object_id not in cls.__fcd_objects:
            cls.__fcd_objects[fcd_object_id] = cls(fcd_object_id)
        return cls.__fcd_objects[fcd_object_id]


class Area(FcdObject):
    default_length = 3000
    default_width = 3000
    default_height = 10
    # Area design path
    _design_path = AREA_DESIGN_PATH
    # Freecad shape
    fcd_shape = None

    def __init__(self, _design_path=None):
        # Add area
        self.fcd_area = Simulator.fcd_doc.getObject("Area")
        if not self.fcd_area:
            self.fcd_area = Simulator.fcd_doc.addObject("Part::Feature", "Area")
            self.fcd_area.Shape = Area.fcd_shape
            if self.fcd_area.Shape.ShapeType == 'Solid':
                self.fcd_area.Placement.Base = FreeCAD.Vector(-self.default_length / 2, -self.default_width / 2, -10)
        else:
            self.fcd_area.ViewObject.Visibility = True


class Robot(FcdObject):
    # Default box dimensions if no robot design is provided
    default_length = 300
    default_width = 300
    default_height = 375
    # Offsets for placement
    offset_length = 0
    offset_width = 0
    offset_height = 0
    offset_angle = 0
    # Robot design path
    _design_path = ROBOT_DESIGN_PATH
    fcd_shape = None

    def __init__(self, robot_id):
        self.robot_id = robot_id

        self.fcd_robot_pose_current = Simulator.fcd_doc.getObject(FCD_ROBOT_NAME.format(int(self.robot_id), "current"))
        self.fcd_robot_pose_order = Simulator.fcd_doc.getObject(FCD_ROBOT_NAME.format(int(self.robot_id), "order"))

        if not self.fcd_robot_pose_current or not self.fcd_robot_pose_order:
            # Add pose current object
            self.fcd_robot_pose_current = Simulator.fcd_doc.addObject("Part::Feature",
                                                                      FCD_ROBOT_NAME.format(int(self.robot_id), "current"))
            self.fcd_robot_pose_current.ViewObject.Visibility = True
            self.fcd_robot_pose_current.Shape = Robot.fcd_shape

            # Add pose order object
            self.fcd_robot_pose_order = Simulator.fcd_doc.addObject("Part::Feature",
                                                                      FCD_ROBOT_NAME.format(int(self.robot_id), "order"))
            self.fcd_robot_pose_order.ViewObject.Visibility = True
            self.fcd_robot_pose_order.Shape = Robot.fcd_shape

            Simulator.fcd_doc.recompute()
        else:
            self.fcd_robot_pose_current.ViewObject.Visibility = True
            self.fcd_robot_pose_order.ViewObject.Visibility = True

        if self.fcd_robot_pose_current.Shape.ShapeType == 'Solid' \
           and self.fcd_robot_pose_order.Shape.ShapeType == 'Solid':
            # Define box offset as reference point is a box corner
            self.offset_length = self.default_length / 2
            self.offset_width = self.default_width / 2
            self.offset_height = 0

        # Set pose order object transparency for visibility
        self.fcd_robot_pose_order.ViewObject.Transparency = 90

    def set_pose(self, fcd_obj, params):
        x = float(params[0])
        y = float(params[1])
        teta = float(params[2])

        # Reference point for Part::Box is the (min(x), min(y), min(z)) corner.
        # Replace the center of the box on the target point by adding half of robot lentgh.
        position = FreeCAD.Vector(x - self.offset_length, y - self.offset_width, self.offset_height)
        # Rotation
        rotation = FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), teta + self.offset_angle)
        # Rotation axis is the center relatively to the position point
        center = FreeCAD.Vector(self.offset_length, self.offset_width, 0)
        fcd_obj.Placement = FreeCAD.Placement(position, rotation, center)

    def set_pose_order(self, params):
        self.set_pose(self.fcd_robot_pose_order, params)

    def set_pose_current(self, params):
        self.set_pose(self.fcd_robot_pose_current, params)

class Obstacle(FcdObject):
    # Default box dimensions if no obstacle design is provided
    default_length = 300
    default_width = 300
    default_height = 375
    # Offsets for placement
    offset_length = 0
    offset_width = 0
    offset_height = 0
    offset_angle = 0
    # Obstacle design path
    _design_path = OBSTACLE_DESIGN_PATH
    fcd_shape = None

    def __init__(self, obstacle_id):
        self.obstacle_id = obstacle_id

        self.fcd_obstacle_pose = Simulator.fcd_doc.getObject(FCD_OBSTACLE_NAME.format(int(self.obstacle_id), "current"))
        self.fcd_obstacle_pose_order = Simulator.fcd_doc.getObject(FCD_OBSTACLE_NAME.format(int(self.obstacle_id), "order"))

        if not self.fcd_obstacle_pose:
            # Add pose current object
            self.fcd_obstacle_pose = Simulator.fcd_doc.addObject("Part::Feature",
                                                                      FCD_OBSTACLE_NAME.format(int(self.obstacle_id), "current"))
            self.fcd_obstacle_pose.ViewObject.Visibility = True
            self.fcd_obstacle_pose.Shape = Obstacle.fcd_shape

            Simulator.fcd_doc.recompute()
        else:
            self.fcd_obstacle_pose.ViewObject.Visibility = True

        if self.fcd_obstacle_pose.Shape.ShapeType == 'Solid':
            # Define box offset as reference point is a box corner
            self.offset_length = self.default_length / 2
            self.offset_width = self.default_width / 2
            self.offset_height = 0

    def set_pose(self, fcd_obj, params):
        x = float(params[0])
        y = float(params[1])
        teta = float(params[2])

        # Reference point for Part::Box is the (min(x), min(y), min(z)) corner.
        # Replace the center of the box on the target point by adding half of obstacle lentgh.
        position = FreeCAD.Vector(x - self.offset_length, y - self.offset_width, self.offset_height)
        # Rotation
        rotation = FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), teta + self.offset_angle)
        # Rotation axis is the center relatively to the position point
        center = FreeCAD.Vector(self.offset_length, self.offset_width, 0)
        fcd_obj.Placement = FreeCAD.Placement(position, rotation, center)


class Parser(Thread):
    STARTLINE_PATTERN = b'>>>>'
    ROBOT_OBJECT_PATTERN = b'@robot@'
    OBSTACLE_OBJECT_PATTERN = b'@obstacle@'
    POSE_ORDER_PATTERN = b'@pose_order@'
    POSE_CURRENT_PATTERN = b'@pose_current@'

    def _decode(self, line):
        # Remove trailing spaces or line return
        line = line.rstrip().rstrip(b',')
        output = sys.stdout
        if re.match(b'@.*@,.*', line) is not None:
            # Split parameters
            params = line.split(b',')

            # Extract objects parameters
            try:
                obj_type = params[0]
                obj_id = params[1]
                obj_param = params[3]
            except IndexError:
                return

            if obj_type == self.ROBOT_OBJECT_PATTERN:
                robot = Robot.get_fcd_object(obj_id)
                if obj_param == self.POSE_ORDER_PATTERN:
                    robot.set_pose_order(params[4:])
                elif obj_param == self.POSE_CURRENT_PATTERN:
                    robot.set_pose_current(params[4:])
                else:
                    output = sys.stderr
                print(line, file=output)
            elif obj_type == self.OBSTACLE_OBJECT_PATTERN:
                if len(params) == 11:
                    obstacle = Obstacle.get_fcd_object(obj_id)
                    x=(int(params[3]) + int(params[7])) / 2
                    y=(int(params[4]) + int(params[8])) / 2
                    O=int(params[11])

                    obstacle.set_pose(obstacle.fcd_obstacle_pose, (x, y, O))

                    print(line, file=output)

    def parse(self, flow):
        while True:
            try:
                line = bytes(flow.readline())
            except serial.serialutil.SerialException:
                self.ser.close()
                self.ser.open()
            if line.startswith(self.STARTLINE_PATTERN):
                break
            self._decode(line)
        print('Parsing finished')


class NativeParser(Parser):

    def parse(self, cmd):
        p = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        super(NativeParser, self).parse(p.stdout)

class SerialParser(Parser):
    def __init__(self, serial_port):
        try:
            self.ser = Serial(port=serial_port, baudrate=115200, timeout=1, writeTimeout=1)
        except serial.serialutil.SerialException:
            self.ser.close()
            self.ser.open()

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

    sys.stdout = FreeCAD.Console

    # Create FreeCAD document
    Simulator.init_fcd_doc()

    # Create robot area
    area = Area()

    # 3D view
    FreeCAD.Gui.ActiveDocument.activeView().viewAxonometric()

    # Scale display to fit all elements
    FreeCADGui.SendMsgToActiveView("ViewFit")

    parser = NativeParser()
    thread = Thread(target=parser.parse, args=(BIN_PATH,))
    #parser = SerialParser("/dev/ttyUSB0")
    #thread = Thread(target=parser.parse)
    thread.daemon = True
    thread.start()
