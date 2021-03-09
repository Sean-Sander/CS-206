import pybullet as p
import pyrosim.pyrosim as pyrosim
import numpy as np
from sensor import SENSOR
from motor import MOTOR

import constants as c
class ROBOT:
    def __init__(self):
        self.robot = p.loadURDF('body.urdf')
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkname in pyrosim.linkNamesToIndices:
            self.sensors[linkname] = SENSOR(linkname)
    def Sense(self, time):
        for b in self.sensors:
            self.sensors[b].Get_Value(time)
    def Prepare_To_Act(self):
        self.motors = {}
        for jointname in pyrosim.jointNamesToIndices:
            self.motors[jointname] = MOTOR(jointname)
    def Act(self, time, p):
        for b in self.motors:
            self.motors[b].Set_Value(time, self.robot, p)

