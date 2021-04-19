from world import *
from robot import *
from sensor import *
from motor import *

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim

import constants as c
import time

class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.solutionID = solutionID
        self.world = WORLD()
        self.robot = ROBOT(self.solutionID)

    def Run(self):
        for i in range(c.count):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i, p)
            # backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
            if self.directOrGUI == "GUI":
                time.sleep(1/60)

    def Get_Fitness(self):
        self.robot.Get_Fitness(self.solutionID)

    def __del__(self):
        p.disconnect()
