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
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(c.count):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i, p)
            # backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

            time.sleep(1/60)

    def __del__(self):
        p.disconnect()
