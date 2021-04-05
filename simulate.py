import numpy
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time
import math
import random

import constants as c
from simulation import *
import sys

if len(sys.argv) >= 1:
    directOrGUI = sys.argv[1]
else:
    directOrGUI = ""

simulation = SIMULATION(directOrGUI)

simulation.Run()

simulation.Get_Fitness()

# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
#
# p.setGravity(0, 0, -9.8)
# planeId = p.loadURDF("plane.urdf")
# bodyId = p.loadURDF("body.urdf")
# p.loadSDF("world.sdf")
#
# backLegSensorValues = numpy.zeros(1000)
# frontLegSensorValues = numpy.zeros(1000)
# targetAnglesVectorFront = numpy.zeros(1000)
# targetAngles = numpy.linspace(-numpy.pi, numpy.pi, 1000)
# targetAnglesVectorBack = numpy.zeros(1000)
#
#
# for i in range(1000):
#     targetAnglesVectorFront[i] = c.amplitude_front * numpy.sin(c.frequency_front * targetAngles[i] + c.phaseOffset_front)
#     targetAnglesVectorBack[i] = c.amplitude_back * numpy.sin(c.frequency_back * targetAngles[i] + c.phaseOffset_back)
#
# numpy.save("data/front_targetAngles.npy", targetAnglesVectorFront)
# numpy.save("data/back_targetAngles.npy", targetAnglesVectorBack)
#
# pyrosim.Prepare_To_Simulate("body.urdf")
# for i in range(1000):
#     p.stepSimulation()
#     backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#     pyrosim.Set_Motor_For_Joint(bodyIndex=bodyId, jointName="Torso_BackLeg",
#                                 controlMode=p.POSITION_CONTROL, targetPosition=targetAnglesVectorBack[i],
#                                 maxForce=100)
#     pyrosim.Set_Motor_For_Joint(bodyIndex=bodyId, jointName="Torso_FrontLeg",
#                                 controlMode=p.POSITION_CONTROL, targetPosition=targetAnglesVectorFront[i],
#                                 maxForce=100)
#     time.sleep(1/60)
#
# p.disconnect()
# #print(backLegSensorValues)
#
# numpy.save("data/backLeg_sensor_data.npy", backLegSensorValues)
# numpy.save("data/frontLeg_sensor_data.npy", frontLegSensorValues)
