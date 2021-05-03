import numpy as np

DEBUG = False

arms = True

amplitude = np.pi/4.0
frequency = 10
phaseOffset = 0

numSensorNeurons = 11
numMotorNeurons = 10

if not arms:
    numSensorNeurons = 7
    numMotorNeurons = 6

motorJointRange = 0.35

numberOfGenerations = 15
populationSize = 20

count = 1000
maxForceConst = 20