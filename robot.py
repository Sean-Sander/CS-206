import pybullet as p
import pyrosim.pyrosim as pyrosim
import numpy as np
from sensor import SENSOR
from motor import MOTOR

from pyrosim.neuralNetwork import NEURAL_NETWORK

import constants as c
class ROBOT:
    def __init__(self):
        self.robot = p.loadURDF('body.urdf')
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain.nndf")
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
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                #print(jointName, neuronName, desiredAngle)
                self.motors[jointName].Set_Value(desiredAngle, self.robot, p)

        #for b in self.motors:
            #self.motors[b].Set_Value(time, self.robot, p)
    def Think(self):
        self.nn.Update()
        self.nn.Print()

    def Get_Fitness(self):
        self.stateOfLinkZero = p.getLinkState(self.robot, 0)
        self.positionOfLinkZero = self.stateOfLinkZero[0]
        self.XCoordinateOfLinkZero = self.positionOfLinkZero[0]
        #print(self.stateOfLinkZero)
        #print(self.positionOfLinkZero)
        #print(self.XCoordinateOfLinkZero)
        with open("data/fitness.txt", "w") as f:
            f.write(str(self.XCoordinateOfLinkZero))

