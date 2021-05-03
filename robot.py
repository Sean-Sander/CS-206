import pybullet as p
import pyrosim.pyrosim as pyrosim
import numpy as np
from sensor import SENSOR
from motor import MOTOR
from constants import *
import time
import os

from pyrosim.neuralNetwork import NEURAL_NETWORK

import constants as c
class ROBOT:
    def __init__(self, solutionID):
        while not os.path.exists('body.urdf'):
            time.sleep(0.01)
        self.robot = p.loadURDF('body.urdf')
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        os.system("del brain" + solutionID + ".nndf")

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
                desiredAngle = motorJointRange * self.nn.Get_Value_Of(neuronName)
                if DEBUG:
                    print(jointName, neuronName, desiredAngle)
                self.motors[jointName].Set_Value(desiredAngle, self.robot, p)

        #for b in self.motors:
            #self.motors[b].Set_Value(time, self.robot, p)
    def Think(self):
        self.nn.Update()
        if DEBUG:
            self.nn.Print()

    def Get_Fitness(self, solutionID):
        # Torso X
        self.stateOfLinkZero = p.getLinkState(self.robot, 0)
        self.positionOfLinkZero = self.stateOfLinkZero[0]
        self.XCoordinateOfLinkZero = self.positionOfLinkZero[0]

        # Torso Z
        self.ZCoordinateOfLinkZero = self.positionOfLinkZero[2]

        #print('HEIGHT:', self.ZCoordinateOfLinkZero)
        x_val = np.abs(self.XCoordinateOfLinkZero)
        z_val = self.ZCoordinateOfLinkZero
        #print('HEIGHT:', z_val)
        if z_val >= 1 and x_val >= 1:
            self.fitness = np.abs(x_val * z_val)
        elif z_val < 1.5:
            self.fitness = np.abs(x_val * .075*z_val)
        elif x_val < 1:
            self.fitness = np.abs(.075*x_val * z_val)
        else:
            self.fitness = np.abs(0.05*(x_val * z_val))

        with open("tmp" + str(solutionID) + ".txt", "w") as f:
            f.write(str(self.fitness))
            print(self.fitness)
        os.system("rename tmp" + str(solutionID) + ".txt fitness" + str(solutionID) + ".txt")
        #print(solutionID)
        #with open("fitness" + str(solutionID) + ".txt", "w") as f:
        #     f.write(str(self.XCoordinateOfLinkZero))

