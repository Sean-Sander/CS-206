import numpy as np
import pyrosim.pyrosim as pyrosim
import os
from constants import *
import random
import time

length = 0.2
width = 1
height = 0.2

class SOLUTION:
    def __init__(self, ID):
        self.weights = (np.random.rand(numSensorNeurons, numMotorNeurons) * 2) - 1
        self.myID = ID

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2, 2, .5],
                          size=[length, width, height])
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        # Torso
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1],
                          size=[1, 1, 1])
        # Back Leg
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso",
                           child="BackLeg", type="revolute",
                           position="0 -.5 1", jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0, -.5, 0],
                          size=[length, width, height])
        # Back Leg Bottom
        pyrosim.Send_Joint(name="BackLeg_BackLowerLeg", parent="BackLeg",
                           child="BackLowerLeg", type="revolute",
                           position="0 -1 0", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -.5],
                          size=[.2, .2, 1])
        # Left Leg
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso",
                           child="LeftLeg", type="revolute",
                           position="-.5 0 1", jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-.5, 0, 0],
                          size=[width, length, height])
        # Left Leg Bottom
        pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg", parent="LeftLeg",
                           child="LeftLowerLeg", type="revolute",
                           position="-1 0 0", jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -.5],
                          size=[.2, .2, 1])
        # Right Leg
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso",
                           child="RightLeg", type="revolute",
                           position=".5 0 1", jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[.5, 0, 0],
                          size=[width, length, height])
        # Right Leg Bottom
        pyrosim.Send_Joint(name="RightLeg_RightLowerLeg", parent="RightLeg",
                           child="RightLowerLeg", type="revolute",
                           position="1 0 0", jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -.5],
                          size=[.2, .2, 1])
        # Front Leg
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso",
                           child="FrontLeg", type="revolute",
                           position="0 .5 1", jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0],
                          size=[length, width, height])
        # Front Leg Bottom
        pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg", parent="FrontLeg",
                           child="FrontLowerLeg", type="revolute",
                           position="0 1 0", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -.5],
                          size=[.2, .2, 1])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        time.sleep(.1)
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="FrontLeg")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name=5, linkName="LeftLeg")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="RightLeg")
        pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLowerLeg")
        pyrosim.Send_Motor_Neuron(name=9, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=10, jointName="BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=12, jointName="LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron(name=13, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name=14, jointName="RightLeg_RightLowerLeg")
        pyrosim.Send_Motor_Neuron(name=15, jointName="Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name=16, jointName="FrontLeg_FrontLowerLeg")
        for currentRow in range(0, numSensorNeurons):
            for currentColumn in range(0, numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+numSensorNeurons,
                                     weight=self.weights[currentRow][currentColumn])

        pyrosim.End()

    def Mutate(self):
        self.weights[random.randint(0, numMotorNeurons),
                     random.randint(0, 1)] = (random.random() * numMotorNeurons) - 1

    def Evaluate(self, directOrGUI):
        pass
        #time.sleep(1)


    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        if (directOrGUI == "GUI"):
            os.system("start python3 simulate.py " + directOrGUI + " " + str(self.myID))
        else:
            os.system("start /b python3 simulate.py " + directOrGUI + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        fitnessFile = "fitness" + str(self.myID) + ".txt"
        #print(fitnessFile)
        while not os.path.exists(fitnessFile):
            time.sleep(0.01)
        with open(fitnessFile, "r") as f:
            self.fitness = float(f.read())
            #print(self.fitness)
        os.system("del fitness" + str(self.myID) + ".txt")

    def Set_ID(self, ID):
        self.myID = ID
