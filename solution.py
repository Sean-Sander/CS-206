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
        pyrosim.Send_Cube(name="Box", pos=[0, 0, 0.5],
                          size=[2, 30, 1])
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        # Torso
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 4],
                          size=[.75, .5, 1.5])
        # Legs
        # Left Thigh
        pyrosim.Send_Joint(name="Torso_LeftThigh", parent="Torso",
                           child="LeftThigh", type="revolute",
                           position=".5 0 3.5", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LeftThigh", pos=[0, 0, -.5],
                          size=[.25, .25, 1])
        # Right Thigh
        pyrosim.Send_Joint(name="Torso_RightThigh", parent="Torso",
                           child="RightThigh", type="revolute",
                           position="-.5 0 3.5", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="RightThigh", pos=[0, 0, -.5],
                          size=[.25, .25, 1])
        # Left Calf
        pyrosim.Send_Joint(name="LeftThigh_LeftCalf", parent="LeftThigh",
                           child="LeftCalf", type="revolute",
                           position="0 0 -1", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LeftCalf", pos=[0, 0, -.5],
                          size=[.25, .25, 1])
        # Right Calf
        pyrosim.Send_Joint(name="RightThigh_RightCalf", parent="RightThigh",
                           child="RightCalf", type="revolute",
                           position="0 0 -1", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="RightCalf", pos=[0, 0, -.5],
                          size=[.25, .25, 1])
        # # Left Foot
        pyrosim.Send_Joint(name="LeftCalf_LeftFoot", parent="LeftCalf",
                           child="LeftFoot", type="revolute",
                           position="0 0 -1", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LeftFoot", pos=[0, 0, 0],
                          size=[.5, .65, .125])
        # Right Foot
        pyrosim.Send_Joint(name="RightCalf_RightFoot", parent="RightCalf",
                           child="RightFoot", type="revolute",
                           position="0 0 -1", jointAxis="1 0 0")
        pyrosim.Send_Cube(name="RightFoot", pos=[0, 0, 0],
                          size=[.5, .65, .125])

        # Arms
        # Left Upper Arm
        if arms:
            pyrosim.Send_Joint(name="Torso_LeftUpperArm", parent="Torso",
                               child="LeftUpperArm", type="revolute",
                               position=".5 0 4.5", jointAxis="1 0 0")
            pyrosim.Send_Cube(name="LeftUpperArm", pos=[0, 0, -.375],
                              size=[.25, .25, .75])
            # Right Upper Arm
            pyrosim.Send_Joint(name="Torso_RightUpperArm", parent="Torso",
                               child="RightUpperArm", type="revolute",
                               position="-.5 0 4.5", jointAxis="1 0 0")
            pyrosim.Send_Cube(name="RightUpperArm", pos=[0, 0, -.375],
                              size=[.25, .25, .75])
            # Left Lower Arm
            pyrosim.Send_Joint(name="LeftUpperArm_LeftLowerArm", parent="LeftUpperArm",
                               child="LeftLowerArm", type="revolute",
                               position=".25 0 -.625", jointAxis="1 0 0")
            pyrosim.Send_Cube(name="LeftLowerArm", pos=[0, 0, -.5],
                              size=[.25, .25, 1])
            # Right Lower Arm
            pyrosim.Send_Joint(name="RightUpperArm_RightLowerArm", parent="RightUpperArm",
                               child="RightLowerArm", type="revolute",
                               position="-.25 0 -.625", jointAxis="1 0 0")
            pyrosim.Send_Cube(name="RightLowerArm", pos=[0, 0, -.5],
                              size=[.25, .25, 1])

        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        time.sleep(.1)
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")

        # Legs
        pyrosim.Send_Sensor_Neuron(name=1, linkName="RightThigh")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="LeftThigh")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="RightCalf")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="LeftCalf")
        if arms:
            pyrosim.Send_Sensor_Neuron(name=5, linkName="LeftUpperArm")
            pyrosim.Send_Sensor_Neuron(name=6, linkName="RightUpperArm")
            pyrosim.Send_Sensor_Neuron(name=7, linkName="LeftLowerArm")
            pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLowerArm")
            pyrosim.Send_Sensor_Neuron(name=9, linkName="LeftFoot")
            pyrosim.Send_Sensor_Neuron(name=10, linkName="RightFoot")
            pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_RightUpperArm")
            pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_LeftUpperArm")
            pyrosim.Send_Motor_Neuron(name=13, jointName="RightUpperArm_RightLowerArm")
            pyrosim.Send_Motor_Neuron(name=14, jointName="LeftUpperArm_LeftLowerArm")
            pyrosim.Send_Motor_Neuron(name=15, jointName="Torso_RightThigh")
            pyrosim.Send_Motor_Neuron(name=16, jointName="Torso_LeftThigh")
            pyrosim.Send_Motor_Neuron(name=17, jointName="RightThigh_RightCalf")
            pyrosim.Send_Motor_Neuron(name=18, jointName="LeftThigh_LeftCalf")
            pyrosim.Send_Motor_Neuron(name=19, jointName="LeftCalf_LeftFoot")
            pyrosim.Send_Motor_Neuron(name=20, jointName="RightCalf_RightFoot")
        else:
            pyrosim.Send_Sensor_Neuron(name=5, linkName="LeftFoot")
            pyrosim.Send_Sensor_Neuron(name=6, linkName="RightFoot")
            pyrosim.Send_Motor_Neuron(name=7, jointName="Torso_RightThigh")
            pyrosim.Send_Motor_Neuron(name=8, jointName="Torso_LeftThigh")
            pyrosim.Send_Motor_Neuron(name=9, jointName="RightThigh_RightCalf")
            pyrosim.Send_Motor_Neuron(name=10, jointName="LeftThigh_LeftCalf")
            pyrosim.Send_Motor_Neuron(name=11, jointName="LeftCalf_LeftFoot")
            pyrosim.Send_Motor_Neuron(name=12, jointName="RightCalf_RightFoot")

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
