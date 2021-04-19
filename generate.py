import pyrosim.pyrosim as pyrosim
import random
import time

length = 1
width = 1
height = 1

def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[2, 2, .5],
                      size=[length, width, height])
    pyrosim.End()
    #time.sleep(1)

def Generate_Body():
    pyrosim.Start_URDF("body.urdf")
    # Torso
    pyrosim.Send_Cube(name="Torso", pos=[1.5, 0, 1.5],
                      size=[length, width, height])
    # Back Leg
    pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso",
                       child="BackLeg", type="revolute",
                       position="1 0 1")
    pyrosim.Send_Cube(name="BackLeg", pos=[-.5, 0, -.5],
                      size=[length, width, height])
    # Front Leg
    pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso",
                       child="FrontLeg", type="revolute",
                       position="2 0 1")
    pyrosim.Send_Cube(name="FrontLeg", pos=[.5, 0, -.5],
                      size=[length, width, height])
    pyrosim.End()
    #time.sleep(1)

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
    pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")
    for i in range(3):
        for j in range(3, 5):
            pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j, weight=random.randint(-1, 1))
    pyrosim.End()
    #time.sleep(1)

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    #Torso
    pyrosim.Send_Cube(name="Torso", pos=[1.5, 0, 1.5],
                      size=[length, width, height])
    #Back Leg
    pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso",
                      child="BackLeg", type="revolute",
                      position="1 0 1")
    pyrosim.Send_Cube(name="BackLeg", pos=[-.5, 0, -.5],
                      size=[length, width, height])
    #Front Leg
    pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso",
                       child="FrontLeg", type="revolute",
                       position="2 0 1")
    pyrosim.Send_Cube(name="FrontLeg", pos=[.5, 0, -.5],
                      size=[length, width, height])
    pyrosim.End()

Create_World()
Generate_Body()
Generate_Brain()




