import pyrosim.pyrosim as pyrosim
import numpy
import math
import constants as c

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffset

        self.motorValues = numpy.ones(c.count)

        for i in range(c.count):
            if (self.jointName == "Torso_BackLeg"):
                self.motorValues[i] = self.amplitude * math.sin(
                self.frequency * i / (c.count / (2 * math.pi)) + self.offset)
            else:
                self.motorValues[i] = self.amplitude * math.sin(
                self.frequency / 2 * i / (c.count / (2 * math.pi)) + self.offset)

    def Set_Value(self, desiredAngle, robotID, p):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotID, jointName=self.jointName, controlMode=p.POSITION_CONTROL,
                                    targetPosition=desiredAngle, maxForce=c.maxForceConst)

    def Save_Values(self):
        pass #TODO do it