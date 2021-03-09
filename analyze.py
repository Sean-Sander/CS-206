import numpy
import matplotlib.pyplot
import matplotlib.pylab

targetAnglesFront = numpy.load("data/front_targetAngles.npy")
targetAnglesBack = numpy.load("data/back_targetAngles.npy")
matplotlib.pylab.plot(targetAnglesFront, label="Target Angles")
matplotlib.pylab.plot(targetAnglesBack, label="Target Angles")
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

"""
backLegSensorValues = numpy.load("data/backLeg_sensor_data.npy")
matplotlib.pyplot.plot(backLegSensorValues, label="Back Leg", linewidth=1.5)
#matplotlib.pyplot.show()

frontLegSensorValues = numpy.load("data/frontLeg_sensor_data.npy")
matplotlib.pyplot.plot(frontLegSensorValues, label="Front Leg")
matplotlib.pyplot.legend()
"""