import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = .5

for k in range(10):
    for j in range(6):
        for i in range(6):
            pyrosim.Send_Cube(pos=[i, j, k],
                              size=[length, width, height])
    length *= .9
    width *= .9
    height *= .9
"""
pyrosim.Send_Cube(name="Box", pos=[x, y, z],
                  size=[length, width, height])


pyrosim.Send_Cube(name="Box2", pos=[x+1, y, z+1],
                  size=[length, width, height])
"""
pyrosim.End()