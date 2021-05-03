import os
import sys
from parallelHillClimber import *

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()

#for i in range(2):
#    os.system("python3 generate.py")
#    os.system("python3 simulate.py")