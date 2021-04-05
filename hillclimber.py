import copy
from solution import SOLUTION
from constants import *

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Print(self):
        print("Parent: " + str(self.parent.fitness), "Child: " + str(self.child.fitness))

    def Select(self):
        if (self.parent.fitness > self.child.fitness):
            self.parent = self.child

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()

    def Show_Best(self):
        self.parent.Evaluate("GUI")

    def Evolve(self):
        self.parent.Evaluate("GUI")
        for currentGeneration in range(numberOfGenerations):
            self.Evolve_For_One_Generation()

