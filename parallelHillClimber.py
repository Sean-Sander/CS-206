import copy
from solution import SOLUTION
from constants import *
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Spawn(self):
        self.children = {}
        for id, robot in self.parents.items():
            self.children[id] = copy.deepcopy(robot)
            self.children[id].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            #print(self.children)

        #self.child = copy.deepcopy(self.parent)
        #self.child.Set_ID(self.nextAvailableID)
        #self.nextAvailableID += 1

    def Mutate(self):
        for keys, value in self.children.items():
            value.Mutate()

    def Print(self):
        #DEBUG = True
        if DEBUG:
            for keys, values in self.parents.items():
                print(' ')
                print(values.fitness, self.children[keys].fitness)
                print(' ')

    def Select(self):
        for keys, values in self.parents.items():
            if self.children[keys].fitness > values.fitness:
                self.parents[keys] = self.children[keys]

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Evaluate(self, solutions):
        for robot in solutions.keys():
            solutions[robot].Start_Simulation("DIRECT")
        for robot in solutions.keys():
            solutions[robot].Wait_For_Simulation_To_End()

    def Show_Best(self):
        lowest = 0
        lowest_key = -1
        for keys, values in self.parents.items():
            if values.fitness > lowest:
                lowest = values.fitness
                lowest_key = keys
                print(values.fitness)
        self.parents[lowest_key].Start_Simulation('GUI')
        # with open(self.weights_file + ".txt", 'w') as f:
        #     np.save(f, self.parents[lowest_key].weights)
        #print(self.parents[lowest_key].weights)

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(numberOfGenerations):
            print('\n\nGeneration:', currentGeneration, "\n\n")
            self.Evolve_For_One_Generation()


