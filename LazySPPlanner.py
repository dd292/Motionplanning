import numpy
from PriorityQueue import PriorityQueue
import math
import copy
import time

class LazySPPlanner(object):

    def __init__(self, planning_env,visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config):
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        #   Starting of planner
        start_time=time.time()
        startID= self.planning_env.find_ID(start_config)
        goalID= self.planning_env.find_ID(goal_config)
        print "IDs",startID,goalID
        PQ= PriorityQueue()
        g={}
        PQ.put(startID,0)
        g[startID]=0
        parent={}
        parent[startID]=-1
        x=0
        memory=[]
        #   Starting of planner
        while not PQ.empty():
            current= PQ.get()
            if current==goalID:
                x=x+1
                print "goal reached ",x
                thing=copy.copy(goalID)
                path= numpy.array([self.planning_env.IDtoconfig(thing)])
                npath= numpy.array([thing])
                thing= parent[thing]

                while thing!=-1:
                    path= numpy.vstack((numpy.array(self.planning_env.IDtoconfig(thing)),path))
                    npath= numpy.vstack((numpy.array(thing),npath))
                    thing = parent[thing]
                thing=copy.copy(goalID)
                collision= False
                while thing!= startID:
                    temp= self.planning_env.gettruecost(parent[thing],thing)
                    if temp==float('inf'):
                        memory.append([parent[thing],thing])
                        collision =True
                        #print collision
                    thing = parent[thing]
                #path= numpy.vstack((numpy.array(self.planning_env.IDtoconfig(thing)),path))
                #npath= numpy.vstack((numpy.array(thing),npath))
                #print "npath:",npath
                #print "memory",memory
                #c=input("stop")

                if collision==True:
                    PQ= PriorityQueue()
                    g={}
                    PQ.put(startID,0)
                    g[startID]=0
                    parent={}
                    parent[startID]=-1
                    continue
                print "Number of edge evalualtion",self.planning_env.count
                print "FInal TIme ", time.time()-start_time
                print "npath:",path
                #print "self.planning_env.count",self.planning_env.count
                print "Final cost",g[goalID]
                #if self.planning_env.space_dim==2:
                #    self.planning_env.plotthegraph(path)
                return path
                break
            for neighbor in self.planning_env.get_successors(current):
                if [current,neighbor] not in memory:
                    if [neighbor,current] not in memory:
                        tempcost= g[current] + self.planning_env.compute_distance(current,neighbor)
                        if neighbor not in g or tempcost < g[neighbor]:
                            g[neighbor]=tempcost
                            priority= tempcost+ self.planning_env.get_heuristic(neighbor,goalID)
                            PQ.put(neighbor,priority)
                            parent[neighbor]= current


        print "Goal not found"
