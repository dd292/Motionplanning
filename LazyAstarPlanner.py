import numpy
from PriorityQueue import PriorityQueue
import math
import copy
import time

class LazyAstarPlanner(object):

    def __init__(self, planning_env,visualize):
        self.planning_env = planning_env
        self.visualize = visualize
    def Plan(self, start_config, goal_config):
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        ## INITIALIZATIONS
        #print "start_config",start_config
        #print "goal_config",goal_config
        start_time= time.time()
        startID= self.planning_env.find_ID(start_config)
        goalID= self.planning_env.find_ID(goal_config)
        print "IDs",startID,goalID
        PQ= PriorityQueue()
        g={}
        closedlist=[]
        PQ.put(startID,0)
        truecost={}
        g[startID]=0
        truecost[startID]=True
        parent={}
        parent[startID]=0
        #   Starting of planner
        while not PQ.empty():
            current= PQ.get()

            #print "current",current
            if current==goalID:
                print "goal reached"
                thing=copy.copy(goalID)
                path= numpy.array([self.planning_env.IDtoconfig(thing)])
                npath= numpy.array([thing])
                thing= parent[thing]
                while thing!=0:
                    path= numpy.vstack((numpy.array(self.planning_env.IDtoconfig(thing)),path))
                    npath= numpy.vstack((numpy.array(thing),npath))
                    thing = parent[thing]
                path= numpy.vstack((numpy.array(self.planning_env.IDtoconfig(thing)),path))
                npath= numpy.vstack((numpy.array(thing),npath))
                #print "nodes expanded:",count
                print "npath:",path
                #print "self.planning_env.count",self.planning_env.count
                print "Number of edge evalualtion",self.planning_env.count
                print "FInal TIme ", time.time()-start_time
                print "Final cost",g[goalID]
                #if self.planning_env.space_dim==2:
                #    self.planning_env.plotthegraph(path)
                return path
                break
            if current in closedlist:
                continue
            elif truecost[current]:# tells if its in collision or not
                closedlist.append(current)
                for neighbor in self.planning_env.get_successors(current):
                    if neighbor not in closedlist:
                        #print "neighbor",neighbor

                        tempcost= g[current] + self.planning_env.compute_distance(current,neighbor)
                        if neighbor not in g or tempcost < g[neighbor]:
                            g[neighbor]=tempcost
                            priority= tempcost+ self.planning_env.get_heuristic(neighbor,goalID)
                            PQ.put(neighbor,priority)
                            truecost[neighbor]= False
                            parent[neighbor]= current


            else:
                temp= self.planning_env.gettruecost(parent[current],current)
                #print "temp,node",temp,current
                if temp<float('inf'):
                    truecost[current]=True
                    g_here= g[parent[current]]+temp
                    #self.planning_env.PlotEdge(neighbour,new_guy)
                    if current not in g or g_here<=g[current]:
                        #print "coming here"
                        g[current]= g_here
                        priority= g_here + self.planning_env.get_heuristic(neighbor,goalID)
                        PQ.put(current,priority)


        print "Goal not found"



        #startID= self.planning_env.configuration_to_nodeID(start_config)
        plan = []

        plan.append(start_config)
        plan.append(goal_config)

        return plan
