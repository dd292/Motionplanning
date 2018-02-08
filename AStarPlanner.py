import sys
import time
import numpy
#import IPython
import SimpleEnvironment as SE
from PriorityQueue import PriorityQueue
import copy
import time

class AStarPlanner(object):
    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.nodes = dict()
        self.PQ = PriorityQueue()

    def Plan(self, start_config, goal_config):
        print start_config
        print goal_config
        waypoints = {}
        cost_at_way = {}
        plan=[]
        start= self.planning_env.discrete_env.configuration_to_nodeID(start_config)
        goal= self.planning_env.discrete_env.configuration_to_nodeID(goal_config)
        print "start:",start
        print "goal:",goal
        x=[]
        y=[]
        count=0
        waypoints={}
        cost_at_way[start] = 0
        waypoints[start]=0
        self.PQ.put(start,0)
        weight=1

        while not self.PQ.empty():
            current= self.PQ.get()
            count = count +1
            #print "current",current
            if (current == goal):
                print "goal reached"
                thing=copy.copy(goal)
                path= numpy.array([self.planning_env.discrete_env.nodeID_to_configuration(thing)])
                npath= numpy.array([thing])
                thing= waypoints[thing]
                while thing!=0:

                    path= numpy.vstack((numpy.array(self.planning_env.discrete_env.nodeID_to_configuration(thing)),path))
                    npath= numpy.vstack((numpy.array(thing),npath))
                    thing = waypoints[thing]
                print "nodes expanded:",count
                print "npath:",path
                print "self.planning_env.count",self.planning_env.count
                print "Final cost",cost_at_way[goal]
                if self.planning_env.space_dim==2:
                    self.planning_env.plotthegraph(path)
                return path

            #print self.planning_env.get_successors(current)
            for next in self.planning_env.get_successors(current):
                #print "next:",next
                #print "waypoints[current]",waypoints[current]
                new_cost =  cost_at_way[current] +self.planning_env.compute_distance(current, next)+ 5*self.planning_env.modeswitch(current,next,waypoints[current])

                if next not in cost_at_way or new_cost < cost_at_way[next]:
                    #if next not in cost_at_way:
                        #print "new cost:",new_cost
                    #else:
                        #print "updated cost:",new_cost

                    cost_at_way[next] = new_cost
                    priority = new_cost +weight*self.planning_env.get_heuristic(next,goal)
                    self.PQ.put(next,priority)
                    waypoints[next]= current
                    #print "waypoint:",waypoints

        print "Path Not Found"
        return 0
        #waypoints_1d_array = [numpy.array(self.planning_env.discrete_env.nodeID_to_configuration(item)) for item in waypoints]

        #waypoints_array = numpy.array(waypoints_1d_array)
        #print waypoints_array
        #x= waypoints_array[:,0]
        #y= waypoints_array[:,1]
        #fig1= plt.figure ()
        #ax = fig1.add_subplot(111, aspect='equal')
        #plt.plot(x,y)

        #for val in self.planning_env.obstacles_list:
        #    width =0
        #    height=0
        #    x_obs= min([val[2*+i] for i in range(self.planning_env.discrete_env.dimension)])
        #    y_obs= min([val[int(2*(i+0.5))] for i in range(self.planning_env.discrete_env.dimension)])
        #    for i in range(self.planning_env.discrete_env.dimension):
        #        width= abs(width-val[2*i])
        #        height= abs(height-val[int(2*(i+0.5))])

#            ax.add_patch(patches.Rectangle((x_obs, y_obs), width, height,hatch='/'))
        #plt.show()


        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
#        planx
        #waypoints.append(start_config)
        #waypoints.append(goal_config)

        #return plan

if __name__ == '__main__':
    start_time = time.time()
    l= SE.SimpleEnvironment(0.025,2)
    d= AStarPlanner(l)

    source_config = numpy.ones(2)*0.1
    target_config = numpy.ones(2)*0.9
    plan = d.Plan(source_config, target_config)

    print("--- %s seconds ---" % (time.time() - start_time))
    print "plan:",plan
