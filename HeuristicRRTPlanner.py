import numpy
from RRTTree import RRTTree
import random
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        tree = RRTTree(self.planning_env, start_config,goal_config)
        plan = []
        start_time = time.time()
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the hrrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        count =0
        start = [round(q,2) for q in start_config]
        goal= [round(f,2) for f in goal_config]
        num_turns= 1/epsilon
        goal_prob=10
        prob=0.3


        optcost = self.planning_env.compute_distance(start,goal)
        #print "optcost",optcost
        while count != num_turns:
            count= count+1
            #print "count",count
            if goal_prob>3:
                if count>2:
                    r=0.5
                    m=0
                    while r>m :
                        r= random.random()
                        rand= self.planning_env.generate_random_configuration()
                        Nid,neighbour= tree.GetNearestVertex(rand)
                        #print "tree.fvalue[Nid]",tree.fvalue[Nid]
                        #print "max fvalue",max(tree.fvalue)
                        m= 1- ((tree.fvalue[Nid]-optcost)/(max(tree.fvalue)-optcost))
                        #print "m",m
                        m= min(m,prob)
                        #print "r",r
                        #print "out of this loop"
                        #c= input ("enter guy")

                else:
                    rand= self.planning_env.generate_random_configuration()
                    Nid,neighbour= tree.GetNearestVertex(rand)
                    #print "just came of the else"
                goal_prob= goal_prob-1
            else:
                rand= goal
                Nid,neighbour= tree.GetNearestVertex(rand)
                goal_prob=10

            new_guy= self.planning_env.extend(neighbour,rand)

            if self.planning_env.state_validity_checker(new_guy):
                count= count-1
                continue
            New_id= tree.AddVertex(new_guy)
            tree.AddCost(New_id,Nid)
            #print "neighbour",neighbour,"new_guy",new_guy
            tree.AddEdge(Nid,New_id)
            if self.planning_env.dimension==2:
                self.planning_env.PlotEdge(neighbour,new_guy)
            #print "new_guy",new_guy

            if new_guy==goal:

                print "Goal Reached"
                last=New_id
                book=[]
                while last!=0:
                    book.append(tree.vertices[last])
                    last= tree.edges[last]
                book.append(tree.vertices[last])
                x= len(book)
                nicebook= [0]*x
                for i in range(x):
                    nicebook[i]= book[x-i-1]
                dist= 0
                f=start
                for i in nicebook:
                    dist = dist + self.planning_env.compute_distance(f,i)
                    f = i
                print "total path =",dist
                print("--- %s seconds ---" % (time.time() - start_time))
                print "count",count
                return nicebook
        #c= input ("enter guy")
        #print "cost",tree.cost
        print "Goal NOT foud"
        return 0
