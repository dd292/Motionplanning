import numpy
from RRTTree import RRTTree
import math
import time

class AnytimeRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
            start_time = time.time()
            start = [round(q,3) for q in start_config]
            goal= [round(f,3) for f in goal_config]
            num_turns= 1/epsilon
            sumel= float("inf")
            anytime =0
            tree=[0]*3
            while anytime<3:
                tree[anytime] = RRTTree(self.planning_env, start_config,goal_config)
                count =0
                prob=10
                while count != num_turns:
                    count= count+1
                    sumrand=0
                    it = True
                    if prob>3:
                        while (it==True):
                            rand= self.planning_env.generate_random_configuration()
                            sumrand= self.planning_env.compute_distance(rand,start)+self.planning_env.compute_distance(rand,goal)
                            if sumrand<= sumel:
                                it =False
                        prob = prob-1
                    else:
                        rand= goal
                        prob=10
                    #print "reached here"
                    Nid,neighbour= tree[anytime].GetNearestVertex(rand)
                    new_guy= self.planning_env.extend(neighbour,rand)
                    #print "count",count
                    if self.planning_env.state_validity_checker(new_guy):
                        count= count-1
                        continue

                    New_id= tree[anytime].AddVertex(new_guy)
                    #print "neighbour",neighbour,"new_guy",new_guy
                    tree[anytime].AddEdge(Nid,New_id)
                    #if self.planning_env.dimension==2:
                    self.planning_env.PlotEdge(neighbour,new_guy)
                    #print "new_guy",new_guy
                    #c= input("stop")

                    if new_guy==goal:
                        print "Goal Reached"
                        last=New_id
                        book=[]
                        while last!=0:
                            book.append(tree[anytime].vertices[last])
                            last= tree[anytime].edges[last]
                        book.append(tree[anytime].vertices[last])
                        x= len(book)
                        nicebook= [0]*x
                        for i in range(x):
                            nicebook[i]= book[x-i-1]
                        #print nicebook
                        dist= 0
                        f=start
                        for i in nicebook:
                            dist = dist + self.planning_env.compute_distance(f,i)
                            f = i
                        print "total path =",dist
                        sumel= self.planning_env.calc_elipse(nicebook)
                        print "sumel",sumel
                        print "count",count
                        anytime=anytime+1
                        if anytime==3:

                            print("--- %s seconds ---" % (time.time() - start_time))
                            return nicebook
                        break
                if count >=num_turns:
                    print "Goal NOT foud"
                    return 0
