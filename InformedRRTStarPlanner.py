import numpy
import math
from RRTTree import RRTTree
import time

class InformedRRTStarPlanner(object):

    def __init__(self, planning_env,visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.rgoal=0.2

    def Plan(self, start_config, goal_config):
        epsilon=0.001
        start_time= time.time()
        tree = RRTTree(self.planning_env, start_config,goal_config)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        start_time = time.time()
        count =0
        start = [round(q,2) for q in start_config]
        goal= [round(f,2) for f in goal_config]
        num_turns= 1/epsilon
        prob=10
        constant=10
        #solution_set={}
        n=0
        cost={}
        cost[0]=0
        ellipse= float("inf")
        informed=1
        while informed==1:
            while count != num_turns:
                count= count+1
                sumrand=0
                it = True
                if prob>3:
                    while (it==True):
                        rand= self.planning_env.generate_random_configuration()
                        sumrand= self.planning_env.compute_distance(rand,start)+self.planning_env.compute_distance(rand,goal)
                        if sumrand<= ellipse:
                            it =False
                    prob = prob-1
                else:
                    rand= goal
                    prob=10

                #c= input ("stop here")
                Nid,neighbour= tree.GetNearestVertex(rand)
                new_guy= self.planning_env.extend(neighbour,rand)
                if self.planning_env.state_validity_checker(new_guy):
                    count= count-1
                    continue
                if self.planning_env.edge_validity_checker(new_guy,neighbour):
                    count=count-1
                    continue

                r= ((math.log10(count)/count)**(1/len(new_guy)))*constant
                near ={}
                for nearID,i in enumerate(tree.vertices,0):
                    if self.planning_env.compute_distance(new_guy,i)< min(r,0.05):
                        near[nearID]=i
                xmin= Nid
                xmincon= neighbour
                cmin= cost[Nid]+self.planning_env.compute_distance(neighbour,new_guy)
                for nearID,nodes in near.iteritems():
                    #print "nearID",nearID,nodes
                    cnew= cost[nearID]+ self.planning_env.compute_distance(nodes,new_guy)
                    if cnew< cmin:
                        if self.planning_env.edge_validity_checker(nodes,new_guy)==False:
                            xmin = nearID
                            xmincon= nodes
                            cmin = cnew

                New_id= tree.AddVertex(new_guy)
                tree.AddEdge(xmin,New_id)
                #self.planning_env.PlotEdge(xmincon,new_guy)
                cost[New_id]=cmin
                #print "reached here",xmin,New_id
                for nearID,nodes in near.iteritems():
                    cnear = cost[nearID]
                    cnew= cost[New_id] +self.planning_env.compute_distance(nodes,new_guy)
                    if cnew< cnear:
                        if self.planning_env.edge_validity_checker(nodes,new_guy)==False:
                            del tree.edges[nearID]
                            tree.AddEdge(New_id,nearID)
                            #self.planning_env.PlotEdge(nodes,new_guy)
                if new_guy==goal:
                    n=n+1
                    #print "Goal Reached",n
                    #c= input("stop here")
                    #print "edges",tree.edges[New_id]

                    #print "vertices",tree.vertices

                    last=New_id
                    book=[]
                    while last!=0:
                        book.append(tree.vertices[last])
                        last= tree.edges[last]
                        #print "last",last
                        #print "tree.vertices",tree.vertices[last]

                    book.append(tree.vertices[last])
                    # "book",book
                    #c= input("stop here")
                    x= len(book)
                    nicebook= [0]*x
                    for i in range(x):
                        nicebook[i]= book[x-i-1]
                    #print "something is done"
                    if (time.time() - start_time)< 5:
                        ellipse= self.planning_env.calc_elipse(nicebook)
                        continue
                    #print "count",count
                    dist =0
                    f = start
                    for i in nicebook:
                        dist = dist + self.planning_env.compute_distance(f,i)
                        f = i
                    print "time take=",start_time-time.time()
                    print "total path =",dist
                    print "nicebookend ",nicebook[len(nicebook)-1]
                    print "goal",goal

                    return nicebook
                if new_guy==goal:
                    continue
                else:
                    break

        print "count",count
        print "Goal NOT foud"
        plan = []

        plan.append(start_config)
        plan.append(goal_config)

        return plan
