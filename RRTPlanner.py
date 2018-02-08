import numpy
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        tree = RRTTree(self.planning_env, start_config,goal_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_time = time.time()
        count =0
        start = [round(q,2) for q in start_config]
        goal= [round(f,2) for f in goal_config]
        num_turns= 1/epsilon
        prob=10
        #print "rached here"
        while count != num_turns:
            count= count+1
            if prob>3:
                rand= self.planning_env.generate_random_configuration()
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
            New_id= tree.AddVertex(new_guy)
            #print "neighbour",neighbour,"new_guy",new_guy
            tree.AddEdge(Nid,New_id)
            #if self.planning_env.dimension==2:
                #self.planning_env.PlotEdge(neighbour,new_guy)
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

                print("--- %s seconds ---" % (time.time() - start_time))
                print "count",count
                dist =0
                f = start
                for i in nicebook:
                    dist = dist + self.planning_env.compute_distance(f,i)
                    f = i
                print "total path =",dist
                print "nicebookend ",nicebook[len(nicebook)-1]
                print "goal",goal
                return nicebook
        print "count",count
        print "Goal NOT foud"
        return 0
