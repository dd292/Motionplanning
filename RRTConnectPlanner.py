import numpy, operator
from RRTPlanner import RRTTree
import time

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        ftree = RRTTree(self.planning_env, start_config,goal_config)
        rtree = RRTTree(self.planning_env, goal_config,goal_config)
        plan = []
        start_time = time.time()
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        count =0
        start = [round(q,3) for q in start_config]
        goal= [round(f,3) for f in goal_config]
        num_turns= 1/epsilon
        while count != num_turns:
            count= count+1
            rand= self.planning_env.generate_random_configuration()
            #print "rand",rand
            #get neighbours
            fNid,fneighbour= ftree.GetNearestVertex(rand)
            rNid,rneighbour= rtree.GetNearestVertex(rand)

            fnew_guy= self.planning_env.connect(fneighbour,rand)

            rnew_guy= self.planning_env.connect(rneighbour,rand)
            fNew_id= ftree.AddVertex(fnew_guy)
            rNew_id= rtree.AddVertex(rnew_guy)
            #print "neighbour",neighbour,"new_guy",new_guy
            ftree.AddEdge(fNid,fNew_id)
            rtree.AddEdge(rNid,rNew_id)
            #print "new_guy",new_guy
            if self.planning_env.dimension==2:
                self.planning_env.PlotEdge(fneighbour,fnew_guy)
                self.planning_env.PlotEdge(rneighbour,rnew_guy)
            if fnew_guy==rnew_guy:
                print "Goal Reached"
                last=fNew_id
                book=[]
                while last!=0:
                    book.append(ftree.vertices[last])
                    last= ftree.edges[last]
                book.append(ftree.vertices[last])
                x= len(book)
                nicebook= [0]*x
                for i in range(x):
                    nicebook[i]= book[x-i-1]
                last = rNew_id
                while last!=0:
                    nicebook.append(rtree.vertices[last])
                    last= rtree.edges[last]
                nicebook.append(rtree.vertices[last])
                print("--- %s seconds ---" % (time.time() - start_time))
                print "count",count
                dist= 0
                f=start
                for i in nicebook:
                    dist = dist + self.planning_env.compute_distance(f,i)
                    f = i
                print "total path =",dist
                return nicebook
        print "Goal NOT foud"
        return 0

        return plan
