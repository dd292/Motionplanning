import operator
import numpy


class RRTTree(object):

    def __init__(self, planning_env, start_config,goal_config):

        self.planning_env = planning_env
        self.vertices = []
        start= [round(i,2) for i in start_config]
        self.vertices.append(start)
        self.cost=[]
        self.goal=goal_config
        self.cost.append(0)
        self.fvalue=[]
        self.fvalue.append(self.planning_env.compute_distance(start_config,goal_config))
        self.edges = dict()


    def GetRootID(self):
        return 0

    def GetNearestVertex(self, config):

        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]

    def GetKNN(self, config, k):

        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.ComputeDistance(config, v))

        dists = numpy.array(dists)
        knnIDs = numpy.argpartition(dists, k)
        knnDists = [dists[i] for i in knnIDs]

        return knnIDs, [self.vertices[vid] for vid in knnIDs]


    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddCost(self, new,neighbour):
        H=self.planning_env.compute_distance(self.vertices[new],self.goal)
        G=self.planning_env.compute_distance(self.vertices[new],self.vertices[neighbour])
        self.cost.append(self.cost[neighbour]+G)
        self.fvalue.append(self.cost[neighbour]+G+H)


    def AddEdge(self, sid, eid):
        self.edges[eid] = sid
