import numpy
import matplotlib.pyplot as pl
import random
import openravepy
import math
from SimpleRobot import SimpleRobot
import copy

class SimpleEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        # goal sampling probability
        self.p = 0.0
        # extend parameter
        self.extendEpsilon = 0.05
        self.dimension= len(self.boundary_limits[0])
        # connect parameter
        self.connectStepSize = 0.01


    def set_goal_parameter(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def generate_random_configuration(self):
        config = [0] * self.dimension
        # TODO: Generate and return a random configuration
        #print self.dimension
        x= True
        while x==True:
            for i in range(self.dimension):
                config[i] = random.uniform(self.boundary_limits[0][i],self.boundary_limits[1][i])
            ret_rand= [round(i,2) for i in config]
            if self.state_validity_checker(ret_rand)==False:
                x=False
                return ret_rand


    def compute_distance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        ## TODO: Here you will implement a function which computes the
        # Euclidean distance between two states given their vertex IDs.
        #print "Input:u:",u," v:",v
        #print "vstate:",v_state
        #print "start and end",start_config, end_config
        dist = 0
        for x in range(self.dimension):
            dist = dist + math.pow(start_config[x] - end_config[x], 2)
        return math.sqrt(dist)



    def state_validity_checker(self, conf):

        # TODO: Implement a state validity checker

        #print "limits",limits
        #print "conf:",conf

        Trans =numpy.array([[ 1, 0, 0, conf[0]],
                            [0, 1,  0, conf[1]],
                            [ 0, 0, 1, 0],
                            [ 0, 0, 0, 1]])
        # old = self.robot.GetActiveDOFLimits()
        self.robot.SetTransform(Trans)
        invalid = self.robot.GetEnv().CheckCollision(self.robot)
        #print "invalid",invalid
        return invalid


    def extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration towards goal configuration
        #

        p1= numpy.array(start_config)
        p2= numpy.array(end_config)
        #print ";p1",p1
        #print "p2",p2
        if self.compute_distance(p1,p2)<self.extendEpsilon:
            return end_config

        new_config= p1 + (((p2-p1)/numpy.linalg.norm(p2-p1))*self.extendEpsilon)
        ret= [round(i,2) for i in new_config]
        #print "ret",ret
        return ret

    def connect(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to connect
        # start configuration to a goal configuration
        #
        if self.compute_distance(start_config,end_config)<self.extendEpsilon:
            return end_config
        #x=1/self.connectStepSize
        p1= numpy.array(start_config)
        p2= numpy.array(end_config)
        y=[]
        for i in range(1,int(1/self.connectStepSize)):
            y.append(i*self.connectStepSize)
        new_config= p1

        for id in y:
            x= copy.copy(new_config)
            new_config= p1 + (p2-p1)*(id)
            ##print "new_config",new_config
            if self.state_validity_checker(new_config):
                x_ret= [round(i,3) for i in x]
                #print "in collision"
                return x_ret
        #print "joined to the end"
        if self.state_validity_checker(end_config)==False:
            return end_config
        else:
            re_new_config= [round(i,2) for i in new_config]
            return re_new_config
    def connectV2(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to connect
        # start configuration to a goal configuration
        #
        if self.compute_distance(start_config,end_config)<self.extendEpsilon:
            return True
        #x=1/self.connectStepSize
        p1= numpy.array(start_config)
        p2= numpy.array(end_config)
        y=[]
        for i in range(1,int(1/self.connectStepSize)):
            y.append(i*self.connectStepSize)
        new_config= p1

        for id in y:
            x= copy.copy(new_config)
            new_config= p1 + (p2-p1)*(id)
            ##print "new_config",new_config
            if self.state_validity_checker(new_config):
                x_ret= [round(i,3) for i in x]
                #print "in collision"
                return False
        #print "joined to the end"
        if self.state_validity_checker(end_config)==False:
            return True

        else:
            return False

    def calc_elipse(self,path):
        # this function calculates the a and b of the elipse
        #for anytime RRT
        max_dista=0
        max_distb=0
        y= len(path)
        for i in path:
            a= self.compute_distance(i,path[0])
            b= self.compute_distance(i,path[y-1])
            if (max_dista+max_distb<a+b):
                max_dista=a
                max_distb=b
        return max_dista+max_distb
    def shorten_path(self, path, timeout=5.0):
        #print "path",path
        x= len(path)
        newpath =[]
        y=1
        count=1
        i=path[0]
        l= path [x-1]
        newpath.append(i)
        while y==1:
            if path[x-1] in newpath:
                print "newpath",newpath
                return newpath
            if self.connectV2(i,l):
                newpath.append(l)
                i=copy.copy(l)
                l= path[x-1]
                count=1
                continue
            if l==i:
                i= copy.copy(m)
                newpath.append[i]
                l= path[x-1]
                count=1
                continue
            #print "l",l
            #print"i",i
            count= count+1
            m=copy.copy(l)
            l= path[x-count]







    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')


        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        self.fig.canvas.draw()
if __name__ == '__main__':
    env = openravepy.Environment()
    robot = SimpleRobot(env)
    d = SimpleEnvironment(robot)
    x=[-4,0]
    y=[-3,0]
    print(d.connect(x,y))
