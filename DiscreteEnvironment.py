import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits
	#print "self.lower_limits:"+str(self.lower_limits)
	#print "self.upper_limits:"+str(self.upper_limits)
        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
			self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)
	#print "number of cells:"+str(self.num_cells)
    def configuration_to_nodeID(self, config):
        # TODO: This function maps a node configuration in full configuration
        # space to a node in discrete space
        nodeID=0
        fact=1
        #print "config:",config.tolist()
        #print "config:",config
        #print "self.resolution",self.resolution
        coordinate = [numpy.floor(round((x/self.resolution),2)) for x in config]
        #print "coordinate:",coordinate
        point=[]
        #print "coordinate:",coordinate
        for id1 in range(self.dimension):
            Glower_limit=  self.configuration_to_gridCoord(self.lower_limits)
            #rint self.lower_limits[id1]
            #point[id1]=self.lower_limits[id1]-coordinate[id1]
            nodeID = nodeID + fact*abs(Glower_limit[id1]-coordinate[id1])
            #print "fact",fact
            fact = fact*(self.num_cells[id1])

        return int(nodeID)

    def nodeID_to_configuration(self, nid):
        # TODO: This function maps a node in discrete space to a configuraiton
        # in the full configuration space
	#print "nid:",nid
        coord = [0] * self.dimension
    	#print "nid:",node_id
        Glower_limit=  self.configuration_to_gridCoord(self.lower_limits)
        #print "Glower_limit:",Glower_limit
        nodetemp=int(nid)

        for id1 in range(self.dimension-1):
            divisor= self.getdivisor(id1)
            #print "divisor",divisor
            coord[self.dimension-1-id1]= nodetemp//divisor+ Glower_limit[self.dimension-1-id1]
            #print "coord[self.dimension-1-id1]:",coord[self.dimension-1-id1]
            nodetemp=nodetemp % divisor
            #print "nodetemp:",nodetemp

    	coord[0]=nodetemp+ Glower_limit[0]
        #coords= [i+Glower_limit[self.dimension-1-id1] for i in coord ]
        ret_coord= [int(i) for i in coord]
        config= self.gridCoord_to_configuration(numpy.asarray(ret_coord))
        a=[round(n,3) for n in config]
        return a

    def getdivisor(self,index):
	#present divisor calculations
	divisor=1
	for m in range(self.dimension-index-1):
		divisor= divisor*self.num_cells[m]
	return int(divisor)

    def configuration_to_gridCoord(self, config):

        # TODO: This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [numpy.floor(round((x/self.resolution),2)) for x in config]
        return coord

    def gridCoord_to_configuration(self, coord):

        # TODO: This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config =[float(x*self.resolution) for x in coord]
        a=[round(n,3) for n in config]
        return a

    def gridCoord_to_nodeID(self,coord):

        # TODO: This function maps a grid coordinate to the associated
        # node id
        node_id = 0
        fact=1
        Glower_limits=  self.configuration_to_gridCoord(self.lower_limits)
        for id1 in range(self.dimension):
            node_id = node_id + fact*abs(Glower_limits[id1]-numpy.floor(coord[id1]))
            fact = fact*self.num_cells[id1]
        return int(node_id)

    def nodeID_to_gridCoord(self, node_id):
        # TODO: This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
    	#print "nid:",node_id
        Glower_limit=  self.configuration_to_gridCoord(self.lower_limits)
        #print "Glower_limit:",Glower_limit
        nodetemp=int(node_id)

        for id1 in range(self.dimension-1):
            divisor= self.getdivisor(id1)
            #print "divisor",divisor
            coord[self.dimension-1-id1]= nodetemp//divisor+ Glower_limit[self.dimension-1-id1]
            #print "coord[self.dimension-1-id1]:",coord[self.dimension-1-id1]
            nodetemp=nodetemp % divisor
            #print "nodetemp:",nodetemp

    	coord[0]=nodetemp+ Glower_limit[0]
        #coords= [i+Glower_limit[self.dimension-1-id1] for i in coord ]
        ret_coord= [int(i) for i in coord]
        return ret_coord

if __name__=='__main__':
    r=0.1#input('Enter your Resolution')
    l=[-5,-5,-5,-5,-5,-5,-5]#input('Enter your Lower Limits')
    u=[5,5,5,5,5,5,5]#input('Enter your upper limits')
    d= DiscreteEnvironment(r, l, u)
    x=numpy.array([4.8, 3.4, 1.8, 2.6, -3.1, -4.0, -5.0])
    print(d.nodeID_to_configuration(101976688498))
    print(d.configuration_to_nodeID(x))
