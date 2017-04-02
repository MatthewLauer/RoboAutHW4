import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        import IPython
        #IPython.embed()
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])
        
    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #

        coord = [0] * self.dimension
        for i in range(self.dimension):
            coord[i] = (config[i]-self.lower_limits[i])//self.resolution[i]

        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for i in range(self.dimension):
            config[i] = (coord[i]+.5)*self.resolution[i]+self.lower_limits[i]
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0

        for i in range(self.dimension-1, -1, -1):
            spacesize = 1;
            for j in range(0, i):
                spacesize = spacesize*self.num_cells[j]
            node_id = node_id + spacesize*coord[i]

        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension

        for i in range(self.dimension-1, -1, -1):
            spacesize = 1;
            for j in range(0, i):
                spacesize = spacesize*self.num_cells[j]
            coord[i] = node_id//spacesize
            node_id = node_id - coord[i]*spacesize
        return coord

