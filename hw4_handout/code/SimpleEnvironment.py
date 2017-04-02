import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        left_wheel = 1
        right_wheel = 1
        duration = 1.57 #I think this makes 90 degree turns

        forward = Control(left_wheel, right_wheel, duration)
        turnright = Control(-left_wheel, right_wheel, duration)
        turnleft = Control(left_wheel, -right_wheel, duration)
        # Iterate through each possible starting orientation
        #I don't know why we iterate, but okay


        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            forward_print = self.GenerateFootprintFromControl(start_config, forward, 0.01)
            turnright_print = self.GenerateFootprintFromControl(start_config, turnright, 0.01)
            turnleft_print = self.GenerateFootprintFromControl(start_config, turnleft, 0.01)
         
            forward_action = Action(forward, numpy.array(forward_print))
            turnright_action = Action(turnright, numpy.array(turnright_print))
            turnleft_action = Action(turnleft, numpy.array(turnleft_print))

            self.actions[idx] = [Ac_forward,Ac_backward,Ac_rightturn,Ac_leftturn]

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        
        start_config = self.discrete_env.NodeIdToConfiguration(node_id)
        coord = numpy.array(self.discrete_env.NodeIdToGridCoord(node_id))
        myactions = self.actions[coord[2]] #because someone chose to iterate through angles for no reason

        for i in range(len(actions)):
            fp = myactions[i].footprint
            suc_config = numpy.array(start_config).copy()
            suc_config[2] = 0 #more bullshit due to iterating through angles
            suc_config = suc_config + foot_print[-1] #add the movement from the action

            suc_id = self.discrete_env.ConfigurationToNodeId(suc_config)

            if (self.robot.GetEnv().CheckCollision(self.robot) == False):
                successors.append([suc_id, myactions[i]])

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        

        dist = dist = numpy.linalg.norm(goal_config - start_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        

        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))

        
        #we ignore angle here on purpose
        #Since we non-holonomic angle will tend to mess us up more
        #Or maybe not, but I'm going with my gut here        
        cost = 10*abs(start_config[0] - goal_config[0]) + abs(start_config[1] - goal_config[1])
        #Also weighting this bad boy
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return cost

