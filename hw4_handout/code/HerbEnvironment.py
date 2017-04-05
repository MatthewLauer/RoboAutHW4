import numpy 
import time
class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.herb = herb

        # add a table and move the robot into place
        #table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        #self.robot.GetEnv().Add(table)

        #table_pose = numpy.array([[ 0, 0, -1, 0.6], 
        #                          [-1, 0,  0, 0], 
        #                          [ 0, 1,  0, 0], 
        #                          [ 0, 0,  0, 1]])
        #table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        num_dof = len(self.robot.GetActiveDOFIndices())
        config = [0] * num_dof

        # TODO: Generate and return a random configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        for dim in range (num_dof):
            config[dim] = numpy.random.uniform(low=lower_limits[dim], high=upper_limits[dim]);

        return numpy.array(config)


    
    def ComputeDistance(self, start_config, end_config):
        
        # TODO: Implement a function which computes the distance between two configurations

        return numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config));


    def Extend(self, start_config, end_config):
        
        # TODO: Implement a function which attempts to extend from 
        # a start configuration to a goal configuration
        num_dof = len(self.robot.GetActiveDOFIndices())
        steps = 25

        if(start_config == None):
            return None
        # Generate interpolations (joint by joint?)
        JointSteps = numpy.transpose(numpy.array([start_config] * steps));
        #import IPython
        #IPython.embed()
        for dim in range(num_dof):
            JointSteps[dim] = numpy.linspace(start_config[dim], end_config[dim], steps);

        for i in range(steps):
            self.robot.SetActiveDOFValues(JointSteps[:,i]);
            
            # Check collision
            for body in self.robot.GetEnv().GetBodies():
                if ((body.GetName() != self.robot.GetName() and
                    self.robot.GetEnv().CheckCollision(self.robot, body)) or
                    self.robot.CheckSelfCollision()):
                    # Check first step
                    if (i == 0): 
                        return None
                    else:
                        #JointSteps[dim] = [JointSteps[dim,i-1]] * steps
                        return None#end_config[:] = JointSteps[:,i-1]
        
        # No collision detected 
        return numpy.array(end_config)
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        initTime = time.time()
        while(time.time()-initTime<timeout):
	    ind = 1
	    while(ind < len(path)-1):
                start_config = path[ind-1]
                final_config = path[ind+1]
                config = self.Extend(start_config,final_config)
                
                
		if config != None:
		    same_bool = 1
                    for i in range(len(config)):
                        if config[i]!=final_config[i]:
                            same_bool = 0
                    if same_bool ==1:
                        del path[ind]
	        ind = ind + 1
                
            
        return path
