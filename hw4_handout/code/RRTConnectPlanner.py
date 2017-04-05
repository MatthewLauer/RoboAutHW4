import numpy, operator
import random
import time
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        start_time = time.time()
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        rplan = []
        fplan = []
        expansions = 0;

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space


        #print "startConfig = [%.2f, %.2f]" %(start_config[0], start_config[1])
        #print "goalConfig = [%.2f, %.2f]" %(goal_config[0], goal_config[1])
        disc = True;
        while (disc):
            if(random.random() < 0.95):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = goal_config;
            expansions+=1
            [nearID, nearConfig] = ftree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDf = ftree.AddVertex(extension);
                ftree.AddEdge(nearID, extIDf);
                #self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, goal_config)):
            #    disc = False

            if(random.random() < 0.95):
                desConfig = self.planning_env.GenerateRandomConfiguration();
            else:
                desConfig = start_config;
            expansions+=1
            [nearID, nearConfig] = rtree.GetNearestVertex(desConfig);            
            extension = self.planning_env.Extend(nearConfig, desConfig)
            if (extension != None):
                extIDr = rtree.AddVertex(extension);
                rtree.AddEdge(nearID, extIDr);
                #self.planning_env.PlotEdge(nearConfig, extension);
            
            #if(numpy.array_equal(extension, start_config)):
            #    disc = False

            for i in range(len(ftree.vertices)):
                if (numpy.linalg.norm(numpy.array(desConfig) - numpy.array(ftree.vertices[i])) < 5):
                    lastlink = self.planning_env.Extend(extension, ftree.vertices[i])
                    if (numpy.array_equal(lastlink,ftree.vertices[i])):
                        lastIDr = rtree.AddVertex(lastlink);
                        rtree.AddEdge(extIDr, lastIDr);
                        #self.planning_env.PlotEdge(extension, lastlink);
                        disc = False
                        extIDf = i
                        expansions+=1
                        break
        lastlink_cp = lastlink
        rplan.insert(0,goal_config)
        while 1:
            if (extIDr == rtree.GetRootId()):
                break;
            else:
                rplan.insert(1, extension);
            extIDr = rtree.edges[extIDr];
            extension = rtree.vertices[extIDr];

 
        plan = list(reversed(rplan))

        plan.insert(0,start_config)
        while 1:
            if (extIDf == ftree.GetRootId()):
                break;
            else:
                plan.insert(1, lastlink);
            extIDf = ftree.edges[extIDf];
            lastlink = ftree.vertices[extIDf];


        for config in plan:
            print "fconfig = [%.2f, %.2f]" %(config[0], config[1])
        print("--- %s seconds ---" % (time.time() - start_time))
        print "Total expansions = %d" %(expansions)
        print("--- %s seconds ---" % (time.time() - start_time))
        print("--- %s path length ---" % len(plan))
        print("--- %s vertices ---" % (len(ftree.vertices) + len(rtree.vertices) - 1))

        return plan