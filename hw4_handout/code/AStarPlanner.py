import time
import bisect
import numpy as np
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.open = Openlist(planning_env)
        self.close = Closelist(planning_env)


    def Plan(self, start_config, goal_config):
        start_time = time.time()

        plan = []
        
        start_node = Node(0,None,0,self.planning_env.discrete_env.ConfigurationToNodeId(start_config), None)
        self.open.addNode(start_node)
        #if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        self.planning_env.InitializePlot(goal_config)
        
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        self.open.setGoal(goal_id)
        suc_node = start_node
        self.close.addNode(start_node.id)
        count = 0

        while (self.open.isEmpty() == False):
            #time.sleep(.5)
            import IPython
            #IPython.embed()
            curr = self.open.getlowest()
            if(curr.parent != None):
                #if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                currconf = np.array(self.planning_env.discrete_env.NodeIdToConfiguration(curr.id))
                parconf = np.array(self.planning_env.discrete_env.NodeIdToConfiguration(curr.parent.id))
                self.planning_env.PlotEdge(currconf[:2], parconf[:2])
            count = count + 1
            #curr = self.open.getlowest()
            if curr.id == goal_id:
                suc_node = curr
                break
            successors = self.planning_env.GetSuccessors(curr.id)
            #import IPython
            #IPython.embed()
            for i in range(0, len(successors)):
                if (self.close.isDuplicate(successors[i][0]) == False):
                    newnode = Node(curr.cost+successors[i][1].control.dt,curr,curr.depth+1, successors[i][0], successors[i][1])
                    self.open.addNode(newnode)
                    self.close.addNode(successors[i][0])



        while (suc_node.id != start_node.id):
            plan.insert(0,suc_node.action)
            suc_node = suc_node.parent
        #plan.insert(0,start_config)
        print("--- %s seconds ---" % (time.time() - start_time))
        print("--- %d expansions ---" % count)
        #plan.append(goal_config)
        transform = self.planning_env.robot.GetTransform() #hacky
        transform[0, 3] = start_config[0]
        transform[1, 3] = start_config[1]
        self.planning_env.robot.SetTransform(transform);

        import IPython
        IPython.embed()
        return plan


class Node:
    def __init__(self, cost, parent, depth, id, action):
        self.cost = cost
        self.parent = parent
        self.depth = depth
        self.id = id
        self.action = action

    def __str__(self):
        #Spits out a full list the node's state/operator and states/operators leading to it
        result = "Cost: " +  str(self.cost)
        result += " Depth: " + str(self.depth)
        result += " ID: " + str(self.id)
        if self.parent != None:
            result += " Parent: " + str(self.parent.id)
        return result


class Closelist:

    def __init__(self, env):
        #import IPython
        #IPython.embed()
        self.close = []

    def addNode(self, id):
        bisect.insort(self.close,id)

    def isDuplicate(self, id):
        #import IPython
        #IPython.embed()
        if(bisect.bisect_left(self.close,id) == len(self.close)):
            return False
        if (self.close[bisect.bisect_left(self.close,id)] == id):
            return True
        return False


class Openlist:
    def __init__(self, env):
        self.env = env
        self.open = []
        self.goal_id = None
    def __str__(self):
        result = "Open List contains " + str(len(self.open)) + " items\n"
        for item in self.open:
            result += str(item) + "\n"
        return result
    def setGoal(self, goal_id):
        self.goal_id = goal_id

    def addNode(self, node):
        lowest = 0
        greatest = len(self.open)
        oldmid = -1
        midpoint = 0
        weight = 1
        while(lowest < greatest):
            midpoint = (lowest + greatest)/2
            if(self.open[midpoint].cost+weight*self.env.ComputeHeuristicCost(self.open[midpoint].id, self.goal_id) == node.cost+weight*self.env.ComputeHeuristicCost(node.id, self.goal_id) or oldmid == midpoint):
                lowest = midpoint
                break;
            else:
                if (node.cost+weight*self.env.ComputeHeuristicCost(node.id, self.goal_id) < self.open[midpoint].cost+weight*self.env.ComputeHeuristicCost(self.open[midpoint].id, self.goal_id)):
                    greatest = midpoint
                else:
                    lowest = midpoint+1
            oldmid = midpoint
        self.open.insert(lowest, node)

    def getlowest(self):
        if not self.isEmpty():
            return self.open.pop(0)
        else:
            raise RunTimeError

    def isEmpty(self):
        return len(self.open) == 0