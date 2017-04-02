
import logging, openravepy
import os
import copy
import time
import math
import numpy
numpy.random.seed(0)
import scipy
from numpy import linalg
import time
from DiscreteEnvironment import DiscreteEnvironment
import IPython
import random

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

    def eval_grasp(self, grasp):
        with self.robot:
            # contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp, translate=True,
                                                                               forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3, 3]
                # for each contact
                G = numpy.array([[], [], [], [], [], []])  # the wrench matrix
                for c in contacts:
                    pos = c[0:3] - obj_position
                    dir = -c[3:]  # this is already a unit vector

                    w = numpy.append(pos, numpy.cross(pos, dir));
                    wt = numpy.reshape(w, (6, 1));

                    G = numpy.concatenate((G, wt), axis=1);

                if not G.size:
                    return 0.00
                U, s, V = numpy.linalg.svd(G, full_matrices=True);
                s = s[::-1];
                print("s==", s);
                sigma_min = s[0];
                # print(sigma_min);

                # TODO use G to compute scrores as discussed in class
                # return len(contacts)
                return sigma_min * len(contacts) * len(contacts)  # change this

            except openravepy.planning_error, e:
        # you get here if there is a failure in planning
        # example: if the hand is already intersecting the object at the initial position/orientation

                return 0.00  # TODO you may want to change this

    def showPossibleBasePoses(self, Tgrasp, N=1,Timeout = 5):
        """visualizes possible base poses for a grasp specified by Tgrasp and gripper_angle

        :param Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame. equals manip.GetTransform() in the goal state
        :param gripper_angle: float, the gripper angle
        :param N: int, the number of sample poses we want to get 
        """

        # find the robot base distribution for the grasp specified by Tgrasp
        # Input for computeBaseDistribution():
        #      Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame
        #              equals manip.GetTransform() in the goal state
        # Output for computeBaseDistribution():
        #      densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
        #      samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
        #      bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]

        densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp, logllthresh=1.8)
        if densityfn == None:
            print 'the specified grasp is not reachable!'
            return

        # Code fragmenmanip = self.robot.SetActiveManipulator('left_wam')t from `examples.mobilemanipulation`
        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = Timeout
        self.manip = self.robot.SetActiveManipulator('left_wam')
        # self.manip = self.robot.GetActiveManipulator()
        self.env = self.robot.GetEnv()
        with self.robot:
            while len(goals) < N:
                if time.time() - starttime > timeout:
                    break
                poses, jointstate = samplerfn(N - len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp, filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            # values = self.robot.GetDOFValues()
                            # values[self.manip.GetArmIndices()] = q
                            pose = self.robot.GetTransform()
                            xy_pose = [pose[0][3], pose[1][3]]
                            goals.append((Tgrasp, xy_pose,q, pose))
                        elif self.manip.FindIKSolution(Tgrasp, 0) is None:
                            numfailures += 1
                            print "base pose is in collision"
        return goals

    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################
        # get possible grasps
        grasps = gmodel.grasps
        # order grasps
        grasps_ordered = grasps.copy()
        for grasp in grasps_ordered:
            grasp[gmodel.graspindices.get('performance')] = self.eval_grasp(grasp)

            # sort!
        order = numpy.argsort(gmodel.grasps_ordered[:, gmodel.graspindices.get('performance')[0]])
        # maximum of sigma_min
        order = order[::-1]

        grasps_ordered = grasps_ordered[order]
        # get best grasps
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot)
        print 'loading irmodel'
        # make sure the robot and manipulator match the database
        assert self.irmodel.robot == self.robot and self.irmodel.manip == self.robot.GetActiveManipulator()

        goals = []
        for validgrasp in grasps_ordered:
            Tgrasp = gmodel.getGlobalGraspTransform(validgrasp, collisionfree=True)

            # print "Compute base distribution for grasp..."
            # densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)

            print "Finding a base pose..."
            manip = self.robot.SetActiveManipulator('left_wam')
            goals = self.showPossibleBasePoses(Tgrasp, 1, 5)  # timeout in seconds

            if len(goals) > 0:
                print "Found grasp"
                break

            self.Tgrasp = Tgrasp
        # goals is a list of : (Tgrasp,pose,values)
        goal_idx = 0
        base_pose = goals[goal_idx][1]  # Don't care which, just need one that works
        grasp_config = goals[goal_idx][2]
        T_pose = goals[goal_idx][3]

        print "Base_pose: %r\n Grasp_config: %r" % (base_pose, grasp_config)

        start_pose = self.robot.GetTransform()
        start_config = self.robot.GetActiveDOFValues()
        self.robot.SetTransform(T_pose)
        self.robot.SetActiveDOFValues(grasp_config)

        raw_input("Target pose found.  Press enter to continue")

        self.robot.SetTransform(start_pose)

        self.robot.SetActiveDOFValues(start_config)


        return base_pose, grasp_config



    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
    
