#!/usr/bin/env python
# coding: utf-8

# In[ ]:


"""
This jupyter notebook loads the point cloud from pcd file saved locally. Then it generates a plan for the loaded
scene
Usage:
    1. launch gazebo_model.launch
    2. specify the environment file, and run the code
"""


# In[1]:


import numpy as np
from numpy import matlib
import rospy
import baxter_interface
from moveit_msgs.msg import RobotState, DisplayRobotState, PlanningScene, RobotTrajectory, ObjectColor
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander, MoveItCommanderException, roscpp_initialize
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
from std_msgs.msg import Header
import random
import sys
import tf
import os

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates

import pickle
import subprocess
import time

import numpy as np
import pickle
import rospy
import argparse
import baxter_interface
import os
import sys, os
import rospkg
import fnmatch
rospack = rospkg.RosPack()
top_path = rospack.get_path('baxter_mpnet_with_sensor')
sys.path.insert(0, top_path)
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from tools.planning_scene_editor import *
import pyquaternion
# In[2]:
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


class GoalSampler():
    def __init__(self, group, limb, robot_state, sv):
        self.x_bounds = [[0.89, 1.13], [-0.2, 1.13]] #0 is the right half of the right table, 1 is the right table
        self.y_bounds = [[-0.66, 0], [-0.9, -0.66]]

        self.x_ranges = [max(self.x_bounds[0]) - min(self.x_bounds[0]), max(self.x_bounds[1]) - min(self.x_bounds[1])]
        self.y_ranges = [max(self.y_bounds[0]) - min(self.y_bounds[0]), max(self.y_bounds[1]) - min(self.y_bounds[1])]

        self.xy_bounds_dict = {'x': self.x_bounds, 'y': self.y_bounds, 'x_r': self.x_ranges, 'y_r': self.y_ranges}
        self.group = group
        self.rs_man = RobotState()  # constructed manually for comparison
        self.rs_man.joint_state.name = robot_state.joint_state.name
        self.filler_robot_state = list(robot_state.joint_state.position)
        self.sv = sv  # collision checking service
        self.limb = limb

    def ik_test(self, pose, printing=False):
        """
        From baxter example code to return a joint_angle dictionary from an input
        workspace pose
        """
        #print('ik test...')
        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose.header = hdr
        ikreq.pose_stamp.append(pose)


        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
        limb_joints = None
        if (resp.isValid[0]):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if printing:
                print("SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                print limb_joints
        else:
            if printing:
                print("INVALID POSE - No Valid Joint Solution Found.")
            return (1, limb_joints)
        return (0, limb_joints)

    def xy_check(self, position, bounds_dict):
        valid0 = False
        valid1 = False
        validx0 = (position.x < max(xy_bounds_dict['x'][0]) and position.x > min(xy_bounds_dict['x'][0]))
        validy0 = (position.y < max(xy_bounds_dict['y'][0]) and position.y > min(xy_bounds_dict['y'][0]))

        valid0 = validx0 and validy0

        validx1 = (position.x < max(xy_bounds_dict['x'][1]) and position.x > min(xy_bounds_dict['x'][1]))
        validy1 = (position.y < max(xy_bounds_dict['y'][1]) and position.y > min(xy_bounds_dict['y'][1]))

        valid1 = validx1 and validy1

        valid = valid0 or valid1
        print("valid0: ", valid0)
        print("valid1: ", valid1)
        print("valid: ", valid)
        return valid

    def sample_from_boundary(self, bounds_dict):
        """
        Sample a workspace position from within a boundary, described by bounds_dict
        """
        area = 0 if np.random.random() < 0.5 else 1 #area 0 is half the center table, area 1 is the whole right table

        x = np.random.random() * bounds_dict['x_r'][area] + min(bounds_dict['x'][area])
        y = np.random.random() * bounds_dict['y_r'][area] + min(bounds_dict['y'][area])

        z = np.random.random() * 0.15 + 0.24
        return([x, y, z])

    def sample(self):
        # sample one collision free goal location
        itr = 0
        while True:
            #print('trail: %d' % (itr))
            itr += 1
            pose = self.sample_from_boundary(self.xy_bounds_dict)
            #print('getting random pose...')
            check_pose = PoseStamped()
            #check_pose = self.group.get_random_pose()
            #print('got random pose.')
            check_pose.pose.position.x = pose[0]
            check_pose.pose.position.y = pose[1]
            check_pose.pose.position.z = pose[2]
            q = pyquaternion.Quaternion.random()
            check_pose.pose.orientation.x = q[0]
            check_pose.pose.orientation.y = q[1]
            check_pose.pose.orientation.z = q[2]
            check_pose.pose.orientation.w = q[3]

            res = self.ik_test(check_pose)
            if res[0]:
                continue

            joint_angles = res[1]
            self.filler_robot_state[10:17] = moveit_scrambler(joint_angles.values())
            self.rs_man.joint_state.position = tuple(self.filler_robot_state)
            if self.sv.getStateValidity(self.rs_man, group_name = 'right_arm'):
                break
        return check_pose, joint_angles





def create_call(ex, root, name):
    #input = '/voxel_grid/output'
    frame_id = 'map'
    call = [str(ex), root+name, '1', '_frame_id:=%s' % (frame_id)]
    return call


def compute_cost(path):
    state_dists = []
    for i in range(len(path) - 1):
        dist = 0
        for j in range(7):  # baxter DOF
            diff = path[i][j] - path[i+1][j]
            dist += diff*diff

        state_dists.append(np.sqrt(dist))
    total_cost = sum(state_dists)
    return total_cost


def main(args):
    rospy.init_node('path_data_gen')

    # sometimes takes a few tries to connect to robot arm
    limb_init = False
    while not limb_init:
        try:
            limb = baxter_interface.Limb('right')
            limb_init = True
        except OSError:
            limb_init = False

    neutral_start = limb.joint_angles()
    min_goal_cost_threshold = 2.0
    # set speed to make sure execution does not violate the speed constraint
    limb.set_joint_position_speed(0.65)
    # Set up planning scene and Move Group objects
    robot = RobotCommander()
    group = MoveGroupCommander("right_arm")
    sv = StateValidity()

    # Setup tables (geometry and location defined in moveit_functions.py, set_environment function)
    # set_environment(robot, scene)

    # Additional Move group setup

    # group.set_goal_joint_tolerance(0.001)
    # group.set_max_velocity_scaling_factor(0.7)
    # group.set_max_acceleration_scaling_factor(0.1)
    max_time = args.max_time
    group.set_planning_time(max_time)

    # Dictionary to save path data, and filename to save the data to in the end
    pathsDict = {}

    if not os.path.exists(args.path_data_path):
        os.makedirs(args.path_data_path)

    pathsFile = args.path_data_path+args.path_data_file

    # load data from environment files for obstacle locations and collision free goal poses
    # with open("env/environment_data/trainEnvironments_GazeboPatch.pkl", "rb") as env_f:
    with open(args.env_data_path+args.env_data_file, "rb") as env_f:
        envDict = pickle.load(env_f)

    # with open("env/environment_data/trainEnvironments_testGoals.pkl", "rb") as goal_f:
    #with open(args.targets_data_path+args.targets_data_file, "rb") as goal_f:
    #    goalDict = pickle.load(goal_f)

    # Obstacle data stored in environment dictionary, loaded into scene modifier to apply obstacle scenes to MoveIt
    robot_state = robot.get_current_state()
    rs_man = RobotState()

    rs_man.joint_state.name = robot_state.joint_state.name
    filler_robot_state = list(robot_state.joint_state.position)
    goal_sampler = GoalSampler(group, limb, robot_state, sv)
    # Here we go
    # slice this if you want only some environments
    test_envs = envDict['poses'].keys()
    done = False
    iter_num = 0
    print("Testing envs: ")
    print(test_envs)
    joint_state_topic = ['joint_states:=/robot/joint_states']
    roscpp_initialize(joint_state_topic)
    rospy.init_node("path_data_gen")
    #roscpp_initialize(sys.argv)

    # parameters for loading point cloud
    executable = './pcd_to_pointcloud'
    rootPath = '../data/pcd/'

    rospy.wait_for_service('clear_octomap')
    clear_octomap = rospy.ServiceProxy('clear_octomap', Empty)
    respond = clear_octomap()
    print(group.get_end_effector_link())
    # try moving the robot to current joints
    #group.go(joints=group.get_current_joint_values())

    while(not rospy.is_shutdown() and not done):
        for i_env, env_name in enumerate(test_envs):
            print("env iteration number: " + str(i_env))
            print("env name: " + str(env_name))

            # load point cloud saved
            name = ''
            for file in sorted(os.listdir(args.pcd_data_path), key=lambda x: int(x.split('Env_')[1].split('_')[1][:-4])):
                if (fnmatch.fnmatch(file, env_name+"*")):
                    # if found the first one that matches the name env+something, then record it
                    name = file
                    break
            call = create_call(executable, rootPath, name)
            ### Printing the executable call and allowing user to manually cycle through environments for demonstration
            print(call)
            ### Uncomment below to call pointcloud_to_pcd executable which takes snapshot of the streaming pointcloud data
            ### and saves it to a .pcd file in a desired file location (as specified by prefix in the command call)

            print("Calling executable... \n\n\n")
            t = time.time()
            #stderr_f = open('log.txt', 'w')
            # publishing topic of point cloud for planning
            # wait for some time to make sure the point cloud is loaded corretly
            p = subprocess.Popen(call)#, stderr=stderr_f)
            rospy.sleep(10)

            new_pose = envDict['poses'][env_name]

            pathsDict[env_name] = {}
            pathsDict[env_name]['paths'] = []
            pathsDict[env_name]['costs'] = []
            pathsDict[env_name]['times'] = []
            pathsDict[env_name]['total'] = 0
            pathsDict[env_name]['feasible'] = 0


            #collision_free_goals = goalDict[env_name]['Joints']

            total_paths = 0
            feasible_paths = 0
            i_path = 0
            #group.go(joints=group.get_current_joint_values())
            # run until either desired number of total or feasible paths has been found
            while (total_paths < args.paths_to_save):

                #do planning and save data

                # some invalid goal states found their way into the goal dataset, check to ensure goals are "reaching" poses above the table
                # by only taking goals which have a straight path cost above a threshold
                valid_goal = False
                print("FP: " + str(feasible_paths))
                print("TP: " + str(total_paths))
                total_paths += 1
                i_path += 1

                # Uncomment below if using a start state different than the robot current state

                # filler_robot_state = list(robot_state.joint_state.position) #not sure if I need this stuff
                # filler_robot_state[10:17] = moveit_scrambler(start.values())
                # rs_man.joint_state.position = tuple(filler_robot_state)
                # group.set_start_state(rs_man)   # set start
                pos = []
                while not valid_goal:
                    pose, joint = goal_sampler.sample()
                    goal = joint
                    optimal_path = [neutral_start.values(), goal.values()]
                    optimal_cost = compute_cost(optimal_path)

                    if optimal_cost > min_goal_cost_threshold:
                        valid_goal = True

                group.set_start_state_to_current_state()
                group.clear_pose_targets()
                try:
                    group.set_joint_value_target(moveit_scrambler(goal.values())) # set target
                except MoveItCommanderException as e:
                    print(e)
                    continue

                start_t = time.time()
                plan = group.plan()
                t = time.time() - start_t
                #group.execute(plan)
                pos = [plan.joint_trajectory.points[i].positions for i in range(len(plan.joint_trajectory.points))]
                if pos != []:
                    pos = np.asarray(pos)
                    cost = compute_cost(pos)
                    print("Time: " + str(t))
                    print("Cost: " + str(cost))
                    # Uncomment below if using max time as criteria for failure
                    if (t > (max_time*0.99)):
                        print("Reached max time...")
                        continue

                    feasible_paths += 1

                    pathsDict[env_name]['paths'].append(pos)
                    pathsDict[env_name]['costs'].append(cost)
                    pathsDict[env_name]['times'].append(t)
                    pathsDict[env_name]['feasible'] = feasible_paths
                    pathsDict[env_name]['total'] = total_paths

                    # Uncomment below if you want to overwrite data on each new feasible path
                    with open(pathsFile + "_" + env_name + ".pkl", "wb") as path_f:
                        pickle.dump(pathsDict[env_name], path_f)

                print("\n")

            p.terminate()
            p.wait()
            # rosservice call to clear octomap
            rospy.wait_for_service('clear_octomap')
            clear_octomap = rospy.ServiceProxy('clear_octomap', Empty)
            respond = clear_octomap()
            iter_num += 1

            print("Env: " + str(env_name))
            print("Feasible Paths: " + str(feasible_paths))
            print("Total Paths: " + str(total_paths))
            print("\n")

            pathsDict[env_name]['total'] = total_paths
            pathsDict[env_name]['feasible'] = feasible_paths

            with open(pathsFile + "_" + env_name + ".pkl", "wb") as path_f:
                pickle.dump(pathsDict[env_name], path_f)

        print("Done iterating, saving all data and exiting...\n\n\n")

        with open(pathsFile + ".pkl", "wb") as path_f:
            pickle.dump(pathsDict, path_f)

        done = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--env_data_path', type=str, default='../gazebo/env/environment_data/')
    parser.add_argument('--env_data_file', type=str, default='trainEnvironments.pkl')
    parser.add_argument('--targets_data_path', type=str, default='../data/train/targets/')
    parser.add_argument('--targets_data_file', type=str, default='trainTargets.pkl')
    parser.add_argument('--path_data_path', type=str, default='../data/train/paths/')
    parser.add_argument('--path_data_file', type=str, default='path_data_sample')
    parser.add_argument('--pcd_data_path', type=str, default='../data/train/pcd/')
    parser.add_argument('--paths_to_save', type=int, default=5)
    parser.add_argument('--max_time', type=int, default=5)

    args = parser.parse_args()
    main(args)
