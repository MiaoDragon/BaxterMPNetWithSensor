#!/usr/bin/env python
#from motion_planning_dataset import *
import pickle
import argparse
import sys
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

import baxter_interface
from moveit_msgs.msg import RobotState, DisplayRobotState, PlanningScene, RobotTrajectory, ObjectColor
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander, MoveItCommanderException, roscpp_initialize
import sys, os
import rospkg
rospack = rospkg.RosPack()
top_path = rospack.get_path('baxter_mpnet_with_sensor')
sys.path.insert(0, top_path)
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from tools.planning_scene_editor import *
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
def ik_test(pose, printing=False):
    """
    From baxter example code to return a joint_angle dictionary from an input
    workspace pose
    """
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return (1, limb.joint_angles()) #returning current joint angles if not valid

    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if printing:
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            print limb_joints
    else:
        if printing:
            print("INVALID POSE - No Valid Joint Solution Found.")

    return (0, limb_joints)

def xy_check(position, bounds_dict):
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

def sample_from_boundary(bounds_dict):
    """
    Sample a workspace position from within a boundary, described by bounds_dict
    """
    area = 0 if np.random.random() < 0.5 else 1 #area 0 is half the center table, area 1 is the whole right table

    x = np.random.random() * bounds_dict['x_r'][area] + min(bounds_dict['x'][area])
    y = np.random.random() * bounds_dict['y_r'][area] + min(bounds_dict['y'][area])

    z = np.random.random() * 0.15 + 0.24
    return([x, y, z])



def main():
    rospy.init_node(sys.argv[1])
    total_envs = 10
    total_targets = int(sys.argv[2]) #total number of collision free targets to save

    limb = baxter_interface.Limb('right')
    def_angles = limb.joint_angles()
    limb.set_joint_position_speed(0.65)
    #robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    #scene = PlanningSceneInterface()
    #scene._scene_pub = rospy.Publisher('planning_scene',
    #                                          PlanningScene,
    #                                          queue_size=0)
    robot = RobotCommander()
    group = MoveGroupCommander("right_arm")
    rs_man = RobotState()

    sv = StateValidity()
    #set_environment(robot, scene)

    masterModifier = ShelfSceneModifier()
    #sceneModifier = PlanningSceneModifier(masterModifier.obstacles)
    #sceneModifier.setup_scene(scene, robot, group)

    robot_state = robot.get_current_state()
    print(robot_state)
    rs_man = RobotState()  # constructed manually for comparison
    rs_man.joint_state.name = robot_state.joint_state.name
    filler_robot_state = list(robot_state.joint_state.position)

    done = False

    masterEnvDict = {}
    envFileName = "testEnvironments_test1.pkl"

    x_bounds = [[0.89, 1.13], [-0.2, 1.13]] #0 is the right half of the right table, 1 is the right table
    y_bounds = [[-0.66, 0], [-0.9, -0.66]]

    x_ranges = [max(x_bounds[0]) - min(x_bounds[0]), max(x_bounds[1]) - min(x_bounds[1])]
    y_ranges = [max(y_bounds[0]) - min(y_bounds[0]), max(y_bounds[1]) - min(y_bounds[1])]

    xy_bounds_dict = {'x': x_bounds, 'y': y_bounds, 'x_r': x_ranges, 'y_r': y_ranges}


    iter_num = 0
    #sceneModifier.delete_obstacles()
    while (not rospy.is_shutdown() and not done):
        for i_env in range(total_envs):
            new_pose = masterModifier.permute_obstacles()
            key_name = 'TrainEnv_' + str(i_env)
            #sceneModifier.permute_obstacles(new_pose)

            masterEnvDict[key_name] = {}
            masterEnvDict[key_name]['ObsPoses'] = {}
            masterEnvDict[key_name]['Targets'] = {}

            masterEnvDict[key_name]['Targets']['Pose'] = []
            masterEnvDict[key_name]['Targets']['Joints'] = []

            for i_target in range(total_targets):
                pose = sample_from_boundary(xy_bounds_dict)
                check_pose = group.get_random_pose()
                check_pose.pose.position.x = pose[0]
                check_pose.pose.position.y = pose[1]
                check_pose.pose.position.z = pose[2]

                joint_angles = ik_test(check_pose)[1]
                filler_robot_state[10:17] = moveit_scrambler(joint_angles.values())
                rs_man.joint_state.position = tuple(filler_robot_state)

                while (ik_test(check_pose)[0] or not sv.getStateValidity(rs_man, group_name = "right_arm")):
                    pose = sample_from_boundary(xy_bounds_dict)

                    check_pose = group.get_random_pose()
                    check_pose.pose.position.x = pose[0]
                    check_pose.pose.position.y = pose[1]
                    check_pose.pose.position.z = pose[2]

                    if (not ik_test(check_pose)[0]):
                        joint_angles = ik_test(check_pose)[1]

                        filler_robot_state[10:17] = moveit_scrambler(joint_angles.values())
                        rs_man.joint_state.position = tuple(filler_robot_state)

                joint_angles = ik_test(check_pose)[1]
                masterEnvDict[key_name]['Targets']['Pose'].append(check_pose)
                masterEnvDict[key_name]['Targets']['Joints'].append(joint_angles)

            #sceneModifier.delete_obstacles()
            masterEnvDict[key_name]['ObsPoses'] = new_pose
            iter_num += 1

        with open(envFileName, "wb") as env_f:
            pickle.dump(masterEnvDict, env_f)

        print("Done saving... exiting loop \n\n\n")
        done = True

if __name__ == "__main__":
    main()
