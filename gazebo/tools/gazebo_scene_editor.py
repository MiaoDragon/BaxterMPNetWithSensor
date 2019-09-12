import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
from std_msgs.msg import Header
import random
import time
# import matplotlib.pyplot as plt
import sys
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates
import tf
import os
import pickle

_colors = {}  # fix this later


class GazeboSceneModifier():
    def spawn_object(self, name, file, pose, orientation=[0,0,0], z_offset=0., ref_frame='world'):
        """
        Spawn object into Gazebo scene described by URDF/SDF file.
        argument:
        --name: object name in Gazebo
        --file: SDF file path
        --pose: pose of the object
        --orientation: euler orientation of the object
        --z_offset: offset on the z axis, to make sure objects are on table, for instance
        --ref_frame: reference frame for the pose and orientation
        """
        # obtain the extension (SDF/URDF) from filename
        extension = file.split('/')[-1].split('.')[-1]
        orientation = list(tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2]))
        orient_fixed = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        init_pose = Pose(Point(
        x=(pose[0]),
        y=pose[1],
        z=pose[2] + z_offset), orient_fixed) #hacky fix for x value to be correct... TODO

        f = open(file)
        sdf_f = f.read()

        rospy.wait_for_service('/gazebo/spawn_%s_model' % (extension))
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_%s_model' % (extension), SpawnModel)
        try:
          resp1 = spawn_model(name, sdf_f, "", init_pose, ref_frame) #expects model name, model file, robot namespace, initial pose, and reference frame
        except rospy.ServiceException as exc:
          print("Service did not process request (Failed to spawn model): " + str(exc))

    def delete_object(self, name):
        """
        Delete objects in the Gazebo scene by name.
        --name: object name in Gazebo
        """
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        try:
          resp1 = delete_model(name)
        except rospy.ServiceException as exc:
          print("Service did not process request (Failed to delete model): " + str(exc))
