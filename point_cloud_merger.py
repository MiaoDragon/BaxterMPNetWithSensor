#!/usr/bin/env python
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2
import tf2_ros

import numpy as np


class PointCloudMerger():
    '''
    On receiving point cloud messages, merge them together
    Using message_filter for time synchronization
    '''
    def __init__(self, camera_num):
        self.camera_num = camera_num
        self.merge_pub = rospy.Publisher("/camera/depth/points_merged", PointCloud2, queue_size=10)
        camera_subs = []
        for i in range(self.camera_num):
            camera_subs.append(message_filters.Subscriber('/camera/depth/points%d' % (i+1), PointCloud2))
        self.ts = message_filters.ApproximateTimeSynchronizer(camera_subs, 10, 1)
        #self.ts = message_filters.TimeSynchronizer(camera_subs, 10)
        self.ts.registerCallback(self.merge)
        # for transforming the point cloud to the proper cooridnate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    def merge(self, *args):
        """
            merge received point clouds
            also preprocess: delete points that are outside of the boundary
        """
        # take any number of input arguments
        merged_points = []
        for points in args:
            # obtain the tf transform for this link
            trans = self.tf_buffer.lookup_transform('map', points.header.frame_id, points.header.stamp, rospy.Duration(1))
            points = do_transform_cloud(points, trans)
            points_list = []
            for p in pcl2.read_points(points, field_names = ("x", "y", "z"), skip_nans=True):
                # check boundary info
                ### TODO: edit this boundary checking for customization
                if p[0] >= -0.5 and p[0] <= 0.5 and p[1] >= -0.5 and p[1] <= 0.5:
                    # remove this point cloud, otherwise can't generate collision-free planning
                    continue
                if p[2] >= 1.8:
                    # remove camera point cloud
                    continue
                points_list.append([p[0],p[1],p[2]])
            merged_points += points_list
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        #create pcl from points
        merged_pointcloud2 = pcl2.create_cloud_xyz32(header, merged_points)
        #publish
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.merge_pub.publish(merged_pointcloud2)

if __name__ == '__main__':
    rospy.init_node('pcd_merger')
    pointCloudMerger = PointCloudMerger(3)
    rospy.spin()
