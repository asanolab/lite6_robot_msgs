#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys
import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import yaml
import argparse
import os
import time
import sys
import roslib
import tf

from xarm_msgs.msg import XyzCoor
from geometry_msgs.msg import PointStamped

rospy.init_node("Droplet_CamTransToBase", anonymous=True)


def callback(data):
    obj_camera_coor = PointStamped()
    obj_camera_coor.header.frame_id = "camera_link"
    obj_camera_coor.header.stamp = rospy.Time()
    # obj_base_coor = PointStamped()
    # rospy.loginfo(rospy.get_caller_id() + "\nsubscribe xyz:" + "\n%s", data)
    obj_camera_coor.point.x = data.x
    obj_camera_coor.point.y = data.y
    obj_camera_coor.point.z = data.z
    listener = tf.TransformListener()
    listener.waitForTransform('/link_base', '/camera_link', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/link_base', '/camera_link', rospy.Time(0))
    # print(obj_base_coor, obj_camera_coor)
    # print("camera\n", obj_camera_coor)
    # print("base\n", obj_base_coor)
    obj_base_coor = listener.transformPoint("/link_base", obj_camera_coor)
    rospy.loginfo(f'Coordinate in base link system:\n{obj_base_coor}')


def listener():
    rospy.Subscriber("droplet_xyz_in_camera", XyzCoor, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
