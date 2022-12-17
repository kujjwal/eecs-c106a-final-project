#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2022
Author: Amay Saxena, Emma Stephan, Jewook Ryu

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import segment_image
from pointcloud_segmentation import segment_pointcloud


def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.
    return np.reshape(camera_info_msg.K, (3, 3))

def isolate_object_of_interest(points, image, cam_matrix, trans, rot):
    segmented_image = segment_image(image)
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    return points

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')


def main():
    CAM_INFO_TOPIC = '/io/internal_camera/right_hand_camera/camera_info'
    RGB_IMAGE_TOPIC = '/io/internal_camera/right_hand_camera/image_rect'

    rospy.Subscriber("user_messages", TimestampString, callback)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':

    rospy.init_node('image_segmentation')

    main()
