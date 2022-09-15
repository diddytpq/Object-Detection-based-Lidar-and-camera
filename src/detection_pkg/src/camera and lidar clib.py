#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys
import time
import threading
import multiprocessing

# External modules
import cv2
import numpy as np
import matplotlib.cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ROS modules
# PKG = 'auro_calibration'
# import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import yaml
import sensor_msgs.msg
import argparse

# Global variables
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

def points3D_to_points2D(points3D,img):    #Velodyne Points to 2D Points(image plane)
    fx=CAMERA_MODEL.fx()
    fy=CAMERA_MODEL.fy()
    cx=CAMERA_MODEL.cx()
    cy=CAMERA_MODEL.cy()
    Tx=CAMERA_MODEL.Tx()
    Ty=CAMERA_MODEL.Ty()
    px = (fx*points3D[:,0] + Tx) / points3D[:,2] + cx;
    py = (fy*points3D[:,1] + Ty) / points3D[:,2] + cy;
    points2D=np.column_stack((px,py))

    #points2D = np.asarray(points2D)
    inrange = np.where((points2D[:, 0] >= 0) &
                       (points2D[:, 1] >= 0) &
                       (points2D[:, 0] < img.shape[1]) &
                       (points2D[:, 1] < img.shape[0]))
    points2D = points2D[inrange[0]].round().astype('int')
    return points2D

def project_point_cloud(velodyne_link, img_msg, image_pub):
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'mono16')
    except CvBridgeError as e: 
        rospy.logerr(e)
        return
    
    # Transform the point cloud
    try:
        transform = TF_BUFFER.lookup_transform('thermal_frame', 'velodyne_frame_from_thermal', rospy.Time())    #get transform parameter from rgb_frame to velodyne_frame
        velodyne_link = do_transform_cloud(velodyne_link, transform)
    except tf2_ros.LookupException:
        pass

    #cv2.normalize(img, img, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_16U)
    # Extract points from message
    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne_link)
    points3D = np.asarray(points3D.tolist())
    
    # Filter points in front of camera
    #points3D[:,2] : front view direction
    #points3D[:,0] : right area from center(+), left area from center(-)
    #points3D[:,1] : under area from center(+), upper area from center(-)
    inrange = np.where((points3D[:, 0] > -8) &
                       (points3D[:, 0] < 8) &
                       (points3D[:, 1] > -5) &
                       (points3D[:, 1] < 5) &
                       (points3D[:, 2] > -0) &
                       (points3D[:, 2] < 30))
    max_intensity = np.max(points3D[:, -1])
    points3D = points3D[inrange[0]]

    pc_distance=np.sqrt(points3D[:, 0]*points3D[:, 0]+points3D[:, 1]*points3D[:, 1]+points3D[:, 2]*points3D[:, 2]) #Straight distance from camera

    # Color map for the points
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points3D[:, -1] / max_intensity) * 255 #intensity color view
    
    #colors=pc_distance*255/30  #distance value to color value (min 0:0meter ~ max 255:30meter)
    
    points2D = points3D_to_points2D(points3D,img) #3D Points to 2D Points transform
    
    #img3 = np.zeros([img.shape[0],img.shape[1],3],dtype=np.uint8)  
    #img3.fill(0) #blank black image
    # Draw the projected 2D points
    for i in range(len(points2D)):
        #cv2.circle(img3, tuple(points2D[i]), 1, (colors[i],colors[i],colors[i]), -1)
        cv2.circle(img, tuple(points2D[i]), 1, (colors[i]), -1)

    # Publish the projected points image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "mono16"))
        #image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img3, "bgr8"))
    except CvBridgeError as e: 
        rospy.logerr(e)

def callback(image, camera_info,velodyne_link, image_pub=None):
    global CAMERA_MODEL, FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER

    # Setup the pinhole camera model
    if FIRST_TIME:
        FIRST_TIME = False

        # Setup camera model
        rospy.loginfo('Setting up camera model')
        CAMERA_MODEL.fromCameraInfo(camera_info)
        
        # TF listener
        rospy.loginfo('Setting up static transform listener')
        TF_BUFFER = tf2_ros.Buffer()
        TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
    if PROJECT_MODE:
        project_point_cloud(velodyne_link, image, image_pub)

def listener(image_color, velodyne_points, camera_lidar=None):
    # Start node
    rospy.init_node('display_camera_lidar', anonymous=True)
    rospy.loginfo('Projection mode: %s' % PROJECT_MODE)
    rospy.loginfo('CameraInfo topic: %s' % camera_info)
    rospy.loginfo('Image topic: %s' % image_color)
    rospy.loginfo('PointCloud2 topic: %s' % velodyne_points)
    rospy.loginfo('Output topic: %s' % camera_lidar)

    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    image_sub = message_filters.Subscriber(image_color, Image)
    velodyne_sub = message_filters.Subscriber(velodyne_points, PointCloud2)

    # Publish output topic
    image_pub = None
    if camera_lidar: image_pub = rospy.Publisher(camera_lidar, Image, queue_size=5)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub,info_sub, velodyne_sub], queue_size=5, slop=0.1)
    ats.registerCallback(callback, image_pub)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    camera_info = "/osr/image_thermal_camerainfo"
    image_color = "/osr/image_thermal"
    velodyne_points = "/osr/lidar_pointcloud"
    camera_lidar = "/camera_lidar2"  #output topic name
    PROJECT_MODE = "true"

    # Start subscriber
    listener(image_color, velodyne_points, camera_lidar)


