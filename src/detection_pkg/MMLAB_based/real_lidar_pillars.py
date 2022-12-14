import argparse
from pathlib import Path
import sys

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import rospy
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from visualization_msgs.msg import Marker,MarkerArray
import ros_numpy

from pyquaternion import Quaternion


import torch
import os
import cv2
import pdb

from mmdet3d.apis import inference_detector, init_model, show_result_meshlab
from mmdet3d.core.bbox.structures.box_3d_mode import Box3DMode, LiDARInstance3DBoxes
from mmdet3d.core.points.base_points import BasePoints


CLASSES = {
        'Pedestrian': 0, 
        'Cyclist': 1, 
        'Car': 2
        }
LABEL2CLASSES = {v:k for k, v in CLASSES.items()}
pcd_limit_range = np.array([0, -40, -3, 70.4, 40, 0.0], dtype=np.float32)
# pcd_limit_range = np.array([0, -39.68, 0, 69.12, 39.68, 3], dtype=np.float32)

parser = argparse.ArgumentParser(description='Configuration Parameters')
parser.add_argument('--ckpt', default='pretrained/epoch_160.pth', help='your checkpoint for kitti')
parser.add_argument('--pc_folder_path', help='your point cloud file path')
parser.add_argument('--pc_path', help='your point cloud path')
parser.add_argument('--calib_path', default='', help='your calib file path')
parser.add_argument('--gt_path', default='', help='your ground truth path')
parser.add_argument('--img_path', default='', help='your image path')
parser.add_argument('--no_cuda', action='store_true',
                    help='whether to use cuda')
args = parser.parse_args()




class Point_detector:

    def __init__(self):
        rospy.init_node("velodyne_points_sub", anonymous=True)


        config = "mmdetection3d/configs/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class.py"
        checkpoint = "mmdetection3d/checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth"
        
        config = "mmdetection3d/configs/pointpillars/hv_pointpillars_secfpn_sbn-all_4x8_2x_nus-3d.py"
        checkpoint = "mmdetection3d/checkpoints/hv_pointpillars_secfpn_sbn-all_4x8_2x_nus-3d_20210826_225857-f19d00a3.pth"


        
        device = "cuda:0"

        self.model = init_model(config, checkpoint, device=device)

        # self.frame_id = "velo_link"#"velodyne" #"camera_link"
        self.frame_id = "velodyne"#"velodyne" #"camera_link"
        
        rospy.Subscriber('/velodyne_points', PointCloud2, self.callback)
        # rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self.callback)

        self.pub_bbox = rospy.Publisher("/boxes", BoundingBoxArray, queue_size=1)
        self.pub_text=rospy.Publisher("/scores",MarkerArray,queue_size=0)
        # self.pub_cloud = rospy.Publisher("/cloud_filtered", PointCloud2, queue_size=0)

        rospy.spin()

    def callback(self, data):
        N = data.width #changed to pointcloud width
        pc = ros_numpy.numpify(data)
        points = np.zeros((N, 5)).astype(np.float32)
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensity']

        # points=ros_numpy.point_cloud2.pointcloud2_to_array(data)
        # points = structured_to_unstructured(points)
        self.points_data=points.reshape(-1,5)

        self.points_data = torch.from_numpy(self.points_data)

        data = BasePoints(self.points_data, points_dim=5)

        result, data = inference_detector(self.model, data)

        # lidar_bboxes = result[0]['boxes_3d'].tensor.numpy() 
        # labels = result[0]['labels_3d'].numpy()
        # scores = result[0]['scores_3d'].numpy()
        # print(lidar_bboxes)
        # print(labels)
        # print(scores)

        lidar_bboxes = result[0]["pts_bbox"]["boxes_3d"].tensor.numpy()
        labels = result[0]['pts_bbox']['labels_3d'].numpy()
        scores = result[0]['pts_bbox']['scores_3d'].numpy()
        

        self.send_bbox_label(lidar_bboxes, labels, scores)


    def send_bbox_label(self, lidar_bboxes, labels, scores):

        num_detects = len(lidar_bboxes)

        arr_bbox = BoundingBoxArray()
        arr_score=MarkerArray()


        for i in range(num_detects):

            if scores[i] < 0.4 : continue

            bbox = BoundingBox()
            bbox.header.frame_id = self.frame_id
            bbox.header.stamp = rospy.Time.now()

            bbox.pose.position.x = float(lidar_bboxes[i][0])
            bbox.pose.position.y = float(lidar_bboxes[i][1])
            bbox.pose.position.z = float(lidar_bboxes[i][2])
            # bbox.pose.position.z = float(lidar_boxes[i][2]) + float(lidar_boxes[i][5]) / 2
            bbox.dimensions.x = float(lidar_bboxes[i][3])  # width
            bbox.dimensions.y = float(lidar_bboxes[i][4])  # length
            bbox.dimensions.z = float(lidar_bboxes[i][5])  # height

            q = Quaternion(axis=(0, 0, 1), radians=float(lidar_bboxes[i][6]))
            bbox.pose.orientation.x = q.x
            bbox.pose.orientation.y = q.y
            bbox.pose.orientation.z = q.z
            bbox.pose.orientation.w = q.w
            # q = get_quaternion_from_euler(0,0,lidar_bboxes[i][6])
            # bbox.pose.orientation.x = q[0] #x
            # bbox.pose.orientation.y = q[1] #y
            # bbox.pose.orientation.z = q[2] #z
            # bbox.pose.orientation.w = q[3] #w

            arr_bbox.boxes.append(bbox)

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.lifetime=rospy.Duration(0.15)
            marker.scale.x = 4
            marker.scale.y = 4
            marker.scale.z = 4

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1
            marker.color.a = 1
            marker.pose.position.x=float(lidar_bboxes[i][0])
            marker.pose.position.y = float(lidar_bboxes[i][1])
            marker.pose.position.z = float(lidar_bboxes[i][2]) + float(lidar_bboxes[i][5]) / 2
            marker.text=str(np.around(scores[i],2))
            arr_score.markers.append(marker)

        arr_bbox.header.frame_id = self.frame_id
        arr_bbox.header.stamp = rospy.Time.now()
        print("Number of detections: {}".format(num_detects))

        self.pub_bbox.publish(arr_bbox)
        # self.pub_text.publish(arr_score)




def point_range_filter(pts, point_range=[0, -39.68, -3, 69.12, 39.68, 1]):
    '''
    data_dict: dict(pts, gt_bboxes_3d, gt_labels, gt_names, difficulty)
    point_range: [x1, y1, z1, x2, y2, z2]
    '''
    flag_x_low = pts[:, 0] > point_range[0] # 0
    flag_y_low = pts[:, 1] > point_range[1] # -39.68
    flag_z_low = pts[:, 2] > point_range[2] # -3
    flag_x_high = pts[:, 0] < point_range[3] # 69.12
    flag_y_high = pts[:, 1] < point_range[4] # 39.68
    flag_z_high = pts[:, 2] < point_range[5] # 1
    keep_mask = flag_x_low & flag_y_low & flag_z_low & flag_x_high & flag_y_high & flag_z_high
    pts = pts[keep_mask]
    return pts 

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

if __name__ == '__main__':

    pillars_net = Point_detector()
