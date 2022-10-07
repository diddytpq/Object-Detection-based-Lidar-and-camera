import numpy as np
import matplotlib.pyplot as plt
import pickle
from pathlib import Path

import torch
from google.protobuf import text_format
from second.utils import simplevis
from second.pytorch.train import build_network
from second.protos import pipeline_pb2
from second.utils import config_tool

from sensor_msgs.msg import PointCloud2
import rospy
import ros_numpy

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from visualization_msgs.msg import Marker,MarkerArray

from pyquaternion import Quaternion

config_path = "/home/drcl/workspace/Object-Detection-based-Lidar-and-camera/src/detection_pkg/second.pytorch/second/train_model/all_1/pipeline.config"
ckpt_path = "/home/drcl/workspace/Object-Detection-based-Lidar-and-camera/src/detection_pkg/second.pytorch/second/train_model/all_1/voxelnet-55710.tckpt"


class Second_Detector:

    def __init__(self, model, voxel_generator, target_assigner, model_cfg, device) -> None:
        rospy.init_node("object_detection_node", anonymous=True)

        self.device = device

        self.model = model
        self.voxel_generator = voxel_generator
        self.target_assigner = target_assigner

        grid_size = self.voxel_generator.grid_size
        feature_map_size = grid_size[:2] // config_tool.get_downsample_factor(model_cfg)
        feature_map_size = [*feature_map_size, 1][::-1]

        anchors = self.target_assigner.generate_anchors(feature_map_size)["anchors"]
        anchors = torch.tensor(anchors, dtype=torch.float32, device=device)
        self.anchors = anchors.view(1, -1, 7)

        self.frame_id = "velodyne"#"velodyne" #"camera_link"
        rospy.Subscriber('/velodyne_points', PointCloud2, self.callback)

        self.pub_bbox = rospy.Publisher("/boxes", BoundingBoxArray, queue_size=1)


    def point_range_filter(self, pts, point_range=[0, -39.68, -3, 69.12, 39.68, 1]):
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

    def callback(self, data):
        N = data.width #changed to pointcloud width
        pc = ros_numpy.numpify(data)
        points = np.zeros((N, 4)).astype(np.float32)
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensity']

        self.points_data=points.reshape(-1,4)

        self.points_data = self.point_range_filter(points)

        voxel_data = self.voxel_generator.generate(points, max_voxels=90000)

        voxels, coords, num_points = voxel_data['voxels'], voxel_data['coordinates'], voxel_data['num_points_per_voxel']

        coords = np.pad(coords, ((0, 0), (1, 0)), mode='constant', constant_values=0)
        voxels = torch.tensor(voxels, dtype=torch.float32, device=self.device)
        coords = torch.tensor(coords, dtype=torch.int32, device=self.device)
        num_points = torch.tensor(num_points, dtype=torch.int32, device=self.device)

        example = {
        "anchors": self.anchors,
        "voxels": voxels,
        "num_points": num_points,
        "coordinates": coords,
        }

        pred = self.model(example)[0]

        boxes_lidar = pred["box3d_lidar"].detach().cpu().numpy()

        self.send_bbox_label(boxes_lidar)


    def send_bbox_label(self, lidar_bboxes):

        num_detects = len(lidar_bboxes)

        arr_bbox = BoundingBoxArray()
        arr_score=MarkerArray()


        for i in range(num_detects):

            # if scores[i] < 0.5 : continue

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

            print(lidar_bboxes[i][6])

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

            # marker = Marker()
            # marker.header.frame_id = self.frame_id
            # marker.header.stamp = rospy.Time.now()
            # marker.ns = "basic_shapes"
            # marker.id = i
            # marker.type = Marker.TEXT_VIEW_FACING
            # marker.action = Marker.ADD
            # marker.lifetime=rospy.Duration(0.15)
            # marker.scale.x = 4
            # marker.scale.y = 4
            # marker.scale.z = 4

            # marker.color.r = 0.0
            # marker.color.g = 0.0
            # marker.color.b = 1
            # marker.color.a = 1
            # marker.pose.position.x=float(lidar_bboxes[i][0])
            # marker.pose.position.y = float(lidar_bboxes[i][1])
            # marker.pose.position.z = float(lidar_bboxes[i][2]) + float(lidar_bboxes[i][5]) / 2
            # marker.text=str(np.around(scores[i],2))
            # arr_score.markers.append(marker)

        arr_bbox.header.frame_id = self.frame_id
        arr_bbox.header.stamp = rospy.Time.now()
        print("Number of detections: {}".format(num_detects))

        self.pub_bbox.publish(arr_bbox)
        # self.pub_text.publish(arr_score)

def main():

    config = pipeline_pb2.TrainEvalPipelineConfig()
    with open(config_path, "r") as f:
        proto_str = f.read()
        text_format.Merge(proto_str, config)

    model_cfg = config.model.second
    config_tool.change_detection_range(model_cfg, [-50, -50, 50, 50])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    net = build_network(model_cfg).to(device).eval()
    net.load_state_dict(torch.load(ckpt_path))
    target_assigner = net.target_assigner
    voxel_generator = net.voxel_generator

    Second_net = Second_Detector(net, voxel_generator, target_assigner, model_cfg, device)

    rospy.spin()


if __name__ == '__main__':
    main()