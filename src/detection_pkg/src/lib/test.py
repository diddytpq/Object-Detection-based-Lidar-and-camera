import argparse
from pathlib import Path
import sys


FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())  # add code to path

path = str(FILE.parents[0])
sys.path.append(path + '/PointPillars')

import cv2
import numpy as np
import os
import torch
import pdb


from utils import setup_seed, read_points, read_calib, read_label, \
    keep_bbox_from_image_range, keep_bbox_from_lidar_range, vis_pc, \
    vis_img_3d, bbox3d2corners_camera, points_camera2image, \
    bbox_camera2lidar
from model.custom_model import VoxelLayer


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


def main(args):

    np.random.seed(22)

    device = torch.device("cuda:0")
    test_layer = VoxelLayer(voxel_size=[0.16, 0.16, 0.16],
                            point_cloud_range=[0, -39.68, -3, 69.12, 39.68, 1],
                            max_voxels=16000,
                            max_num_points=32,
                            device = device,
                            pillar_flag = True)

    
    pc_folder_path = "datasimple/velodyne/"
    pc_file_list = os.listdir(pc_folder_path)
    pc = read_points(pc_folder_path + "/" + pc_file_list[0])

    pc = point_range_filter(pc)

    pc_tensor = torch.from_numpy(pc).to(device)
    # voxels, coors_batch, npoints_per_voxel = test_layer([pc_tensor])
    voxels, coors_batch, npoints_per_voxel, pillar_coors_batch = test_layer([pc_tensor])


    print(voxels.size())
    print(coors_batch.size())
    print(npoints_per_voxel.size())
    print(pillar_coors_batch.size())

    # # voxels_th = voxels_th.cpu().numpy()
    # # indices_th = indices_th.cpu().numpy()
    # # num_p_in_vx_th = num_p_in_vx_th.cpu().numpy()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Configuration Parameters')
    parser.add_argument('--ckpt', default='pretrained/epoch_160.pth', help='your checkpoint for kitti')
    parser.add_argument('--pc_folder_path', help='your point cloud file path')
    parser.add_argument('--no_cuda', action='store_true',
                        help='whether to use cuda')
    args = parser.parse_args()

    main(args)
