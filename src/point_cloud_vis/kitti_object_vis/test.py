import cv2
from kitti_object import kitti_object, show_lidar_with_depth, show_lidar_on_image, \
                         show_image_with_boxes, show_lidar_topview_with_boxes, get_box3d_pts_list

import torch
import os
import numpy as np
import pickle


def count_point_in_box(box_corner, pc):

    point_count = 0

    x_min = box_corner[:,0].min()
    y_min = box_corner[:,1].min()
    z_min = box_corner[:,2].min()
    x_max = box_corner[:,0].max()
    y_max = box_corner[:,1].max()
    z_max = box_corner[:,2].max()

    # for (x,y,z,r) in pc:
    #     if x_min < x < x_max:
    #         if y_min < y < y_max:
    #             if z_min < z < z_max:
    #                 point_count += 1

    point_index = np.where((x_min < pc[:,0]) & (pc[:,0] < x_max) & (y_min < pc[:,1]) & (pc[:,1] < y_max) & (z_min < pc[:,2]) & (pc[:,2] < z_max) )

    point_count = len(point_index[0])

    return point_count

def save_3d_bbox(data_path, origin_train_data_path, save_data_path):

    """
    kitti label format으로 저장된 텍스트 파일 --> 각 label data의 3D box corner point(총 8개)를 저장
    """

    file_list = os.listdir("/media/drcl/DATADRIVE1/kitti/clocs/data/clocs_data/3D_kitti_format/")
    save_file_list = os.listdir(save_data_path)

    dataset = kitti_object("/media/drcl/DATADRIVE1/kitti", "/media/drcl/DATADRIVE1/kitti/clocs/data/clocs_data/3D_kitti_format/", "training")

    for i in file_list:
        print("==============================================================")
        print(i)

        if (i[:-3] + ".pickle") in save_file_list: continue

        # i = "000189.pt"

        data_idx = int(i[:-3])

        objects = dataset.get_label_objects(data_idx)
        pc_velo = dataset.get_lidar(data_idx)
        calib = dataset.get_calibration(data_idx)
        img = dataset.get_image(data_idx)
        img_height, img_width, _ = img.shape

        # box_3d_points = show_lidar_with_depth(pc_velo, objects, calib, fig_3d, True, img_width, img_height)
        box_3d_points = get_box3d_pts_list(objects, calib)

        with open(save_data_path + i[:-3] + '.pickle', 'wb') as f:
            pickle.dump(box_3d_points, f, pickle.HIGHEST_PROTOCOL)




def save_new_train_data(_3D_bbox_data_path, origin_train_data_path, save_data_path):

    """
    kitti label format으로 저장된 텍스트 파일 --> 각 label data의 3D box corner point(총 8개)를 저장
    """

    _3D_bbox_file_list = os.listdir(_3D_bbox_data_path)

    origin_train_data_file_list = os.listdir(origin_train_data_path)

    save_file_list = os.listdir(save_data_path)

    dataset = kitti_object("/media/drcl/DATADRIVE1/kitti", "/media/drcl/DATADRIVE1/kitti/clocs/data/clocs_data/3D_kitti_format/", "training")


    for i in origin_train_data_file_list:

        i = "000304.pt"

        print("==============================================================")
        print(i)

        if i in save_file_list: continue

        n_count_list = []
        point_list = []

        train_data = torch.load(origin_train_data_path + i)

        data_idx = int(i[:-3])

        pc_velo = dataset.get_lidar(data_idx)

        with open(_3D_bbox_data_path + i[:-3] + '.pickle', 'rb') as f:
            box_3d_points = pickle.load(f)

        print(box_3d_points)

        for box_corner_list in box_3d_points:
            n_count = count_point_in_box(box_corner_list, pc_velo)
            print("n_count : ", n_count)

            n_count_list.append(n_count)


        for (index_2d, index_3d) in train_data['input_data']["tensor_index"]:
            point_list.append(n_count_list[index_3d])

        point_array = np.array([point_list])

        train_data['input_data']['fusion_input'] = np.vstack([train_data['input_data']['fusion_input'],point_array])

        print(save_data_path+i)

        torch.save(train_data, save_data_path+i)

        break

if __name__ == "__main__":

    save_data_path = "/home/drcl/workspace/Project-Clocs/data/clocs_data/3D_box_points/"

    origin_train_data_path = "/media/drcl/DATADRIVE1/kitti/clocs/data/clocs_data/input_data/"

    # save_3d_bbox(origin_train_data_path, save_data_path)

    _3D_bbox_data_path = "/media/drcl/DATADRIVE1/kitti/clocs/data/clocs_data/3D_box_points/"
    save_data_path = "/home/drcl/workspace/Project-Clocs/data/clocs_data/new_input_data/"

    save_new_train_data(_3D_bbox_data_path, origin_train_data_path, save_data_path)