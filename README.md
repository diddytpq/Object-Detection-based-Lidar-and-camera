# Object-Detection-based-Lidar-and-camera

# Install mmlab
    https://mmdetection3d.readthedocs.io/en/stable/getting_started.html#


# Real envs
## 1. Connect lidar and camera
    roslaunch velodyne_camera connect_lidar_camera.launch

## 2. get velodyne data using python
    python src/detection_pkg/MMLAB_based/point_pillar/src/get_velodyne_data.py

## 3. Run PointPillar
    python src/detection_pkg/MMLAB_based/point_pillar/src/real_lidar_pillars.py

    or 

    python src/detection_pkg/MMLAB_based/real_lidar_pillars.py

    or 

    python src/detection_pkg/MMLAB_based/rosbag_pillars.py



# Gazebp envs

## 1. Run gazebo world
    roslaunch mecanum_robot_gazebo mecanum_velodyne.launch

## 2. Run Second
    python src/detection_pkg/second.pytorch/test.py

## 3. Run Second
    python src/detection_pkg/second.pytorch/real_lidar_pillars.py

## utility
* kitti data to rosbag
    ```
    kitti2bag -t {folder-name} -r {folder-number} raw_synced .
    ```
* play rosbag
    ```
    rosbag play {rosbag-name}.bag --clock --pause --loop
    ```