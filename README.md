# Object-Detection-based-Lidar-and-camera

# Real envs
## 1. Connect lidar and camera
    roslaunch velodyne_camera connect_lidar_camera.launch

## 2. get velodyne data using python
    python src/detection_pkg/point_pillar/src/get_velodyne_data.py

## 3. Run PointPillar
    python src/detection_pkg/point_pillar/src/real_lidar_pillars.py



# Gazebp envs

## 1. Run gazebo world
    roslaunch mecanum_robot_gazebo mecanum_velodyne.launch

## 2. Run Second
    python src/detection_pkg/second.pytorch/src/real_lidar_pillars.py

## utility
* kitti data to rosbag
    ```
    kitti2bag -t {folder-name} -r {folder-number} raw_synced .
    ```
* play rosbag
    ```
    rosbag play {rosbag-name}.bag --clock --pause --loop
    ```