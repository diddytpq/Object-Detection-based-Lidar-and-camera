# Object-Detection-based-Lidar-and-camera

## 1. Connect lidar and camera
    roslaunch detection_pkg connect_lidar_camera.launch

## 2. get velodyne data using python
    python src/test_pkg/src/get_velodyne_data.py

## 3. Run PointPillar
    python src/detection_pkg/src/real_lidar_pillars.py


## utility
* kitti data to rosbag
    ```
    kitti2bag -t {folder-name} -r {folder-number} raw_synced .
    ```
* play rosbag
    ```
    rosbag play {rosbag-name}.bag --clock --pause --loop
    ```