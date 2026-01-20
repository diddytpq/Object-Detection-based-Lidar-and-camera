# 🚗 LiDAR-Camera 센서 융합 기반 3D 객체 탐지 시스템

## **📋 프로젝트 개요**

| 항목 | 내용 |
| --- | --- |
| **프로젝트명** | Object-Detection-based-Lidar-and-camera |
| **개발 기간** | 석사 과정 연구 프로젝트 |
| **개발 환경** | Ubuntu, ROS (Robot Operating System) |
| **주요 기술** | Python, PyTorch, ROS, Gazebo, CUDA |
| **역할** | 전체 시스템 설계 및 구현 |

---

## **🎯 프로젝트 목표**

자율주행 시스템에서 핵심적인 **LiDAR와 카메라 센서 융합을 통한 3D 객체 탐지 시스템** 개발

- 실시간 Velodyne LiDAR 포인트 클라우드 처리
- 카메라-LiDAR 센서 융합으로 탐지 정확도 향상
- ROS 기반 실시간 탐지 파이프라인 구축
- Gazebo 시뮬레이션 환경 지원

---

## **🏗️ 시스템 아키텍처**

```python
┌─────────────────────────────────────────────────────────────────────────┐
│                         센서 융합 3D 객체 탐지 시스템                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────────┐ │
│   │ Velodyne    │    │ RealSense   │    │       Gazebo 시뮬레이터      │ │
│   │ VLP-16      │    │ Camera      │    │    (mecanum_robot_gazebo)   │ │
│   │ LiDAR       │    │ (RGB-D)     │    │                             │ │
│   └──────┬──────┘    └──────┬──────┘    └──────────────┬──────────────┘ │
│          │                  │                          │                │
│          ▼                  ▼                          ▼                │
│   ┌────────────────────────────────────────────────────────────────────┐│
│   │                    ROS (Robot Operating System)                    ││
│   │  • /velodyne_points (PointCloud2)                                  ││
│   │  • /camera/color/image_raw (Image)                                 ││
│   │  • /boxes (BoundingBoxArray)                                       ││
│   └───────────────────────────┬────────────────────────────────────────┘│
│                               │                                         │
│                               ▼                                         │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                    3D 객체 탐지 모델                              │   │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │   │
│   │  │ PointPillars │  │   SECOND     │  │       CLOCs          │   │   │
│   │  │ (MMDet3D)    │  │  (Sparse     │  │ (Camera-LiDAR Object │   │   │
│   │  │              │  │   Conv)      │  │  Candidates Fusion)  │   │   │
│   │  └──────────────┘  └──────────────┘  └──────────────────────┘   │   │
│   └───────────────────────────┬─────────────────────────────────────┘   │
│                               │                                         │
│                               ▼                                         │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                      탐지 결과 출력                               │   │
│   │  • 3D Bounding Box (위치, 크기, 방향)                            │   │
│   │  • 객체 분류 (Car, Pedestrian, Cyclist)                         │   │
│   │  • 신뢰도 점수 (Confidence Score)                                │   │
│   └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## **🔧 핵심 기술 구현**

**1. 실시간 LiDAR 기반 객체 탐지 (PointPillars)**

```python
# real_lidar_pillars.py - ROS 기반 실시간 탐지
class Point_detector:
    def __init__(self):
        rospy.init_node("velodyne_points_sub", anonymous=True)
        
        # MMDetection3D 모델 초기화
        config = "mmdetection3d/configs/pointpillars/..."
        self.model = init_model(config, checkpoint, device="cuda:0")
        
        # ROS Subscriber/Publisher 설정
        rospy.Subscriber('/velodyne_points', PointCloud2, self.callback)
        self.pub_bbox = rospy.Publisher("/boxes", BoundingBoxArray, queue_size=1)
    
    def callback(self, data):
        # 포인트 클라우드 처리 및 추론
        points = self.process_pointcloud(data)
        result, _ = inference_detector(self.model, data)
        self.send_bbox_label(lidar_bboxes, labels, scores)
```

**2. CLOCs 센서 융합 네트워크**

카메라 2D 탐지 결과와 LiDAR 3D 탐지 결과를 융합하여 탐지 정확도 향상

```python
# fusion.py - 커스텀 융합 네트워크
class custom_fusion(nn.Module):
    def __init__(self):
        super(custom_fusion, self).__init__()
        
        # Feature extraction blocks with skip connections
        self.block_1 = Sequential(
            nn.Conv2d(5, 16, 1), nn.BatchNorm2d(16), nn.ReLU(),
            nn.Conv2d(16, 32, 1), nn.BatchNorm2d(32), nn.ReLU(),
            nn.Conv2d(32, 64, 1), nn.BatchNorm2d(64), nn.ReLU(),
        )
        self.block_2 = Sequential(...)  # 중간 블록
        self.block_3 = Sequential(...)  # 출력 블록
        
    def forward(self, input_1, tensor_index):
        x_1 = self.block_1(input_1)
        x_2 = self.block_2(x_1)
        x_12 = torch.cat((x_1, x_2), 1)  # Skip connection
        x = self.block_3(x_12)
        return x, flag
```

**3. 메카넘 휠 로봇 플랫폼 통합**

Gazebo 시뮬레이션 환경에서 센서 통합 테스트 가능

```python
<!-- mecanum.urdf.xacro - 로봇 URDF 정의 -->
<robot name="omni_manipulator">
    <xacro:mecanum_robot/>
    <xacro:mecanum_wheel prefix="front_R" parent="base_link" reflect="true">
        <origin xyz="0.23 -0.2541 -0.11838" rpy="${-pi/2} 0 0"/> 
    </xacro:mecanum_wheel>
    <!-- Velodyne LiDAR 센서 마운트 -->
    <xacro:sensor_gazebo/>
</robot>
```

---

## **📊 성능 평가 결과 (KITTI 데이터셋)**

**CLOCs 센서 융합 성능 개선**

| Method | 3D AP (Easy) | 3D AP (Moderate) | 3D AP (Hard) | BEV AP (Easy) | BEV AP (Moderate) | BEV AP (Hard) |
| --- | --- | --- | --- | --- | --- | --- |
| **SECOND (baseline)** | 88.79 | 79.09 | 76.04 | 92.52 | 88.38 | 87.31 |
| **SECOND + Cas-RCNN** | 91.35 | 82.01 | 77.09 | 94.57 | 90.95 | 88.1 |
| **Improvement** | **+2.56** | **+2.92** | **+1.05** | **+2.05** | **+2.57** | **+0.79** |

| Method | 3D AP (Easy) | 3D AP (Moderate) | 3D AP (Hard) | BEV AP (Easy) | BEV AP (Moderate) | BEV AP (Hard) |
| --- | --- | --- | --- | --- | --- | --- |
| **PointPillar (baseline)** | 87.44 | 77.73 | 73.34 | 92.05 | 88.18 | 86.75 |
| **PointPillar + Cas-RCNN** | 88.94 | 79.99 | 74.55 | 93.03 | 89.63 | 86.45 |
| **Improvement** | **+1.5** | **+2.26** | **+1.21** | **+0.98** | **+1.45** | -0.3 |

---

## **📁 프로젝트 구조**

```python
Object-Detection-based-Lidar-and-camera/
├── src/
│   ├── detection_pkg/           # 3D 객체 탐지 패키지
│   │   ├── CLOCs_LQS/           # Camera-LiDAR 융합 네트워크
│   │   │   ├── train.py         # 학습 스크립트
│   │   │   ├── eval.py          # 평가 스크립트
│   │   │   └── tool/
│   │   │       ├── fusion.py    # 융합 네트워크 정의
│   │   │       └── nms.py       # Non-Maximum Suppression
│   │   ├── MMLAB_based/         # MMDetection3D 기반 모델
│   │   │   ├── mmdetection3d/   # 3D 탐지 프레임워크
│   │   │   ├── real_lidar_pillars.py  # 실시간 탐지
│   │   │   └── rosbag_pillars.py      # ROSBag 탐지
│   │   ├── SECOND_based/        # SECOND 모델 구현
│   │   └── custom_model/        # 커스텀 모델 (NuScenes, KITTI)
│   │
│   ├── velodyne_camera/         # LiDAR-카메라 연동 패키지
│   │   └── launch/
│   │       └── connect_lidar_camera.launch
│   │
│   ├── velodyne_pkg/            # Velodyne LiDAR 드라이버
│   │   ├── velodyne_driver/
│   │   └── velodyne_pointcloud/
│   │
│   ├── realsense-ros/           # Intel RealSense 카메라 드라이버
│   │
│   ├── mecanum_pkg/             # 메카넘 로봇 플랫폼
│   │   ├── mecanum_robot_description/  # URDF 모델
│   │   └── mecanum_robot_gazebo/       # Gazebo 시뮬레이션
│   │
│   └── point_cloud_vis/         # KITTI 데이터셋 시각화 도구
│       └── kitti_object_vis/
│
└── rosbag/                      # 테스트용 ROS 데이터
    ├── 1.bag ~ 5.bag
    └── test.bag
```

---

## **🛠️ 사용 기술 스택**

**프레임워크 & 라이브러리**

| 분류 | 기술 |
| --- | --- |
| **로봇 미들웨어** | ROS (Robot Operating System) |
| **딥러닝** | PyTorch, MMDetection3D, OpenPCDet |
| **시뮬레이션** | Gazebo |
| **시각화** | RViz, Mayavi, OpenCV |
| **포인트 클라우드** | PCL (Point Cloud Library), ros_numpy |

**하드웨어 센서**

| 센서 | 용도 |
| --- | --- |
| **Velodyne VLP-16** | 3D LiDAR 포인트 클라우드 취득 |
| **Intel RealSense** | RGB-D 카메라 영상 취득 |

### **데이터셋**

- **KITTI 3D Object Detection Dataset**
- **NuScenes Dataset**

---

## **🚀 실행 방법**

**1. 실제 센서 환경**

```python
# Step 1: LiDAR와 카메라 연결
roslaunch velodyne_camera connect_lidar_camera.launch

# Step 2: PointPillars 실시간 탐지 실행
python src/detection_pkg/MMLAB_based/real_lidar_pillars.py
```

**2. Gazebo 시뮬레이션 환경**

```python
# Step 1: Gazebo 월드 실행
roslaunch mecanum_robot_gazebo mecanum_velodyne.launch

# Step 2: 탐지 모델 실행
python src/detection_pkg/MMLAB_based/rosbag_pillars.py
```

**3. CLOCs 모델 학습**

```python
# 데이터 생성
python generate_data.py --rootpath ./data/clocs_data

# 학습 실행
python train.py --epochs 200 --log-path ./log/second/faster

# 평가
python eval.py
```

---

## **💡 주요 성과 및 기여**

1. **센서 융합 아키텍처 설계**
    - LiDAR 3D 탐지와 카메라 2D 탐지 결과를 효과적으로 융합
    - Skip Connection 기반 커스텀 융합 네트워크 구현
2. **실시간 처리 파이프라인**
    - ROS 기반 실시간 포인트 클라우드 처리
    - GPU 가속 추론으로 실시간 탐지 달성
3. **통합 테스트 환경 구축**
    - Gazebo 시뮬레이션 환경 구축
    - 메카넘 휠 로봇 플랫폼 통합
4. **탐지 성능 개선**
    - CLOCs 융합으로 3D AP 최대 **2.92%p** 향상 (Moderate 난이도)
    - 다양한 난이도에서 일관된 성능 개선 달성

---

## **📚 참고 문헌**

- **CLOCs**: Camera-LiDAR Object Candidates Fusion for 3D Object Detection
- **PointPillars**: Fast Encoders for Object Detection from Point Clouds
- **SECOND**: Sparsely Embedded Convolutional Detection
- **OpenPCDet**: Open-MMLab Point Cloud Detection Toolbox
- **MMDetection3D**: OpenMMLab 3D Detection Toolbox

---

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
