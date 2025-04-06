# LiDAR-Camera Fusion System (creamsooo_fusion)

## Overview
This system is an ROS-based software that performs high-precision object detection and distance measurement by fusing LiDAR and camera data. It extracts accurate object position, distance, and velocity information by combining 2D object detection results from camera images with 3D LiDAR point clouds.

**Developer: creamsooo**

## Key Features
- Precise fusion based on LiDAR-camera calibration
- Multi-modal object detection and tracking
- Real-time object velocity and movement direction calculation
- Advanced object classification based on clustering
- Emergency stop detection and notification
- Intuitive 3D visualization

## Implementation Algorithms and Logic

### 1. LiDAR-Camera Calibration and Coordinate Transformation

#### 1.1 Calibration Parameter Management
- Camera intrinsic parameters (FOV, image size)
- Camera and LiDAR position/orientation parameters (X, Y, Z, Roll, Pitch, Yaw)
- Loading parameters from YAML-based configuration files

#### 1.2 Coordinate Transformation Matrix Calculation
- Rotation matrix (using Roll-Pitch-Yaw angles)
```cpp
Eigen::Matrix3d CreamsoooFusion::getRotationMatrix(double roll, double pitch, double yaw)
```
- LiDAR-camera transformation matrix
```cpp
transform_lidar_to_cam_ = Tr_cam_to_vehicle.inverse() * Tr_lidar_to_vehicle;
```
- Camera intrinsic parameter matrix
```cpp
void CreamsoooFusion::calculateCameraMatrix()
```

#### 1.3 3D-2D Projection
- Transforming LiDAR points to camera coordinate system
```cpp
Eigen::Vector3d CreamsoooFusion::transformLidarToCamera(const Eigen::Vector4d& pc_lidar)
```
- Converting camera coordinates to image pixel coordinates
```cpp
cv::Point2i CreamsoooFusion::transformCameraToImage(const Eigen::Vector3d& pc_camera)
```

### 2. Point Cloud Processing

#### 2.1 Preprocessing and Filtering
- ROI (Region of Interest) setting using PassThrough filter
  - X-axis (forward) filtering: 0~50m
  - Z-axis (height) filtering: -0.3~3.0m
  - Y-axis (lateral) filtering: -10~10m
- Downsampling using VoxelGrid filter (10cm grid)

#### 2.2 Euclidean Clustering
- Density-based clustering using KdTree-based nearest neighbor search
```cpp
pcl::EuclideanClusterExtraction<PointT> ec;
```
- Clustering parameters:
  - Cluster tolerance (default 0.5m)
  - Minimum cluster size (default 10 points)
  - Maximum cluster size (default 2000 points)

#### 2.3 Cluster Merging Algorithm
- Merging similar clusters (distance threshold 1.5m)
- Measuring cluster similarity based on centroid distance
- Merging points and recalculating centroids

#### 2.4 Classification Based on Cluster Characteristics
- Object type estimation based on size:
  - Person: height > 1.5m, width < 1.0m, depth < 1.0m
  - Vehicle: width > 1.5m or depth > 1.5m

### 3. Object Tracking and Velocity Calculation

#### 3.1 Object Tracking System
- Maintaining and managing object IDs between frames
- Distance-based matching algorithm
- Object attribute and history management

#### 3.2 Tracking Maintenance
- Object lifetime (Age) tracking
- Counting lost frames
- Object persistence logic based on duration
```cpp
void CreamsoooFusion::updateClusterPersistence()
```

#### 3.3 Velocity and Movement Calculation
- Velocity calculation based on position changes between frames
- Radial velocity calculation
- Approaching/moving away/stationary state classification
```cpp
void CreamsoooFusion::updateClusterVelocity(ClusterInfo& current_cluster, const std::vector<ClusterInfo>& prev_clusters)
```
- Noise filtering:
  - Minimum time interval check (0.005 seconds)
  - Maximum velocity limit (15.0 m/s)
  - Distance-based object matching threshold (2.0m)

### 4. LiDAR-Camera Fusion Methodology

#### 4.1 Traditional Fusion Method (Bounding Box Based)
- Based on 2D bounding boxes from YOLO detection results
- Filtering LiDAR points projected onto the image plane
- Calculating center points and distances for each object

#### 4.2 Clustering-based IoU Fusion
- Calculating IoU between LiDAR clusters and 2D detection results
```cpp
double CreamsoooFusion::calculateIoU(const vision_msgs::Detection2D& detection, const ClusterInfo& cluster)
```
- Projecting clusters to the image plane (3D to 2D)
```cpp
cv::Rect CreamsoooFusion::computeProjectedRect(const ClusterInfo& cluster)
```
- Matching based on IoU threshold (0.3)

#### 4.3 Multiple Fusion Modes
- Fusion mode 0: Using only traditional fusion method
- Fusion mode 1: Using only clustering-based fusion method
- Fusion mode 2: Using both methods (default)

### 5. Visualization and Result Output

#### 5.1 Object Marker Generation
- Bounding box-based object markers
- Cluster-based sphere markers
- IoU matching-based cylinder markers

#### 5.2 Information Text Display
- Object ID and tracking lifetime (Age)
- Distance information (in meters)
- Relative velocity and movement direction
  - APPROACHING
  - MOVING AWAY
  - STATIONARY
- Object size information

#### 5.3 Color Coding Scheme
- Color changes based on distance
- Color distinction by object type (person, vehicle, other)
- Risk level visualization (within 3m: red/purple, beyond 8m: green/blue)

### 6. Safety Features

#### 6.1 Emergency Stop Function
- Setting emergency stop distance threshold (default 1.0m)
- Calculating and evaluating risk for each object
- Detecting dangerous situations and outputting warning messages
```cpp
void CreamsoooFusion::checkEmergencyStop(const std::vector<creamsooo_fusion::ObjectDistance>& objects)
```

#### 6.2 ROS Topic-based Notifications
- Publishing emergency stop signals (/e_stop)
- Publishing object distance information (/object_distance)
- Publishing filtered LiDAR points (/filtered_points)

## System Features

### Real-time Performance Optimization
- Point cloud downsampling
- Efficient filtering pipeline
- Lightweight object tracking algorithm

### Stable Object Tracking
- Tracking ID management for temporal consistency
- Persistence algorithm to minimize object loss
- Age-based validity check for false detection filtering

### Flexible Configuration
- YAML-based parameter management
- Support for various fusion modes
- ROS framework integration

## Dependencies
- ROS (Robot Operating System)
- PCL (Point Cloud Library)
- OpenCV
- Eigen
- YOLO-based object detection system (ultralytics_ros)

## Conclusion
This system provides a powerful LiDAR-camera fusion solution that can be used in various applications such as autonomous vehicles, robot navigation, and ADAS (Advanced Driver Assistance Systems). It enables a richer and more accurate understanding of the environment through multi-modal sensor fusion and advanced object tracking. 