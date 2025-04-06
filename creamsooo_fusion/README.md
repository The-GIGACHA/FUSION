# LiDAR-Camera Fusion for Autonomous Driving

This package provides a comprehensive LiDAR-camera fusion system for autonomous driving, incorporating both object detection and lane detection. The system fuses LiDAR point cloud data with camera-based detection to enhance performance in various driving conditions.

## System Overview

The fusion system combines multiple sensing modalities to achieve more reliable detection and tracking in challenging scenarios.

### Key Features
- Object detection and tracking with YOLOv8 + LiDAR fusion
- Lane detection using YOLOPv2 + LiDAR fusion
- Multiple fusion strategies for object detection
- Ground plane extraction from LiDAR point cloud
- 3D object and lane clustering
- Emergency detection for collision avoidance
- Rich visualization in RViz with integrated view

## Object Detection Fusion

### Algorithm Details

#### 1. YOLOv8 Object Detection
- High-performance object detection using YOLOv8
- Segmentation capabilities for enhanced object boundary delineation
- Object tracking using ByteTrack

#### 2. LiDAR-Camera Fusion Strategies
The system supports multiple fusion modes:
- **Mode 0**: Traditional bounding box fusion - LiDAR points are assigned to camera-detected objects based on 2D projections
- **Mode 1**: Clustering-based fusion - LiDAR points are first clustered in 3D, then associated with camera detections
- **Mode 2**: Combined approach using both methods for maximum reliability

#### 3. Distance Calculation and Emergency Detection
- Precise distance calculation to detected objects
- Emergency alerts based on configured distance thresholds
- Classification of object distances based on proximity

#### 4. 3D Object Localization
- Transformation of 2D detections to 3D world coordinates
- Generation of 3D bounding boxes for detected objects
- Publication of object positions for path planning

## Lane Detection Fusion

### Algorithm Details

#### 1. Point Cloud Processing
The system processes LiDAR point cloud data through several steps:
- Height-based filtering to extract ground points
- Removal of points outside the region of interest
- Projection of ground points onto the camera image

#### 2. LiDAR-Camera Fusion
The core of the system is the fusion of LiDAR point cloud with camera-based lane segmentation:
- LiDAR points are transformed to the camera coordinate system
- Points are projected onto the image plane
- Points falling within lane segments identified by YOLOPv2 are classified as lane points
- A search radius around each point is used to handle calibration inaccuracies

#### 3. Height-Based Clustering
Lane points are clustered based on their height values to separate different lane markings:
- Points are grouped into bins of 1cm height resolution
- Each height group is processed separately with DBSCAN clustering
- Clusters from different height groups represent different lane segments

#### 4. Lane Selection
The system selects the most likely left and right lane boundaries:
- Clusters are classified as left or right based on their y-coordinate
- The closest cluster to the vehicle on each side is selected
- For partial lane detection (single lane visible), the system estimates the position of the other lane

#### 5. Lane Center Calculation
The lane center is computed to guide vehicle navigation:
- If both lanes are detected, the center is the midpoint
- If only one lane is detected, the center is estimated using standard lane width
- The lane center is published as both a marker for visualization and a point message for navigation

## Integration System

The integration system allows simultaneous operation of both object and lane detection:

### 1. Combined Visualization
- Single RViz instance displaying both object and lane detection results
- Custom configuration for optimal visualization of all data

### 2. Coordinated Processing
- Shared LiDAR point cloud input
- Common calibration parameters
- Synchronized processing of camera and LiDAR data

## Usage

### Launch Files
- `creamsooo_fusion.launch`: Launches the object detection fusion node
- `lane_detection_fusion.launch`: Launches the lane detection fusion node
- `integration.launch`: Launches both object detection and lane detection nodes with integrated RViz visualization

### Object Detection Parameters
- `fusion_mode`: Fusion strategy selection (0, 1, or 2)
- `emergency_distance`: Distance threshold for emergency alerts
- `cluster_tolerance`: Distance threshold for clustering
- `min_cluster_size`, `max_cluster_size`: Size limits for valid clusters

### Lane Detection Parameters
- `image_width`, `image_height`: Camera resolution (default: 640x480)
- `fov`: Camera field of view (default: 90 degrees)
- `lane_width_limit`: Maximum width of lane from vehicle center (default: 3.7m)
- `lane_cluster_tolerance`: Distance threshold for clustering (default: 0.5m)
- `lane_min_cluster_size`: Minimum points for a valid lane cluster (default: 3)

### Subscribed Topics
- `/velodyne_points`: LiDAR point cloud (sensor_msgs/PointCloud2)
- `/image_jpeg/compressed`: Camera image for object detection (sensor_msgs/CompressedImage)
- `/yolo_lane_seg`: Lane segmentation image (sensor_msgs/Image)
- `/yolo_result`: YOLO detection results (ultralytics_ros/YoloResult)

### Published Topics
#### Object Detection
- `/filtered_points`: Filtered point cloud (sensor_msgs/PointCloud2)
- `/object_markers`: Object boundary markers (visualization_msgs/MarkerArray)
- `/object_distance`: Object distance information (creamsooo_fusion/ObjectDistance)
- `/emergency_status`: Emergency status (std_msgs/Bool)

#### Lane Detection
- `/lane_detection_fusion/filtered_ground`: Ground points (sensor_msgs/PointCloud2)
- `/lane_detection_fusion/lane_points`: Detected lane points (sensor_msgs/PointCloud2)
- `/lane_detection_fusion/lane_markers`: Lane boundary markers (visualization_msgs/MarkerArray)
- `/lane_detection_fusion/lane_center`: Lane center point (geometry_msgs/PointStamped)
- `/lane_detection_fusion/lane_debug_image`: Debug image with overlay (sensor_msgs/Image)
- `/lane_detection_fusion/lane_debug_image_large`: High-resolution debug image (sensor_msgs/Image)

## Dependencies
- ROS Noetic
- PCL
- OpenCV
- YOLOv8 (ultralytics_ros)
- YOLOPv2 for lane segmentation
- Eigen
- ByteTrack (for object tracking)

## Quick Start

1. Clone the repository to your ROS workspace
2. Build the workspace: `catkin_make`
3. Source your workspace: `source devel/setup.bash`
4. Launch the integrated system: `roslaunch creamsooo_fusion integration.launch`

## Authors
- Object Detection Fusion: Team Creamsooo
- Lane Detection Fusion: Chaemin Park

## Conclusion
This integrated system provides robust perception capabilities for autonomous vehicles, combining the strengths of camera-based detection with LiDAR point cloud processing. The system enables reliable operation in various driving conditions and challenging environments. 