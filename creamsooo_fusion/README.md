# LiDAR-Camera Fusion for Lane Detection

This package provides a robust lane detection system using LiDAR-camera fusion techniques. The system fuses LiDAR point cloud data with camera-based lane segmentation to accurately detect and track lane boundaries in various driving conditions.

## System Overview

The lane detection fusion system utilizes both LiDAR point clouds and semantic segmentation results from a vision-based lane detector (YOLOPv2). By combining these complementary sensing modalities, it achieves more reliable lane detection in challenging scenarios.

### Key Features
- Ground plane extraction from LiDAR point cloud
- Projection of LiDAR points onto camera image
- Lane boundary identification using semantic segmentation
- 3D lane point clustering based on height (z) values
- Lane center estimation
- Visualization in RViz

## Algorithm Details

### 1. Point Cloud Processing
The system processes LiDAR point cloud data through several steps:
- Height-based filtering to extract ground points
- Removal of points outside the region of interest
- Projection of ground points onto the camera image

### 2. LiDAR-Camera Fusion
The core of the system is the fusion of LiDAR point cloud with camera-based lane segmentation:
- LiDAR points are transformed to the camera coordinate system
- Points are projected onto the image plane
- Points falling within lane segments identified by YOLOPv2 are classified as lane points
- A search radius around each point is used to handle calibration inaccuracies

### 3. Height-Based Clustering
Lane points are clustered based on their height values to separate different lane markings:
- Points are grouped into bins of 1cm height resolution
- Each height group is processed separately with DBSCAN clustering
- Clusters from different height groups represent different lane segments

### 4. Lane Selection
The system selects the most likely left and right lane boundaries:
- Clusters are classified as left or right based on their y-coordinate
- The closest cluster to the vehicle on each side is selected
- For partial lane detection (single lane visible), the system estimates the position of the other lane

### 5. Lane Center Calculation
The lane center is computed to guide vehicle navigation:
- If both lanes are detected, the center is the midpoint
- If only one lane is detected, the center is estimated using standard lane width
- The lane center is published as both a marker for visualization and a point message for navigation

### 6. Debugging Visualization
The system provides comprehensive debugging capabilities:
- Overlay of LiDAR points on lane segmentation image
- Color-coded points to distinguish lane vs. non-lane points
- Multiple resolution outputs (640x480 and 1280x720)
- Markers in RViz for lane boundaries and center line

## Usage

### Launch Files
- `lane_detection_fusion.launch`: Launches the lane detection fusion node
- `integration.launch`: Launches both object detection and lane detection nodes with RViz visualization

### Parameters
- `image_width`, `image_height`: Camera resolution (default: 640x480)
- `fov`: Camera field of view (default: 90 degrees)
- `lane_width_limit`: Maximum width of lane from vehicle center (default: 3.7m)
- `lane_cluster_tolerance`: Distance threshold for clustering (default: 0.5m)
- `lane_min_cluster_size`: Minimum points for a valid lane cluster (default: 3)

### Subscribed Topics
- `/points_raw`: LiDAR point cloud (sensor_msgs/PointCloud2)
- `/lane_segmentation`: Lane segmentation image (sensor_msgs/Image)

### Published Topics
- `/lane_points`: Detected lane points (sensor_msgs/PointCloud2)
- `/lane_markers`: Lane boundary markers (visualization_msgs/MarkerArray)
- `/lane_center`: Lane center point (geometry_msgs/PointStamped)
- `/lane_debug_image`: Debug image with overlay (sensor_msgs/Image)
- `/lane_debug_image_large`: High-resolution debug image (sensor_msgs/Image)

## Dependencies
- ROS Noetic
- PCL
- OpenCV
- YOLOPv2 for lane segmentation
- Eigen

## Author
Chaemin Park

## License
This project is licensed under the MIT License - see the LICENSE file for details. 