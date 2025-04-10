#ifndef CREAMSOOO_FUSION_H
#define CREAMSOOO_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ultralytics_ros/YoloResult.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <creamsooo_fusion/ObjectDistance.h>

// 포인트 클라우드 타입 정의
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 클러스터 정보를 저장하는 구조체
struct ClusterInfo {
    PointCloudT::Ptr cloud;
    Eigen::Vector4f centroid;
    double distance;
    int id;
    std::string class_name;
    bool matched;  // 바운딩 박스와 매칭 여부
    int detection_id;  // 매칭된 감지 ID
    
    // 속도 계산을 위한 필드
    Eigen::Vector4f prev_centroid;
    ros::Time last_update_time;
    Eigen::Vector3f velocity;  // x, y, z 방향 속도
    double relative_speed;     // 상대 속도 (m/s)
    bool has_velocity;         // 속도 정보 유무
    
    // 트래킹을 위한 변수 추가
    int tracking_id;           // 트래킹 ID (프레임 간 유지)
    int lost_frames;           // 연속으로 감지되지 않은 프레임 수
    int age;                   // 트래킹된 프레임 수 (수명)
    
    ClusterInfo() : cloud(new PointCloudT()), 
                   matched(false), 
                   detection_id(-1), 
                   class_name("unknown"),
                   has_velocity(false),
                   relative_speed(0.0),
                   tracking_id(-1),     // 초기값 -1 (할당되지 않음)
                   lost_frames(0),      // 초기값 0
                   age(0) {}            // 초기값 0 (수명)
};

class CreamsoooFusion {
public:
    // 생성자
    CreamsoooFusion(ros::NodeHandle& nh);
    
    // 소멸자
    ~CreamsoooFusion();
    
private:
    // ROS 노드 핸들
    ros::NodeHandle nh_;
    
    // 파라미터
    std::string lidar_topic_;
    std::string yolo_result_topic_;
    std::string frame_id_;
    double emergency_distance_;
    int fusion_mode_;  // 0: 기존 방식, 1: 클러스터링 방식, 2: 두 방식 모두
    
    // 구독자
    ros::Subscriber lidar_sub_;
    ros::Subscriber yolo_result_sub_;
    
    // 퍼블리셔
    ros::Publisher filtered_points_pub_;
    ros::Publisher object_markers_pub_;
    ros::Publisher object_distance_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher cluster_markers_pub_;  // 클러스터링 결과 마커
    ros::Publisher iou_fusion_markers_pub_;  // IOU 퓨전 결과 마커
    
    // 최근 데이터
    PointCloudT::Ptr current_cloud_;
    ultralytics_ros::YoloResult current_detections_;
    std::vector<ClusterInfo> clusters_;  // 라이다 클러스터 정보
    
    // TF 리스너
    tf::TransformListener tf_listener_;

    // 카메라 캘리브레이션 관련 변수
    int image_width_;                  // 이미지 너비
    int image_height_;                 // 이미지 높이
    double fov_;                       // 시야각 (Field of View)
    double cam_x_, cam_y_, cam_z_;     // 카메라 위치
    double cam_roll_, cam_pitch_, cam_yaw_; // 카메라 자세 (RPY)
    double lidar_x_, lidar_y_, lidar_z_; // 라이다 위치
    double lidar_roll_, lidar_pitch_, lidar_yaw_; // 라이다 자세 (RPY)
    Eigen::Matrix4d transform_lidar_to_cam_; // 라이다에서 카메라로의 변환 행렬
    Eigen::Matrix3d camera_matrix_;    // 카메라 내부 파라미터 행렬
    
    // 클러스터링 파라미터
    double cluster_tolerance_;  // 클러스터링 거리 임계값
    int min_cluster_size_;      // 최소 클러스터 크기
    int max_cluster_size_;      // 최대 클러스터 크기
    
    // 트래킹을 위한 변수
    std::vector<ClusterInfo> tracked_clusters_;  // 이전 프레임의 트래킹된 클러스터
    int next_tracking_id_;                       // 다음에 할당할 트래킹 ID
    ros::Duration max_tracking_duration_;        // 클러스터 트래킹 최대 시간
    int max_lost_frames_;                        // 최대 연속 손실 프레임 수
    
    // 콜백 함수
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void yoloResultCallback(const ultralytics_ros::YoloResult::ConstPtr& msg);
    
    // 포인트 클라우드 처리 함수
    void processPointCloud();
    
    // 기존 퓨전 방식 함수
    void processTraditionalFusion();
    void processTraditionalFusionWithoutPublishing();  // 마커 발행 없이 처리하는 함수
    PointCloudT::Ptr filterPointsInBoundingBox(const vision_msgs::Detection2D& detection);
    
    // 새로운 클러스터링 기반 퓨전 함수
    void performClustering();
    void updateClusterVelocity(ClusterInfo& current_cluster, const std::vector<ClusterInfo>& prev_clusters);
    double calculateIoU(const vision_msgs::Detection2D& detection, const ClusterInfo& cluster);
    void matchClustersToDetections();
    
    // 시각화 함수
    void publishObjectMarkers();
    void publishClusterMarkers();
    void publishIoUFusionMarkers();
    
    // 거리 계산 함수
    double calculateDistance(const pcl::PointCloud<PointT>::Ptr& cluster);
    
    // 긴급 정지 함수
    void checkEmergencyStop(const std::vector<creamsooo_fusion::ObjectDistance>& objects);

    // 라이다-카메라 캘리브레이션 관련 함수
    void loadCalibrationParams();
    Eigen::Matrix3d getRotationMatrix(double roll, double pitch, double yaw);
    void calculateTransformMatrix();
    void calculateCameraMatrix();
    Eigen::Vector3d transformLidarToCamera(const Eigen::Vector4d& pc_lidar);
    cv::Point2i transformCameraToImage(const Eigen::Vector3d& pc_camera);
    
    // 버전 2D -> 3D 투영 함수
    cv::Rect computeProjectedRect(const ClusterInfo& cluster);
    
    // 클러스터 트래킹 함수 추가
    void updateClusterTracking();
    void updateClusterPersistence();
    int findMatchingTrackedCluster(const ClusterInfo& cluster);
};

#endif // CREAMSOOO_FUSION_H 