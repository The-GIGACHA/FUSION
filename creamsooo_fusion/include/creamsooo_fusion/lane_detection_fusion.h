#ifndef LANE_DETECTION_FUSION_H
#define LANE_DETECTION_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// 포인트 클라우드 타입 정의
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 차선 정보를 저장하는 구조체
struct LaneInfo {
    PointCloudT::Ptr cloud;       ///< 차선을 구성하는 포인트 클라우드
    Eigen::Vector4f centroid;     ///< 차선 중심점
    double distance;              ///< 차량으로부터의 거리
    bool is_left;                 ///< 왼쪽 차선인지 여부 (false면 오른쪽)
    double height;                ///< 차선의 높이(z) 값
    
    // 생성자: 멤버 초기화
    LaneInfo() : cloud(new PointCloudT()), distance(0.0), is_left(false), height(0.0) {}
};

class LaneDetectionFusion {
public:
    // 생성자
    LaneDetectionFusion(ros::NodeHandle& nh);
    
    // 소멸자
    ~LaneDetectionFusion();
    
private:
    // ROS 노드 핸들
    ros::NodeHandle nh_;
    
    // 파라미터
    std::string lidar_topic_;
    std::string lane_seg_topic_;  // YOLOPv2 차선 세그멘테이션 토픽
    std::string frame_id_;
    
    // 구독자
    ros::Subscriber lidar_sub_;
    ros::Subscriber lane_seg_sub_;
    
    // 퍼블리셔
    ros::Publisher filtered_ground_pub_;
    ros::Publisher lane_markers_pub_;
    ros::Publisher lane_points_pub_;
    ros::Publisher lane_center_pub_;
    ros::Publisher debug_image_pub_;    // 디버그 이미지 퍼블리셔
    ros::Publisher large_debug_image_pub_;  // 큰 디버그 이미지 퍼블리셔
    
    // 최근 데이터
    PointCloudT::Ptr current_cloud_;
    PointCloudT::Ptr ground_cloud_;
    cv::Mat current_lane_seg_;
    std::vector<LaneInfo> lanes_;  // 차선 정보
    
    // TF 리스너
    tf::TransformListener tf_listener_;

    // 카메라 캘리브레이션 관련 변수
    int image_width_;
    int image_height_;
    double fov_;
    double cam_x_, cam_y_, cam_z_;
    double cam_roll_, cam_pitch_, cam_yaw_;
    double lidar_x_, lidar_y_, lidar_z_;
    double lidar_roll_, lidar_pitch_, lidar_yaw_;
    Eigen::Matrix4d transform_lidar_to_cam_;
    Eigen::Matrix3d camera_matrix_;
    Eigen::Matrix3d original_camera_matrix_;  // 원본 카메라 행렬 저장
    
    // 지면 검출 파라미터
    double ground_height_threshold_;  // 지면으로 간주할 높이 임계값
    double ground_max_angle_;         // 지면 평면 최대 각도 (라디안)
    double ground_z_min_;             // 지면 높이 최소값 (라이다 기준, 미터)
    double ground_z_max_;             // 지면 높이 최대값 (라이다 기준, 미터)
    int ground_max_iterations_;       // RANSAC 최대 반복 횟수
    double ground_min_inlier_ratio_;  // 인라이어 최소 비율
    
    // 차선 검출 파라미터
    double lane_cluster_tolerance_;   // 차선 클러스터링 거리 임계값
    int lane_min_cluster_size_;       // 최소 차선 클러스터 크기
    int lane_max_cluster_size_;       // 최대 차선 클러스터 크기
    double lane_width_;               // 차선 폭
    double lane_width_limit_;         // 중앙으로부터 최대 차선 폭 제한 (미터)
    double lane_min_distance_;        // 차선 최소 거리
    double lane_max_distance_;        // 차선 최대 거리
    
    // 콜백 함수
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void laneSegCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    // 포인트 클라우드 처리 함수
    void processPointCloud();
    
    // 지면 검출 함수
    void extractGroundPoints();
    
    // 차선 검출 함수
    void extractLanePoints();
    void segmentLanes();
    
    // 차선 중앙점 계산 함수
    void calculateLaneCenter();
    
    // 시각화 함수
    void publishGroundPoints();
    void publishLaneMarkers();
    void publishLaneCenter();
    
    // 라이다-카메라 캘리브레이션 관련 함수
    void loadCalibrationParams();
    Eigen::Matrix3d getRotationMatrix(double roll, double pitch, double yaw);
    void calculateTransformMatrix();
    void calculateCameraMatrix();
    Eigen::Vector3d transformLidarToCamera(const Eigen::Vector4d& pc_lidar);
    cv::Point2i transformCameraToImage(const Eigen::Vector3d& pc_camera, bool use_original_size = false);
    
    // 3D -> 2D 투영 함수
    bool isPointInSegmentation(const PointT& point, const cv::Mat& seg_image);

    /**
     * @brief 최적의 좌/우 차선을 선택하는 함수
     */
    void selectBestLanes();
};

// Creator: Chaemin Park

#endif // LANE_DETECTION_FUSION_H 