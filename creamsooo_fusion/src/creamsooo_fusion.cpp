#include "creamsooo_fusion/creamsooo_fusion.h"
#include <iomanip>  // setprecision, fixed
#include <sstream>  // stringstream
#include <pcl/common/common.h>  // getMinMax3D 함수 포함

CreamsoooFusion::CreamsoooFusion(ros::NodeHandle& nh) : nh_(nh) {
    // 파라미터 로드
    nh_.param<std::string>("lidar_topic", lidar_topic_, "/velodyne_points");
    nh_.param<std::string>("yolo_result_topic", yolo_result_topic_, "/yolo_result");
    nh_.param<std::string>("frame_id", frame_id_, "ego_car");
    nh_.param<double>("emergency_distance", emergency_distance_, 1.0);
    nh_.param<int>("fusion_mode", fusion_mode_, 2);  // 기본값은 두 방식 모두 사용
    
    // 클러스터링 파라미터 초기화
    nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5);
    nh_.param<int>("min_cluster_size", min_cluster_size_, 10);
    nh_.param<int>("max_cluster_size", max_cluster_size_, 2000);
    
    // 트래킹 파라미터 초기화
    next_tracking_id_ = 0;
    max_tracking_duration_ = ros::Duration(3.0);  // 최대 3초 동안 트래킹
    max_lost_frames_ = 10;  // 10프레임 이상 사라지면 트래킹 종료
    
    // 초기화
    current_cloud_.reset(new PointCloudT());
    
    // 캘리브레이션 파라미터 로드
    loadCalibrationParams();
    
    // 변환 행렬 계산
    calculateTransformMatrix();
    calculateCameraMatrix();
    
    // 구독자 설정
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &CreamsoooFusion::lidarCallback, this);
    yolo_result_sub_ = nh_.subscribe(yolo_result_topic_, 1, &CreamsoooFusion::yoloResultCallback, this);
    
    // 퍼블리셔 설정
    filtered_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
    object_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_markers", 1);
    object_distance_pub_ = nh_.advertise<creamsooo_fusion::ObjectDistance>("/object_distance", 1);
    emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/e_stop", 1);
    cluster_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 1);
    iou_fusion_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/iou_fusion_markers", 1);
    
    ROS_INFO("CreamsoooFusion initialized with fusion mode: %d", fusion_mode_);
}

CreamsoooFusion::~CreamsoooFusion() {
    ROS_INFO("CreamsoooFusion destroyed");
}

void CreamsoooFusion::loadCalibrationParams() {
    // 이미지 파라미터
    nh_.param<int>("/camera/width", image_width_, 640);
    nh_.param<int>("/camera/height", image_height_, 480);
    nh_.param<double>("/camera/fov", fov_, 90.0);
    
    // 카메라 위치/자세 파라미터
    nh_.param<double>("/camera/x", cam_x_, 0.61);
    nh_.param<double>("/camera/y", cam_y_, 0.0);
    nh_.param<double>("/camera/z", cam_z_, 1.15);
    nh_.param<double>("/camera/roll", cam_roll_, 0.0);
    nh_.param<double>("/camera/pitch", cam_pitch_, 0.0);
    nh_.param<double>("/camera/yaw", cam_yaw_, 0.0);
    
    // 라이다 위치/자세 파라미터
    nh_.param<double>("/lidar/x", lidar_x_, 0.36);
    nh_.param<double>("/lidar/y", lidar_y_, 0.0);
    nh_.param<double>("/lidar/z", lidar_z_, 0.65);
    nh_.param<double>("/lidar/roll", lidar_roll_, 0.0);
    nh_.param<double>("/lidar/pitch", lidar_pitch_, 0.0);
    nh_.param<double>("/lidar/yaw", lidar_yaw_, 0.0);
    
    ROS_INFO("Calibration parameters loaded");
}

Eigen::Matrix3d CreamsoooFusion::getRotationMatrix(double roll, double pitch, double yaw) {
    // 각 각도의 코사인, 사인 값 계산
    double cos_roll = cos(roll);
    double cos_pitch = cos(pitch);
    double cos_yaw = cos(yaw);
    double sin_roll = sin(roll);
    double sin_pitch = sin(pitch);
    double sin_yaw = sin(yaw);
    
    // 각 축별 회전 행렬 생성
    Eigen::Matrix3d rot_roll;
    rot_roll << 1, 0, 0,
                0, cos_roll, -sin_roll,
                0, sin_roll, cos_roll;
    
    Eigen::Matrix3d rot_pitch;
    rot_pitch << cos_pitch, 0, sin_pitch,
                 0, 1, 0,
                 -sin_pitch, 0, cos_pitch;
    
    Eigen::Matrix3d rot_yaw;
    rot_yaw << cos_yaw, -sin_yaw, 0,
               sin_yaw, cos_yaw, 0,
               0, 0, 1;
    
    // 전체 회전 행렬 = Yaw * Pitch * Roll (행렬 곱셈 순서 중요)
    return rot_yaw * rot_pitch * rot_roll;
}

void CreamsoooFusion::calculateTransformMatrix() {
    // 카메라 자세에 추가 회전 적용 (카메라 좌표계 변환을 위한 보정)
    double adjusted_cam_roll = cam_roll_ - M_PI/2;  // -90도
    double adjusted_cam_pitch = cam_pitch_;
    double adjusted_cam_yaw = cam_yaw_ - M_PI/2;    // -90도
    
    // 회전 행렬 계산
    Eigen::Matrix3d cam_rot = getRotationMatrix(adjusted_cam_roll, adjusted_cam_pitch, adjusted_cam_yaw);
    Eigen::Matrix3d lidar_rot = getRotationMatrix(lidar_roll_, lidar_pitch_, lidar_yaw_);
    
    // 변환 행렬 생성
    Eigen::Matrix4d Tr_cam_to_vehicle = Eigen::Matrix4d::Identity();
    Tr_cam_to_vehicle.block<3, 3>(0, 0) = cam_rot;
    Tr_cam_to_vehicle(0, 3) = cam_x_;
    Tr_cam_to_vehicle(1, 3) = cam_y_;
    Tr_cam_to_vehicle(2, 3) = cam_z_;
    
    Eigen::Matrix4d Tr_lidar_to_vehicle = Eigen::Matrix4d::Identity();
    Tr_lidar_to_vehicle.block<3, 3>(0, 0) = lidar_rot;
    Tr_lidar_to_vehicle(0, 3) = lidar_x_;
    Tr_lidar_to_vehicle(1, 3) = lidar_y_;
    Tr_lidar_to_vehicle(2, 3) = lidar_z_;
    
    // 라이다-카메라 간 변환 행렬 계산: (카메라-차량)^-1 * (라이다-차량)
    transform_lidar_to_cam_ = Tr_cam_to_vehicle.inverse() * Tr_lidar_to_vehicle;
    
    ROS_INFO_STREAM("Transform matrix from LiDAR to camera: \n" << transform_lidar_to_cam_);
}

void CreamsoooFusion::calculateCameraMatrix() {
    // 초점 거리(focal length) 계산: width / (2 * tan(FOV/2))
    double focal_length = image_width_ / (2 * tan(fov_ * M_PI / 360.0));  // fov를 라디안으로 변환
    
    // 주점(principal point) 좌표 - 일반적으로 이미지 중심
    double cx = image_width_ / 2.0;
    double cy = image_height_ / 2.0;
    
    // 카메라 내부 파라미터 행렬 생성
    camera_matrix_ = Eigen::Matrix3d::Identity();
    camera_matrix_(0, 0) = focal_length;  // fx
    camera_matrix_(1, 1) = focal_length;  // fy
    camera_matrix_(0, 2) = cx;            // cx
    camera_matrix_(1, 2) = cy;            // cy
    
    ROS_INFO_STREAM("Camera intrinsic matrix: \n" << camera_matrix_);
}

Eigen::Vector3d CreamsoooFusion::transformLidarToCamera(const Eigen::Vector4d& pc_lidar) {
    // 변환 행렬을 이용해 라이다 좌표를 카메라 좌표로 변환
    Eigen::Vector4d pc_cam_homogeneous = transform_lidar_to_cam_ * pc_lidar;
    
    // 동차 좌표를 3D 좌표로 변환
    return pc_cam_homogeneous.head<3>();
}

cv::Point2i CreamsoooFusion::transformCameraToImage(const Eigen::Vector3d& pc_camera) {
    // 카메라 뒤에 있는 포인트는 무시
    if (pc_camera(2) <= 0) {
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    // 카메라 내부 파라미터 행렬을 이용해 3D->2D 변환
    Eigen::Vector3d pc_image = camera_matrix_ * pc_camera;
    
    // 동차화 (z로 나누기)
    int u = static_cast<int>(pc_image(0) / pc_image(2));
    int v = static_cast<int>(pc_image(1) / pc_image(2));
    
    // 이미지 영역을 벗어난 포인트 무시
    if (u < 0 || u >= image_width_ || v < 0 || v >= image_height_) {
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    return cv::Point2i(u, v);
}

void CreamsoooFusion::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 포인트 클라우드 변환
    pcl::fromROSMsg(*msg, *current_cloud_);
    
    // 데이터가 있으면 처리
    if (!current_detections_.detections.detections.empty() && current_cloud_->size() > 0) {
        processPointCloud();
    }
}

void CreamsoooFusion::yoloResultCallback(const ultralytics_ros::YoloResult::ConstPtr& msg) {
    // YOLO 결과 저장
    current_detections_ = *msg;
    
    // 데이터가 있으면 처리
    if (!current_detections_.detections.detections.empty() && current_cloud_->size() > 0) {
        processPointCloud();
    }
}

void CreamsoooFusion::performClustering() {
    // 유지해야 할 트래킹된 클러스터 업데이트
    updateClusterPersistence();
    
    // 클러스터 벡터 초기화 (트래킹을 위한 이전 클러스터 정보는 별도로 저장됨)
    clusters_.clear();
    
    // 지면 제거 및 ROI 필터링을 위한 PassThrough 필터
    PointCloudT::Ptr filtered_cloud(new PointCloudT());
    
    // 전방 영역 필터링
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(current_cloud_);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, 50.0);  // 전방 50m 이내로 확장
    pass_x.filter(*filtered_cloud);
    
    // 지면 필터링 개선: 더 정확한 지면 제거를 위한 높이 범위 조정
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(filtered_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-0.3, 3.0);  // 지면 위 3.0m 이내, 지면 아래 0.3m 이내로 조정
    pass_z.filter(*filtered_cloud);
    
    // 측면 영역 제한 (차량에서 너무 멀리 떨어진 측면 포인트 제거)
    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(filtered_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-10.0, 10.0);  // 좌우 10m 이내로 제한
    pass_y.filter(*filtered_cloud);
    
    // 다운샘플링 - 포인트 수를 줄여 처리 속도 향상
    PointCloudT::Ptr downsampled_cloud(new PointCloudT());
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(filtered_cloud);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);  // 10cm 간격으로 다운샘플링
    voxel_grid.filter(*downsampled_cloud);
    
    // 실제 데이터 확인용 로그
    ROS_INFO("Filtered cloud size: %zu points, Downsampled size: %zu points", 
             filtered_cloud->size(), downsampled_cloud->size());
    
    // Euclidean 클러스터링 - 객체 분할
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(downsampled_cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    // 클러스터링 파라미터 조정 - 사람/차량 크기에 맞게 조정
    ec.setClusterTolerance(cluster_tolerance_);  // 점 사이 최대 거리 (기본값 0.5m)
    ec.setMinClusterSize(min_cluster_size_);     // 최소 클러스터 크기 (기본값 10)
    ec.setMaxClusterSize(max_cluster_size_);     // 최대 클러스터 크기 (기본값 2000)
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_cloud);
    ec.extract(cluster_indices);
    
    // 클러스터 추출 및 후처리: 크기, 밀도 등을 기준으로 필터링
    int valid_cluster_count = 0;
    int merged_cluster_count = 0;
    
    // 초기 분할된 클러스터 저장 (병합 전)
    std::vector<ClusterInfo> initial_clusters;
    
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        // 최소 포인트 수 검증 (노이즈 제거)
        if (cluster_indices[i].indices.size() < min_cluster_size_) {
            continue;
        }
        
        ClusterInfo cluster_info;
        cluster_info.cloud.reset(new PointCloudT());
        cluster_info.id = valid_cluster_count++;
        cluster_info.matched = false;
        cluster_info.detection_id = -1;
        cluster_info.class_name = "unknown";
        
        // 클러스터에 포함된 포인트 추출
        for (const auto& idx : cluster_indices[i].indices) {
            cluster_info.cloud->push_back((*downsampled_cloud)[idx]);
        }
        
        // 중심점 계산
        pcl::compute3DCentroid(*(cluster_info.cloud), cluster_info.centroid);
        
        // 거리 계산
        cluster_info.distance = sqrt(
            cluster_info.centroid[0] * cluster_info.centroid[0] + 
            cluster_info.centroid[1] * cluster_info.centroid[1] + 
            cluster_info.centroid[2] * cluster_info.centroid[2]
        );
        
        // 위치 기반 필터링: 너무 높거나 낮은 위치의 클러스터 제외
        if (cluster_info.centroid[2] < -0.2 || cluster_info.centroid[2] > 2.5) {
            continue;
        }
        
        // 초기 클러스터 추가
        initial_clusters.push_back(cluster_info);
    }
    
    // 유사한 클러스터 병합 (같은 객체의 일부인데 나뉘어진 클러스터 병합)
    if (initial_clusters.size() > 1) {
        std::vector<bool> merged(initial_clusters.size(), false);
        
        for (size_t i = 0; i < initial_clusters.size(); i++) {
            if (merged[i]) continue;
            
            ClusterInfo merged_cluster = initial_clusters[i];
            bool did_merge = false;
            
            for (size_t j = i + 1; j < initial_clusters.size(); j++) {
                if (merged[j]) continue;
                
                // 두 클러스터 간 거리 계산
                float dist_x = merged_cluster.centroid[0] - initial_clusters[j].centroid[0];
                float dist_y = merged_cluster.centroid[1] - initial_clusters[j].centroid[1];
                float dist_z = merged_cluster.centroid[2] - initial_clusters[j].centroid[2];
                float distance = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
                
                // 클러스터가 충분히 가까우면 병합 (거리 임계값 1.5m)
                if (distance < 1.5) {
                    // 클러스터 포인트 병합
                    *(merged_cluster.cloud) += *(initial_clusters[j].cloud);
                    
                    // 병합된 클러스터의 중심점 재계산
                    pcl::compute3DCentroid(*(merged_cluster.cloud), merged_cluster.centroid);
                    
                    // 거리 재계산
                    merged_cluster.distance = sqrt(
                        merged_cluster.centroid[0] * merged_cluster.centroid[0] + 
                        merged_cluster.centroid[1] * merged_cluster.centroid[1] + 
                        merged_cluster.centroid[2] * merged_cluster.centroid[2]
                    );
                    
                    merged[j] = true;
                    did_merge = true;
                }
            }
            
            // 병합된 클러스터 또는 단일 클러스터 추가
            if (did_merge) {
                merged_cluster.id = merged_cluster_count++;
                clusters_.push_back(merged_cluster);
            } else {
                clusters_.push_back(initial_clusters[i]);
            }
        }
    } else {
        // 병합 필요 없을 경우 초기 클러스터 그대로 사용
        clusters_ = initial_clusters;
    }
    
    // 클러스터 크기 기반 물체 유형 추정 (사전 분류)
    for (auto& cluster : clusters_) {
        // 클러스터 최소/최대 바운딩 박스 계산
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*(cluster.cloud), min_pt, max_pt);
        
        // 바운딩 박스 크기 계산
        float width = max_pt[0] - min_pt[0];   // x
        float height = max_pt[2] - min_pt[2];  // z
        float depth = max_pt[1] - min_pt[1];   // y
        
        // 크기 기반 객체 유형 추정
        if (height > 1.5 && width < 1.0 && depth < 1.0) {
            // 높이가 높고 폭이 좁으면 사람으로 추정
            cluster.class_name = "person_estimate";
        } else if (width > 1.5 || depth > 1.5) {
            // 폭이나 깊이가 크면 차량으로 추정
            cluster.class_name = "vehicle_estimate";
        }
        
        // 이전 프레임의 클러스터와 매칭하여 속도 계산
        updateClusterVelocity(cluster, tracked_clusters_);
    }
    
    // 클러스터 트래킹 업데이트
    updateClusterTracking();
    
    ROS_INFO("Found %zu clusters after merging and filtering", clusters_.size());
}

void CreamsoooFusion::updateClusterPersistence() {
    // 트래킹된 클러스터 중 일정 시간 이상 보이지 않은 것은 제거
    std::vector<ClusterInfo> retained_clusters;
    
    for (auto& tracked_cluster : tracked_clusters_) {
        // lost_frames 증가 (다음 프레임에서 매칭되면 다시 0으로 리셋됨)
        tracked_cluster.lost_frames++;
        
        // 연속으로 감지되지 않은 프레임 수가 임계값 미만이면 유지
        if (tracked_cluster.lost_frames < max_lost_frames_) {
            retained_clusters.push_back(tracked_cluster);
            ROS_DEBUG("Keeping tracked cluster ID %d, age %d, lost_frames %d", 
                     tracked_cluster.tracking_id, tracked_cluster.age, tracked_cluster.lost_frames);
        } else {
            ROS_DEBUG("Removing tracked cluster ID %d, age %d after %d consecutive lost frames", 
                     tracked_cluster.tracking_id, tracked_cluster.age, tracked_cluster.lost_frames);
        }
    }
    
    tracked_clusters_ = retained_clusters;
}

int CreamsoooFusion::findMatchingTrackedCluster(const ClusterInfo& cluster) {
    // 현재 클러스터와 가장 가까운 이전 트래킹 클러스터 찾기
    double min_distance = 2.0;  // 같은 객체로 볼 최대 거리 (m)
    int matched_idx = -1;
    
    for (size_t i = 0; i < tracked_clusters_.size(); i++) {
        // 이미 매칭된 클러스터는 스킵
        if (tracked_clusters_[i].matched) {
            continue;
        }
        
        // 중심점 간 거리 계산
        double dx = cluster.centroid[0] - tracked_clusters_[i].centroid[0];
        double dy = cluster.centroid[1] - tracked_clusters_[i].centroid[1];
        double dz = cluster.centroid[2] - tracked_clusters_[i].centroid[2];
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        // 거리가 임계값보다 작고 현재까지의 최소 거리보다 작으면 업데이트
        if (distance < min_distance) {
            min_distance = distance;
            matched_idx = i;
        }
    }
    
    return matched_idx;
}

void CreamsoooFusion::updateClusterTracking() {
    // 현재 클러스터와 이전에 트래킹된 클러스터 매칭
    for (auto& cluster : clusters_) {
        // 트래킹된 클러스터와 매칭
        int matched_idx = findMatchingTrackedCluster(cluster);
        
        if (matched_idx >= 0) {
            // 매칭된 트래킹 클러스터 찾음
            ClusterInfo& tracked_cluster = tracked_clusters_[matched_idx];
            
            // 트래킹 ID 유지 및 정보 업데이트
            cluster.tracking_id = tracked_cluster.tracking_id;
            cluster.lost_frames = 0;  // 현재 프레임에서 찾았으므로 0으로 리셋
            
            // 수명(age) 정보 복사 및 증가
            cluster.age = tracked_cluster.age + 1;
            
            // 트래킹 클러스터 매칭 표시 (중복 매칭 방지)
            tracked_cluster.matched = true;
            
            // 클러스터 속성 업데이트 (매칭 정보 유지)
            if (cluster.matched && !tracked_cluster.matched) {
                // 현재 클러스터가 YOLO 객체와 매칭되었으나 이전 트래킹은 매칭이 없었을 경우
                tracked_cluster.matched = true;
                tracked_cluster.detection_id = cluster.detection_id;
                tracked_cluster.class_name = cluster.class_name;
            } else if (!cluster.matched && tracked_cluster.matched) {
                // 이전 트래킹에 YOLO 매칭 정보가 있으나 현재 클러스터는 매칭이 없을 경우
                cluster.matched = true;
                cluster.detection_id = tracked_cluster.detection_id;
                cluster.class_name = tracked_cluster.class_name;
            }
            
            ROS_DEBUG("Cluster matched with tracking ID %d, age %d", cluster.tracking_id, cluster.age);
        } else {
            // 매칭된 트래킹 클러스터 없음 - 새 트래킹 ID 할당
            cluster.tracking_id = next_tracking_id_++;
            cluster.lost_frames = 0;
            cluster.age = 1;  // 새로운 클러스터는 수명 1부터 시작
            
            ROS_DEBUG("New tracking ID %d assigned to cluster", cluster.tracking_id);
        }
    }
    
    // 트래킹된 클러스터 목록 업데이트 (현재 프레임의 모든 클러스터 추가)
    // 기존 트래킹된 클러스터 중 매칭되지 않은 것들은 tracked_clusters_에 유지됨
    for (const auto& cluster : clusters_) {
        bool found = false;
        for (auto& tracked : tracked_clusters_) {
            if (tracked.tracking_id == cluster.tracking_id) {
                // 이미 트래킹 목록에 있는 경우, 정보 업데이트
                tracked.cloud = cluster.cloud;
                tracked.centroid = cluster.centroid;
                tracked.distance = cluster.distance;
                tracked.matched = cluster.matched;
                tracked.detection_id = cluster.detection_id;
                tracked.class_name = cluster.class_name;
                tracked.has_velocity = cluster.has_velocity;
                tracked.velocity = cluster.velocity;
                tracked.relative_speed = cluster.relative_speed;
                tracked.last_update_time = ros::Time::now();
                tracked.lost_frames = 0;  // 현재 프레임에서 매칭됨
                tracked.age = cluster.age;  // 수명 정보 업데이트
                found = true;
                break;
            }
        }
        
        // 새 클러스터인 경우 트래킹 목록에 추가
        if (!found) {
            ClusterInfo tracked_cluster = cluster;
            tracked_cluster.last_update_time = ros::Time::now();
            tracked_clusters_.push_back(tracked_cluster);
        }
    }
    
    // 다음 프레임에서 새로운 매칭을 위해 매칭 플래그 초기화
    for (auto& tracked : tracked_clusters_) {
        tracked.matched = false;
    }
}

void CreamsoooFusion::updateClusterVelocity(ClusterInfo& current_cluster, const std::vector<ClusterInfo>& prev_clusters) {
    // 현재 시간 기록
    ros::Time current_time = ros::Time::now();
    
    // 초기화: 속도 정보 없음으로 설정
    current_cluster.has_velocity = false;
    current_cluster.velocity = Eigen::Vector3f::Zero();
    current_cluster.relative_speed = 0.0;
    
    // 현재 클러스터의 위치
    const Eigen::Vector4f& current_pos = current_cluster.centroid;
    double min_distance = 2.0;  // 같은 객체로 판단할 최대 거리 (1.5m에서 2.0m로 증가)
    int matched_idx = -1;
    
    // 이전 프레임의 클러스터 중 가장 가까운 클러스터 찾기
    for (size_t i = 0; i < prev_clusters.size(); i++) {
        const Eigen::Vector4f& prev_pos = prev_clusters[i].centroid;
        
        // 유클리드 거리 계산
        double dx = current_pos[0] - prev_pos[0];
        double dy = current_pos[1] - prev_pos[1];
        double dz = current_pos[2] - prev_pos[2];
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        // 거리가 임계값보다 작으면 같은 객체로 간주
        if (distance < min_distance) {
            min_distance = distance;
            matched_idx = i;
        }
    }
    
    // 매칭된 이전 클러스터가 있으면 속도 계산
    if (matched_idx >= 0) {
        const ClusterInfo& prev_cluster = prev_clusters[matched_idx];
        const Eigen::Vector4f& prev_pos = prev_cluster.centroid;
        
        // 시간 차이 계산 (초 단위)
        double dt = (current_time - prev_cluster.last_update_time).toSec();
        
        // 시간 차이가 너무 작으면 계산 안함 (노이즈 방지)
        if (dt > 0.005) {  // 0.01초에서 0.005초로 임계값 감소
            // 3D 속도 벡터 계산 (x, y, z 방향)
            current_cluster.velocity[0] = (current_pos[0] - prev_pos[0]) / dt;  // x 방향 속도
            current_cluster.velocity[1] = (current_pos[1] - prev_pos[1]) / dt;  // y 방향 속도
            current_cluster.velocity[2] = (current_pos[2] - prev_pos[2]) / dt;  // z 방향 속도
            
            // 상대 속도 계산 (ego 차량 기준 radial 속도)
            // 거리 차이를 시간으로 나누어 계산
            double prev_distance = sqrt(prev_pos[0] * prev_pos[0] + 
                                       prev_pos[1] * prev_pos[1] + 
                                       prev_pos[2] * prev_pos[2]);
            
            double current_distance = current_cluster.distance;
            current_cluster.relative_speed = (current_distance - prev_distance) / dt;
            
            // 속도 정보가 유효함을 표시
            current_cluster.has_velocity = true;
            
            // 이전 위치 저장
            current_cluster.prev_centroid = prev_pos;
            
            // 속도가 비정상적으로 크면 필터링 (노이즈 방지)
            if (fabs(current_cluster.relative_speed) > 15.0) {  // 10.0에서 15.0으로 임계값 증가
                current_cluster.has_velocity = false;
                current_cluster.relative_speed = 0.0;
            }
        }
    }
    
    // 매칭된 이전 클러스터가 없거나 속도를 계산할 수 없는 경우, 기본 정보 설정
    if (!current_cluster.has_velocity) {
        current_cluster.relative_speed = 0.0;
        current_cluster.has_velocity = true; // 속도가 0이라도 표시하기 위해 true로 설정
    }
    
    // 현재 시간 저장
    current_cluster.last_update_time = current_time;
}

cv::Rect CreamsoooFusion::computeProjectedRect(const ClusterInfo& cluster) {
    // 클러스터의 모든 포인트를 이미지 평면에 투영하여 바운딩 박스 계산
    std::vector<cv::Point2i> image_points;
    
    for (const auto& point : cluster.cloud->points) {
        // 라이다 포인트를 동차 좌표로 변환
        Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
        
        // 라이다 좌표를 카메라 좌표로 변환
        Eigen::Vector3d camera_point = transformLidarToCamera(lidar_point);
        
        // 카메라 뒤의 포인트는 무시
        if (camera_point(2) <= 0) {
            continue;
        }
        
        // 카메라 3D 좌표를 이미지 2D 픽셀 좌표로 변환
        cv::Point2i image_point = transformCameraToImage(camera_point);
        
        // 유효한 포인트만 추가
        if (image_point.x >= 0 && image_point.y >= 0) {
            image_points.push_back(image_point);
        }
    }
    
    // 투영된 포인트가 없는 경우
    if (image_points.empty()) {
        return cv::Rect(0, 0, 0, 0);
    }
    
    // 투영된 포인트들의 경계 계산
    int min_x = image_width_;
    int min_y = image_height_;
    int max_x = 0;
    int max_y = 0;
    
    for (const auto& point : image_points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }
    
    // 바운딩 박스가 이미지 영역을 벗어나지 않도록 조정
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    max_x = std::min(image_width_ - 1, max_x);
    max_y = std::min(image_height_ - 1, max_y);
    
    // 바운딩 박스 생성
    return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

double CreamsoooFusion::calculateIoU(const vision_msgs::Detection2D& detection, const ClusterInfo& cluster) {
    // YOLO 바운딩 박스 좌표 계산
    int det_xmin = static_cast<int>(detection.bbox.center.x - detection.bbox.size_x/2);
    int det_ymin = static_cast<int>(detection.bbox.center.y - detection.bbox.size_y/2);
    int det_xmax = static_cast<int>(detection.bbox.center.x + detection.bbox.size_x/2);
    int det_ymax = static_cast<int>(detection.bbox.center.y + detection.bbox.size_y/2);
    
    cv::Rect detection_rect(det_xmin, det_ymin, det_xmax - det_xmin, det_ymax - det_ymin);
    
    // 클러스터를 이미지 평면에 투영하여 바운딩 박스 계산
    cv::Rect cluster_rect = computeProjectedRect(cluster);
    
    // 바운딩 박스가 유효하지 않은 경우
    if (cluster_rect.width <= 0 || cluster_rect.height <= 0) {
        return 0.0;
    }
    
    // 교집합 영역 계산
    cv::Rect intersection = detection_rect & cluster_rect;
    
    // 교집합이 없는 경우
    if (intersection.width <= 0 || intersection.height <= 0) {
        return 0.0;
    }
    
    // IoU 계산: 교집합 / 합집합
    double intersection_area = intersection.width * intersection.height;
    double detection_area = detection_rect.width * detection_rect.height;
    double cluster_area = cluster_rect.width * cluster_rect.height;
    double union_area = detection_area + cluster_area - intersection_area;
    
    // 합집합이 0인 경우 예외 처리
    if (union_area <= 0) {
        return 0.0;
    }
    
    return intersection_area / union_area;
}

void CreamsoooFusion::matchClustersToDetections() {
    // 각 클러스터와 감지 결과 간의 IoU 계산 및 매칭
    for (auto& cluster : clusters_) {
        double max_iou = 0.0;
        int best_match_id = -1;
        int best_match_idx = -1;
        std::string best_class_name = "";
        
        for (size_t i = 0; i < current_detections_.detections.detections.size(); i++) {
            const auto& detection = current_detections_.detections.detections[i];
            
            // IoU 계산
            double iou = calculateIoU(detection, cluster);
            
            // 더 높은 IoU를 가진 감지 결과로 업데이트
            if (iou > max_iou && iou > 0.3) {  // IoU 임계값: 0.3
                max_iou = iou;
                best_match_id = i;
                best_match_idx = i;
                
                // 클래스 이름 가져오기
                if (!detection.results.empty()) {
                    best_class_name = detection.results[0].id;
                }
            }
        }
        
        // 매칭 정보 업데이트
        if (best_match_id >= 0) {
            cluster.matched = true;
            cluster.detection_id = best_match_id;
            cluster.class_name = best_class_name;
            ROS_INFO("Cluster %d matched with detection %d (class: %s, IoU: %.2f)",
                    cluster.id, best_match_id, best_class_name.c_str(), max_iou);
        }
    }
}

PointCloudT::Ptr CreamsoooFusion::filterPointsInBoundingBox(const vision_msgs::Detection2D& detection) {
    PointCloudT::Ptr object_cloud(new PointCloudT());
    
    // 바운딩 박스 좌표 추출
    int xmin = static_cast<int>(detection.bbox.center.x - detection.bbox.size_x/2);
    int ymin = static_cast<int>(detection.bbox.center.y - detection.bbox.size_y/2);
    int xmax = static_cast<int>(detection.bbox.center.x + detection.bbox.size_x/2);
    int ymax = static_cast<int>(detection.bbox.center.y + detection.bbox.size_y/2);
    
    // 각 라이다 포인트를 이미지에 투영
    for (const auto& point : current_cloud_->points) {
        // 필터링: 전방 30m 이내 포인트만 처리
        if (point.x < 0 || point.x > 30) {
            continue;
        }
        
        // 라이다 포인트를 동차 좌표로 변환
        Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
        
        // 라이다 좌표를 카메라 좌표로 변환
        Eigen::Vector3d camera_point = transformLidarToCamera(lidar_point);
        
        // 카메라 뒤의 포인트는 무시
        if (camera_point(2) <= 0) {
            continue;
        }
        
        // 카메라 3D 좌표를 이미지 2D 픽셀 좌표로 변환
        cv::Point2i image_point = transformCameraToImage(camera_point);
        
        // 바운딩 박스 내에 있는 포인트인지 확인
        if (image_point.x >= xmin && image_point.x <= xmax && 
            image_point.y >= ymin && image_point.y <= ymax) {
            object_cloud->push_back(point);
        }
    }
    
    return object_cloud;
}

void CreamsoooFusion::processPointCloud() {
    // 퓨전 모드에 따라 처리
    if (fusion_mode_ == 0 || fusion_mode_ == 2) {
        // 기존 퓨전 방식 (바운딩 박스 기반)
        processTraditionalFusion();
    }
    
    if (fusion_mode_ == 1 || fusion_mode_ == 2) {
        // 클러스터링 기반 퓨전 방식
        performClustering();
        matchClustersToDetections();
        publishClusterMarkers();
        publishIoUFusionMarkers();
    }
}

void CreamsoooFusion::processTraditionalFusion() {
    // 결과를 저장할 마커 배열 초기화
    visualization_msgs::MarkerArray markers;
    // 객체 거리 정보를 저장할 벡터
    std::vector<creamsooo_fusion::ObjectDistance> object_distances;
    
    // 각 감지된 객체에 대해 처리
    for (size_t i = 0; i < current_detections_.detections.detections.size(); i++) {
        const auto& detection = current_detections_.detections.detections[i];
        
        // 객체 바운딩 박스 영역 내의 포인트 필터링 (캘리브레이션 기반)
        PointCloudT::Ptr object_cloud = filterPointsInBoundingBox(detection);
        
        // 필터링된 포인트가 없으면 다음 객체로
        if (object_cloud->empty()) {
            continue;
        }
        
        // 포인트 다운샘플링 (선택 사항)
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(object_cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*object_cloud);
        
        // 객체 중심 좌표 계산
        Eigen::Vector4f centroid;
        if (object_cloud->size() > 0) {
            pcl::compute3DCentroid(*object_cloud, centroid);
            
            // 객체까지의 거리 계산
            double distance = sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
            
            // 객체 정보 메시지 생성
            creamsooo_fusion::ObjectDistance obj_dist;
            obj_dist.header.stamp = ros::Time::now();
            obj_dist.header.frame_id = frame_id_;
            obj_dist.id = i;  // detection ID는 vision_msgs에서 제공하지 않으므로 인덱스 사용
            
            // 클래스 이름 가져오기
            std::string class_name = "";
            if (!detection.results.empty()) {
                class_name = detection.results[0].id;
            }
            obj_dist.class_name = class_name;
            obj_dist.distance = distance;
            
            // 중심점 설정
            obj_dist.center.x = centroid[0];
            obj_dist.center.y = centroid[1];
            obj_dist.center.z = centroid[2];
            
            // 긴급 정지 조건 확인
            obj_dist.emergency = (distance < emergency_distance_);
            
            // 객체 거리 정보 추가
            object_distances.push_back(obj_dist);
            
            // 객체 거리 정보 발행
            object_distance_pub_.publish(obj_dist);
            
            // 마커 생성
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "object_" + std::to_string(i);
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 마커 위치 및 크기
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            
            // 거리에 따른 색상 설정 - 더 명확한 색상 변화를 위해 수정
            // 3m 이내: 빨간색, 8m 이상: 녹색, 중간: 노란색 계열
            if (distance < 3.0) {
                // 위험 - 빨간색
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (distance < 8.0) {
                // 주의 - 노란색 계열
                float t = (distance - 3.0) / 5.0;  // 3~8m 사이를 0~1로 정규화
                marker.color.r = 1.0;
                marker.color.g = t * 1.0;
                marker.color.b = 0.0;
            } else {
                // 안전 - 녹색
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 0.7;
            
            marker.lifetime = ros::Duration(0.1);
            
            markers.markers.push_back(marker);
            
            // 텍스트 마커 추가
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "object_text_" + std::to_string(i);
            text_marker.id = i;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            text_marker.pose.position.x = centroid[0];
            text_marker.pose.position.y = centroid[1];
            text_marker.pose.position.z = centroid[2] + 1.0;  // 객체 위에 텍스트 표시
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.5;  // 텍스트 크기
            
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // ID, 클래스, 거리 정보 표시
            std::stringstream ss;
            ss << "ID: " << i ;
            ss << "\nDist: " << std::fixed << std::setprecision(1) << distance << "m";
            
            text_marker.text = ss.str();
            text_marker.lifetime = ros::Duration(0.1);
            
            markers.markers.push_back(text_marker);
            
            // 필터링된 객체 포인트 클라우드 발행
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*object_cloud, cloud_msg);
            cloud_msg.header.frame_id = frame_id_;
            cloud_msg.header.stamp = ros::Time::now();
            filtered_points_pub_.publish(cloud_msg);
        }
    }
    
    // 마커 배열 발행
    if (!markers.markers.empty()) {
        object_markers_pub_.publish(markers);
    }
    
    // 긴급 정지 조건 확인
    checkEmergencyStop(object_distances);
}

void CreamsoooFusion::publishClusterMarkers() {
    visualization_msgs::MarkerArray markers;
    
    for (size_t i = 0; i < clusters_.size(); i++) {
        const auto& cluster = clusters_[i];
        
        // 마커 생성
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        
        // tracking_id를 사용하여 일관된 네임스페이스와 ID 할당
        marker.ns = "cluster_" + std::to_string(cluster.tracking_id);
        marker.id = cluster.tracking_id;  // tracking_id 사용
        
        marker.type = visualization_msgs::Marker::SPHERE;  // 구 형태로 시각화
        marker.action = visualization_msgs::Marker::ADD;
        
        // 마커 위치 및 크기
        marker.pose.position.x = cluster.centroid[0];
        marker.pose.position.y = cluster.centroid[1];
        marker.pose.position.z = cluster.centroid[2];
        marker.pose.orientation.w = 1.0;
        
        // 클러스터 크기에 비례한 마커 크기
        double size_factor = std::min(1.0, std::max(0.5, (double)cluster.cloud->size() / 100.0));
        marker.scale.x = 0.5 * size_factor;
        marker.scale.y = 0.5 * size_factor;
        marker.scale.z = 0.5 * size_factor;
        
        // 거리에 따른 색상 설정
        if (cluster.distance < 3.0) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // 보라색 (근거리)
        } else if (cluster.distance < 8.0) {
            float t = (cluster.distance - 3.0) / 5.0;
            marker.color.r = 1.0 - t;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // 보라색 -> 파란색
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // 파란색 (원거리)
        }
        marker.color.a = 0.7;
        
        // 마커 지속 시간을 늘려 깜빡임 감소
        marker.lifetime = ros::Duration(0.3);
        
        markers.markers.push_back(marker);
        
        // 트래킹 ID 및 수명(age)을 보여주는 텍스트 마커 추가
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cluster_text_" + std::to_string(cluster.tracking_id);
        text_marker.id = cluster.tracking_id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = cluster.centroid[0];
        text_marker.pose.position.y = cluster.centroid[1];
        text_marker.pose.position.z = cluster.centroid[2] + 0.5;  // 객체 위에 텍스트 표시
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;  // 텍스트 크기
        
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // ID 및 수명(age) 표시
        std::stringstream ss;
        ss << "T" << cluster.tracking_id;
        ss << " (Age:" << cluster.age << ")";
        
        text_marker.text = ss.str();
        text_marker.lifetime = ros::Duration(0.3);
        
        markers.markers.push_back(text_marker);
    }
    
    // 마커 배열 발행
    if (!markers.markers.empty()) {
        cluster_markers_pub_.publish(markers);
    }
}

void CreamsoooFusion::publishIoUFusionMarkers() {
    visualization_msgs::MarkerArray markers;
    
    // 매칭된 클러스터만 시각화
    for (const auto& cluster : clusters_) {
        if (cluster.matched) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            
            // tracking_id를 사용하여 일관된 네임스페이스와 ID 할당
            marker.ns = "iou_fusion_" + std::to_string(cluster.tracking_id);
            marker.id = cluster.tracking_id;  // tracking_id 사용
            
            marker.type = visualization_msgs::Marker::CYLINDER;  // 원통 형태로 시각화
            marker.action = visualization_msgs::Marker::ADD;
            
            // 클러스터의 바운딩 박스 크기 계산
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*(cluster.cloud), min_pt, max_pt);
            float width = max_pt[0] - min_pt[0];   // x
            float depth = max_pt[2] - min_pt[2];  // z
            float height = max_pt[1] - min_pt[1];   // y
            
            // 최소 크기 보장
            width = std::max(width, 0.5f);
            depth = std::max(depth, 0.5f);
            height = std::max(height, 1.0f);
            
            // 너무 작은 객체 크기 조정 (시각화를 위해)
            if (cluster.class_name == "person" || cluster.class_name == "person_estimate") {
                width = std::max(width, 0.6f);
                depth = std::max(depth, 0.6f);
                height = std::max(height, 1.7f);
            } else if (cluster.class_name == "car" || cluster.class_name == "truck" || 
                       cluster.class_name == "vehicle_estimate") {
                width = std::max(width, 1.8f);
                depth = std::max(depth, 1.8f);
                height = std::max(height, 1.5f);
            }
            
            // 마커 위치 및 크기
            marker.pose.position.x = cluster.centroid[0];
            marker.pose.position.y = cluster.centroid[1];
            marker.pose.position.z = min_pt[2] + height/2;  // 바닥에 위치하도록 z 위치 조정
            marker.pose.orientation.w = 1.0;
            
            // 원통형 마커의 반지름은 width와 depth의 평균으로 설정
            float radius = (width + depth) / 4.0;
            marker.scale.x = radius * 2;
            marker.scale.y = radius * 2;
            marker.scale.z = height;
            
            // 매칭된 클래스에 따른 색상 설정
            if (cluster.class_name.find("person") != std::string::npos) {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;  // 주황색 (사람)
            } else if (cluster.class_name.find("car") != std::string::npos || 
                      cluster.class_name.find("truck") != std::string::npos ||
                      cluster.class_name.find("vehicle") != std::string::npos) {
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 1.0;  // 하늘색 (차량)
            } else {
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 0.5;  // 자주색 (기타)
            }
            marker.color.a = 0.8;
            
            // 마커 지속 시간을 늘려 깜빡임 감소
            marker.lifetime = ros::Duration(0.3);
            
            markers.markers.push_back(marker);
            
            // 텍스트 마커 추가
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "iou_fusion_text_" + std::to_string(cluster.tracking_id);
            text_marker.id = cluster.tracking_id;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            text_marker.pose.position.x = cluster.centroid[0];
            text_marker.pose.position.y = cluster.centroid[1];
            text_marker.pose.position.z = max_pt[2] + 0.5;  // 객체 위에 텍스트 표시
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.5;  // 텍스트 크기
            
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // 객체 ID, 거리, 크기, 속도, 수명(age) 정보 표시
            std::stringstream ss;
            ss << "ID: " << cluster.detection_id;
            ss << " (Age:" << cluster.age << ")";
            ss << "\nDist: " << std::fixed << std::setprecision(1) << cluster.distance << "m";
            
            // 속도 정보가 있는 경우만 표시
            if (cluster.has_velocity) {
                // 접근하는 경우 음수, 멀어지는 경우 양수로 표시
                ss << "\nVel: " << std::fixed << std::setprecision(1) 
                   << -cluster.relative_speed << " m/s";
                   
                // 접근/멀어짐 표시
                if (cluster.relative_speed < -0.5) {
                    ss << " [APPROACHING]";
                } else if (cluster.relative_speed > 0.5) {
                    ss << " [MOVING AWAY]";
                } else {
                    ss << " [STATIONARY]";
                }
            }
            
            // 크기 정보 추가
            ss << "\nSize: " << std::fixed << std::setprecision(1) 
               << width << "x" << depth << "x" << height << "m";
               
            text_marker.text = ss.str();
            text_marker.lifetime = ros::Duration(0.3);
            
            markers.markers.push_back(text_marker);
        }
    }
    
    // 마커 배열 발행
    if (!markers.markers.empty()) {
        iou_fusion_markers_pub_.publish(markers);
    }
}

double CreamsoooFusion::calculateDistance(const pcl::PointCloud<PointT>::Ptr& cluster) {
    // 중심점 계산
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    // 원점으로부터의 유클리드 거리
    return sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
}

void CreamsoooFusion::publishObjectMarkers() {
    // processPointCloud 메서드에서 처리됨
}

void CreamsoooFusion::checkEmergencyStop(const std::vector<creamsooo_fusion::ObjectDistance>& objects) {
    // 긴급 정지 조건 확인
    bool emergency = false;
    
    for (const auto& obj : objects) {
        if (obj.emergency) {
            emergency = true;
            // 터미널에 눈에 띄는 형태로 경고 메시지 출력
            ROS_ERROR("=====================================");
            ROS_ERROR("!!!!! EMERGENCY STOP TRIGGERED !!!!!");
            ROS_ERROR("Object: %s (ID: %d) at distance %.2f m",
                    obj.class_name.c_str(), obj.id, obj.distance);
            ROS_ERROR("=====================================");
            break;
        }
    }
    
    // 긴급 정지 신호 발행
    std_msgs::Bool e_stop_msg;
    e_stop_msg.data = emergency;
    emergency_stop_pub_.publish(e_stop_msg);
} 