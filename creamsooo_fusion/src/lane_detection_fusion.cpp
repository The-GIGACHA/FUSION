#include "creamsooo_fusion/lane_detection_fusion.h"

LaneDetectionFusion::LaneDetectionFusion(ros::NodeHandle& nh) : nh_(nh) {
    // 초기화
    current_cloud_.reset(new PointCloudT());
    ground_cloud_.reset(new PointCloudT());
    
    // 파라미터 로딩
    nh_.param<std::string>("lidar_topic", lidar_topic_, "/velodyne_points");
    nh_.param<std::string>("lane_seg_topic", lane_seg_topic_, "/yolo_lane_seg");
    nh_.param<std::string>("frame_id", frame_id_, "ego_car");
    
    // 지면 검출 파라미터
    nh_.param<double>("ground_height_threshold", ground_height_threshold_, 0.2);
    nh_.param<double>("ground_max_angle", ground_max_angle_, 0.15);
    nh_.param<double>("ground_z_min", ground_z_min_, -2.0);
    nh_.param<double>("ground_z_max", ground_z_max_, 0.5);
    nh_.param<int>("ground_max_iterations", ground_max_iterations_, 200);
    nh_.param<double>("ground_min_inlier_ratio", ground_min_inlier_ratio_, 0.3);
    
    // 차선 검출 파라미터
    nh_.param<double>("lane_cluster_tolerance", lane_cluster_tolerance_, 0.1);
    nh_.param<int>("lane_min_cluster_size", lane_min_cluster_size_, 10);
    nh_.param<int>("lane_max_cluster_size", lane_max_cluster_size_, 1000);
    nh_.param<double>("lane_width", lane_width_, 0.2);
    nh_.param<double>("lane_min_distance", lane_min_distance_, 0.5);
    nh_.param<double>("lane_max_distance", lane_max_distance_, 20.0);
    nh_.param<double>("lane_width_limit", lane_width_limit_, 3.7);
    
    // 캘리브레이션 파라미터 로딩
    loadCalibrationParams();
    
    // 변환 행렬 계산
    calculateTransformMatrix();
    calculateCameraMatrix();
    
    // 구독자 및 발행자 설정
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &LaneDetectionFusion::lidarCallback, this);
    lane_seg_sub_ = nh_.subscribe(lane_seg_topic_, 1, &LaneDetectionFusion::laneSegCallback, this);
    
    filtered_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_ground", 1);
    lane_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lane_points", 1);
    lane_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_markers", 1);
    lane_center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("lane_center", 1);
    debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("lane_debug_image", 1);
    large_debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("lane_debug_image_large", 1);
    
    ROS_INFO("Lane Detection Fusion Node Initialized");
}

LaneDetectionFusion::~LaneDetectionFusion() {
    ROS_INFO("Lane Detection Fusion Node Terminated");
}

void LaneDetectionFusion::loadCalibrationParams() {
    // 카메라 파라미터 로드
    nh_.param<int>("/camera/width", image_width_, 640);
    nh_.param<int>("/camera/height", image_height_, 480);
    nh_.param<double>("/camera/fov", fov_, 90.0);
    fov_ = fov_ * M_PI / 180.0;  // 도에서 라디안으로 변환
    
    nh_.param<double>("/camera/x", cam_x_, 0.0);
    nh_.param<double>("/camera/y", cam_y_, 0.0);
    nh_.param<double>("/camera/z", cam_z_, 0.0);
    nh_.param<double>("/camera/roll", cam_roll_, 0.0);
    nh_.param<double>("/camera/pitch", cam_pitch_, 0.0);
    nh_.param<double>("/camera/yaw", cam_yaw_, 0.0);
    
    // 라디안으로 변환
    cam_roll_ = cam_roll_ * M_PI / 180.0;
    cam_pitch_ = cam_pitch_ * M_PI / 180.0;
    cam_yaw_ = cam_yaw_ * M_PI / 180.0;
    
    // 라이다 파라미터 로드 (생략되었으면 0으로 설정됨)
    nh_.param<double>("/lidar/x", lidar_x_, 0.0);
    nh_.param<double>("/lidar/y", lidar_y_, 0.0);
    nh_.param<double>("/lidar/z", lidar_z_, 0.0);
    nh_.param<double>("/lidar/roll", lidar_roll_, 0.0);
    nh_.param<double>("/lidar/pitch", lidar_pitch_, 0.0);
    nh_.param<double>("/lidar/yaw", lidar_yaw_, 0.0);
    
    // 라디안으로 변환
    lidar_roll_ = lidar_roll_ * M_PI / 180.0;
    lidar_pitch_ = lidar_pitch_ * M_PI / 180.0;
    lidar_yaw_ = lidar_yaw_ * M_PI / 180.0;
    
    ROS_INFO("Camera parameters:");
    ROS_INFO("Resolution: %dx%d, FOV: %.1f degrees", image_width_, image_height_, fov_ * 180.0 / M_PI);
    ROS_INFO("Position: (%.2f, %.2f, %.2f)", cam_x_, cam_y_, cam_z_);
    ROS_INFO("Orientation (deg): roll=%.2f, pitch=%.2f, yaw=%.2f", 
             cam_roll_ * 180.0 / M_PI, cam_pitch_ * 180.0 / M_PI, cam_yaw_ * 180.0 / M_PI);
    
    ROS_INFO("Lidar parameters:");
    ROS_INFO("Position: (%.2f, %.2f, %.2f)", lidar_x_, lidar_y_, lidar_z_);
    ROS_INFO("Orientation (deg): roll=%.2f, pitch=%.2f, yaw=%.2f", 
             lidar_roll_ * 180.0 / M_PI, lidar_pitch_ * 180.0 / M_PI, lidar_yaw_ * 180.0 / M_PI);
}

Eigen::Matrix3d LaneDetectionFusion::getRotationMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    return q.matrix();
}

void LaneDetectionFusion::calculateTransformMatrix() {
    // 카메라 자세에 추가 회전 적용 (카메라 좌표계 변환을 위한 보정)
    double adjusted_cam_roll = cam_roll_ - M_PI/2;  // -90도
    double adjusted_cam_pitch = cam_pitch_;
    double adjusted_cam_yaw = cam_yaw_ - M_PI/2;    // -90도
    
    // 회전 행렬 계산
    Eigen::Matrix3d cam_rot = getRotationMatrix(adjusted_cam_roll, adjusted_cam_pitch, adjusted_cam_yaw);
    Eigen::Matrix3d lidar_rot = getRotationMatrix(lidar_roll_, lidar_pitch_, lidar_yaw_);
    
    // 디버그 정보
    ROS_INFO("Adjusted Camera rotation matrix:");
    ROS_INFO("%.3f %.3f %.3f", cam_rot(0,0), cam_rot(0,1), cam_rot(0,2));
    ROS_INFO("%.3f %.3f %.3f", cam_rot(1,0), cam_rot(1,1), cam_rot(1,2));
    ROS_INFO("%.3f %.3f %.3f", cam_rot(2,0), cam_rot(2,1), cam_rot(2,2));
    
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
    
    // 변환 행렬 출력 (디버깅용)
    ROS_INFO("Lidar to camera transform matrix:");
    ROS_INFO("%.3f %.3f %.3f %.3f", transform_lidar_to_cam_(0,0), transform_lidar_to_cam_(0,1), transform_lidar_to_cam_(0,2), transform_lidar_to_cam_(0,3));
    ROS_INFO("%.3f %.3f %.3f %.3f", transform_lidar_to_cam_(1,0), transform_lidar_to_cam_(1,1), transform_lidar_to_cam_(1,2), transform_lidar_to_cam_(1,3));
    ROS_INFO("%.3f %.3f %.3f %.3f", transform_lidar_to_cam_(2,0), transform_lidar_to_cam_(2,1), transform_lidar_to_cam_(2,2), transform_lidar_to_cam_(2,3));
    ROS_INFO("%.3f %.3f %.3f %.3f", transform_lidar_to_cam_(3,0), transform_lidar_to_cam_(3,1), transform_lidar_to_cam_(3,2), transform_lidar_to_cam_(3,3));
}

void LaneDetectionFusion::calculateCameraMatrix() {
    // 핀홀 카메라 모델 내부 파라미터 계산
    double fx = image_width_ / (2 * tan(fov_ / 2));
    double fy = fx;  // 정사각형 픽셀 가정
    double cx = image_width_ / 2;
    double cy = image_height_ / 2;
    
    camera_matrix_ = Eigen::Matrix3d::Zero();
    camera_matrix_(0, 0) = fx;
    camera_matrix_(1, 1) = fy;
    camera_matrix_(0, 2) = cx;
    camera_matrix_(1, 2) = cy;
    camera_matrix_(2, 2) = 1.0;
    
    // 원본 카메라 행렬 저장
    original_camera_matrix_ = camera_matrix_;
    
    ROS_INFO("Camera matrix for %dx%d:", image_width_, image_height_);
    ROS_INFO("fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", fx, fy, cx, cy);
    ROS_INFO("Field of view: %.1f degrees", fov_ * 180.0 / M_PI);
    
    // 원본 이미지 크기 (1280x720)에 대한 카메라 행렬도 계산
    // 일단 640x480 기준으로 계산한 다음 스케일링 적용
    double scale_x = 1280.0 / image_width_;
    double scale_y = 720.0 / image_height_;
    
    ROS_INFO("Scaled camera matrix for 1280x720:");
    ROS_INFO("fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
             fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y);
}

Eigen::Vector3d LaneDetectionFusion::transformLidarToCamera(const Eigen::Vector4d& pc_lidar) {
    Eigen::Vector4d pc_camera = transform_lidar_to_cam_ * pc_lidar;
    
    // 디버깅을 위해 10번에 한 번 첫 번째 포인트의 변환 결과 출력
    static int transform_count = 0;
    if (transform_count % 1000 == 0) {
        ROS_INFO("Transform: Lidar (%.2f, %.2f, %.2f) -> Camera (%.2f, %.2f, %.2f)",
                 pc_lidar(0), pc_lidar(1), pc_lidar(2),
                 pc_camera(0), pc_camera(1), pc_camera(2));
    }
    transform_count++;
    
    return pc_camera.head<3>();
}

cv::Point2i LaneDetectionFusion::transformCameraToImage(const Eigen::Vector3d& pc_camera, bool use_original_size) {
    // 카메라 뒤에 있는 포인트는 무시
    if (pc_camera(2) <= 0) {
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    // 기본 카메라 매트릭스 파라미터
    double fx = camera_matrix_(0, 0);
    double fy = camera_matrix_(1, 1);
    double cx = camera_matrix_(0, 2);
    double cy = camera_matrix_(1, 2);
    
    // 원본 크기(1280x720)로 투영할 경우 스케일링 적용
    if (use_original_size) {
        double scale_x = 1280.0 / image_width_;
        double scale_y = 720.0 / image_height_;
        fx *= scale_x;
        fy *= scale_y;
        cx *= scale_x;
        cy *= scale_y;
    }
    
    // 디버깅용 (500번에 한 번씩 출력)
    static int count = 0;
    bool debug_this_point = (count % 500 == 0);
    count++;
    
    if (debug_this_point) {
        ROS_INFO("Camera matrix %s: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                 use_original_size ? "(1280x720)" : "(640x480)",
                 fx, fy, cx, cy);
        ROS_INFO("Camera point: (%.2f, %.2f, %.2f)", 
                 pc_camera(0), pc_camera(1), pc_camera(2));
    }
    
    // 카메라 내부 파라미터 행렬을 이용해 3D->2D 변환
    // 투영 방법: x' = fx * X / Z + cx, y' = fy * Y / Z + cy
    int u = static_cast<int>(std::round(fx * pc_camera(0) / pc_camera(2) + cx));
    int v = static_cast<int>(std::round(fy * pc_camera(1) / pc_camera(2) + cy));
    
    if (debug_this_point) {
        ROS_INFO("Image coordinates %s: (%d, %d)", 
                 use_original_size ? "(1280x720)" : "(640x480)", u, v);
    }
    
    // 이미지 영역을 벗어난 포인트 체크
    int width = use_original_size ? 1280 : image_width_;
    int height = use_original_size ? 720 : image_height_;
    
    if (u < 0 || u >= width || v < 0 || v >= height) {
        if (debug_this_point) {
            ROS_WARN("Point outside image bounds: (%d, %d), image size: %dx%d", 
                     u, v, width, height);
        }
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    return cv::Point2i(u, v);
}

bool LaneDetectionFusion::isPointInSegmentation(const PointT& point, const cv::Mat& seg_image) {
    static int total_points = 0;
    static int valid_points = 0;
    static ros::Time last_log_time = ros::Time::now();
    
    total_points++;
    
    if (seg_image.empty()) {
        ROS_WARN_THROTTLE(5.0, "Segmentation image is empty");
        return false;
    }
    
    // 라이다 포인트를 이미지 평면으로 투영
    Eigen::Vector4d pc_lidar(point.x, point.y, point.z, 1.0);
    Eigen::Vector3d pc_camera = transformLidarToCamera(pc_lidar);
    
    // 카메라 뒤에 있는 점은 제외
    if (pc_camera(2) <= 0) {
        return false;
    }
    
    cv::Point2i pixel = transformCameraToImage(pc_camera, false);
    
    // 이미지 경계 확인
    if (pixel.x < 0 || pixel.x >= seg_image.cols || pixel.y < 0 || pixel.y >= seg_image.rows) {
        return false;
    }
    
    // 세그멘테이션 값이 차선(흰색)인지 확인, YOLOPV2에서 차선은 흰색(255)으로 표시
    bool is_lane = seg_image.at<uchar>(pixel.y, pixel.x) > 127;
    
    if (is_lane) {
        valid_points++;
    }
    
    // 주기적으로 통계 로그 출력 (5초마다)
    ros::Time now = ros::Time::now();
    if ((now - last_log_time).toSec() >= 5.0) {
        ROS_INFO("Point projection stats: %d / %d points (%.2f%%) mapped to lane pixels", 
                valid_points, total_points, 100.0 * valid_points / total_points);
        // 통계 리셋
        total_points = 0;
        valid_points = 0;
        last_log_time = now;
        
        // 디버그용: 투영된 예제 점 좌표 출력
        ROS_INFO("Example point - Lidar: (%.2f, %.2f, %.2f) -> Camera: (%.2f, %.2f, %.2f) -> Image: (%d, %d)", 
                point.x, point.y, point.z, 
                pc_camera(0), pc_camera(1), pc_camera(2), 
                pixel.x, pixel.y);
    }
    
    return is_lane;
}

void LaneDetectionFusion::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("Received lidar message with frame_id: %s", msg->header.frame_id.c_str());
    
    // 좌표계 변환이 필요한지 확인
    if (msg->header.frame_id != frame_id_) {
        ROS_INFO("Transforming points from %s to %s", msg->header.frame_id.c_str(), frame_id_.c_str());
        
        // 임시적인 해결책: velodyne 프레임을 그대로 사용
        // 실제 환경에서는 tf를 사용하여 변환해야 함
        sensor_msgs::PointCloud2 transformed_cloud = *msg;
        transformed_cloud.header.frame_id = frame_id_;
        pcl::fromROSMsg(transformed_cloud, *current_cloud_);
    } else {
        pcl::fromROSMsg(*msg, *current_cloud_);
    }
    
    ROS_INFO("Received lidar points: %zu", current_cloud_->size());
    
    if (current_cloud_->empty()) {
        ROS_WARN("Received empty point cloud");
        return;
    }
    
    if (current_lane_seg_.empty()) {
        ROS_WARN("Lane segmentation image is empty, waiting for data");
        return;
    }
    
    processPointCloud();
}

void LaneDetectionFusion::laneSegCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received lane segmentation message with frame_id: %s", msg->header.frame_id.c_str());
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        cv::Mat orig_lane_seg = cv_ptr->image;
        
        // 원본 해상도 출력
        ROS_INFO("Original lane segmentation image: %dx%d", orig_lane_seg.cols, orig_lane_seg.rows);
        
        // 변환 비율 계산 (카메라 행렬 스케일링에 사용)
        double scale_x = static_cast<double>(image_width_) / orig_lane_seg.cols;
        double scale_y = static_cast<double>(image_height_) / orig_lane_seg.rows;
        
        // 캘리브레이션 해상도(640x480)로 리사이즈
        cv::resize(orig_lane_seg, current_lane_seg_, cv::Size(image_width_, image_height_), 0, 0, cv::INTER_LINEAR);
        
        ROS_INFO("Resized lane segmentation image to match calibration: %dx%d (scale: %.2f, %.2f)", 
                 current_lane_seg_.cols, current_lane_seg_.rows, scale_x, scale_y);
        
        // 변환 정보를 로그로 출력 (디버깅용)
        ROS_INFO("Camera matrix (original): fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                 camera_matrix_(0, 0), camera_matrix_(1, 1), 
                 camera_matrix_(0, 2), camera_matrix_(1, 2));
        
        // 임시로 조정된 카메라 행렬 계산 (디버깅 정보용)
        double scaled_fx = camera_matrix_(0, 0) / scale_x;
        double scaled_fy = camera_matrix_(1, 1) / scale_y;
        double scaled_cx = camera_matrix_(0, 2) / scale_x;
        double scaled_cy = camera_matrix_(1, 2) / scale_y;
        
        ROS_INFO("Camera matrix (scaled to original): fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                 scaled_fx, scaled_fy, scaled_cx, scaled_cy);
        
        // 간단한 통계 - 흰색(차선) 픽셀 수 카운트
        int white_pixels = 0;
        for (int y = 0; y < current_lane_seg_.rows; y++) {
            for (int x = 0; x < current_lane_seg_.cols; x++) {
                if (current_lane_seg_.at<uchar>(y, x) > 50) {  // 임계값 낮춤
                    white_pixels++;
                }
            }
        }
        ROS_INFO("Lane segmentation white pixels: %d (%.2f%%)", 
                white_pixels, 100.0 * white_pixels / (current_lane_seg_.rows * current_lane_seg_.cols));
        
        // 디버깅을 위해 세그멘테이션 이미지 저장
        static int img_count = 0;
        if (img_count % 30 == 0) {  // 30프레임마다 저장
            // 원본 이미지 저장
            std::string orig_filename = "/tmp/lane_seg_orig_" + std::to_string(img_count) + ".png";
            cv::imwrite(orig_filename, orig_lane_seg);
            
            // 리사이즈된 이미지 저장
            std::string filename = "/tmp/lane_seg_resized_" + std::to_string(img_count) + ".png";
            cv::imwrite(filename, current_lane_seg_);
            ROS_INFO("Saved lane segmentation images to %s and %s", 
                     orig_filename.c_str(), filename.c_str());
            
            // 컬러 시각화 이미지 생성
            cv::Mat color_vis = cv::Mat::zeros(current_lane_seg_.size(), CV_8UC3);
            for (int y = 0; y < current_lane_seg_.rows; y++) {
                for (int x = 0; x < current_lane_seg_.cols; x++) {
                    if (current_lane_seg_.at<uchar>(y, x) > 50) {
                        color_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);  // 빨간색으로 표시
                    }
                }
            }
            
            std::string color_filename = "/tmp/lane_seg_color_" + std::to_string(img_count) + ".png";
            cv::imwrite(color_filename, color_vis);
        }
        img_count++;
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void LaneDetectionFusion::processPointCloud() {
    if (current_cloud_->empty() || current_lane_seg_.empty()) {
        ROS_WARN("Empty point cloud or lane segmentation image");
        return;
    }
    
    ROS_INFO("Processing point cloud with %zu points", current_cloud_->size());
    
    // 1. 지면 포인트 추출
    extractGroundPoints();
    
    // 지면 포인트 발행
    publishGroundPoints();
    
    // 비교를 위해 클라우드 복사
    PointCloudT::Ptr ground_copy(new PointCloudT(*ground_cloud_));
    
    // 2. 차선 포인트 추출
    extractLanePoints();
    
    // 검출된 차선 포인트 발행
    sensor_msgs::PointCloud2 lane_cloud_msg;
    pcl::toROSMsg(*current_cloud_, lane_cloud_msg);
    lane_cloud_msg.header.frame_id = frame_id_;
    lane_cloud_msg.header.stamp = ros::Time::now();
    lane_points_pub_.publish(lane_cloud_msg);
    
    // 3. 차선 클러스터링 및 분류
    segmentLanes();
    
    // 4. 차선 중앙 계산
    calculateLaneCenter();
    
    // 5. 시각화
    publishLaneMarkers();
    publishLaneCenter();
}

void LaneDetectionFusion::extractGroundPoints() {
    if (current_cloud_->empty()) {
        ROS_WARN("Current point cloud is empty");
        return;
    }
    
    // 새로운 접근법: 라이다 위치를 고려한 지면 포인트 추출
    PointCloudT::Ptr ground_candidates(new PointCloudT());
    
    // 라이다의 상대적 위치 고려 (라이다는 지면에서 lidar_z_ 높이에 설치됨)
    double relative_z_min = ground_z_min_ + lidar_z_;
    double relative_z_max = ground_z_max_ + lidar_z_;
    
    ROS_INFO("Extracting ground points with height range [%.2f, %.2f] relative to lidar at height %.2f", 
             ground_z_min_, ground_z_max_, lidar_z_);
    
    // 높이 범위 내에 있는 포인트만 선택 (라이다 좌표계 기준)
    for (const auto& point : *current_cloud_) {
        // 라이다 설치 높이를 고려한 지면 포인트 추출
        if (point.z >= ground_z_min_ && point.z <= ground_z_max_) {
            // 거리에 따른 가중치를 적용 (가까운 포인트는 높이 제한을 더 엄격하게, 먼 포인트는 더 널널하게)
            double distance_xy = std::sqrt(point.x * point.x + point.y * point.y);
            
            // 거리에 따라 허용 높이 범위 확장 (먼 포인트일수록 높이 범위를 넓게)
            double height_tolerance = ground_height_threshold_ * (1.0 + 0.1 * distance_xy);
            
            // 기본 지면 높이(약 0.0)에서 허용 범위 내에 있거나, 거리에 따른 기준 적용
            if (std::abs(point.z) <= height_tolerance || 
                (distance_xy > 5.0 && point.z <= ground_z_max_)) {
                ground_candidates->push_back(point);
            }
        }
    }
    
    ROS_INFO("Height filtered points: %zu out of %zu (%.2f%%)", 
             ground_candidates->size(), current_cloud_->size(), 
             100.0 * ground_candidates->size() / current_cloud_->size());
    
    if (ground_candidates->empty()) {
        ROS_WARN("No points in adjusted height range, trying broader range");
        
        // 더 넓은 범위로 재시도
        for (const auto& point : *current_cloud_) {
            if (point.z >= ground_z_min_ - 0.5 && point.z <= ground_z_max_ + 0.5) {
                ground_candidates->push_back(point);
            }
        }
        
        ROS_INFO("Broader height filter: %zu points found", ground_candidates->size());
    }
    
    if (ground_candidates->empty()) {
        ROS_ERROR("Still no ground points detected! Check lidar position and ground parameters");
        return;
    }
    
    // 지면 포인트 클라우드 설정
    ground_cloud_ = ground_candidates;
    
    ROS_INFO("Using %zu ground points", ground_cloud_->size());
    
    // 추가적인 디버깅 정보
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    
    for (const auto& point : *ground_cloud_) {
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
    }
    
    ROS_INFO("Ground points height range: [%.2f, %.2f]", min_z, max_z);
}

void LaneDetectionFusion::extractLanePoints() {
    if (ground_cloud_->empty() || current_lane_seg_.empty()) {
        ROS_WARN("Empty ground cloud or lane segmentation image!");
        return;
    }
    
    // 디버깅 정보 - 원래 ground_cloud_ 크기
    ROS_INFO("Ground cloud size before lane filtering: %zu", ground_cloud_->size());
    
    // 차선 포인트 클라우드 초기화
    PointCloudT::Ptr lane_cloud(new PointCloudT());
    
    // 카운터 변수 (디버깅용)
    int projected_points = 0;
    int points_in_image = 0;
    int lane_points = 0;
    int width_filtered = 0;
    
    // 세그멘테이션 이미지 필터링 (Python 코드의 seg_img_filter 함수와 유사)
    cv::Mat filtered_seg_img = current_lane_seg_.clone();
    
    // 이미지 하단부만 사용 (상단부는 차선 검출에 적합하지 않을 수 있음)
    int threshold_height = static_cast<int>(filtered_seg_img.rows * 0.75); // 이미지 3/4 아래 부분만 사용
    
    // 상단 부분 마스킹
    for (int y = 0; y < threshold_height; y++) {
        for (int x = 0; x < filtered_seg_img.cols; x++) {
            filtered_seg_img.at<uchar>(y, x) = 0;
        }
    }
    
    // 디버깅용 시각화 이미지
    cv::Mat debug_img;
    if (!current_lane_seg_.empty()) {
        cv::cvtColor(filtered_seg_img, debug_img, cv::COLOR_GRAY2BGR);
    } else {
        debug_img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    
    // 원본 세그멘테이션 이미지를 컬러로 변환해서 오버레이용 이미지 준비
    cv::Mat overlay_img;
    cv::cvtColor(current_lane_seg_, overlay_img, cv::COLOR_GRAY2BGR);
    
    // 원본 크기(1280x720)용 오버레이 이미지
    cv::Mat large_overlay_img;
    cv::cvtColor(current_lane_seg_, large_overlay_img, cv::COLOR_GRAY2BGR);
    cv::resize(large_overlay_img, large_overlay_img, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);

    ROS_INFO("Lane width limit: %.2f meters", lane_width_limit_);
    
    // 지면 포인트를 이미지에 투영하여 차선인지 확인
    for (const auto& point : *ground_cloud_) {
        projected_points++;
        
        // 좌우 폭 제한 (중앙으로부터 ±lane_width_limit_ m)
        if (std::abs(point.y) > lane_width_limit_) {
            width_filtered++;
            continue;
        }
        
        // 최대 거리 제한 확인
        double distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance < lane_min_distance_ || distance > lane_max_distance_) {
            continue;
        }
        
        // 라이다 포인트를 이미지 평면으로 투영
        Eigen::Vector4d pc_lidar(point.x, point.y, point.z, 1.0);
        Eigen::Vector3d pc_camera = transformLidarToCamera(pc_lidar);
        
        // 카메라 뒤에 있는 점은 제외
        if (pc_camera(2) <= 0) {
            continue;
        }
        
        // 640x480 해상도로 투영
        cv::Point2i pixel = transformCameraToImage(pc_camera, false);
        
        // 무효한 포인트 체크 (이미지 경계 바깥)
        if (pixel.x < 0 || pixel.y < 0) {
            continue;
        }
        
        points_in_image++;
        
        // 디버그 이미지에 투영된 포인트 그리기 (파란색)
        cv::circle(debug_img, pixel, 2, cv::Scalar(255, 0, 0), -1);
        
        // 640x480 오버레이 이미지에 포인트 그리기 (라이다 포인트: 파란색)
        cv::circle(overlay_img, pixel, 2, cv::Scalar(255, 0, 0), -1);
        
        // 1280x720 오버레이 이미지에 포인트 그리기 (라이다 포인트: 파란색)
        // 1280x720 해상도로 투영
        cv::Point2i large_pixel = transformCameraToImage(pc_camera, true);
        if (large_pixel.x >= 0 && large_pixel.y >= 0) {
            cv::circle(large_overlay_img, large_pixel, 3, cv::Scalar(255, 0, 0), -1);
        }
        
        // 차선 세그멘테이션 확인 - 주변 픽셀 검사
        bool is_lane = false;
        int search_radius = 5;  // 작게 조정
        
        for (int dy = -search_radius; dy <= search_radius && !is_lane; dy++) {
            for (int dx = -search_radius; dx <= search_radius && !is_lane; dx++) {
                int nx = pixel.x + dx;
                int ny = pixel.y + dy;
                
                // 경계 확인
                if (nx >= 0 && nx < filtered_seg_img.cols && ny >= 0 && ny < filtered_seg_img.rows) {
                    // 세그멘테이션 값이 차선(흰색)인지 확인 (임계값을 조금 낮춤)
                    if (filtered_seg_img.at<uchar>(ny, nx) > 20) {
                        is_lane = true;
                        break;
                    }
                }
            }
        }
        
        if (is_lane) {
            lane_points++;
            
            // 차선 포인트 추가
            PointT lane_point = point;
            lane_point.intensity = 255.0;  // 차선 포인트 표시용
            lane_cloud->push_back(lane_point);
            
            // 디버그 이미지에 차선 포인트 그리기 (녹색)
            cv::circle(debug_img, pixel, 3, cv::Scalar(0, 255, 0), -1);
            
            // 640x480 오버레이 이미지에 차선 포인트 그리기 (녹색)
            cv::circle(overlay_img, pixel, 3, cv::Scalar(0, 255, 0), -1);
            
            // 1280x720 오버레이 이미지에 차선 포인트 그리기 (녹색)
            if (large_pixel.x >= 0 && large_pixel.y >= 0) {
                cv::circle(large_overlay_img, large_pixel, 5, cv::Scalar(0, 255, 0), -1);
            }
            
            // 디버깅용 - 제대로 검출된 포인트 좌표 출력 (처음 10개만)
            if (lane_points <= 10) {
                ROS_INFO("Lane point: (%.2f, %.2f, %.2f), Pixel: (%d, %d), Large: (%d, %d)",
                         point.x, point.y, point.z, pixel.x, pixel.y, large_pixel.x, large_pixel.y);
            }
        }
    }
    
    // 오버레이 이미지에 중앙 기준선 표시 및 좌우 폭 제한 영역 표시
    // 640x480 오버레이 이미지
    int mid_x = overlay_img.cols / 2;
    cv::line(overlay_img, cv::Point(mid_x, 0), cv::Point(mid_x, overlay_img.rows), cv::Scalar(0, 255, 255), 1);

    // 텍스트 정보 추가
    std::string info_text = "Lane points: " + std::to_string(lane_points) + 
                           ", Width limit: " + std::to_string(lane_width_limit_) + "m";
    cv::putText(overlay_img, info_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
               cv::Scalar(0, 165, 255), 2);

    // 1280x720 오버레이 이미지
    int large_mid_x = large_overlay_img.cols / 2;
    cv::line(large_overlay_img, cv::Point(large_mid_x, 0), 
             cv::Point(large_mid_x, large_overlay_img.rows), cv::Scalar(0, 255, 255), 2);

    // 텍스트 정보 추가 (큰 이미지)
    cv::putText(large_overlay_img, info_text, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                cv::Scalar(0, 165, 255), 2);
    
    // 디버그 이미지 저장
    static int debug_count = 0;
    if (debug_count % 10 == 0) {  // 10프레임마다 저장
        std::string debug_filename = "/tmp/lane_detection_debug_" + std::to_string(debug_count) + ".png";
        cv::imwrite(debug_filename, debug_img);
        
        std::string overlay_filename = "/tmp/lane_overlay_" + std::to_string(debug_count) + ".png";
        cv::imwrite(overlay_filename, overlay_img);
        
        std::string large_overlay_filename = "/tmp/lane_overlay_large_" + std::to_string(debug_count) + ".png";
        cv::imwrite(large_overlay_filename, large_overlay_img);
        
        ROS_INFO("Saved debug images to %s, %s, and %s", 
                 debug_filename.c_str(), overlay_filename.c_str(), large_overlay_filename.c_str());
    }
    debug_count++;
    
    // 오버레이 이미지를 ROS 메시지로 변환해서 퍼블리시 (640x480)
    sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", overlay_img).toImageMsg();
    debug_img_msg->header.stamp = ros::Time::now();
    debug_img_msg->header.frame_id = frame_id_;
    debug_image_pub_.publish(debug_img_msg);

    // 큰 오버레이 이미지 퍼블리시 (1280x720)
    sensor_msgs::ImagePtr large_debug_img_msg = 
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", large_overlay_img).toImageMsg();
    large_debug_img_msg->header.stamp = ros::Time::now();
    large_debug_img_msg->header.frame_id = frame_id_;
    large_debug_image_pub_.publish(large_debug_img_msg);
    
    // 차선 포인트 설정
    *current_cloud_ = *lane_cloud;
    
    // 디버깅 정보
    ROS_INFO("Lane detection stats: width filtered %d, lane/image/total: %d/%d/%d (%.2f%%)",
             width_filtered, lane_points, points_in_image, projected_points,
             projected_points > 0 ? 100.0 * lane_points / projected_points : 0.0);
    
    if (lane_cloud->empty()) {
        ROS_WARN("No lane points detected!");
    } else {
        ROS_INFO("Detected %zu lane points in %.2fm lane width", lane_cloud->size(), lane_width_limit_);
    }
}

void LaneDetectionFusion::segmentLanes() {
    if (current_cloud_->empty()) {
        ROS_WARN("No lane points detected!");
        return;
    }
    
    ROS_INFO("Starting lane segmentation with %zu points", current_cloud_->size());
    
    // 높이(z값)에 따라 포인트를 그룹화
    std::map<int, PointCloudT::Ptr> height_groups;
    std::map<int, double> height_group_values;
    
    // 높이 그룹 생성 (1cm 간격으로 구분)
    double height_resolution = 0.01;  // 1cm 단위로 높이 그룹화
    
    // 최소/최대 z값을 추적하여 높이 범위 확인
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    
    for (const auto& point : *current_cloud_) {
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
        
        // 높이(z값)에 따라 그룹 인덱스 계산
        int height_group = static_cast<int>(point.z / height_resolution);
        
        // 해당 높이 그룹 생성
        if (height_groups.find(height_group) == height_groups.end()) {
            height_groups[height_group] = boost::make_shared<PointCloudT>();
            height_group_values[height_group] = height_group * height_resolution;
        }
        
        // 포인트를 해당 높이 그룹에 추가
        height_groups[height_group]->push_back(point);
    }
    
    ROS_INFO("Height range: %.3f to %.3f, divided into %zu groups", 
             min_z, max_z, height_groups.size());
    
    // 최종 차선 클러스터 저장
    lanes_.clear();
    
    // 각 높이 그룹별로 클러스터링 진행
    for (const auto& group_pair : height_groups) {
        int height_group = group_pair.first;
        PointCloudT::Ptr cloud = group_pair.second;
        double group_height = height_group_values[height_group];
        
        if (cloud->size() < lane_min_cluster_size_) {
            ROS_DEBUG("Skipping height group %.3f with only %zu points", 
                     group_height, cloud->size());
            continue;
        }
        
        ROS_INFO("Processing height group %.3f with %zu points", 
                 group_height, cloud->size());
        
        // 클러스터링 수행
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);
        
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(lane_cluster_tolerance_);
        ec.setMinClusterSize(lane_min_cluster_size_);
        ec.setMaxClusterSize(lane_max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        
        ROS_INFO("Found %zu clusters at height %.3f", cluster_indices.size(), group_height);
        
        // 각 클러스터 처리
        for (size_t i = 0; i < cluster_indices.size(); i++) {
            // 클러스터 추출
            PointCloudT::Ptr cluster_cloud(new PointCloudT);
            for (const auto& idx : cluster_indices[i].indices) {
                cluster_cloud->push_back((*cloud)[idx]);
            }
            
            // 중심점 계산
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster_cloud, centroid);
            
            // 차선 정보 생성
            LaneInfo lane_info;
            lane_info.cloud = cluster_cloud;
            lane_info.centroid = centroid;
            lane_info.distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);
            lane_info.is_left = centroid[1] > 0;  // y좌표 기준 좌/우 구분
            lane_info.height = group_height;
            
            // 유효한 차선 클러스터인지 확인
            // 너무 가까운 포인트는 노이즈일 수 있음
            if (centroid[0] < 1.0) {
                ROS_DEBUG("Ignoring cluster: too close (%.2f m)", centroid[0]);
                continue;
            }
            
            // 차선 클러스터 추가
            lanes_.push_back(lane_info);
            
            ROS_INFO("Added %s lane at height %.3f: centroid (%.2f, %.2f, %.2f), distance %.2f, size %zu",
                    lane_info.is_left ? "left" : "right",
                    lane_info.height,
                    lane_info.centroid[0], lane_info.centroid[1], lane_info.centroid[2],
                    lane_info.distance, cluster_cloud->size());
        }
    }
    
    // 최적의 좌/우 차선 선택
    selectBestLanes();
    
    ROS_INFO("Lane segmentation completed: %zu lanes found", lanes_.size());
}

// 최적의 좌/우 차선 선택 함수 추가
void LaneDetectionFusion::selectBestLanes() {
    // 좌/우 차선 후보 저장
    std::vector<LaneInfo*> left_candidates;
    std::vector<LaneInfo*> right_candidates;
    
    // 좌/우로 분류
    for (auto& lane : lanes_) {
        if (lane.is_left) {
            left_candidates.push_back(&lane);
        } else {
            right_candidates.push_back(&lane);
        }
    }
    
    ROS_INFO("Lane candidates: %zu left, %zu right", 
             left_candidates.size(), right_candidates.size());
    
    // 차선 찾기 결과를 저장할 배열
    std::vector<LaneInfo> final_lanes;
    
    // 왼쪽 차선 선택 (차량에 가장 가까운 것)
    if (!left_candidates.empty()) {
        std::sort(left_candidates.begin(), left_candidates.end(), 
                 [](LaneInfo* a, LaneInfo* b) { return std::abs(a->centroid[1]) < std::abs(b->centroid[1]); });
        
        final_lanes.push_back(*left_candidates[0]);
        ROS_INFO("Selected left lane: centroid (%.2f, %.2f, %.2f), distance %.2f",
                 left_candidates[0]->centroid[0], left_candidates[0]->centroid[1], 
                 left_candidates[0]->centroid[2], left_candidates[0]->distance);
    }
    
    // 오른쪽 차선 선택 (차량에 가장 가까운 것)
    if (!right_candidates.empty()) {
        std::sort(right_candidates.begin(), right_candidates.end(), 
                 [](LaneInfo* a, LaneInfo* b) { return std::abs(a->centroid[1]) < std::abs(b->centroid[1]); });
        
        final_lanes.push_back(*right_candidates[0]);
        ROS_INFO("Selected right lane: centroid (%.2f, %.2f, %.2f), distance %.2f",
                 right_candidates[0]->centroid[0], right_candidates[0]->centroid[1], 
                 right_candidates[0]->centroid[2], right_candidates[0]->distance);
    }
    
    // 최종 선택된 차선으로 교체
    lanes_ = final_lanes;
}

void LaneDetectionFusion::calculateLaneCenter() {
    if (lanes_.empty()) {
        return;
    }
    
    // 왼쪽과 오른쪽 차선 찾기
    LaneInfo* left_lane = nullptr;
    LaneInfo* right_lane = nullptr;
    
    for (auto& lane : lanes_) {
        if (lane.is_left && !left_lane) {
            left_lane = &lane;
        } else if (!lane.is_left && !right_lane) {
            right_lane = &lane;
        }
        
        if (left_lane && right_lane) break;
    }
    
    // 차선 중앙점 마커 발행
    visualization_msgs::Marker center_marker;
    center_marker.header.frame_id = frame_id_;
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "lane_center";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.lifetime = ros::Duration(0.1);
    
    // 중앙점 색상 (녹색)
    center_marker.color.r = 0.0;
    center_marker.color.g = 1.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 1.0;
    
    center_marker.scale.x = 0.3;
    center_marker.scale.y = 0.3;
    center_marker.scale.z = 0.3;
    
    // 중앙점 위치 계산
    if (left_lane && right_lane) {
        // 양쪽 차선 모두 있을 때 - 두 차선의 중간
        center_marker.pose.position.x = (left_lane->centroid[0] + right_lane->centroid[0]) / 2.0;
        center_marker.pose.position.y = (left_lane->centroid[1] + right_lane->centroid[1]) / 2.0;
        center_marker.pose.position.z = (left_lane->centroid[2] + right_lane->centroid[2]) / 2.0;
    } 
    else if (left_lane) {
        // 왼쪽 차선만 있을 때 - 추정된 중앙
        double estimated_right_y = left_lane->centroid[1] - 3.5;  // 예상 차선 폭 3.5m
        center_marker.pose.position.x = left_lane->centroid[0];
        center_marker.pose.position.y = (left_lane->centroid[1] + estimated_right_y) / 2.0;
        center_marker.pose.position.z = left_lane->centroid[2];
    }
    else if (right_lane) {
        // 오른쪽 차선만 있을 때 - 추정된 중앙
        double estimated_left_y = right_lane->centroid[1] + 3.5;  // 예상 차선 폭 3.5m
        center_marker.pose.position.x = right_lane->centroid[0];
        center_marker.pose.position.y = (right_lane->centroid[1] + estimated_left_y) / 2.0;
        center_marker.pose.position.z = right_lane->centroid[2];
    }
    else {
        // 둘 다 없을 때는 마커를 발행하지 않음
        return;
    }
    
    center_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::MarkerArray center_marker_array;
    center_marker_array.markers.push_back(center_marker);
    
    lane_markers_pub_.publish(center_marker_array);
    
    // 중앙점 발행 (PointStamped 메시지로도 발행)
    geometry_msgs::PointStamped center_point;
    center_point.header.frame_id = frame_id_;
    center_point.header.stamp = ros::Time::now();
    center_point.point.x = center_marker.pose.position.x;
    center_point.point.y = center_marker.pose.position.y;
    center_point.point.z = center_marker.pose.position.z;
    
    lane_center_pub_.publish(center_point);
}

void LaneDetectionFusion::publishGroundPoints() {
    if (ground_cloud_->empty()) {
        return;
    }
    
    sensor_msgs::PointCloud2 ground_cloud_msg;
    pcl::toROSMsg(*ground_cloud_, ground_cloud_msg);
    ground_cloud_msg.header.frame_id = frame_id_;
    ground_cloud_msg.header.stamp = ros::Time::now();
    filtered_ground_pub_.publish(ground_cloud_msg);
}

void LaneDetectionFusion::publishLaneMarkers() {
    visualization_msgs::MarkerArray markers;
    
    for (size_t i = 0; i < lanes_.size(); i++) {
        // 차선 포인트 클라우드를 마커로 변환
        visualization_msgs::Marker lane_marker;
        lane_marker.header.frame_id = frame_id_;
        lane_marker.header.stamp = ros::Time::now();
        lane_marker.ns = "lane_points";
        lane_marker.id = i;
        lane_marker.type = visualization_msgs::Marker::POINTS;
        lane_marker.action = visualization_msgs::Marker::ADD;
        lane_marker.lifetime = ros::Duration(0.1);
        
        // 좌/우 차선에 따라 색상 설정
        if (lanes_[i].is_left) {
            lane_marker.color.r = 1.0; // 좌측 차선은 빨간색
            lane_marker.color.g = 0.0;
            lane_marker.color.b = 0.0;
        } else {
            lane_marker.color.r = 0.0; // 우측 차선은 파란색
            lane_marker.color.g = 0.0;
            lane_marker.color.b = 1.0;
        }
        lane_marker.color.a = 1.0;
        
        lane_marker.scale.x = 0.05;
        lane_marker.scale.y = 0.05;
        
        for (const auto& point : *(lanes_[i].cloud)) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            lane_marker.points.push_back(p);
        }
        
        markers.markers.push_back(lane_marker);
        
        // 차선 중심점 마커 추가
        visualization_msgs::Marker centroid_marker;
        centroid_marker.header.frame_id = frame_id_;
        centroid_marker.header.stamp = ros::Time::now();
        centroid_marker.ns = "lane_centroids";
        centroid_marker.id = i;
        centroid_marker.type = visualization_msgs::Marker::SPHERE;
        centroid_marker.action = visualization_msgs::Marker::ADD;
        centroid_marker.lifetime = ros::Duration(0.1);
        
        if (lanes_[i].is_left) {
            centroid_marker.color.r = 1.0;
            centroid_marker.color.g = 0.0;
            centroid_marker.color.b = 0.0;
        } else {
            centroid_marker.color.r = 0.0;
            centroid_marker.color.g = 0.0;
            centroid_marker.color.b = 1.0;
        }
        centroid_marker.color.a = 1.0;
        
        centroid_marker.scale.x = 0.2;
        centroid_marker.scale.y = 0.2;
        centroid_marker.scale.z = 0.2;
        
        centroid_marker.pose.position.x = lanes_[i].centroid[0];
        centroid_marker.pose.position.y = lanes_[i].centroid[1];
        centroid_marker.pose.position.z = lanes_[i].centroid[2];
        centroid_marker.pose.orientation.w = 1.0;
        
        markers.markers.push_back(centroid_marker);
    }
    
    lane_markers_pub_.publish(markers);
}

void LaneDetectionFusion::publishLaneCenter() {
    if (lanes_.empty()) {
        return;
    }
    
    // 왼쪽과 오른쪽 차선 찾기
    LaneInfo* left_lane = nullptr;
    LaneInfo* right_lane = nullptr;
    
    for (auto& lane : lanes_) {
        if (lane.is_left && !left_lane) {
            left_lane = &lane;
        } else if (!lane.is_left && !right_lane) {
            right_lane = &lane;
        }
        
        if (left_lane && right_lane) break;
    }
    
    // 차선 중앙점 마커 발행
    visualization_msgs::Marker center_marker;
    center_marker.header.frame_id = frame_id_;
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "lane_center";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.lifetime = ros::Duration(0.1);
    
    // 중앙점 색상 (녹색)
    center_marker.color.r = 0.0;
    center_marker.color.g = 1.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 1.0;
    
    center_marker.scale.x = 0.3;
    center_marker.scale.y = 0.3;
    center_marker.scale.z = 0.3;
    
    // 중앙점 위치 계산
    if (left_lane && right_lane) {
        // 양쪽 차선 모두 있을 때 - 두 차선의 중간
        center_marker.pose.position.x = (left_lane->centroid[0] + right_lane->centroid[0]) / 2.0;
        center_marker.pose.position.y = (left_lane->centroid[1] + right_lane->centroid[1]) / 2.0;
        center_marker.pose.position.z = (left_lane->centroid[2] + right_lane->centroid[2]) / 2.0;
    } 
    else if (left_lane) {
        // 왼쪽 차선만 있을 때 - 추정된 중앙
        double estimated_right_y = left_lane->centroid[1] - 3.5;  // 예상 차선 폭 3.5m
        center_marker.pose.position.x = left_lane->centroid[0];
        center_marker.pose.position.y = (left_lane->centroid[1] + estimated_right_y) / 2.0;
        center_marker.pose.position.z = left_lane->centroid[2];
    }
    else if (right_lane) {
        // 오른쪽 차선만 있을 때 - 추정된 중앙
        double estimated_left_y = right_lane->centroid[1] + 3.5;  // 예상 차선 폭 3.5m
        center_marker.pose.position.x = right_lane->centroid[0];
        center_marker.pose.position.y = (right_lane->centroid[1] + estimated_left_y) / 2.0;
        center_marker.pose.position.z = right_lane->centroid[2];
    }
    else {
        // 둘 다 없을 때는 마커를 발행하지 않음
        return;
    }
    
    center_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::MarkerArray center_marker_array;
    center_marker_array.markers.push_back(center_marker);
    
    lane_markers_pub_.publish(center_marker_array);
    
    // 중앙점 발행 (PointStamped 메시지로도 발행)
    geometry_msgs::PointStamped center_point;
    center_point.header.frame_id = frame_id_;
    center_point.header.stamp = ros::Time::now();
    center_point.point.x = center_marker.pose.position.x;
    center_point.point.y = center_marker.pose.position.y;
    center_point.point.z = center_marker.pose.position.z;
    
    lane_center_pub_.publish(center_point);
} 