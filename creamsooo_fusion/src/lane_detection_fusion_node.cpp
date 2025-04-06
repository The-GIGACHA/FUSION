#include "creamsooo_fusion/lane_detection_fusion.h"

int main(int argc, char** argv) {
    // ROS 초기화
    ros::init(argc, argv, "lane_detection_fusion_node");
    ros::NodeHandle nh("~");
    
    // LaneDetectionFusion 객체 생성
    LaneDetectionFusion fusion(nh);
    
    // ROS 스핀
    ros::spin();
    
    return 0;
} 