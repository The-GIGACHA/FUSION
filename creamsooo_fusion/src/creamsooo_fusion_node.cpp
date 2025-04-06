#include "creamsooo_fusion/creamsooo_fusion.h"

int main(int argc, char** argv) {
    // ROS 초기화
    ros::init(argc, argv, "creamsooo_fusion_node");
    ros::NodeHandle nh("~");
    
    // CreamsoooFusion 객체 생성
    CreamsoooFusion fusion(nh);
    
    // ROS 스핀
    ros::spin();
    
    return 0;
} 