#include "pose_estimation.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_estimation_demo");
    pose_estimation pose_estimation;
    ros::spin();
    return 0;
}