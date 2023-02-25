// ROS
#include <ros/ros.h>

// DETECTION PACKAGE
#include <detection/pose_table_detector/pose_table_detector_server.h>

// CONSTANTS
#include <detection/constants/constants.h>

/**
 * @author Francesco Caldivezzi
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(detection::NODE_POSE_TABLE_DETECTOR << " NODE STARTED");

    // Init
    ros::init(argc, argv, detection::NODE_POSE_TABLE_DETECTOR);

    // Start Node
    ros::NodeHandle node;

    // Manipulation Server
    PoseTableDetectorServer server(node);

    // Spin
    ros::spin();
    return 0;

} // main