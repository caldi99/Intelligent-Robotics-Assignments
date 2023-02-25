
#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// DETECTION PACKAGE
#include <detection/poses_detection/poses_detection_publisher.h>

/**
 * @author Francesco Caldivezzi
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(detection::NODE_POSES_DETECTION << " NODE STARTED");

    // Initializer
    ros::init(argc, argv, detection::NODE_POSES_DETECTION);

    // Create node
    ros::NodeHandle node;

    // Create publisher
    PosesDetectionPublisher publisher(node);

    // Define loop rate
    ros::Rate loopRate(detection::LOOP_RATE);

    while (ros::ok())
    {
        publisher.publish();
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}