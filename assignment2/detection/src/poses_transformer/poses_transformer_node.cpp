#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// DETECTION PACKAGE
#include <detection/poses_transformer/poses_transformer_server.h>

/**
 * @author Francesco Caldivezzi
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(detection::NODE_POSES_TRANSFORMER << " NODE STARTED");
    // Initializer
    ros::init(argc, argv, detection::NODE_POSES_TRANSFORMER);

    // Create node
    ros::NodeHandle node;

    // Advertize service
    PosesTransformerServer server(node);

    ros::spin();

    return 0;
}