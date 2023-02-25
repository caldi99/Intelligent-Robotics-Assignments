#include <ros/ros.h>

// CONSTANTS
#include <manipulation/constants/constants.h>

// MANIPULATION PACKAGE PACKAGE
#include <manipulation/head_movement/head_movement_server.h>

/**
 * @author Manuel Barusco
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(manipulation::NODE_HEAD_MOVEMENT << " NODE STARTED");
    // Initializer
    ros::init(argc, argv, manipulation::NODE_HEAD_MOVEMENT);
    
    // Create node
    ros::NodeHandle node;

    HeadMovementServer server;

    ros::spin();

    return 0;
}