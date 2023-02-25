#include <ros/ros.h>

// CONSTANTS
#include <manipulation/constants/constants.h>

// MANIPULATION PACKAGE PACKAGE
#include <manipulation/torso_lifter/torso_lifter_server.h>

/**
 * @author Riccardo Rampon
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(manipulation::NODE_TORSO_LIFTER << " NODE STARTED");
    // Initializer
    ros::init(argc, argv, manipulation::NODE_TORSO_LIFTER);

    // Create node
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Advertize service
    TorsoLifterServer server(node);

    ros::waitForShutdown();

    return 0;
}