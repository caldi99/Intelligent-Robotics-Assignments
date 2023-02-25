// ROS
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MANIPULATION PACKAGE
#include <manipulation/pick_place/pick_place_server.h>

// CONSTANTS
#include <manipulation/constants/constants.h>

/**
 * @author Riccardo Rampon, Francesco Caldivezzi
*/
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(manipulation::NODE_PICK_PLACE << " NODE STARTED");
    // Init
    ros::init(argc, argv, manipulation::NODE_PICK_PLACE);

    // Start Node
    ros::NodeHandle node;

    // Spinner
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Manipulation Server
    PickPlaceServer pickPlace(node);

    ros::waitForShutdown();

    // Stop
    spinner.stop();

    return 0;

} // main