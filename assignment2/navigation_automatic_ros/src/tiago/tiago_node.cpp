#include <ros/ros.h>
//CONSTANTS
#include <navigation_automatic_ros/constants/constants.h>

//SERVER
#include <navigation_automatic_ros/tiago/tiago_server.h>

/**
 * @author Manuel Barusco
*/
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(navigation_automatic_ros::NODE_TIAGO_SERVER << " NODE STARTED");

    //Init
    ros::init(argc, argv, navigation_automatic_ros::NODE_TIAGO_SERVER);

    //Create server and wait
    TiagoServer server(navigation_automatic_ros::TOPIC_TIAGO_SERVER);

    ros::spin();
    return 0;
}