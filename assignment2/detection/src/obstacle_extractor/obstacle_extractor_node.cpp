#include <ros/ros.h>

//CONSTANTS
#include <detection/constants/constants.h>

//DETECTION PACKAGE
#include <detection/obstacle_extractor/obstacle_extractor_server.h>

/**
 * @author Francesco Caldivezzi
*/
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(detection::NODE_OBSTACLE_EXTRACTOR << " NODE STARTED");
    //Initializer
    ros::init(argc, argv, detection::NODE_OBSTACLE_EXTRACTOR);

    //Create node
    ros::NodeHandle node;

    //Advertize service   
    ObstacleExtractorServer server(node);

    ros::spin();

    return 0;
}