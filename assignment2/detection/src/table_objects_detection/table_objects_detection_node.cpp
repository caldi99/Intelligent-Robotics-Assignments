#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// DETECTION PACKAGE
#include <detection/table_objects_detection/table_objects_detection_server.h>

/**
 * @author Manuel Barusco
*/
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(detection::NODE_TABLE_DETECTION << " NODE STARTED");
    // Initializer
    ros::init(argc, argv, detection::NODE_TABLE_DETECTION);

    // Create node
    ros::NodeHandle node;

    TableObjectsDetectionServer server; 

    ros::spin();
        
    return 0;
}