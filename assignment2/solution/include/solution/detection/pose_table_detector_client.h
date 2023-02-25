#ifndef POSETABLEDETECTORCLIENT_H
#define POSETABLEDETECTORCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

//SERVICE FILE
#include <detection_msgs/PoseTableDetector.h>

/**
 * This class is a client for the PoseTableDetectorServer
 * @author Francesco Caldivezzi
*/
class PoseTableDetectorClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     */
    PoseTableDetectorClient(ros::NodeHandle node);

    /**
     * This function call the serverCall the server with the given message
     * @param msg : The message to send to the server which contains also the response
     * @return True if everything went ok
     */
    bool callServer(detection_msgs::PoseTableDetector &msg);

private:   

    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // POSETABLEDETECTORCLIENT_H