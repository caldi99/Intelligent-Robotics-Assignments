#ifndef POSESTRANSFORMERCLIENT_H
#define POSESTRANSFORMERCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

//SERVICE FILE
#include <detection_msgs/Transform.h>

/**
 * This class is a client for the PosesTransformerServer
 * @author Francesco Caldivezzi
*/
class PosesTransformerClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     */
    PosesTransformerClient(ros::NodeHandle node);

    /**
     * This function call the serverCall the server with the given message
     * @param msg : The message to send to the server which contains also the response
     * @return True if everything went ok
     */
    bool callServer(detection_msgs::Transform &msg);

private:   

    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // POSESTRANSFORMERCLIENT_H