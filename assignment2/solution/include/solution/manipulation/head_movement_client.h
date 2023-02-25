#ifndef HEADMOVEMENTCLIENT_H
#define HEADMOVEMENTCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

//SERVICE FILE
#include <manipulation_msgs/HeadMovement.h>

/**
 * This class is a client for the HeadMovementServer
 * @author Manuel Barusco
*/
class HeadMovementClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     */
    HeadMovementClient(ros::NodeHandle node);

    /**
     * This function call the serverCall the server with the given message
     * @param msg : The message to send to the server which contains also the response
     * @return True if everything went ok
     */
    bool callServer(manipulation_msgs::HeadMovement &msg);

private:   

    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // HEADMOVEMENTCLIENT_H