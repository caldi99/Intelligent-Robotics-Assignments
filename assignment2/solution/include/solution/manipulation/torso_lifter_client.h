#ifndef TORSOLIFTERCLIENT_H
#define TORSOLIFTERCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

//SERVICE FILE
#include <manipulation_msgs/TorsoLifter.h>

/**
 * This class is a client for the TorsoLifterServer
 * @author Riccardo Rampon
*/
class TorsoLifterClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     */
    TorsoLifterClient(ros::NodeHandle node);

    /**
     * This function call the serverCall the server with the given message
     * @param msg : The message to send to the server which contains also the response
     * @return True if everything went ok
     */
    bool callServer(manipulation_msgs::TorsoLifter &msg);

private:   

    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // TORSOLIFTERCLIENT_H