#ifndef OBSTACLEEXTRACTORCLIENT_H
#define OBSTACLEEXTRACTORCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

//SERVICE FILE
#include <detection_msgs/Circles.h>

/**
 * This class is a client for the ObstacleExtractor server
 * @author Francesco Caldivezzi
*/
class ObstacleExtractorClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     */
    ObstacleExtractorClient(ros::NodeHandle node);

    /**
     * This function call the serverCall the server with the given message
     * @param msg : The message to send to the server which contains also the response
     * @return True if everything went ok
     */
    bool callServer(detection_msgs::Circles &msg);

private:   

    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // OBSTACLEEXTRACTORCLIENT_H