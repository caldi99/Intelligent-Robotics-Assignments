#ifndef POSESTRANSFORMERSERVER_H
#define POSESTRANSFORMERSERVER_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// SERVICE MESSAGE
#include <detection_msgs/Transform.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>

/**
 * This class is used to Transform the poses from two reference frames
 * @author Francesco Caldivezzi, Manuel Barusco 
 */
class PosesTransformerServer
{
public:
    /**
     * Creates a service server that will manage the response to a client on the topic TOPIC_POSES_TRANSFORMER
     * @param node : The node of the server
     * */
    PosesTransformerServer(ros::NodeHandle node);

private:
    /**
     * This function is used to manage the response to the client
     * @param req : The request of the client
     * @param res : The response of the server
     * @return True if everything went ok
     */
    bool sendResponse(detection_msgs::Transform::Request &req, detection_msgs::Transform::Response &res);

    ros::NodeHandle node;
    ros::ServiceServer server;
};

#endif // POSESTRANSFORMERSERVER_H
