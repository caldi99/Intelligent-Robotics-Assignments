#ifndef OBSTACLEEXTRACTORSERVER_H
#define OBSTACLEEXTRACTORSERVER_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// LASERSCAN MESSAGE
#include <sensor_msgs/LaserScan.h>

// CIRCLE MESSAGE
#include <detection_msgs/Circle.h>

// SERVICE MESSAGE
#include <detection_msgs/Circles.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>

// DETECTION PACKAGE
#include <detection/obstacle_extractor/obstacle_extractor.h>

/**
 * This class is used to extract circles
 * @author Francesco Caldivezzi
*/
class ObstacleExtractorServer
{
public:
    /**
     * Creates a service server that will manage the response to a client on the topic TOPIC_OBSTACLE_EXTRACTOR
     * @param node : The node of the server
     * */
    ObstacleExtractorServer(ros::NodeHandle node);

private:
    /**
     * This function a LaserScan message, returns the points detected by it
     * @param msg : The LaserScan message
     * @return A list of points
     */
    std::list<Point> computePoints(sensor_msgs::LaserScan msg);

    /**
     * This function is used to manage the response to the client
     * @param req : The request of the client
     * @param res : The response of the server
     * @return True if everything went ok
     */
    bool sendResponse(detection_msgs::Circles::Request &req, detection_msgs::Circles::Response &res);

    ros::NodeHandle node;
    ros::ServiceServer server;
};

#endif // OBSTACLEEXTRACTORSERVER_H