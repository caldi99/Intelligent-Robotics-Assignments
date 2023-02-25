#ifndef POSETABLEDETECTORSERVER_H
#define POSETABLEDETECTORSERVER_H

// ROS
#include <ros/ros.h>

// ACTION MESSAGE
#include <detection_msgs/PoseTableDetector.h>

// CONSTANTS
#include <detection/constants/constants.h>

// MANIPULATION
#include <manipulation_msgs/HeadMovement.h>

// OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// SENSOR MSGS
#include <sensor_msgs/image_encodings.h>

/**
 * This class is used to find the pose of the place table of the current picked object
 * @author Francesco Caldivezzi
 */
class PoseTableDetectorServer
{
private:
    ros::NodeHandle node;
    ros::ServiceServer server;
    ros::ServiceClient headClient;

    /**
     * This function returns true if the y of circle c1 is smaller than the y of cirlce c1
     * @param c1 : A Circle
     * @param c2 : A Circle
     * @return True if the y of circle c1 is smaller than the y of cirlce c1
     */
    static bool compareCircles(detection_msgs::Circle c1, detection_msgs::Circle c2);

    /**
     * This function checks if x is the smallest among x,y,z
     * @param x : A value
     * @param y : A value
     * @param z : A value
     * @return True if it is the biggest false otherwise
     */
    bool isSmallest(int x, int y, int z);

    /**
     * This function checks if x is the biggest among x,y,z
     * @param x : A value
     * @param y : A value
     * @param z : A value
     * @return True if it is the biggest false otherwise
     */
    bool isBiggest(int x, int y, int z);

    /**
     * This function computes the average point given a set of points
     * @param points : An array of points
     * @return The avg point
     */
    cv::Point2i getAvgPoint(const std::vector<cv::Point2i> &points);

    /**
     * Given a B&W image, this function returns the array of points where the image is white.
     * @param image : The image B&W
     * @return The arrray of points
     */
    std::vector<cv::Point2i> getPoints(const cv::Mat &image);

public:
    /**
     * Constructor of the class
     * @param node : The node of the server
     */
    PoseTableDetectorServer(ros::NodeHandle node);

    /**
     * This function is used to manage the response to the client
     * @param req : The request of the client
     * @param res : The response of the server
     * @return True if everything went ok
     */
    bool sendResponse(detection_msgs::PoseTableDetector::Request &req, detection_msgs::PoseTableDetector::Response &res);
};

#endif // POSETABLEDETECTORSERVER_H