#ifndef HEADMOVEMENT_H
#define HEADMOVEMENT_H

// ROS
#include <ros/ros.h>
#include <ros/topic.h>

// ACTIONLIB
#include <actionlib/client/simple_action_client.h>

// CONSTANTS
#include <manipulation/constants/constants.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

// Geometry message
#include <geometry_msgs/PointStamped.h>

// SERVICE
#include <manipulation_msgs/HeadMovement.h>

// ACTION FOR TRAJECTOR
#include <control_msgs/FollowJointTrajectoryAction.h>

// ACTION FOR HEAD PIXEL POINTING
#include <control_msgs/PointHeadAction.h>

// Action interface type for moving TIAGo's head
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControllClient;

// Action interface type for moving TIAGo's head with the image pixel pointing
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;

/**
 * This class is used to move the head
 * @author Manuel Barusco, Francesco Caldivezzi
 */
class HeadMovementServer
{
private:
    PointHeadClient pointHeadClient;              // action client for moving the Tiago Head
    HeadControllClient headClient;                // action client for moving the Tiago Head
    control_msgs::FollowJointTrajectoryGoal goal; // goal to move the head
    cv::Mat cameraIntrinsics;                     // camera intrinsics parameters
    ros::NodeHandle node;
    ros::NodeHandle serverNode;
    ros::ServiceServer server;

public:
    // constructor
    HeadMovementServer();

    /** service callback
     * @param &req reference to the service request
     * @param &res reference to the service response
     */
    bool headMovementCallback(manipulation_msgs::HeadMovement::Request &req, manipulation_msgs::HeadMovement::Response &res);

    /** track for color
     * @param center point where TIAGo head must point
     */
    void pointHead(cv::Point2d center);

    /** move head to navigation position (standard position)
     */
    void pointHeadForNavigation();
};

#endif // HEADMOVEMENT_H