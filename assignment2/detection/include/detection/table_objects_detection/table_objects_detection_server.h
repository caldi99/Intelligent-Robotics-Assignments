#ifndef TABLEOBJECTSDETECTIONSERVER_H
#define TABLEOBJECTSDETECTIONSERVER_H

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// CONSTANTS
#include <detection/constants/constants.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// SERVICE
#include <manipulation_msgs/HeadMovement.h>

// MESSAGE FILE
#include <detection_msgs/Detections.h>
#include <detection_msgs/Detection.h>

// GEOMETRY MESSAGES
#include <sensor_msgs/JointState.h>

// ACTION
#include <detection_msgs/ObjectsDetectionAction.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

/**
 * @author Manuel Barusco
 */
class TableObjectsDetectionServer
{
private:
    int colorID; // id of the color to search (it arrives from the action request)
    ros::NodeHandle nh;
    ros::NodeHandle nodeHead;
    actionlib::SimpleActionServer<detection_msgs::ObjectsDetectionAction> actionServer; // action server for the ObjectsDetection action

    ros::ServiceClient clientHead = nodeHead.serviceClient<manipulation_msgs::HeadMovement>(detection::TOPIC_HEAD_MOVEMENT); // client to the head movement manipulation service

    detection_msgs::ObjectsDetectionFeedback feedback; // feedback to the action, see the ObjectsDetection action message
    detection_msgs::ObjectsDetectionResult result;     // results to the action, see the ObjectsDetection action message

    double joint1, joint2;     // variables with the head joints variables
    std::set<int> observedIds; // set with the april tag ids already detected

    /**
     * @param center point where to look in the tiago acquired image
     */
    void lookToPoint(cv::Point2d center);

    /** point the table with the head by using its color
     */
    void pointTable();

    /** point colored object with the head by using its color
     */
    void pointColor();

    /** point a given area in the image (it will point to the center of the area)
     */
    void pointArea(cv::Rect colorizedArea);

    /** move the head down a little
     */
    void moveHeadDown();

    /** start the tracking of the collision objects
     */
    void pointObstacleTags();

    /** reset head position to the position where the table is well localized
     */
    void resetPosition();

public:
    /** constructor
     */
    TableObjectsDetectionServer();

    /** callback that is executed when an action arrives
     */
    void startDetection(const detection_msgs::ObjectsDetectionGoalConstPtr &goal);
};

#endif // TABLEOBJECTSDETECTIONSERVER_H