#ifndef TABLEOBJECTSDETECTIONCLIENT_H
#define TABLEOBJECTSDETECTIONCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

// SERVICE FILE
#include <detection_msgs/ObjectsDetectionAction.h>
#include <detection_msgs/Detection.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

/**
 * This class is a client for the TableDetectionServer
 * @author Manuel Barusco
 */
class TableDetectionClient
{
public:
    /**
     * Constructor of the class
     * @param node : The node of the client
     * @param colorID : color to search on the table
     */
    TableDetectionClient(int colorID, std::string serverName);

    /**
     * Callback for action done
     * @param state : final state
     * @param result_ptr : boost pointer to the final result of the table and objects detection action
     */
    void doneCb(const actionlib::SimpleClientGoalState &state, const detection_msgs::ObjectsDetectionResultConstPtr &result_ptr);

    /**
     * Callback for when the action is starting
     */
    void activeCb();

    /**
     * Callback for feedback messages
     * @param feedback_ptr : pointer to the ObjectsDetection Feedback messages
     */
    void feedbackCb(const detection_msgs::ObjectsDetectionFeedbackConstPtr &feedback_ptr);

    /**
     * Method for sending the goal and start the task
     */
    std::vector<detection_msgs::Detection> sendGoal();

private:
    int colorID;
    ros::NodeHandle node;
    std::vector<detection_msgs::Detection> detections;
    actionlib::SimpleActionClient<detection_msgs::ObjectsDetectionAction> client;
};

#endif // TABLEOBJECTSDETECTIONCLIENT_H