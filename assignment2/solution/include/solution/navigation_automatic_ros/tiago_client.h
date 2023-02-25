#ifndef TIAGOCLIENT_H
#define TIAGOCLIENT_H

// ROS
#include <ros/ros.h>

//CONSTANTS
#include <solution/constants/constants.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// CUSTOM ACTION
#include <navigation_automatic_ros/MoveDetectAction.h>

/**
 * This class is a client for the TiagoServer
 * @author Manuel Barusco
*/
class TiagoClient
{
private:
    actionlib::SimpleActionClient<navigation_automatic_ros::MoveDetectAction> client;
    double x;
    double y;
    double orZ;
    double orW;

    /**
     * Callback for action done
     * @param state : final state
     * @param result_ptr : boost pointer to the final result of the move and detect action
     */
    void doneCb(const actionlib::SimpleClientGoalState &state, const navigation_automatic_ros::MoveDetectResultConstPtr &result_ptr);

    /**
     * Callback for when robot starts the task
     */
    void activeCb();

    /**
     * Callback for feedback messages
     * @param feedback_ptr : pointer to the MoveDetect Feedback messages
     */
    void feedbackCb(const navigation_automatic_ros::MoveDetectFeedbackConstPtr &feedback_ptr);

public:
    /**
     * Constructor of the class
     * @param x : x position parameter
     * @param y : y position parameter
     * @param orZ : z orientation parameter
     * @param orW : w orientation parameter
     * @param serverName : name of the server
     */
    TiagoClient(double x = 0.0, double y = 0.0, double orZ = 0.0, double orW = 1.0, std::string serverName = solution::TOPIC_TIAGO_SERVER);

    /**
     * Method for sending the goal and start the task
     */
    void sendGoal();
};

#endif // TIAGOCLIENT_H