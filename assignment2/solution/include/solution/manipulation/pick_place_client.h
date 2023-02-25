#ifndef PICKPLACECLIENT_H
#define PICKPLACECLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ACTION
#include <manipulation_msgs/PickPlaceAction.h>

// DETECTION MESSAGE
#include <detection_msgs/Detection.h>

/**
 * This class is a client for the PickPlaceServer
 * @author Francesco Caldivezzi, Riccardo Rampon
*/
class PickPlaceClient
{
public:

    /**
     * Constructor of the class
    */
    PickPlaceClient(unsigned char mode, std::vector<detection_msgs::Detection> poses, int idObjectPick);

    /**
     * Method for sending the goal and start the task
     */
    void sendGoal();    

private:
    /**
     * Callback for action done
     * @param state : final state
     * @param resultPtr : boost pointer to the final result of the pick place action
     */
    void doneCb(const actionlib::SimpleClientGoalState &state, const manipulation_msgs::PickPlaceResultConstPtr &resultPtr);

    /**
     * Callback for when robot starts the task
     */
    void activeCb();

    /**
     * Callback for feedback messages
     * @param feedbackPtr : pointer to the PickPlace Feedback messages
     */
    void feedbackCb(const manipulation_msgs::PickPlaceFeedbackConstPtr &feedbackPtr);

    // MEMBERS
    unsigned char mode;
    std::vector<detection_msgs::Detection> poses;
    int idObjectPick;
    actionlib::SimpleActionClient<manipulation_msgs::PickPlaceAction> client;
};

#endif // PICKPLACECLIENT_H