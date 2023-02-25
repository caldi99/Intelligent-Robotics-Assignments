#ifndef TIAGO_CLIENT_COMMANDS_H
#define TIAGO_CLIENT_COMMANDS_H

// ROS
#include <ros/ros.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// CUSTOM ACTION
#include <navigation_commands_ros/MoveDetectAction.h>

/**
 * This class implement a simple action client that sends the goal to the robot.
 * @author : Riccardo Rampon, Manuel Barusco
 */
class TiagoClientCommands{

    private:

        actionlib::SimpleActionClient<navigation_commands_ros::MoveDetectAction> client;
        float x;
        float y;
        float orZ;
        
        /** callback for action done
         * @param &state final state 
         * @param &result_ptr Boost Pointer to the final result of the move and detect action
        */
        void doneCb(const actionlib::SimpleClientGoalState &state, const navigation_commands_ros::MoveDetectResultConstPtr &result_ptr);
        
        //callback for when robot starts the task
        void activeCb();

        /** callback for feedback messages
         * @param &feedback_ptr pointer to the MoveDetect Feedback messages
        */
        void feedbackCb(const navigation_commands_ros::MoveDetectFeedbackConstPtr &feedback_ptr);

    public:

        /*  
        @param x coordinate x of the final pose (w.r.t. map ref. frame)
        @param y coordinate y of the final pose (w.r.t. map ref. frame)
        @param orZ orientation in the z axis of the final pose (w.r.t. map ref. frame)
        */
        TiagoClientCommands(float x, float y, float orZ, const std::string server_name);

        /* method for sending the goal and start the task */
        int sendGoal();

};//TiagoClientCommands

#endif // TIAGO_CLIENT_COMMANDS_H