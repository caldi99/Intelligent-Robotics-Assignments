#ifndef TIAGO_SERVER_COMMANDS_H
#define TIAGO_SERVER_COMMANDS_H

// ROS
#include <ros/ros.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// WALL_DETECTOR
#include <detection/wall_detector/wall_detector.h>

// MOVE_BASE
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

// CUSTOM ACTION
#include <navigation_commands_ros/MoveDetectAction.h>

// GEOMETRY_MSGS
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// SENSOR_MSGS
#include <sensor_msgs/LaserScan.h>

// STL
#include <list>

// DETECTION
#include <detection/obstacle_detection/point.h>
#include <detection/obstacle_detection/circle.h>
#include <detection/obstacle_detection/obstacle_extractor.h>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * This class implement a simple action server that mangaes the varius behaviours of the robot.
 * @author : Riccardo Rampon, Manuel Barusco
 */
class TiagoServerCommands
{

private:

    ros::NodeHandle nh;                                                              // Node for managing the action server
    actionlib::SimpleActionServer<navigation_commands_ros::MoveDetectAction> server; // The action server
    MoveBaseClient client;                                                           // Client to the Navigation Stack
    navigation_commands_ros::MoveDetectFeedback feedback;                            // Feedback message
    navigation_commands_ros::MoveDetectResult result;                                // Result message
    navigation_commands_ros::MoveDetectGoal goal;                                    // Goal message

    ros::Subscriber sub;
    ros::Publisher pub;
    const float FORWARD_SPEED_MPS = 5.0;
    const float ANGULAR_SPEED_RPS = 0.0;

    /**
     * This function publish the velocity vector for the robot with the topic "/mobile_base_controller/cmd_vel"
     */
    void setVelocityVector();

    /**
     * This function implements the navigation of the robot when it is in a narrow space
     */
    void doNavigationNarrowSpaces();

    /**
     * This function implements the navigation of the robot whenever the robot is no more in a narrow space
     * @param goal : The goal position to reach
     */
    void doNavigationMoveBase(const navigation_commands_ros::MoveDetectGoalConstPtr& goal);

    /**
     * This function implements the detection of the robot
     */
    void doDetection();

public:

    /**
     * Constructor of the class
     * @param serverName : The name of the server
     */
    TiagoServerCommands(const std::string serverName);

    /**
     * This function is the callback called when a goal is sent to the server
     * @param goal : The goal that the robot need to achieve
    */
    void navAndDetectCallback(const navigation_commands_ros::MoveDetectGoalConstPtr& goal);

};

#endif // TIAGO_SERVER_COMMANDS_H