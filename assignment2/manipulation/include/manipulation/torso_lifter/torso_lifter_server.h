#ifndef TORSOLIFTERSERVER_H
#define TORSOLIFTERSERVER_H

//ROS
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// SERVICE MESSAGE
#include <manipulation_msgs/TorsoLifter.h>

//CONSTANTS
#include <manipulation/constants/constants.h>

/**
 * This class is used to move up and down the torso of the robot
 * @author Riccardo Rampon
*/
class TorsoLifterServer
{
private:
    ros::NodeHandle node;
    ros::ServiceServer server;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm;

    /**
     *
     * @param req : The value of the joint with which lifting the torso
     * @param res : True if lifting finished
     * @return True if everything was ok
     */
    bool sendResponse(manipulation_msgs::TorsoLifter::Request &req, manipulation_msgs::TorsoLifter::Response &res);

public:
    /**
     * Create a node that will manage the lift of the torso of the robot
     * @param node : The node of the server
     */
    TorsoLifterServer(ros::NodeHandle node);
};

#endif // TORSOLIFTERSERVER_H