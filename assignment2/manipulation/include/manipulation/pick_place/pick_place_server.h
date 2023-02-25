#ifndef PICKPLACESERVER_H
#define PICKPLACESERVER_H

// ROS
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// GEOMETRY_MSGS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

// GAZEBO ROS LINK
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// CONSTANTS
#include <manipulation/constants/constants.h>

// ACTION MESSAGE
#include <manipulation_msgs/PickPlaceAction.h>

/**
 * This Class is used for manipualting the Tiago's components
 * @author : Riccardo Rampon, Francesco Caldivezzi
 */
class PickPlaceServer
{
private:
    actionlib::SimpleActionServer<manipulation_msgs::PickPlaceAction> server;
    manipulation_msgs::PickPlaceFeedback feedback;
    manipulation_msgs::PickPlaceResult result;
    manipulation_msgs::PickPlaceGoal goal;

    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceGripper;
    std::map<int, geometry_msgs::Pose> detectionsPosesIds;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;

    /**
     * This function moves the arm to a default pose.
     * @return True if the procedure ended correctly
     */
    bool moveToDefaultPose();

    /**
     * This function open or closes the gripper depending on the parameter mode.
     * @param mode : Either Open or Close
     * @return True if the procedure ended correctly
     */
    bool setGripper(std::string mode);

    /**
     * This function sets up the planning scene.
     * @param mode : Either PICK (0) or PLACE (1)
     */
    void setupPlannigScene(int mode);

    /**
     * This function deletes the current planning scene and the collision objects inside the collisionObjects vector
     */
    void unSetupPlanningScene();

    /**
     * This function is used to pick the object defined with a certain id.
     * @param pose : The pose of the object to pick (top if Red or Blue, COM if green)
     * @param id : The id of the object to pick
     * @param offset : The offset of the object to pick
     * @return True if the procedure ended correctly
     */
    bool moveToPosePick(geometry_msgs::Pose &pose, int id, double offset);

    /**
     * This function is used to place the object defined with a certain id.
     * @param pose : The pose of the top part of the table where to place the object.
     * @param id : The id of the object to place
     * @param offset : The offset of the object to place
     * @return True if the procedure ended correctly
     */
    bool moveToPosePlace(geometry_msgs::Pose &pose, int id, double offset);

    /**
     * This function removed from the planning scene a collision object.
     * @param name : The name of the collision object to remove
     */
    void removeCollisionObject(std::string name);

    /**
     * This function virtual attach or detach of an object to another
     * @param modelName1 : Name of the model of the first object
     * @param linkName1 : Name of the link of the first object
     * @param modelName2 : Name of the model of the second object
     * @param linkName2 : Name of the link of the second object
     * @param mode : either "attach" or "detach"
     * @return True if the procedure ended correctly
     */
    bool virtualAttachDetach(std::string modelName1, std::string linkName1, std::string modelName2, std::string linkName2, std::string mode);

    // DEBUG FUNCTIONS
    /**
     * This function publish the planning scene in order to visualize it into Rviz.
     */
    void publishPlanningScene();

public:
    /**
     * Constructor of the class
     * @param node : The node of the server
     */
    PickPlaceServer(ros::NodeHandle node);

    /**
     * Callback for the pick and place
     * @param goal : Goal of pick and place
     */
    void callback(const manipulation_msgs::PickPlaceGoalConstPtr &goal);
};

#endif // PICKPLACESERVER_H