#include <manipulation/torso_lifter/torso_lifter_server.h>

TorsoLifterServer::TorsoLifterServer(ros::NodeHandle node) : moveGroupInterfaceArm(manipulation::INTERFACE_ARM_TORSO)
{
    ROS_INFO_STREAM("(Server) manipulation TorsoLifterServer TorsoLifterServer");
    this->node = node;
    this->server = this->node.advertiseService(manipulation::TOPIC_TORSO_LIFTER, &TorsoLifterServer::sendResponse, this);
}

bool TorsoLifterServer::sendResponse(manipulation_msgs::TorsoLifter::Request &req, manipulation_msgs::TorsoLifter::Response &res)
{
    ROS_INFO_STREAM("(Server) manipulation TorsoLifterServer sendResponse");
    moveit::planning_interface::MoveGroupInterface::Plan planTorso;

    // Set planner ID
    moveGroupInterfaceArm.setPlannerId(manipulation::PLANNER_ID);

    // Set state to current state
    moveGroupInterfaceArm.setStartStateToCurrentState();

    // Max velocity scaling factor
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(manipulation::MAX_VELOCITY_SCALING_FACTOR);

    // Get the value of the joints
    std::vector<double> jointValues = moveGroupInterfaceArm.getCurrentJointValues();

    // Get the name of the joints
    std::vector<std::string> jointNames = moveGroupInterfaceArm.getJoints();

    // Get the correct index of the joint to move
    int index = std::distance(jointNames.begin(), std::find(jointNames.begin(), jointNames.end(), manipulation::TORSO_LIFT_JOINT));

    // Check if index is ok
    bool success = false;
    if (index >= 0 && index < jointValues.size())
    {
        // Set the value of the joints
        jointValues.at(index) = req.joint_value;
        moveGroupInterfaceArm.setJointValueTarget(jointValues);

        // Set planning time
        moveGroupInterfaceArm.setPlanningTime(manipulation::PLANNING_TIME);

        // Make the plan
        if (moveGroupInterfaceArm.plan(planTorso) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO_STREAM("LIFTING ROBOT");

            // Set success to true
            success = true;

            // Move the robot
            moveGroupInterfaceArm.move();
        }
    }
    res.state = success;
    return success;
}