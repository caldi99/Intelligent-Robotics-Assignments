#include <manipulation/pick_place/pick_place_server.h>

PickPlaceServer::PickPlaceServer(ros::NodeHandle node) : server(node, manipulation::TOPIC_PICK_PLACE, boost::bind(&PickPlaceServer::callback, this, _1), false),
                                                         moveGroupInterfaceArm(manipulation::INTERFACE_ARM),
                                                         moveGroupInterfaceGripper(manipulation::INTERFACE_GRIPPER)
{
    server.start();
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer PickPlace");
}

void PickPlaceServer::callback(const manipulation_msgs::PickPlaceGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer callback");

    // Variables
    bool success = true;
    geometry_msgs::Pose pickPose;
    geometry_msgs::Pose placePose;

    // Populate map of detections
    for (int i = 0; i < goal->poses.size(); i++)
        this->detectionsPosesIds.insert({goal->poses.at(i).id, goal->poses.at(i).pose});

    // Setup Planning Scene
    setupPlannigScene(goal->mode);

    // TODO: UNCCOMENT THIS TO SEE IT IN RVIZ
    // publishPlanningScene();

    if (goal->mode == 0)
    {
        // Get pickPose
        pickPose = detectionsPosesIds.at(goal->id_object_pick_place);

        // Move to default pose
        success &= moveToDefaultPose();

        if (success)
        {
            feedback.status = "Default Pose Reached";
            server.publishFeedback(feedback);
            ROS_INFO_STREAM("\t Feedback : " << feedback.status);

            // Open gripper
            success &= setGripper(manipulation::OPEN_GRIPPER_MODALITY);

            if (success)
            {
                feedback.status = "Open Gripper";
                server.publishFeedback(feedback);
                ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                // Approach the object
                success &= moveToPosePick(pickPose, goal->id_object_pick_place, manipulation::OFFSET_APPROACH_DEPART);

                if (success)
                {
                    feedback.status = "Approach Position Reached";
                    server.publishFeedback(feedback);
                    ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                    // Move to the object
                    success &= moveToPosePick(pickPose, goal->id_object_pick_place, 0.0);

                    if (success)
                    {
                        feedback.status = "Position Object Reached";
                        server.publishFeedback(feedback);
                        ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                        // Attach virtually the object
                        switch (goal->id_object_pick_place)
                        {
                        case 1: // Blue hexagon
                            success &= virtualAttachDetach(manipulation::HEXAGON, manipulation::HEXAGON_LINK,
                                                           manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                           manipulation::ATTACH_MODALITY);
                            break;
                        case 2: // Green Triangle
                            success &= virtualAttachDetach(manipulation::TRIANGLE, manipulation::TRIANGLE_LINK,
                                                           manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                           manipulation::ATTACH_MODALITY);
                            break;
                        case 3: // Red Cube
                            success &= virtualAttachDetach(manipulation::CUBE, manipulation::CUBE_LINK,
                                                           manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                           manipulation::ATTACH_MODALITY);
                            break;
                        }

                        if (success)
                        {
                            feedback.status = "Object Attached";
                            server.publishFeedback(feedback);
                            ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                            // Remove collision object to pick
                            switch (goal->id_object_pick_place)
                            {
                            case 1: // Blue hexagon
                                removeCollisionObject(manipulation::HEXAGON_COLLISION_OBJECT);
                                break;
                            case 2: // Green Triangle
                                removeCollisionObject(manipulation::TRIANGLE_COLLISION_OBJECT);
                                break;
                            case 3: // Red Cube
                                removeCollisionObject(manipulation::CUBE_COLLISION_OBJECT);
                                break;
                            }

                            feedback.status = "Collision Object Removed";
                            server.publishFeedback(feedback);
                            ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                            // Close gripper
                            success &= setGripper(manipulation::CLOSE_GRIPPER_MODALITY);

                            if (success)
                            {
                                feedback.status = "Close Gripper";
                                server.publishFeedback(feedback);
                                ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                                // Move back to offset position
                                success &= moveToPosePick(pickPose, goal->id_object_pick_place, manipulation::OFFSET_APPROACH_DEPART); 

                                if (success)
                                {
                                    feedback.status = "Depart Position Reached";
                                    server.publishFeedback(feedback);
                                    ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                                    // Move back to default Pose
                                    success &= moveToDefaultPose();
                                    if (success)
                                    {
                                        feedback.status = "Default Pose Reached";
                                        server.publishFeedback(feedback);
                                        ROS_INFO_STREAM("\t Feedback : " << feedback.status);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Get place Pose
        placePose = detectionsPosesIds.at(-1);

        // Correct the pose
        placePose.position.z += manipulation::PLACE_TABLE_HEIGHT;

        // Move to approach position
        success &= moveToPosePlace(placePose, goal->id_object_pick_place, manipulation::OFFSET_APPROACH_DEPART);

        if (success)
        {
            feedback.status = "Approach Position Reached";
            server.publishFeedback(feedback);
            ROS_INFO_STREAM("\t Feedback : " << feedback.status);

            // Move to place position
            success &= moveToPosePlace(placePose, goal->id_object_pick_place, 0.0);
            if (success)
            {
                feedback.status = "Position Object Reached";
                server.publishFeedback(feedback);
                ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                // Detach virtually the object
                switch (goal->id_object_pick_place)
                {
                case 1: // Blue hexagon
                    success &= virtualAttachDetach(manipulation::HEXAGON, manipulation::HEXAGON_LINK,
                                                   manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                   manipulation::DETACH_MODALITY);
                    break;
                case 2: // Green Triangle
                    success &= virtualAttachDetach(manipulation::TRIANGLE, manipulation::TRIANGLE_LINK,
                                                   manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                   manipulation::DETACH_MODALITY);
                    break;
                case 3: // Red Cube
                    success &= virtualAttachDetach(manipulation::CUBE, manipulation::CUBE_LINK,
                                                   manipulation::TIAGO, manipulation::ARM_7_LINK,
                                                   manipulation::DETACH_MODALITY);
                    break;
                }

                if (success)
                {
                    feedback.status = "Object Detached";
                    server.publishFeedback(feedback);
                    ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                    // Open the gripper
                    success &= setGripper(manipulation::OPEN_GRIPPER_MODALITY);

                    if (success)
                    {

                        feedback.status = "Open Gripper";
                        server.publishFeedback(feedback);
                        ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                        // Move to depart position
                        success &= moveToPosePlace(placePose, goal->id_object_pick_place, manipulation::OFFSET_APPROACH_DEPART);

                        if (success)
                        {
                            feedback.status = "Depart Position Reached";
                            server.publishFeedback(feedback);
                            ROS_INFO_STREAM("\t Feedback : " << feedback.status);

                            // Move back to default Pose
                            success &= moveToDefaultPose();
                            if (success)
                            {
                                feedback.status = "Default Pose Reached";
                                server.publishFeedback(feedback);
                                ROS_INFO_STREAM("\t Feedback : " << feedback.status);
                            }
                        }
                    }
                }
            }
        }
    }

    // Unsetup planning scene
    unSetupPlanningScene();

    // Remove current detection poses
    detectionsPosesIds.clear();

    // Set success
    this->result.state = success;
    if (success)
        server.setSucceeded(this->result);
    else
        server.setAborted(this->result);
}

void PickPlaceServer::setupPlannigScene(int mode)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer setupPlannigScene");

    // Varaibles
    std::map<int, geometry_msgs::Pose>::iterator it;
    geometry_msgs::Pose currentPose;
    int currentId;

    for (it = this->detectionsPosesIds.begin(); it != this->detectionsPosesIds.end(); it++)
    {
        currentId = it->first;
        currentPose = it->second;
        moveit_msgs::CollisionObject collisionObject;

        // frame_id
        collisionObject.header.frame_id = manipulation::BASE_LINK_REFERENCE_FRAME;

        // operation
        collisionObject.operation = collisionObject.ADD;

        // primitives resize
        collisionObject.primitives.resize(1);

        // primitives dimensions resize
        collisionObject.primitives.at(0).dimensions.resize(3);

        // primitive_poses
        collisionObject.primitive_poses.resize(1);
        collisionObject.primitive_poses.at(0).orientation = currentPose.orientation;

        if (mode == 0)
        {
            if (currentId < 0)
            {
                // id
                collisionObject.id = manipulation::PICK_TABLE_COLLISION_OBJECT;

                // primitives
                collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                collisionObject.primitives.at(0).dimensions.at(0) = manipulation::PICK_TABLE_LENGTH;
                collisionObject.primitives.at(0).dimensions.at(1) = manipulation::PICK_TABLE_WIDTH;
                collisionObject.primitives.at(0).dimensions.at(2) = manipulation::PICK_TABLE_HEIGHT;

                // primitive_poses position
                currentPose.position.z += (manipulation::PICK_TABLE_HEIGHT / 2.0);
                collisionObject.primitive_poses.at(0).position = currentPose.position;
            }
            else if (currentId <= 3)
            {
                switch (currentId)
                {
                case 1:
                    // id
                    collisionObject.id = manipulation::HEXAGON_COLLISION_OBJECT;

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
                    collisionObject.primitives.at(0).dimensions.at(0) = manipulation::HEXAGON_HEIGHT;
                    collisionObject.primitives.at(0).dimensions.at(1) = (manipulation::HEXAGON_DIAMETER / 2.0);

                    // primitive_poses position
                    currentPose.position.z -= (manipulation::HEXAGON_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                case 2:
                    // id
                    collisionObject.id = manipulation::TRIANGLE_COLLISION_OBJECT;

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                    collisionObject.primitives.at(0).dimensions.at(0) = manipulation::TRIANGLE_LENGTH;
                    collisionObject.primitives.at(0).dimensions.at(1) = manipulation::TRIANGLE_WIDTH;
                    collisionObject.primitives.at(0).dimensions.at(2) = manipulation::TRIANGLE_HEIGHT;

                    // primitive_poses position
                    currentPose.position.z -= (manipulation::TRIANGLE_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                case 3:
                    // id
                    collisionObject.id = manipulation::CUBE_COLLISION_OBJECT;

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                    collisionObject.primitives.at(0).dimensions.at(0) = manipulation::CUBE_LENGTH;
                    collisionObject.primitives.at(0).dimensions.at(1) = manipulation::CUBE_WIDTH;
                    collisionObject.primitives.at(0).dimensions.at(2) = manipulation::CUBE_HEIGHT;

                    // primitive_poses position
                    currentPose.position.z -= (manipulation::CUBE_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                }
            }
            else
            {
                // id
                collisionObject.id = manipulation::COLLISION_OBJECTS_COLLISION_OBJECT + std::to_string(currentId);

                // primitives
                collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
                collisionObject.primitives.at(0).dimensions.at(0) = manipulation::COLLISION_HEIGHT;
                collisionObject.primitives.at(0).dimensions.at(1) = (manipulation::COLLISION_DIAMETER / 2.0);

                // primitive_poses position
                currentPose.position.z -= (manipulation::COLLISION_HEIGHT / 2.0);
                collisionObject.primitive_poses.at(0).position = currentPose.position;
            }
        }
        else
        {
            // id
            collisionObject.id = manipulation::PLACE_TABLE_COLLISION_OBJECT;

            // primitives
            collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
            collisionObject.primitives.at(0).dimensions.at(0) = manipulation::PLACE_TABLE_HEIGHT;
            collisionObject.primitives.at(0).dimensions.at(1) = manipulation::PLACE_TABLE_RADIUS;

            // primitive_poses position
            currentPose.position.z += (manipulation::PLACE_TABLE_HEIGHT / 2.0);
            collisionObject.primitive_poses.at(0).position = currentPose.position;
        }

        collisionObjects.push_back(collisionObject);
    }
    planningSceneInterface.applyCollisionObjects(collisionObjects);
} // setupPlannigScene

void PickPlaceServer::unSetupPlanningScene()
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer unSetupPlanningScene");

    // Variables
    std::vector<std::string> collisionObjectsIds;

    for (moveit_msgs::CollisionObject collisionObject : collisionObjects)
    {
        collisionObjectsIds.push_back(collisionObject.id);
    }

    // Delete planning scene
    planningSceneInterface.removeCollisionObjects(collisionObjectsIds);

    // Clear current collision objects
    collisionObjects.clear();
} // unSetupPlanningScene

bool PickPlaceServer::moveToDefaultPose()
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer moveToDefaultPose");

    // Variables declaration
    std::map<std::string, double> targetPosition;
    std::vector<std::string> torsoArmJointNames;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    // Set target pose
    targetPosition.insert({manipulation::ARM_1_JOINT, manipulation::ARM_1_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_2_JOINT, manipulation::ARM_2_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_3_JOINT, manipulation::ARM_3_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_4_JOINT, manipulation::ARM_4_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_5_JOINT, manipulation::ARM_5_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_6_JOINT, manipulation::ARM_6_JOINT_VALUE_DEFAULT});
    targetPosition.insert({manipulation::ARM_7_JOINT, manipulation::ARM_7_JOINT_VALUE_DEFAULT});

    // Get tors arm joint names
    torsoArmJointNames = moveGroupInterfaceArm.getJoints();

    // Set group interface
    moveGroupInterfaceArm.setPlannerId(manipulation::PLANNER_ID);
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(manipulation::MAX_VELOCITY_SCALING_FACTOR);
    moveGroupInterfaceArm.setPlanningTime(manipulation::PLANNING_TIME);

    for (int i = 0; i < torsoArmJointNames.size(); i++)
        if (targetPosition.count(torsoArmJointNames.at(i)) > 0)
            moveGroupInterfaceArm.setJointValueTarget(torsoArmJointNames.at(i), targetPosition.at(torsoArmJointNames.at(i)));

    if (moveGroupInterfaceArm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("\t Plan found!! ");
        success = true;
        moveGroupInterfaceArm.move();
    }
    return success;
} // moveToDefaultPose

bool PickPlaceServer::virtualAttachDetach(std::string modelName1, std::string linkName1, std::string modelName2, std::string linkName2, std::string mode)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer virtualAttachDetach");

    // Variables
    ros::NodeHandle n;
    ros::ServiceClient client;
    gazebo_ros_link_attacher::Attach service;
    bool success = false;
    std::string topic;

    // Define service message
    service.request.model_name_1 = modelName1;
    service.request.link_name_1 = linkName1;
    service.request.model_name_2 = modelName2;
    service.request.link_name_2 = linkName2;

    // Select correct topic
    if (mode.compare(manipulation::ATTACH_MODALITY) == 0)
        topic = manipulation::TOPIC_LINK_ATTACHER_NODE + "/" + manipulation::ATTACH_MODALITY;
    else
        topic = manipulation::TOPIC_LINK_ATTACHER_NODE + "/" + manipulation::DETACH_MODALITY;

    // Create client
    client = n.serviceClient<gazebo_ros_link_attacher::Attach>(topic);

    // Call server
    if (client.call(service))
    {
        success = true;
        ROS_INFO_STREAM("Attach success");
    }
    else
        ROS_ERROR_STREAM("Failed to call service '/link_attacher_node/attach'");

    return success;
} // virtualAttachDetach

bool PickPlaceServer::moveToPosePick(geometry_msgs::Pose &pose, int id, double offset)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer moveToPosePick");

    // Variables
    geometry_msgs::PoseStamped targetPose;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    tf2::Quaternion start;
    tf2::Quaternion rotation;
    tf2::Quaternion end;

    // Define reference frame of the pose
    targetPose.header.frame_id = manipulation::BASE_LINK_REFERENCE_FRAME;

    // Set orientation
    targetPose.pose.orientation.w = 1.0;
    targetPose.pose.orientation.z = 0.0;
    targetPose.pose.orientation.y = 0.0;
    targetPose.pose.orientation.x = 0.0;

    switch (id)
    {
    case 1: // Blue

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(M_PI_2, 0.0, 0.0); // Frontal pick
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
        {
            targetPose.pose.position.x -= (manipulation::GRIPPER_LENGTH + (manipulation::HEXAGON_DIAMETER / 2.0) + offset);
            targetPose.pose.position.z -= (manipulation::HEXAGON_HEIGHT / 3.0);
        }
        else
        {
            targetPose.pose.position.x -= (manipulation::GRIPPER_LENGTH - (manipulation::HEXAGON_DIAMETER / 2.0)); //(manipulation::GRIPPER_LENGTH - (manipulation::HEXAGON_DIAMETER / 2.0));
            targetPose.pose.position.z -= (manipulation::HEXAGON_HEIGHT / 3.0); 
        } // if-else
        break;

    case 2: // Green

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(0.0, M_PI_2, -M_PI_2); // Top pick
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::TRIANGLE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += manipulation::GRIPPER_LENGTH;
        break;
    case 3: // Red

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(0.0, M_PI_2, -M_PI_2);
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + offset);
        else
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH - (manipulation::CUBE_HEIGHT / 2.0));

        break;
    } // switch-case

    ROS_INFO_STREAM("\t Pose : " << targetPose.pose);

    // TODO : UNCOMMENT THIS CODE ONLY IF YOU NEED TO SEE THE grasp_pose
    /*ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>(manipulation::TOPIC_MY_POSE, 1);
    ros::WallDuration sleep_t(0.5);
    while (pub.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    for (int i = 0; i < 1000; i++)
        pub.publish(targetPose);*/

    // Set Parameters interface
    moveGroupInterfaceArm.setPlannerId(manipulation::PLANNER_ID);
    moveGroupInterfaceArm.setPoseReferenceFrame(manipulation::BASE_LINK_REFERENCE_FRAME);
    moveGroupInterfaceArm.setPoseTarget(targetPose, manipulation::ARM_TOOL_LINK);
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(manipulation::MAX_VELOCITY_SCALING_FACTOR);
    moveGroupInterfaceArm.setPlanningTime(manipulation::PLANNING_TIME);
    //moveGroupInterfaceArm.setSupportSurfaceName(manipulation::PICK_TABLE_COLLISION_OBJECT);

    if (moveGroupInterfaceArm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        success = true;
        moveGroupInterfaceArm.move();
    }

    return success;
} // moveToPosePick

bool PickPlaceServer::setGripper(std::string mode)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer setGripper");

    // Declare variables
    std::map<std::string, double> targetPosition;
    std::vector<std::string> torsoGripperJointNames;
    moveit::planning_interface::MoveGroupInterface::Plan planGripper;
    bool success = false;

    // Set correct value of gripper depending on modality
    if (mode.compare(manipulation::OPEN_GRIPPER_MODALITY) == 0)
    {
        ROS_INFO_STREAM("\t Open gripper");
        targetPosition.insert({manipulation::GRIPPER_LEFT_FINGER_JOINT, manipulation::OPEN_GRIPPER_MAX_SIZE});
        targetPosition.insert({manipulation::GRIPPER_RIGHT_FINGER_JOINT, manipulation::OPEN_GRIPPER_MAX_SIZE});
    }
    else if (mode.compare(manipulation::CLOSE_GRIPPER_MODALITY) == 0)
    {
        ROS_INFO_STREAM("\t Close gripper!");
        targetPosition.insert({manipulation::GRIPPER_LEFT_FINGER_JOINT, 0.0});
        targetPosition.insert({manipulation::GRIPPER_RIGHT_FINGER_JOINT, 0.0});
    }
    else
    {
        ROS_ERROR_STREAM("Set gripper modality properly!");
    }

    // Get joint names
    torsoGripperJointNames = moveGroupInterfaceGripper.getJoints();

    // Set interface params
    moveGroupInterfaceGripper.setPlannerId(manipulation::PLANNER_ID);
    moveGroupInterfaceGripper.setStartStateToCurrentState();
    moveGroupInterfaceGripper.setMaxVelocityScalingFactor(manipulation::MAX_VELOCITY_SCALING_FACTOR);
    moveGroupInterfaceGripper.setPlanningTime(manipulation::PLANNING_TIME);

    for (int i = 1; i < torsoGripperJointNames.size(); i++)
        if (targetPosition.count(torsoGripperJointNames.at(i)) > 0)
            moveGroupInterfaceGripper.setJointValueTarget(torsoGripperJointNames.at(i), targetPosition.at(torsoGripperJointNames.at(i)));

    if (moveGroupInterfaceGripper.plan(planGripper) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("Moving gripper " << mode);
        success = true;
        moveGroupInterfaceGripper.move();
    }

    return success;
} // setGripper

void PickPlaceServer::removeCollisionObject(std::string name)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer removeCollisionObject");

    // Variables
    std::vector<std::string> collisionObjectsIds;

    // Remove from planning scene
    collisionObjectsIds.push_back(name);
    planningSceneInterface.removeCollisionObjects(collisionObjectsIds);

    // Remove from vector of collision objects
    int index = 0;
    for (int i = 0; i < collisionObjects.size(); i++)
    {
        if (collisionObjects.at(i).id.compare(name) == 0)
        {
            index = i;
            break;
        }
    }
    collisionObjects.erase(collisionObjects.begin() + index);

} // removeCollisionObject

void PickPlaceServer::publishPlanningScene()
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer publishPlanningScene");

    // Variables
    ros::NodeHandle node;
    ros::Publisher publisher;
    std::map<std::string, moveit_msgs::CollisionObject> map;
    ros::WallDuration sleep_t(0.5);
    moveit_msgs::PlanningScene planningSceneMessage;
    std::map<std::string, moveit_msgs::CollisionObject>::iterator it;

    // Get publisher
    publisher = node.advertise<moveit_msgs::PlanningScene>(manipulation::TOPIC_PLANNING_SCENE, 1);

    // Wait that a client connects (rviz)
    while (publisher.getNumSubscribers() < 1)
        sleep_t.sleep();

    // Get the objects
    map = planningSceneInterface.getObjects();

    // Add collision objects
    for (it = map.begin(); it != map.end(); it++)
        planningSceneMessage.world.collision_objects.push_back(it->second);
    planningSceneMessage.is_diff = true;

    // Publish scene
    publisher.publish(planningSceneMessage);
} // publishPlanningScene

bool PickPlaceServer::moveToPosePlace(geometry_msgs::Pose &pose, int id, double offset)
{
    ROS_INFO_STREAM("(Server) Manipulation PickPlaceServer moveToPosePlace");

    // Variables
    geometry_msgs::PoseStamped targetPose;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    tf2::Quaternion start;
    tf2::Quaternion rotation;
    tf2::Quaternion end;

    // Define reference frame of the pose
    targetPose.header.frame_id = manipulation::BASE_LINK_REFERENCE_FRAME;

    // Set orientation
    targetPose.pose.orientation.w = 1.0;
    targetPose.pose.orientation.z = 0.0;
    targetPose.pose.orientation.y = 0.0;
    targetPose.pose.orientation.x = 0.0;

    // Convert a PoseStamped to Quaternion
    tf2::convert(targetPose.pose.orientation, start);
    rotation.setRPY(0.0, M_PI_2, -M_PI_2); // Top place
    end = (rotation * start);
    end = end.normalize();
    // Set orientation
    tf2::convert(end, targetPose.pose.orientation);

    // Set position
    targetPose.pose.position = pose.position;
    switch (id)
    {
    case 1: // Blue
        targetPose.pose.position.y -= 0.05; 
        targetPose.pose.position.x += 0.1; 
        if (offset != 0.0)
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::HEXAGON_DIAMETER / 2.0) + offset);
        else
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::HEXAGON_DIAMETER / 2.0));
        break;

    case 2: // Green
        // targetPose.pose.position.x -= 0.2;
        if (offset != 0.0)
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::TRIANGLE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += manipulation::GRIPPER_LENGTH + (manipulation::TRIANGLE_HEIGHT / 2.0);
        break;
    case 3: // Red
        if (offset != 0.0)
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::CUBE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += (manipulation::GRIPPER_LENGTH + (manipulation::CUBE_HEIGHT / 2.0));
        break;
    } // switch-case

    ROS_INFO_STREAM("\t Pose : " << targetPose.pose);

    // TODO : UNCOMMENT THIS CODE ONLY IF YOU NEED TO SEE THE grasp_pose
    /*ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>(manipulation::TOPIC_MY_POSE, 1);
    ros::WallDuration sleep_t(0.5);
    while (pub.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    for (int i = 0; i < 1000; i++)
        pub.publish(targetPose);*/

    // Set Parameters interface
    moveGroupInterfaceArm.setPlannerId(manipulation::PLANNER_ID);
    moveGroupInterfaceArm.setPoseReferenceFrame(manipulation::BASE_LINK_REFERENCE_FRAME);
    moveGroupInterfaceArm.setPoseTarget(targetPose, manipulation::ARM_TOOL_LINK);
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(manipulation::MAX_VELOCITY_SCALING_FACTOR);
    moveGroupInterfaceArm.setPlanningTime(manipulation::PLANNING_TIME);
    //moveGroupInterfaceArm.setSupportSurfaceName(manipulation::PICK_TABLE_COLLISION_OBJECT);

    if (moveGroupInterfaceArm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        success = true;
        moveGroupInterfaceArm.move();
    }

    return success;
}
