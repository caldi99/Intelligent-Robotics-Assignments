#include <ros/ros.h>

// CLIENTS
#include <solution/tiago_iaslab_simulation/human_client.h>
#include <solution/navigation_automatic_ros/tiago_client.h>
#include <solution/manipulation/torso_lifter_client.h>
#include <solution/detection/obstacle_extractor_client.h>
#include <solution/detection/poses_transformer_client.h>
#include <solution/manipulation/pick_place_client.h>
#include <solution/detection/table_objects_detection_client.h>
#include <solution/manipulation/head_movement_client.h>
#include <solution/detection/pose_table_detector_client.h>

// TIAGO IASLAB SIMULATION
#include <tiago_iaslab_simulation/Objs.h>

// DETECTION
#include <detection_msgs/Detections.h>
#include <detection_msgs/Transform.h>
#include <detection_msgs/Circles.h>
#include <detection_msgs/ObjectsDetectionAction.h>
#include <detection_msgs/PoseTableDetector.h>

// MANIPULATION
#include <manipulation_msgs/TorsoLifter.h>

// SENSORS MSGS
#include <sensor_msgs/LaserScan.h>

// CONSTANTS
#include <solution/constants/constants.h>

// FUNCTIONS

/**
 * This function moves the robot to the specified location given by the parameters of this function
 * @param positionX : The x coordinate where to move the robot
 * @param positionY : The y coordinate where to move the robot
 * @param orientationZ : The z value for the orientation (quaternion)
 * @param orientationW : The w value for the orientation (quaternion)
 */
void moveRobotTo(double positionX, double positionY, double orientationZ, double orientationW);

/**
 * This function lifts the torso of the robot
 * @param node : A ros::NodeHandle node
 * @param mode : True if lift-up, false if lift-down
 */
void liftTorso(ros::NodeHandle node, bool mode);

/**
 * This function computes the pose of the pick rectangular table w.r.t. "base_link" reference frame
 * @param : A ros::NodeHandle node
 * @return The pose of the pick table ()
 */
geometry_msgs::Pose getPosePickTable(ros::NodeHandle node);

/**
 * This function move the head in the standard position
 * @param node : A ros::NodeHandle node
 */
void moveHead(ros::NodeHandle node);

/**
 * This function converts a pose from a source_reference frame to a target_reference frame
 * @param node : A ros::NodeHandle node
 * @param pose : The pose to be converted
 * @param sourceFrame : The source_reference frame
 * @param targetFrame : The target_reference frame
 * @return The pose in the target reference frame
 */
geometry_msgs::Pose convertPose(ros::NodeHandle node, geometry_msgs::Pose pose, std::string sourceFrame, std::string targetFrame);

/**
 * This function returns an array of circles where their coordinates are with respect to the "map" refernce frame
 * @param node : A ros::NodeHandle node
 * @return The array of circles
 */
std::vector<detection_msgs::Circle> detectCircles(ros::NodeHandle node);

/**
 * @author Francesco Caldivezzi
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM(solution::NODE_SOLUTION << " NODE STARTED");
    // Init ros
    ros::init(argc, argv, solution::NODE_SOLUTION);

    // Create node
    ros::NodeHandle node;

    // Declare varibale
    bool ready = true;
    bool allObjs = true;
    std::vector<int> ids;
    geometry_msgs::Pose pose;
    std::vector<geometry_msgs::Pose> poses;
    detection_msgs::PoseTableDetector poseTableDetectorMessage;
    geometry_msgs::Pose posePickTable;
    geometry_msgs::Pose posePlaceTableGlobal;
    geometry_msgs::Pose posePlaceTable;
    std::vector<detection_msgs::Detection> detectionPoses;
    detection_msgs::Detection detection;

    // Client declarations
    HumanClient *humanClient;
    PickPlaceClient *pickPlaceClient;
    PoseTableDetectorClient *poseTableDetectorClient;
    TableDetectionClient *tableDetectionClient;

    // TODO CHANGE THIS
    /*
    ids.push_back(1); 
    ids.push_back(2);
    ids.push_back(3);
    */

    // Create HumanClient 1 = Blue, 2 = Green, 3 = Red
    humanClient = new HumanClient(node, solution::TOPIC_HUMAN_SERVER);
    humanClient->getResponse(ready, allObjs, ids);
    humanClient->convertIdsToDestinationPoses(ids, poses);
       
    // Move robot to waypoint
    moveRobotTo(solution::POSE_POSITION_X_WAYPOINT1, solution::POSE_POSITION_Y_WAYPOINT1, solution::POSE_ORIENTATION_Z_WAYPOINT1, solution::POSE_ORIENTATION_W_WAYPOINT1);

    for (int i = 0; i < poses.size(); i++)
    {
        // Get current pose
        pose = poses.at(i);

        // Move robot to the correct position
        moveRobotTo(pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w);

        // Lift Up the torso
        liftTorso(node, true);

        // Get the poses of the tags
        tableDetectionClient = new TableDetectionClient(ids.at(i), solution::TOPIC_TABLE_OBJECTS_DETECTION);
        detectionPoses = tableDetectionClient->sendGoal();

        // Move head back to original place
        moveHead(node);

        // Get the pose of the "pick" / "rectangular" table
        posePickTable = getPosePickTable(node);

        // Pick procedure
        // Add pick table
        detection.pose = posePickTable;
        detection.id = -1;
        detectionPoses.push_back(detection);

        // Send message to pick server
        pickPlaceClient = new PickPlaceClient(solution::PICK_MODE, detectionPoses, ids.at(i));
        pickPlaceClient->sendGoal();
        detectionPoses.clear();

        // Move torso down
        liftTorso(node, false);

        // Move to waypoint 3 if blue or red (1 or 3)
        if (ids.at(i) == 1 || ids.at(i) == 3)
            moveRobotTo(solution::POSE_POSITION_X_WAYPOINT3, solution::POSE_POSITION_Y_WAYPOINT3, solution::POSE_ORIENTATION_Z_WAYPOINT3, solution::POSE_ORIENTATION_W_WAYPOINT3);

        // Move to waypoint 2 (between rooms)
        moveRobotTo(solution::POSE_POSITION_X_WAYPOINT2, solution::POSE_POSITION_Y_WAYPOINT2, solution::POSE_ORIENTATION_Z_WAYPOINT2, solution::POSE_ORIENTATION_W_WAYPOINT2);

        // Move to waypoint 4 (docking position)
        moveRobotTo(solution::POSE_POSITION_X_WAYPOINT4, solution::POSE_POSITION_Y_WAYPOINT4, solution::POSE_ORIENTATION_Z_WAYPOINT4, solution::POSE_ORIENTATION_W_WAYPOINT4);

        // Find positions of the tables
        poseTableDetectorMessage.request.id = ids.at(i);
        poseTableDetectorMessage.request.circles = detectCircles(node);
        poseTableDetectorClient = new PoseTableDetectorClient(node);
        poseTableDetectorClient->callServer(poseTableDetectorMessage);

        // Save global position of the table
        posePlaceTableGlobal.position.x = poseTableDetectorMessage.response.circle.x;
        posePlaceTableGlobal.position.y = poseTableDetectorMessage.response.circle.y;

        // Move to correct position
        moveRobotTo(posePlaceTableGlobal.position.x, posePlaceTableGlobal.position.y + solution::OFFSET_PLACE_POSITION_TABLE, solution::ORIENTATION_Z_PLACE_TABLES, solution::ORIENTATION_W_PLACE_TABLES);

        // Lift Up the torso
        liftTorso(node, true);

        // Place procedure
        // Convert the globalPose of the place table to the base_link_reference frame
        posePlaceTable = convertPose(node, posePlaceTableGlobal, solution::MAP_REFERENCE_FRAME, solution::BASE_LINK_REFERENCE_FRAME);
        posePlaceTable.position.z = -solution::HEIGHT_LASER;

        // Add place table
        detection.pose = posePlaceTable;
        detection.id = -1;
        detectionPoses.push_back(detection);

        // Send message to place server
        pickPlaceClient = new PickPlaceClient(solution::PLACE_MODE, detectionPoses, ids.at(i));
        pickPlaceClient->sendGoal();
        detectionPoses.clear();

        // Move torso down
        liftTorso(node, false);

        // Move to waypoint 5 (near rooms, rotated)
        moveRobotTo(solution::POSE_POSITION_X_WAYPOINT5, solution::POSE_POSITION_Y_WAYPOINT5, solution::POSE_ORIENTATION_Z_WAYPOINT5, solution::POSE_ORIENTATION_W_WAYPOINT5);
    }

    ros::spin();
    return 0;
}

void moveRobotTo(double positionX, double positionY, double orientationZ, double orientationW)
{
    TiagoClient *client = new TiagoClient(positionX, positionY, orientationZ, orientationW, solution::TOPIC_TIAGO_SERVER);
    client->sendGoal();
}

void liftTorso(ros::NodeHandle node, bool mode)
{
    // Variables
    TorsoLifterClient *torsoLifterClient = new TorsoLifterClient(node);
    manipulation_msgs::TorsoLifter torsoLifterMessage;

    if (mode)
        torsoLifterMessage.request.joint_value = solution::TORSO_LIFT_JOINT_TARGET;
    else
        torsoLifterMessage.request.joint_value = solution::TORSO_LIFT_JOINT_INITIAL;

    torsoLifterClient->callServer(torsoLifterMessage);
}

geometry_msgs::Pose getPosePickTable(ros::NodeHandle node)
{
    // Variables
    // Messages
    detection_msgs::Circles circlesMessage;
    detection_msgs::Circle circleMessage;
    geometry_msgs::Pose poseRaw;
    geometry_msgs::Pose poseRet;

    // Clients
    ObstacleExtractorClient *obstacleExtractorClient;

    // Read once the /scan
    sensor_msgs::LaserScanConstPtr scanMessagePointer = ros::topic::waitForMessage<sensor_msgs::LaserScan>(solution::TOPIC_SCAN, node);
    circlesMessage.request.scan_data = *scanMessagePointer;
    circlesMessage.request.max_circle_radius = solution::MAX_CIRCLE_RADIUS_TABLE_PICK;
    circlesMessage.request.min_circle_radius = solution::MIN_CIRCLE_RADIUS_TABLE_PICK;
    circlesMessage.request.radius_enlargment = solution::RADIUS_ENLARMENT_TABLE_PICK;

    // Extract circles
    obstacleExtractorClient = new ObstacleExtractorClient(node);
    obstacleExtractorClient->callServer(circlesMessage);

    // Find circle corresponding to current id
    circleMessage = circlesMessage.response.cirles.at(0);

    // Convert to pose
    poseRaw.position.x = circleMessage.y;
    poseRaw.position.y = -circleMessage.x;
    poseRaw.position.z = 0.0;
    poseRaw.orientation.x = 0.0;
    poseRaw.orientation.y = 0.0;
    poseRaw.orientation.z = 0.0;
    poseRaw.orientation.w = 1.0;

    // Convert pose to correct reference frame
    poseRet = convertPose(node, poseRaw, solution::BASE_LASER_LINK_REFERENCE_FRAME, solution::BASE_LINK_REFERENCE_FRAME);

    // Set height of rectangular table
    poseRet.position.z = -solution::HEIGHT_LASER;

    return poseRet;
}

void moveHead(ros::NodeHandle node)
{
    // Variables
    // Clients
    HeadMovementClient *headMovementClient;

    // Messages
    manipulation_msgs::HeadMovement headMovementMessage;

    headMovementMessage.request.mode = 0; // Std movement of the head
    headMovementClient = new HeadMovementClient(node);
    headMovementClient->callServer(headMovementMessage);
}

geometry_msgs::Pose convertPose(ros::NodeHandle node, geometry_msgs::Pose pose, std::string sourceFrame, std::string targetFrame)
{
    // Variables
    // Messages
    detection_msgs::Transform transformMessage;
    // Clients
    PosesTransformerClient *posesTransformerClient;

    // Convert pose to correct reference frame
    transformMessage.request.pose = pose;
    transformMessage.request.source_frame = sourceFrame;
    transformMessage.request.target_frame = targetFrame;
    posesTransformerClient = new PosesTransformerClient(node);
    posesTransformerClient->callServer(transformMessage);

    return transformMessage.response.pose;
}

std::vector<detection_msgs::Circle> detectCircles(ros::NodeHandle node)
{
    // Variables
    // Messages
    detection_msgs::Circles circlesMessage;
    geometry_msgs::Pose poseRaw;
    std::vector<detection_msgs::Circle> thresholdedCircles;
    std::vector<detection_msgs::Circle> ret;
    double sum = 0.0;

    // Clients
    ObstacleExtractorClient *obstacleExtractorClient;

    sensor_msgs::LaserScanConstPtr scanMessagePointer = ros::topic::waitForMessage<sensor_msgs::LaserScan>(solution::TOPIC_SCAN, node);
    circlesMessage.request.scan_data = *scanMessagePointer;
    circlesMessage.request.max_circle_radius = solution::MAX_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.min_circle_radius = solution::MIN_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.radius_enlargment = solution::RADIUS_ENLARMENT_DETECTION;

    // Extract circles
    obstacleExtractorClient = new ObstacleExtractorClient(node);
    obstacleExtractorClient->callServer(circlesMessage);

    // Threshold small circles
    for (detection_msgs::Circle c : circlesMessage.response.cirles)
        if (c.radius > solution::THRESHOLD_RADIUS_CIRCLES_PLACE_TABLE)
            thresholdedCircles.push_back(c);

    // Make the y of each circle equivalent to the avarege y
    for (detection_msgs::Circle c : thresholdedCircles)
        sum += c.y;
    sum /= thresholdedCircles.size();

    for (int i = 0; i < thresholdedCircles.size(); i++)
        thresholdedCircles.at(i).y = sum;

    for (detection_msgs::Circle c : thresholdedCircles)
    {
        // Convert to pose
        poseRaw.position.x = c.y;
        poseRaw.position.y = -c.x;
        poseRaw.position.z = 0.0;
        poseRaw.orientation.x = 0.0;
        poseRaw.orientation.y = 0.0;
        poseRaw.orientation.z = 0.0;
        poseRaw.orientation.w = 1.0;

        // Convert the pose to the correct reference frame
        geometry_msgs::Pose convertedPose = convertPose(node, poseRaw, solution::BASE_LASER_LINK_REFERENCE_FRAME, solution::MAP_REFERENCE_FRAME);

        // Add circle conveted to the array to return
        detection_msgs::Circle circle;
        circle.x = convertedPose.position.x;
        circle.y = convertedPose.position.y;
        circle.radius = c.radius;
        ret.push_back(circle);
    }

    return ret;
}