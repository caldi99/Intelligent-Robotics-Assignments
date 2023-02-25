#include <manipulation/head_movement/head_movement_server.h>

HeadMovementServer::HeadMovementServer() : headClient(manipulation::FOLLOW_JOINT_TRAJECTORY_ACTION, true), pointHeadClient(manipulation::POINT_HEAD_ACTION, true)
{
    ROS_INFO_STREAM("(Server) manipulation HeadMovementServer HeadMovementServer");
    this->node = node;
    this->server = this->node.advertiseService(manipulation::TOPIC_HEAD_MOVEMENT, &HeadMovementServer::headMovementCallback, this);
    this->goal.trajectory.joint_names.push_back(manipulation::HEAD_1_JOINT);
    this->goal.trajectory.joint_names.push_back(manipulation::HEAD_2_JOINT);
    this->goal.trajectory.points.resize(1); 
    
    // Get the camera intrinsic parameters from the ROS topic
    ROS_INFO("Waiting for camera intrinsics parameters");
    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage <sensor_msgs::CameraInfo>(manipulation::TOPIC_CAMERA_INFO, ros::Duration(10.0));
    if(msg.use_count() > 0)
    {
        cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
        cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
        cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
        cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
        cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
        cameraIntrinsics.at<double>(2, 2) = 1;
    }

    pointHeadClient.waitForServer(ros::Duration(20.0)); 
    ROS_INFO("HeadMovementServer started");
}

bool HeadMovementServer::headMovementCallback(manipulation_msgs::HeadMovement::Request &req, manipulation_msgs::HeadMovement::Response &res)
{
    ROS_INFO_STREAM("(Server) manipulation HeadMovementServer headMovementCallback");
    int modality = req.mode;

    // Resize to put the value of the joints
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    goal.trajectory.points.at(0).positions.resize(goal.trajectory.joint_names.size());

    if (modality == 3)
    { // move the head according to pixel passed
        pointHead(cv::Point2d(req.point.x, req.point.y));
        return true;
    }

    switch (modality)
    {
    case 0: // move the head to the initial position
        goal.trajectory.points.at(0).positions.at(0) = 0.0;
        goal.trajectory.points.at(0).positions.at(1) = 0.0;
        break;
    case 1: // move the head down
        goal.trajectory.points.at(0).positions.at(0) = 0.0;
        goal.trajectory.points.at(0).positions.at(1) = -0.4; 
        break;
    case 2: // move the head according to the joints values passed as parameters
        goal.trajectory.points.at(0).positions.at(0) = req.rotation_horizontal;
        goal.trajectory.points.at(0).positions.at(1) = req.rotation_vertical;
        break;
    }
    goal.trajectory.points.at(0).time_from_start = ros::Duration(2.0);

    headClient.sendGoal(goal);
    ros::Duration(3).sleep();
    res.state = true;

    return true;
}

void HeadMovementServer::pointHead(cv::Point2d center)
{
    ROS_INFO_STREAM("(Server) manipulation HeadMovementServer pointHead");
    ROS_INFO_STREAM(center.x << " " << center.y);

    geometry_msgs::PointStamped pointStamped;

    pointStamped.header.frame_id = manipulation::XTION_RGB_OPTICAL_FRAME_REFERENCE_FRAME;

    // compute normalized coordinates of the selected pixel
    double x = (center.x - cameraIntrinsics.at<double>(0, 2)) / cameraIntrinsics.at<double>(0, 0);
    double y = (center.y - cameraIntrinsics.at<double>(1, 2)) / cameraIntrinsics.at<double>(1, 1);
    double Z = 1.0; // define an arbitrary distance
    pointStamped.point.x = x * Z;
    pointStamped.point.y = y * Z;
    pointStamped.point.z = Z;

    // build the action goal
    control_msgs::PointHeadGoal goal;
    // the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
    goal.pointing_frame = manipulation::XTION_RGB_OPTICAL_FRAME_REFERENCE_FRAME;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    pointHeadClient.sendGoal(goal);
    ros::Duration(3).sleep();
}
