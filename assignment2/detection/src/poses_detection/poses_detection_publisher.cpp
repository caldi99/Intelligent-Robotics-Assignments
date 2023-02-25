#include <detection/poses_detection/poses_detection_publisher.h>

PosesDetectionPublisher::PosesDetectionPublisher(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Server) detection PosesDetectionPublisher PosesDetectionPublisher");
    this->node = node;
    this->publisher = this->node.advertise<detection_msgs::Detections>(detection::TOPIC_POSES_DETECTION, detection::QUEUE_SIZE);
}

void PosesDetectionPublisher::publish()
{
    // ROS_INFO_STREAM("(Server) detection PosesDetectionPublisher publish");
    // Save only once the detections
    apriltag_ros::AprilTagDetectionArray::ConstPtr detections = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>(detection::TOPIC_TAG_DETECTIONS, node);

    // Convert detections
    detection_msgs::Detections msg;
    for (int i = 0; i < detections->detections.size(); i++)
    {
        
        apriltag_ros::AprilTagDetection detection = (detections->detections)[i];
        geometry_msgs::PoseWithCovarianceStamped poseCovarianceStamped = detection.pose;
        geometry_msgs::PoseWithCovariance poseCovariance = poseCovarianceStamped.pose;
        
        detection_msgs::Detection detectionMessage;
        detectionMessage.pose = transformPose(poseCovariance.pose);
        detectionMessage.id = detection.id.at(0);
        msg.poses.push_back(detectionMessage);
    }

    // Publish the message
    publisher.publish(msg);
}

geometry_msgs::Pose PosesDetectionPublisher::transformPose(geometry_msgs::Pose pose)
{
    // ROS_INFO_STREAM("(Server) detection PosesDetectionPublisher transformPose");
    geometry_msgs::Pose ret;

    // Convert the pose
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<detection_msgs::Transform>(detection::TOPIC_POSES_TRANSFORMER);
    detection_msgs::Transform msg;
    msg.request.source_frame = detection::XTION_RGB_OPTICAL_FRAME_REFERENCE_FRAME;
    msg.request.target_frame = detection::BASE_LINK_REFERENCE_FRAME;
    msg.request.pose = pose;

    if (client.call(msg))
        ret = msg.response.pose;
    else
        ROS_ERROR("Failed to call service");

    return ret;
}