#include <solution/detection/pose_table_detector_client.h>

PoseTableDetectorClient::PoseTableDetectorClient(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Client) solution PoseTableDetectorClient PoseTableDetectorClient");
    this->node = node;

    client = this->node.serviceClient<detection_msgs::PoseTableDetector>(solution::TOPIC_POSE_TABLE_DETECTOR);
}

bool PoseTableDetectorClient::callServer(detection_msgs::PoseTableDetector &msg)
{
    ROS_INFO_STREAM("(Client) solution PoseTableDetectorClient callServer");
    return client.call(msg);
}