#include <solution/detection/obstacle_extractor_client.h>

ObstacleExtractorClient::ObstacleExtractorClient(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Client) solution ObstacleExtractorClient ObstacleExtractorClient");
    this->node = node;

    client = this->node.serviceClient<detection_msgs::Circles>(solution::TOPIC_OBSTACLE_EXTRACTOR);
}

bool ObstacleExtractorClient::callServer(detection_msgs::Circles &msg)
{
    ROS_INFO_STREAM("(Client) solution ObstacleExtractorClient callServer");
    return client.call(msg);
}