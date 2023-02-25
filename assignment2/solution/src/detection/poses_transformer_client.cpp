#include <solution/detection/poses_transformer_client.h>

PosesTransformerClient::PosesTransformerClient(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Client) solution PosesTransformerClient PosesTransformerClient");
    this->node = node;

    client = this->node.serviceClient<detection_msgs::Transform>(solution::TOPIC_POSES_TRANSFORMER);
}

bool PosesTransformerClient::callServer(detection_msgs::Transform &msg)
{
    ROS_INFO_STREAM("(Client) solution PosesTransformerClient callServer");
    return client.call(msg);
}