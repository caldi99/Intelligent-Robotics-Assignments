#include <solution/manipulation/head_movement_client.h>

HeadMovementClient::HeadMovementClient(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Client) solution HeadMovementClient HeadMovementClient");
    this->node = node;

    client = this->node.serviceClient<manipulation_msgs::HeadMovement>(solution::TOPIC_HEAD_MOVEMENT);
}

bool HeadMovementClient::callServer(manipulation_msgs::HeadMovement &msg)
{
    ROS_INFO_STREAM("(Client) solution HeadMovementClient callServer");
    return client.call(msg);
}