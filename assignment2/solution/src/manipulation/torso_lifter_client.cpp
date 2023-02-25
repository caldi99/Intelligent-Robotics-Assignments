#include <solution/manipulation/torso_lifter_client.h>

TorsoLifterClient::TorsoLifterClient(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Client) solution TorsoLifterClient TorsoLifterClient");
    this->node = node;

    client = this->node.serviceClient<manipulation_msgs::TorsoLifter>(solution::TOPIC_TORSO_LIFTER);
}

bool TorsoLifterClient::callServer(manipulation_msgs::TorsoLifter &msg)
{
    ROS_INFO_STREAM("(Client) solution TorsoLifterClient callServer");
    return client.call(msg);
}