#include <solution/tiago_iaslab_simulation/human_client.h>

HumanClient::HumanClient(ros::NodeHandle node, std::string serverName)
{
    ROS_INFO_STREAM("(Client) solution HumanClient HumanClient");
    this->node = node;
    this->client = node.serviceClient<tiago_iaslab_simulation::Objs>(serverName);
}

bool HumanClient::getResponse(bool ready, bool allObjs, std::vector<int> &ids)
{
    ROS_INFO_STREAM("(Client) solution HumanClient getResponse");
    tiago_iaslab_simulation::Objs srv;
    srv.request.all_objs = allObjs;
    srv.request.ready = ready;

    bool ret = client.call(srv);
    ids = srv.response.ids;

    return ret;
}

void HumanClient::convertIdsToDestinationPoses(std::vector<int> &ids, std::vector<geometry_msgs::Pose> &poses)
{
    ROS_INFO_STREAM("(Client) solution HumanClient getResponse");
    for (int id : ids)
    {
        geometry_msgs::Pose pose;
        switch (id)
        {
        case 1:
            pose.position.x = solution::POSE_POSITION_X_ID1;
            pose.position.y = solution::POSE_POSITION_Y_ID1;
            pose.orientation.z = solution::POSE_ORIENTATION_Z_ID1;
            pose.orientation.w = solution::POSE_ORIENTATION_W_ID1;
            break;
        case 2:
            pose.position.x = solution::POSE_POSITION_X_ID2;
            pose.position.y = solution::POSE_POSITION_Y_ID2;
            pose.orientation.z = solution::POSE_ORIENTATION_Z_ID2;
            pose.orientation.w = solution::POSE_ORIENTATION_W_ID2;
            break;
        case 3:
            pose.position.x = solution::POSE_POSITION_X_ID3;
            pose.position.y = solution::POSE_POSITION_Y_ID3;
            pose.orientation.z = solution::POSE_ORIENTATION_Z_ID3;
            pose.orientation.w = solution::POSE_ORIENTATION_W_ID3;
            break;
        }
        poses.push_back(pose);
    }
}