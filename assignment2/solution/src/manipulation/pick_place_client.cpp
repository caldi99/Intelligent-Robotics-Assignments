#include <solution/manipulation/pick_place_client.h>

PickPlaceClient::PickPlaceClient(unsigned char mode, std::vector<detection_msgs::Detection> poses, int idObjectPick) : client(solution::TOPIC_PICK_PLACE, true)
{
    ROS_INFO("(Client) solution PickPlaceClient PickPlaceClient");
    this->mode = mode;
    this->poses = poses;
    this->idObjectPick = idObjectPick;
}

void PickPlaceClient::sendGoal()
{
    ROS_INFO("(Client) solution PickPlaceClient sendGoal");

    client.waitForServer();

    manipulation_msgs::PickPlaceGoal goal;
    
    // set the goal position
    goal.id_object_pick_place = idObjectPick;
    goal.mode = mode;
    goal.poses = poses;
       
    client.sendGoal(goal, boost::bind(&PickPlaceClient::doneCb, this, _1, _2),
                                  boost::bind(&PickPlaceClient::activeCb, this),
                                  boost::bind(&PickPlaceClient::feedbackCb, this, _1));    

    client.waitForResult();    
}

void PickPlaceClient::doneCb(const actionlib::SimpleClientGoalState &state, const manipulation_msgs::PickPlaceResultConstPtr &resultPtr)
{
    ROS_INFO("(Client) solution PickPlaceClient doneCb");

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_DEBUG_STREAM("\tState : " << resultPtr->state);
    else    
        ROS_WARN_STREAM("\tState : " << resultPtr->state);    
}

void PickPlaceClient::activeCb()
{
    ROS_INFO("(Client) solution PickPlaceClient activeCb");
}

void PickPlaceClient::feedbackCb(const manipulation_msgs::PickPlaceFeedbackConstPtr &feedbackPtr)
{
    ROS_INFO("(Client) solution PickPlaceClient feedbackCb");
    ROS_DEBUG_STREAM("\tStatus : " << feedbackPtr->status);
}