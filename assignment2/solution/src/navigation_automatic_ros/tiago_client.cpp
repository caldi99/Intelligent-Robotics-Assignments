#include <solution/navigation_automatic_ros/tiago_client.h>

TiagoClient::TiagoClient(double x, double y, double orZ, double orW, std::string clientName) : client(clientName, true)
{
    ROS_INFO_STREAM("(Client) solution TiagoClient TiagoClient");
    this->x = x;
    this->y = y;
    this->orZ = orZ;
    this->orW = orW;
}

void TiagoClient::doneCb(const actionlib::SimpleClientGoalState &state, const navigation_automatic_ros::MoveDetectResultConstPtr &result)
{
    ROS_INFO_STREAM("(Client) solution TiagoClient doneCb");
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)    
        ROS_INFO("(Client) ROBOT HAS FINISHED"); // successed    
    else    
        ROS_WARN("(Client) ROBOT HAS FAILED: "); // failed
    
}

void TiagoClient::activeCb()
{
    ROS_INFO_STREAM("(Client) solution TiagoClient activeCb");
    ROS_INFO("(Client) GOAL SENT TO THE ROBOT.");
}

void TiagoClient::feedbackCb(const navigation_automatic_ros::MoveDetectFeedbackConstPtr &feedbackPtr)
{
    ROS_INFO_STREAM("(Client) solution TiagoClient feedbackCb");
    int state = feedbackPtr->state;
    if (state == 0)
        ROS_INFO("(Client) ROBOT IS MOVING.");
    else if (state == 1)
        ROS_INFO("(Client) ROBOT IS ARRIVED TO THE FINAL POSE.");
    else if (state == -1)
        ROS_INFO("(Client) ROBOT ERROR IN NAVIGATION.");
}

void TiagoClient::sendGoal()
{
    ROS_INFO_STREAM("(Client) solution TiagoClient sendGoal");
    // Wait for the action server to come up so that we can begin processing goals.
    ROS_INFO("(Client) Waiting for the TiagoServerCommands to come up.");
    client.waitForServer();

    navigation_automatic_ros::MoveDetectGoal goal;

    // set the goal position
    goal.x = x;
    goal.y = y;
    goal.orZ = orZ;
    goal.orW = orW;

    ROS_INFO("(Client) Sending goal");
    
    client.sendGoal(goal, boost::bind(&TiagoClient::doneCb, this, _1, _2),
                                  boost::bind(&TiagoClient::activeCb, this),
                                  boost::bind(&TiagoClient::feedbackCb, this, _1));    

    client.waitForResult();    
}
