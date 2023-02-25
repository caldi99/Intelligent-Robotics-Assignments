#include <solution/detection/table_objects_detection_client.h>

TableDetectionClient::TableDetectionClient(int colorID, std::string serverName) : client(serverName, true)
{
    ROS_INFO("(Client) Solution TableDetectionClient started");
    this->colorID = colorID;
}

void TableDetectionClient::doneCb(const actionlib::SimpleClientGoalState &state, const detection_msgs::ObjectsDetectionResultConstPtr &resultPtr)
{    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        this->detections = resultPtr->detections.poses;
        ROS_INFO("(Client) Solution detection is finished"); // successed
    }
    else
    {
        ROS_WARN("(Client) Solution ROBOT HAS FAILED: "); // failed
    }
}

void TableDetectionClient::activeCb()
{
    ROS_INFO("(Client) Solution Goal sent to the robot.");
}

void TableDetectionClient::feedbackCb(const detection_msgs::ObjectsDetectionFeedbackConstPtr &feedback)
{
    int state = feedback->state;
    if (state == 0)
        ROS_INFO("(Client) Solution The robot is searching for the table");
    else if (state == 1)
        ROS_INFO("(Client) Solution The robot is searching for the colored object tag");
    else if (state == 2)
        ROS_INFO("(Client) Solution The robot is searching for the collision objects tag");
}

std::vector<detection_msgs::Detection> TableDetectionClient::sendGoal()
{
    // Wait for the action server to come up so that we can begin processing goals.
    ROS_INFO("(Client) Waiting for the server to come up.");
    client.waitForServer();

    detection_msgs::ObjectsDetectionGoal goal;

    // set the goal position
    goal.colorID = this->colorID;

    ROS_INFO("(Client) Solution Sending goal");

    client.sendGoal(goal, boost::bind(&TableDetectionClient::doneCb, this, _1, _2),
                    boost::bind(&TableDetectionClient::activeCb, this),
                    boost::bind(&TableDetectionClient::feedbackCb, this, _1));

    client.waitForResult();

    return this->detections;
}