#include <navigation_commands_ros/tiago_client_commands.h>

/*  
@param x coordinate x of the final pose (w.r.t. map ref. frame)
@param y coordinate y of the final pose (w.r.t. map ref. frame)
@param orZ orientation in the z axis of the final pose (w.r.t. map ref. frame)
@param server_name name of the TiagoServer 
*/
TiagoClientCommands::TiagoClientCommands(float x, float y, float orZ, const std::string server_name) : client(server_name, true)
{
    ROS_INFO("(Client) Tiago Client started");
    this->x = x;
    this->y = y;
    this->orZ = orZ;

} // TiagoClientCommands

/** callback for action done
 * @param &state final state 
 * @param &result_ptr Boost Pointer to the final result of the move action
*/
void TiagoClientCommands::doneCb(const actionlib::SimpleClientGoalState &state, const navigation_commands_ros::MoveDetectResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("(Client) ROBOT HAS FINISHED: NAVIGATION AND DETECTION ARE DONE"); // successed
        for (int i = 0; i < result->points.poses.size(); i++)
            ROS_INFO_STREAM("OBJECT DETECTED X : " << result->points.poses.at(i).position.x << " OBJECT DETECTED Y : " << result->points.poses.at(i).position.y);
    }
    else
    {
        ROS_WARN("(Client) ROBOT HAS FAILED: "); // failed
    }                                            // if-else
} // doneCb

// Callback for when robot starts the navigation
void TiagoClientCommands::activeCb()
{
    ROS_INFO("(Client) GOAL SENT TO THE ROBOT.");
} // activeCb

/** callback for feedback messages
 * @param &feedback_ptr pointer to the MoveBaseFeedback message
*/
void TiagoClientCommands::feedbackCb(const navigation_commands_ros::MoveDetectFeedbackConstPtr &feedback)
{
    int state = feedback->state;
    if (state == 0)
        ROS_INFO("(Client) ROBOT IS MOVING.");
    else if (state == 1)
        ROS_INFO("(Client) ROBOT IS ARRIVED TO THE FINAL POSE.");
    else if (state == 2)
        ROS_INFO("(Client) ROBOT IS SCANNING THE ENVIRONMENT.");
    else if (state == -1)
        ROS_INFO("(Client) ROBOT ERROR IN NAVIGATION.");
    else if (state == -2)
        ROS_INFO("(Client) ROBOT ERROR IN DETECTION.");
} // feedbackCb

//function that send the goal to our Tiago Server
int TiagoClientCommands::sendGoal()
{
    // Wait for the action server to come up so that we can begin processing goals.
    ROS_INFO("(Client) Waiting for the TiagoServerCommands to come up.");
    client.waitForServer();

    navigation_commands_ros::MoveDetectGoal goal;

    // set the goal position
    goal.x = x;
    goal.y = y;
    goal.orZ = orZ;

    ROS_INFO("(Client) Sending goal");
    client.sendGoal(goal, boost::bind(&TiagoClientCommands::doneCb, this, _1, _2),
                          boost::bind(&TiagoClientCommands::activeCb, this),
                          boost::bind(&TiagoClientCommands::feedbackCb, this, _1));

} // sendGoal