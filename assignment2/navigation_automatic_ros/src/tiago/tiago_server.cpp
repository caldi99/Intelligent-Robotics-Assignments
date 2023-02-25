#include <navigation_automatic_ros/tiago/tiago_server.h>

TiagoServer::TiagoServer(std::string name) : server(nh, name, boost::bind(&TiagoServer::navAndDetectCallback, this, _1), false), client(navigation_automatic_ros::TOPIC_MOVE_BASE, true)
{
    server.start();
    ROS_INFO_STREAM("(Server) navigation_automatic_ros TiagoServer TiagoServer");
}

void TiagoServer::doNavigation(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) navigation_automatic_ros TiagoServer doNavigation");
    // Wait for the action server to come up so that we can begin processing goals.
    client.waitForServer();

    // create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    // set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.orientation.z = goal->orZ;
    goalMsg.target_pose.pose.orientation.w = goal->orW;

    // Send the goal and wait
    actionlib::SimpleClientGoalState result = client.sendGoalAndWait(goalMsg);

    ROS_INFO_STREAM("(Server) ROBOT IS ARRIVED AT THE FINAL POSITION");
    feedback.state = 1; //(Client) ROBOT IS ARRIVED TO THE FINAL POSE.
    server.publishFeedback(feedback);

    // Finished
    this->result.arrived = true;
    server.setSucceeded(this->result);
}

void TiagoServer::navAndDetectCallback(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) navigation_automatic_ros TiagoServer navAndDetectCallback");
    // navigate to the final pose
    doNavigation(goal);
}
