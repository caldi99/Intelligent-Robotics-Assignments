#include <navigation_automatic_ros/tiago_server.h>


/** constructor
 * @param name of the action server
*/
TiagoServer::TiagoServer(std::string name) : server(nh, name, boost::bind(&TiagoServer::navAndDetectCallback, this, _1), false), client("move_base", true)
{
    server.start();
    ROS_INFO_STREAM("(Server) TIAGO SERVER STARTED");
}

/** method for managing all the navigation part of the task
@param &goal reference to the MoveDetectAction message goal
*/
void TiagoServer::doNavigation(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal){

    // Wait for the action server to come up so that we can begin processing goals.
    client.waitForServer();
    
    //create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    //set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.orientation.z = goal->orZ;

    //all is wrt. world (map) ref frame
    goalMsg.target_pose.pose.orientation.w = 1.0;

    //Send the goal and wait
    actionlib::SimpleClientGoalState result = client.sendGoalAndWait(goalMsg);

    ROS_INFO_STREAM("(Server) ROBOT IS ARRIVED AT THE FINAL POSITION");
    feedback.state = 1; //(Client) ROBOT IS ARRIVED TO THE FINAL POSE.
    server.publishFeedback(feedback); 
}

/** method for the detection task
*/
void TiagoServer::doDetection(){
    // do the detection and publish the final message with all the detected moving objects
    ROS_INFO_STREAM("(Server) ROBOT IS STARTING THE DETECTION");
    feedback.state = 2; //(Client) ROBOT IS SCANNING THE ENVIRONMENT.
    server.publishFeedback(feedback);

    // Read scan data
    sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

    float angleMin = msg->angle_min + M_PI_2;
    float angleIncrement = msg->angle_increment;
    std::list<Point> inputPoints;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        float x = msg->ranges.at(i) * std::cos(angleMin);
        float y = msg->ranges.at(i) * std::sin(angleMin);

        inputPoints.push_back(Point(cv::Point2d(x, y)));
        angleMin += angleIncrement;
    }

    // Extract obstacles :
    ObstacleExtractor ex(inputPoints, msg->angle_increment);
    std::list<Segment> segments;
    std::list<Circle> circles;
    ex.processPoints(segments, circles);
    // result.state =
    //  Find centers of the objctes
    for (Circle c : circles)
    {
        geometry_msgs::Pose point;
        point.position.x = c.getCenter().getPoint().y;
        point.position.y = -c.getCenter().getPoint().x;

        result.points.poses.push_back(point);
    }
    ROS_INFO_STREAM("(Server) DETECTION IS FINISHED");

    // Finished
    server.setSucceeded(result);

}


/** callback for the main task of the robot
 * @param &goal final pose of the navigation stack 
*/
void TiagoServer::navAndDetectCallback(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal){
    //navigate to the final pose
    doNavigation(goal);

    //do the detection of the moving obstacles
    doDetection();
} 
