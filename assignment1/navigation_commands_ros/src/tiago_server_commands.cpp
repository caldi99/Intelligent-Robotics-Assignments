#include <navigation_commands_ros/tiago_server_commands.h>

/** constructor
 * @param serverName name of the action server
*/
TiagoServerCommands::TiagoServerCommands(const std::string serverName) : server(nh, serverName, boost::bind(&TiagoServerCommands::navAndDetectCallback, this, _1), false), client("move_base", true)
{
    server.start();
    ROS_INFO_STREAM("(Server) TIAGO SERVER STARTED");
} // TiagoServerCommands

void TiagoServerCommands::setVelocityVector()
{
    geometry_msgs::Twist velVec;
    velVec.linear.x = FORWARD_SPEED_MPS;  // 5.0 m/s
    velVec.angular.z = ANGULAR_SPEED_RPS; // 0.0 rad/s
    pub.publish(velVec);
} // setVelocityVector

void TiagoServerCommands::doNavigationNarrowSpaces()
{
    // Wait for the action server to come up so that we can begin processing goals.
    ROS_INFO_STREAM("(Server) ROBOT IS MOVING");
    feedback.state = 0; //(Client) ROBOT IS MOVING.
    server.publishFeedback(feedback);

    ROS_INFO_STREAM("(Server) ROBOT IS MOVING IN A NARROW SPACE");
    pub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
    ros::Rate loop(10);
    bool fail = false;    
    do
    {
        sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
        WallDetector detector(msg->angle_increment, msg->angle_min, msg->angle_max, msg->range_max, msg->range_min);
        std::vector<cv::Vec4f> finalLines;
        fail = detector.findLines(msg->ranges, finalLines);
        if (!fail)
            setVelocityVector();        
        loop.sleep();
    } while (!fail);
}

/** method for managing all the navigation part of the task
@param &goal reference to the MoveDetectAction message goal
*/
void TiagoServerCommands::doNavigationMoveBase(const navigation_commands_ros::MoveDetectGoalConstPtr& goal)
{
    // Now we are no more in a narrow space => go to the goal position from here
    ROS_INFO_STREAM("(Server) ROBOT IS NO MORE MOVING IN NARROW SPACE");
    client.waitForServer();

    // create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    // set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.orientation.z = goal->orZ;

    // all is wrt. world (map) ref frame
    goalMsg.target_pose.pose.orientation.w = 1.0;

    //Send the goal and wait
    actionlib::SimpleClientGoalState result = client.sendGoalAndWait(goalMsg);

    ROS_INFO_STREAM("(Server) ROBOT IS ARRIVED AT THE FINAL POSITION");
    feedback.state = 1; //(Client) ROBOT IS ARRIVED TO THE FINAL POSE.
    server.publishFeedback(feedback);
}

/** method for the detection task
*/
void TiagoServerCommands::doDetection()
{
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
} // doDetection

/** callback for the main task of the robot
 * @param &goal final pose of the navigation stack 
*/
void TiagoServerCommands::navAndDetectCallback(const navigation_commands_ros::MoveDetectGoalConstPtr& goal)
{
    doNavigationNarrowSpaces();
    doNavigationMoveBase(goal);
    doDetection();    
}