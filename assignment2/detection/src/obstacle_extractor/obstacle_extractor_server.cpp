#include <detection/obstacle_extractor/obstacle_extractor_server.h>

ObstacleExtractorServer::ObstacleExtractorServer(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Server) detection ObstacleExtractorServer ObstacleExtractorServer");
    this->node = node;
    this->server = this->node.advertiseService(detection::TOPIC_OBSTACLE_EXTRACTOR, &ObstacleExtractorServer::sendResponse, this);
}

bool ObstacleExtractorServer::sendResponse(detection_msgs::Circles::Request &req, detection_msgs::Circles::Response &res)
{
    ROS_INFO_STREAM("(Server) detection ObstacleExtractorServer sendResponse");
    ObstacleExtractor ex(this->computePoints(req.scan_data),req.scan_data.angle_increment,req.max_circle_radius, req.min_circle_radius, req.radius_enlargment);
    std::list<Segment> segments;
    std::list<Circle> circles;
    ex.processPoints(segments, circles);

    for(Circle circle : circles)
    {
        detection_msgs::Circle c;
        c.x = circle.getCenter().getPoint().x;
        c.y = circle.getCenter().getPoint().y;
        c.radius = circle.getRadius();
        res.cirles.push_back(c);
    }

    return true;
}

std::list<Point> ObstacleExtractorServer::computePoints(sensor_msgs::LaserScan msg)
{
    ROS_INFO_STREAM("(Server) detection ObstacleExtractorServer computePoints");
    float angleMin = msg.angle_min + M_PI_2;
    float angleIncrement = msg.angle_increment;
    std::list<Point> points;

    for (int i = 0; i < msg.ranges.size(); i++)
    {
        float x = msg.ranges.at(i) * std::cos(angleMin);
        float y = msg.ranges.at(i) * std::sin(angleMin);

        points.push_back(Point(cv::Point2d(x, y)));
        angleMin += angleIncrement;
    }
    return points;
}