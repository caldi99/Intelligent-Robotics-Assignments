#include <detection/pose_table_detector/pose_table_detector_server.h>

PoseTableDetectorServer::PoseTableDetectorServer(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer PoseTableDetectorServer");
    this->node = node;
    this->server = this->node.advertiseService(detection::TOPIC_POSE_TABLE_DETECTOR, &PoseTableDetectorServer::sendResponse, this);
    this->headClient = this->node.serviceClient<manipulation_msgs::HeadMovement>(detection::TOPIC_HEAD_MOVEMENT);
}

bool PoseTableDetectorServer::sendResponse(detection_msgs::PoseTableDetector::Request& req, detection_msgs::PoseTableDetector::Response& res)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer sendResponse");

    // Move head down
    manipulation_msgs::HeadMovement headMovementMessage;
    headMovementMessage.request.mode = 1;
    headClient.call(headMovementMessage);

    // Get the image
    cv::Mat image = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>(detection::TOPIC_IMAGE, node), sensor_msgs::image_encodings::BGR8)->image;

    // Threashold image
    cv::Mat imageHSV;
    cv::Mat imageThresholdedBlue;
    cv::Mat imageThresholdedGreen;
    cv::Mat imageThresholdedRed;

    // Convert image to HSV
    cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

    // Compute thresholded images
    cv::inRange(imageHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), imageThresholdedBlue);
    cv::inRange(imageHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), imageThresholdedGreen);
    cv::inRange(imageHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), imageThresholdedRed);


    // Get the points of each thresholded image
    std::vector<cv::Point2i> pointsBlue = getPoints(imageThresholdedBlue);
    std::vector<cv::Point2i> pointsGreen = getPoints(imageThresholdedGreen);
    std::vector<cv::Point2i> pointsRed = getPoints(imageThresholdedRed);

    // Find the avg of each of the points
    cv::Point2i avgBlue = getAvgPoint(pointsBlue);
    cv::Point2i avgGreen = getAvgPoint(pointsGreen);
    cv::Point2i avgRed = getAvgPoint(pointsRed);

    unsigned char mode = 0; // 0 biggest, 1 smallest, 2 middle

    switch (req.id)
    {
    case 1: // Blue
        if (isBiggest(avgBlue.y, avgRed.y, avgGreen.y))
            mode = 0;
        else if (isSmallest(avgBlue.y, avgRed.y, avgGreen.y))
            mode = 1;
        else
            mode = 2;
        break;
    case 2: // Blue
        if (isBiggest(avgGreen.y, avgRed.y, avgBlue.y))
            mode = 0;
        else if (isSmallest(avgGreen.y, avgRed.y, avgBlue.y))
            mode = 1;
        else
            mode = 2;
        break;
    case 3: // Blue
        if (isBiggest(avgRed.y, avgBlue.y, avgGreen.y))
            mode = 0;
        else if (isSmallest(avgRed.y, avgBlue.y, avgGreen.y))
            mode = 1;
        else
            mode = 2;
        break;
    }

    // Get the correct circle
    std::vector<detection_msgs::Circle> circles = req.circles;
    std::sort(circles.begin(), circles.end(), PoseTableDetectorServer::compareCircles);

    switch (mode)
    {
    case 0: // biggest
        res.circle = circles.at(circles.size() -1);
        break;
    case 1: // smallest
        res.circle = circles.at(0);
        break;
    case 2: // middle
        res.circle = circles.at(circles.size() /2);
        break;
    }        
}

std::vector<cv::Point2i> PoseTableDetectorServer::getPoints(const cv::Mat &image)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer getPoints");

    std::vector<cv::Point2i> points;
    for (int r = 0; r < image.rows; r++)
        for (int c = 0; c < image.cols; c++)
            if (image.at<unsigned char>(r, c) == 255)
                points.push_back(cv::Point2f(r, c));
    return points;
}

cv::Point2i PoseTableDetectorServer::getAvgPoint(const std::vector<cv::Point2i> &points)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer getAvgPoint");

    cv::Point2i p;
    p.x = 0;
    p.y = 0;

    for (cv::Point2i point : points)
        p += point;

    p.x /= points.size();
    p.y /= points.size();
    return p;
}

bool PoseTableDetectorServer::isSmallest(int x, int y, int z)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer isSmallest");

    return std::min(std::min(x, y), z) == x;
}

bool PoseTableDetectorServer::isBiggest(int x, int y, int z)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer isBiggest");

    return std::max(std::max(x, y), z) == x;
}

bool PoseTableDetectorServer::compareCircles(detection_msgs::Circle c1, detection_msgs::Circle c2)
{
    ROS_INFO_STREAM("(Server) Detection PoseTableDetectorServer compareCircles");

    return c1.x < c2.x;
}