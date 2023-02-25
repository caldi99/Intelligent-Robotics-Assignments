/**
 * Debugging puposes file
 * @author : Francesco Caldivezzi
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

// PROCESS
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

// STL
#include <list>
#include <iostream>

// DETECTION
#include <detection/obstacle_detection/obstacle_extractor.h>

// OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// FUNCTIONS DEFINITION
void showSegmentsAndCircleRviz(std::list<Segment>& segments, std::list<Circle>& circles);
void publishLines(std::list<Segment> &segments, ros::Publisher &markerPub);
void publishCircles(std::list<Circle> &circles, ros::Publisher &markerPub);
void createTrackBars();

// TRACKBAR CALLBACKS

// SEGMENTS
static void onMinGroupPointsTrackbar(int, void *);
static void onMaxGroupDistanceTrackbar(int, void *);
static void onMaxSplitDistanceTrackbar(int, void *);
static void onMaxMergeSeparationTrackbar(int, void *);
static void onMaxMergeSpreadTrackbar(int, void *);

// CIRCLES
static void onRadiusEnlargementTrackbar(int, void *);
static void onMaxCircleRadiusTrackbar(int, void *);

//CALLBACK SCAN
void chatterCallback(const sensor_msgs::LaserScanConstPtr &msg);

// DEFINITION OF SHARED VARIABLES BETWEEN PROCESSES

// SEGMENTS
int minGroupPointsTrackbar = 2;
static int *minGroupPoints;
int maxGroupDistanceTrackbar = 1;
static double *maxGroupDistance;
int maxSplitDistanceTrackbar = 1;
static double *maxSplitDistance;
int maxMergeSeparationTrackbar = 1;
static double *maxMergeSeparation;
int maxMergeSpreadTrackbar = 1;
static double *maxMergeSpread;

// CIRCLES
int radiusEnlargementTrackbar = 1;
static double *radiusEnlargement;
int maxCircleRadiusTrackbar = 1;
static double *maxCircleRadius;

// MIN VALUES (FACTORS USED TO CHANGE WITH CALLBACKS OF TRACKBARS)
// SEGMENTS
int minValueMinGroupPoints = 2;
double minValueMaxGroupDistance = 0.01;
double minValueMaxSplitDistance = 0.01;
double minValueMaxMergeSeparation = 0.01;
double minValueMaxMergeSpread = 0.1;

//CENTERS
double minValueRadiusEnlargement = 0.05;
double minValueMaxCircleRadius = 0.05;


int main(int argc, char **argv)
{
    // CREATE SHARED VARIABLES

    // SHARED VARIABLES FOR SEGMENTS
    minGroupPoints = (int *)mmap(NULL, sizeof *minGroupPoints, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    maxGroupDistance = (double *)mmap(NULL, sizeof *maxGroupDistance, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    maxSplitDistance = (double *)mmap(NULL, sizeof *maxSplitDistance, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    maxMergeSeparation = (double *)mmap(NULL, sizeof *maxMergeSeparation, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    maxMergeSpread = (double *)mmap(NULL, sizeof *maxMergeSpread, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    // SHARED VARIABLES FOR CIRCLE
    radiusEnlargement = (double *)mmap(NULL, sizeof *maxMergeSpread, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    maxCircleRadius = (double *)mmap(NULL, sizeof *maxMergeSpread, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    // SET INITIAL VALUES SHARED VARIABLES

    // SET INITIAL VALUES FOR SEGMENTS
    *minGroupPoints = 5;
    *maxGroupDistance = 0.05;
    *maxSplitDistance = 0.3;
    *maxMergeSeparation = 0.03;
    *maxMergeSpread = 0.03;

    // SET INITIAL VALUES FOR CIRCLES 
    *radiusEnlargement = 0.05;
    *maxCircleRadius = 0.35;

    pid_t pid = fork();

    if(pid == -1)
    {
        ROS_INFO_STREAM("FAIL");
        return 1;
    }else if(pid > 0)
    {
        // Create a Node
        ros::init(argc, argv, "wall_detector");

        // Start the Node
        ros::NodeHandle node;

        // Subscribe to the topic
        ros::Subscriber sub = node.subscribe("scan", 1000, chatterCallback);

        ros::spin();
    }else
    {
        //Create Trackbars
        createTrackBars();
    }

    return 0;
}

void chatterCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
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

    ROS_INFO_STREAM("minGroupPoints " << *minGroupPoints);
    ROS_INFO_STREAM("maxGroupDistance " << *maxGroupDistance);
    ROS_INFO_STREAM("maxSplitDistance " << *maxSplitDistance);
    ROS_INFO_STREAM("maxMergeSeparation " << *maxMergeSeparation);
    ROS_INFO_STREAM("maxMergeSpread " << *maxMergeSpread);
    ROS_INFO_STREAM("radiusEnlargement " << *radiusEnlargement);
    ROS_INFO_STREAM("maxCircleRadius " << *maxCircleRadius);

    try
    {
        /*ObstacleExtractor ex(inputPoints, msg->angle_increment,
                             *minGroupPoints, *maxGroupDistance, *maxSplitDistance, *maxMergeSeparation, *maxMergeSpread, *radiusEnlargement, *maxCircleRadius);*/
        ObstacleExtractor ex(inputPoints, msg->angle_increment,*radiusEnlargement, *maxCircleRadius);
        std::list<Segment> segments;
        std::list<Circle> circles;
        ex.processPoints(segments, circles);
        ROS_INFO_STREAM("NUMBER OF SEGMENTS " << segments.size());
        ROS_INFO_STREAM("NUMBER OF CIRCLES " << circles.size());
        for(Circle circle : circles)
        {
            ROS_INFO_STREAM("RADIUS CIRCLE " << circle.getRadius());
            ROS_INFO_STREAM("CIRCLE X : " << circle.getCenter().getPoint().y);
            ROS_INFO_STREAM("CIRCLE Y : " << -circle.getCenter().getPoint().x);
        }
        showSegmentsAndCircleRviz(segments, circles);
    }
    catch (std::exception e)
    {
        ROS_INFO_STREAM("ERROR " << e.what());
    }
}//chatterCallback

// Create Trackbars
void createTrackBars()
{
    // Define window
    cv::String nameWindow = "trackbars";
    cv::namedWindow(nameWindow, cv::WINDOW_AUTOSIZE);

    // Trackbar for MIN_GROUP_POINTS 
    cv::String trackbarName1 = "MIN_GROUP_POINTS : ";
    int maxValue1 = 10;
    cv::createTrackbar(trackbarName1, nameWindow, &minGroupPointsTrackbar, maxValue1, onMinGroupPointsTrackbar);

    // Trackbar for MAX_GROUP_DISTANCE 
    cv::String trackbarName2 = "MAX_GROUP_DISTANCE : ";
    int maxValue2 = 10;
    cv::createTrackbar(trackbarName2, nameWindow, &maxGroupDistanceTrackbar, maxValue2, onMaxGroupDistanceTrackbar);

    // Trackbar for MAX_SPLIT_DISTANCE 
    cv::String trackbarName3 = "MAX_SPLIT_DISTANCE : ";
    int maxValue3 = 10;
    cv::createTrackbar(trackbarName3, nameWindow, &maxSplitDistanceTrackbar, maxValue3, onMaxSplitDistanceTrackbar);

    // Trackbar for MAX_MERGE_SEPARATION 
    cv::String trackbarName4 = "MAX_MERGE_SEPARATION : ";
    int maxValue4 = 10;
    cv::createTrackbar(trackbarName4, nameWindow, &maxMergeSeparationTrackbar, maxValue4, onMaxMergeSeparationTrackbar);

    // Trackbar for MAX_MERGE_SPREAD 
    cv::String trackbarName5 = "MAX_MERGE_SPREAD : ";
    int maxValue5 = 10;
    cv::createTrackbar(trackbarName5, nameWindow, &maxMergeSpreadTrackbar, maxValue5, onMaxMergeSpreadTrackbar);

    // Trackbar for RADIUS_ENLARGEMENT 
    cv::String trackbarName6 = "RADIUS_ENLARGEMENT : ";
    int maxValue6 = 10;
    cv::createTrackbar(trackbarName6, nameWindow, &radiusEnlargementTrackbar, maxValue6, onRadiusEnlargementTrackbar);

    // Trackbar for RADIUS_ENLARGEMENT 
    cv::String trackbarName7 = "MAX_CIRCLE_RADIUS : ";
    int maxValue7 = 10;
    cv::createTrackbar(trackbarName7, nameWindow, &maxCircleRadiusTrackbar, maxValue7, onMaxCircleRadiusTrackbar);

    cv::imshow(nameWindow, cv::Mat(10, 750, CV_8UC1));
    cv::waitKey();
}//createTrackBars

void showSegmentsAndCircleRviz(std::list<Segment>& segments, std::list<Circle>& circles)
{
    // Create Node
    ros::NodeHandle node;

    // Create message to publish
    ros::Publisher markerPub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Publish rate
    ros::Rate rate(30); // 1/30 = 0.03 sec

    for (int i = 0; i < 75; i++) // 2.33 sec
    {
        publishLines(segments,markerPub);
        publishCircles(circles,markerPub);        
        rate.sleep();
    }
}//showSegmentsAndCircleRviz

void publishLines(std::list<Segment> &segments, ros::Publisher &markerPub)
{
    visualization_msgs::Marker points;
    visualization_msgs::Marker line;

    // General infos of the msgs
    points.header.frame_id = line.header.frame_id = "/base_link";
    points.header.stamp = line.header.stamp = ros::Time::now();
    points.ns = line.ns = "points_and_lines_and_circles";
    points.action = line.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line.pose.orientation.w = 1.0;

    // Id of the marker
    points.id = 0;
    line.id = 1;

    // Type of the marker
    points.type = visualization_msgs::Marker::POINTS;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Line markers use only the x component of scale, for the line width
    line.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0;
    points.color.a = 1.0;

    // Line is blue
    line.color.b = 1.0;
    line.color.a = 1.0;

    for (Segment s : segments)
    {
        geometry_msgs::Point point1;
        point1.y = -s.getFirstPoint().getPoint().x;
        point1.x = s.getFirstPoint().getPoint().y;
        point1.z = 0;
        points.points.push_back(point1);
        line.points.push_back(point1);

        geometry_msgs::Point point2;
        point2.y = -s.getLastPoint().getPoint().x;
        point2.x = s.getLastPoint().getPoint().y;
        point2.z = 0;
        line.points.push_back(point2);
        points.points.push_back(point2);
    }

    markerPub.publish(points);
    markerPub.publish(line);
}//publishLines

void publishCircles(std::list<Circle> &circles, ros::Publisher &markerPub)
{
    std::vector<visualization_msgs::Marker> circlesMarker;

    int i = 0;
    for (Circle circle : circles)
    {
        visualization_msgs::Marker circleMarker;

        // General Info
        circleMarker.header.frame_id = "/base_link";
        circleMarker.header.stamp = ros::Time::now();
        circleMarker.ns = "circles";
        circleMarker.action = visualization_msgs::Marker::ADD;
        circleMarker.pose.orientation.w = 1.0;

        // Id
        circleMarker.id = i;

        // Type Cylinder
        circleMarker.type = visualization_msgs::Marker::CYLINDER;

        // Color red
        circleMarker.color.r = 1.0;
        circleMarker.color.a = 1.0;

        // Set center of the circle
        geometry_msgs::Point center;
        center.x = circle.getCenter().getPoint().y;
        center.y = -circle.getCenter().getPoint().x;
        center.z = 0.0;
        circleMarker.pose.position = center;

        // Set diameter of the circle
        circleMarker.scale.x = circle.getRadius() * 2;
        circleMarker.scale.y = circle.getRadius() * 2;
        circleMarker.scale.z = 0.0;

        //Publish circle
        markerPub.publish(circleMarker);

        i++;        
    }    
}//publishCircles

static void onMinGroupPointsTrackbar(int, void *)
{
    if (minGroupPointsTrackbar >= minValueMinGroupPoints)
    {
        *minGroupPoints = minGroupPointsTrackbar;
    }
}//onMinGroupPointsTrackbar

static void onMaxGroupDistanceTrackbar(int, void *)
{
    *maxGroupDistance = minValueMaxGroupDistance * maxGroupDistanceTrackbar;
}//onMaxGroupDistanceTrackbar

static void onMaxSplitDistanceTrackbar(int, void *)
{

    *maxSplitDistance = minValueMaxSplitDistance * maxSplitDistanceTrackbar;
}//onMaxSplitDistanceTrackbar

static void onMaxMergeSeparationTrackbar(int, void *)
{

    *maxMergeSeparation = minValueMaxMergeSeparation * maxMergeSeparationTrackbar;
}//onMaxMergeSeparationTrackbar

static void onMaxMergeSpreadTrackbar(int, void *)
{
    *maxMergeSpread = minValueMaxMergeSpread * maxMergeSpreadTrackbar;
}//onMaxMergeSpreadTrackbar

static void onRadiusEnlargementTrackbar(int, void *)
{
    *radiusEnlargement = minValueRadiusEnlargement * radiusEnlargementTrackbar;
}//onRadiusEnlargementTrackbar

static void onMaxCircleRadiusTrackbar(int, void *)
{
    *maxCircleRadius = minValueMaxCircleRadius * maxCircleRadiusTrackbar;
}//onMaxCircleRadiusTrackbar