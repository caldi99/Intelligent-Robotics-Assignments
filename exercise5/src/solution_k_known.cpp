//GROUP : MANUEL BARUSCO 2053083, FRANCESCO CALDIVEZZI 2037893, RICCARDO RAMPON 2052416

//ROS
#include <ros/ros.h>

//SENSORS_MSGS
#include <sensor_msgs/LaserScan.h>

//OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//LIMITS
#include <limits>

//MY LIBRARY
#include <laser_scanner/clusterizer.h>

int main(int argc, char** argv)
{
    //Node Creation
    ros::init(argc, argv, "solution_k_not_known");

    //Start Node
    ros::NodeHandle node;

    //Read data of the scanner only once because it is always the same
    sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",node);

    //Convert to polar coordinates the points
    float increment = msg->angle_increment;    
    std::vector<cv::Point2f> points;

    ROS_INFO_STREAM("INPUT POINTS : " << std::endl);

    for(int i = 0; i < msg->ranges.size(); i++)
    {
        if(msg->ranges.at(i) != std::numeric_limits<float>::infinity())        
        {
            cv::Point2f point(msg->ranges.at(i) * std::cos(i*increment),msg->ranges.at(i) * std::sin(i*increment));
            points.push_back(point); 
            ROS_INFO_STREAM(point << std::endl);
        }
    }
    //Cluster points
    int k = 3;
    Clusterizer clusterizer(k,k,true);
    cv::Mat centers;  
    std::vector<int> labels; 
    clusterizer.clusterize(points,labels,centers);

    ROS_INFO_STREAM("CENTERS : " << std::endl);
    ROS_INFO_STREAM(centers);

    return 0;
}