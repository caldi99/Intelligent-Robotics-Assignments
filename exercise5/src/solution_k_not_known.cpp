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

#include <laser_scanner/clusterizer.h>

int main(int argc, char** argv)
{
    //Node Creation
    ros::init(argc, argv, "solution_node");

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


    //Idea : kMin = 2, kMax = ((2 * pi * (maxRadiusCircumference = 3.5)) /(minSpaceOccupiedByAPerson = 0.75)) / numberOfLegs = 2
    Clusterizer cluterizer(2,29,false);
    std::vector<int> labels;
    cv::Mat centers;
    cluterizer.clusterize(points,labels,centers);

    ROS_INFO_STREAM("K : " << centers.rows <<std::endl);

    ROS_INFO_STREAM("CENTERS : " << std::endl);
    ROS_INFO_STREAM(centers);

    return 0;
}

