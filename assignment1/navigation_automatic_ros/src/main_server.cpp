#include <ros/ros.h>
#include <navigation_automatic_ros/tiago_server.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "start2");

    TiagoServer server ("TiagoServer");

    ros::spin();
    return 0;
}