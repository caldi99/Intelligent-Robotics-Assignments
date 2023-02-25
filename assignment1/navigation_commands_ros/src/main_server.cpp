#include <ros/ros.h>
#include <navigation_commands_ros/tiago_server_commands.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "start2");

    TiagoServerCommands server("TiagoServer");

    ros::spin();
    return 0;
}//main