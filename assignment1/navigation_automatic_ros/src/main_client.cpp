#include <ros/ros.h>
#include <navigation_automatic_ros/tiago_client.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "start1");

    //read the command lines argument 
    if (argc != 4) {
        ROS_INFO("Please provide x, y and orientazione wrt. z axis of the goal position!");
        return 1;  
    } 

    float x,y, orZ; 

    char* endPtr; 
    x = strtof(argv[1], &endPtr);
    y = strtof(argv[2], &endPtr);
    orZ = strtof(argv[3], &endPtr);

    //construct the client
    TiagoClient client(x,y,orZ,"TiagoServer"); 

    //send the goal
    client.sendGoal(); 

    ros::spin();
    return 0;
}