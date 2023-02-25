#ifndef HUMANCLIENT_H
#define HUMANCLIENT_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <solution/constants/constants.h>

// CUSTOM SERVICE
#include <tiago_iaslab_simulation/Objs.h>

// GEOMETRY MSGS
#include <geometry_msgs/Pose.h>

/**
 * This class is a client for the Human 
 * @author Francesco Caldivezzi
*/
class HumanClient
{
public:
    /**
     * Constructor of the class
     * @param node : node with which creating the client
     * @param serverName : name of the server
     */
    HumanClient(ros::NodeHandle node, std::string serverName);

    /**
     * Function that return the response obtained from the server
     * @param ready : ready parameter of the Obj service message
     * @param allObjs : all_objs parameter of the Obj service message
     * @param ids : ids returned by the service message
     * @return : True if communication was succeded
     */
    bool getResponse(bool ready, bool allObjs, std::vector<int> &ids);

    /**
     * Converts the ids of the objects to the destinations where the object are
     * @param ids : ids to convert
     * @param poses : poses where the objects can be found
     */
    void convertIdsToDestinationPoses(std::vector<int> &ids, std::vector<geometry_msgs::Pose> &poses);

private:
    ros::NodeHandle node;
    ros::ServiceClient client;
};

#endif // HUMANCLIENT_H