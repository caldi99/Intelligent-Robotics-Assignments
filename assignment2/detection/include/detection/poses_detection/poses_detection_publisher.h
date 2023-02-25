#ifndef POSESDETECTIONPUBLISHER_H
#define POSESDETECTIONPUBLISHER_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <detection/constants/constants.h>

// SERVICE FILE
#include <detection_msgs/Transform.h>

// MESSAGE FILE
#include <detection_msgs/Detections.h>
#include <detection_msgs/Detection.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>

// APRIL TAG
#include <apriltag_ros/AprilTagDetectionArray.h>

// GEOMETRY MESSAGES
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

/**
 * This class publishes the poses detected.
 * @author Francesco Caldivezzi, Manuel Barusco 
 */
class PosesDetectionPublisher
{
private:
    ros::NodeHandle node;
    ros::Publisher publisher;

    /**
     * This function transforms the given pose from detection::SOURCE_REFERENCE_FRAME to detection::TARGET_REFERENCE_FRAME
     * @param pose : The pose in the detection::SOURCE_REFERENCE_FRAME
     * @return The pose in the detection::TARGET_REFERENCE_FRAME
     */
    geometry_msgs::Pose transformPose(geometry_msgs::Pose pose);

public:
    /**
     * Create a node that will manage the detections of the apriltag and return the poses in the correct reference frame
     * @param node : The node of the publisher
     */
    PosesDetectionPublisher(ros::NodeHandle node);

    /**
     * This function publish a single message with the poses of the detections in the correct reference frame
     */
    void publish();
};

#endif // POSESDETECTIONPUBLISHER_H
