#include <detection/poses_transformer/poses_transformer_server.h>

PosesTransformerServer::PosesTransformerServer(ros::NodeHandle node)
{
    ROS_INFO_STREAM("(Server) detection PosesTransformerServer PosesTransformerServer");
    this->node = node;
    this->server = this->node.advertiseService(detection::TOPIC_POSES_TRANSFORMER, &PosesTransformerServer::sendResponse, this);
}

bool PosesTransformerServer::sendResponse(detection_msgs::Transform::Request &req, detection_msgs::Transform::Response &res)
{
    // ROS_INFO_STREAM("(Server) detection PosesTransformerServer sendResponse");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transform;
    bool success = true;

    try
    {
        transform = tfBuffer.lookupTransform(req.target_frame, req.source_frame, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(req.pose, res.pose, transform);
    }
    catch (tf2::LookupException ex)
    {
        ROS_WARN("Error in transformation: %s", ex.what());
        success = false;
    }

    return success;
}
