#ifndef CONSTANTSDETECTION_H
#define CONSTANTSDETECTION_H

#include <string>

/**
 * Constants file
 * @author Francesco Caldivezzi, Riccardo Rampon, Manuel Barusco
*/
namespace detection
{
    // NODES
    const static std::string NODE_POSES_TRANSFORMER = "poses_transformer_node";
    const static std::string NODE_POSES_DETECTION = "poses_detection_node";
    const static std::string NODE_OBSTACLE_EXTRACTOR = "obstacle_extractor_node";
    const static std::string NODE_TABLE_DETECTION = "table_detection_node";
    const static std::string NODE_POSE_TABLE_DETECTOR = "pose_table_detector";

    // TOPICS
    const static std::string TOPIC_JOINT_STATUS = "/joint_states";
    const static std::string TOPIC_POSES_DETECTION = "/poses_detection";
    const static std::string TOPIC_TAG_DETECTIONS = "/tag_detections";
    const static std::string TOPIC_POSES_TRANSFORMER = "/poses_transformer";
    const static std::string TOPIC_OBSTACLE_EXTRACTOR = "/obstacle_extractor";
    const static std::string TOPIC_CAMERA_INFO = "/xtion/rgb/camera_info";
    static const std::string TOPIC_IMAGE = "/xtion/rgb/image_raw";
    static const std::string TOPIC_HEAD_MOVEMENT = "/head_movement";
    static const std::string TOPIC_TABLE_OBJECTS_DETECTION = "/table_objects_detection";
    static const std::string TOPIC_POSE_TABLE_DETECTOR = "/pose_table_detector";

    // FRAMES
    const static std::string XTION_RGB_OPTICAL_FRAME_REFERENCE_FRAME = "xtion_rgb_optical_frame";
    const static std::string BASE_LINK_REFERENCE_FRAME = "base_link";

    // QUEUE SIZE
    const static int QUEUE_SIZE = 1000;
    // LOOP RATE
    const static int LOOP_RATE = 10;

    // CONSTANTS OBSTACLE EXTRACTOR
    // LINES
    const bool USE_SPLIT_AND_MERGE = true;    // choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments
    const int MIN_GROUP_POINTS = 5;           // minimum number of points comprising a group to be further processed
    const double MAX_GROUP_DISTANCE = 0.05;   //(d_group in the paper) if the distance between two points is greater than this value, start a new group
    const double MAX_SPLIT_DISTANCE = 0.3;    //(d_split in the paper) if a point in a group lays further from the fitted line than this value, split the group
    const double MAX_MERGE_SEPARATION = 0.03; //(d_0 in the paper) if distance between lines is smaller than this value, consider merging them
    const double MAX_MERGE_SPREAD = 0.03;     //(d_spread in the paper) merge two segments if all of their extreme points lay closer to the leading line than this value,
    // CIRCLES
    const bool CIRCLE_FROM_VISIBLE = true;        // detect circular obstacles only from fully visible (not occluded) segments
    const bool DISCARD_CONVERTED_SEGMENTS = true; // remove segments that where transformed into circles
}

#endif // CONSTANTSDETECTION_H