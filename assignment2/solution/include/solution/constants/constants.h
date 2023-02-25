#ifndef CONSTANTSSOLUTION_H
#define CONSTANTSSOLUTION_H

#include <string>

/**
 * Constants of solution package
 * @author Manuel Barusco, Francesco Caldivezzi, Riccardo Rampon
 */
namespace solution
{
    // NODES
    const static std::string NODE_SOLUTION = "solution";

    // TOPICS
    const static std::string TOPIC_TIAGO_SERVER = "/tiago_server";
    const static std::string TOPIC_HUMAN_SERVER = "/human_objects_srv";
    const static std::string TOPIC_POSES_DETECTION = "/poses_detection";
    static const std::string TOPIC_TORSO_LIFTER = "/torso_lifter";
    const static std::string TOPIC_POSES_TRANSFORMER = "/poses_transformer";
    const static std::string TOPIC_OBSTACLE_EXTRACTOR = "/obstacle_extractor";
    const static std::string TOPIC_SCAN = "/scan";
    static const std::string TOPIC_TABLE_OBJECTS_DETECTION = "/table_objects_detection";
    static const std::string TOPIC_HEAD_MOVEMENT = "/head_movement";
    static const std::string TOPIC_PICK_PLACE = "/pick_place";
    static const std::string TOPIC_POSE_TABLE_DETECTOR = "/pose_table_detector";

    // POSES
    // BLUE
    const static double POSE_POSITION_X_ID1 = 8.17761;
    const static double POSE_POSITION_Y_ID1 = -1.9486;
    const static double POSE_ORIENTATION_Z_ID1 = -0.708202;
    const static double POSE_ORIENTATION_W_ID1 = 0.70601;

    // RED
    const static double POSE_POSITION_X_ID3 = 7.44911;
    const static double POSE_POSITION_Y_ID3 = -1.9486;
    const static double POSE_ORIENTATION_Z_ID3 = -0.709458;
    const static double POSE_ORIENTATION_W_ID3 = 0.704748;

    // GREEN
    const static double POSE_POSITION_X_ID2 = 7.69915;
    const static double POSE_POSITION_Y_ID2 = -4.02856;
    const static double POSE_ORIENTATION_Z_ID2 = 0.686435;
    const static double POSE_ORIENTATION_W_ID2 = 0.727191;

    // WAYPOINT1 (NEAR BROWN CIRCLE OBSTACLE)
    const static double POSE_POSITION_X_WAYPOINT1 = 8.44668;
    const static double POSE_POSITION_Y_WAYPOINT1 = -0.0281095;
    const static double POSE_ORIENTATION_Z_WAYPOINT1 = -0.709564;
    const static double POSE_ORIENTATION_W_WAYPOINT1 = 0.704641;

    // WAYPOINT2 (NEAR SECOND ROOM ANDATA)
    const static double POSE_POSITION_X_WAYPOINT2 = 8.83445;
    const static double POSE_POSITION_Y_WAYPOINT2 = -4.12874;
    const static double POSE_ORIENTATION_Z_WAYPOINT2 = 0.0;
    const static double POSE_ORIENTATION_W_WAYPOINT2 = 1.0;

    // WAYPOINT3 (ONLY FOR RED AND BLUE)
    const static double POSE_POSITION_X_WAYPOINT3 = 8.80689464438;
    const static double POSE_POSITION_Y_WAYPOINT3 = -1.54279238433;
    const static double POSE_ORIENTATION_Z_WAYPOINT3 = -0.65988535475;
    const static double POSE_ORIENTATION_W_WAYPOINT3 = 0.751366301205;

    // WAYPOINT4 (DOCKING POSITION)
    const static double POSE_POSITION_X_WAYPOINT4 = 11.463908;
    const static double POSE_POSITION_Y_WAYPOINT4 = -2.100030;
    const static double POSE_ORIENTATION_Z_WAYPOINT4 = 0.710590;
    const static double POSE_ORIENTATION_W_WAYPOINT4 = 0.703605;

    // WAYPOINT5 (NEAR SECOND ROOM RITORNO)
    const static double POSE_POSITION_X_WAYPOINT5 = 8.83445;
    const static double POSE_POSITION_Y_WAYPOINT5 = -4.12874;
    const static double POSE_ORIENTATION_Z_WAYPOINT5 = 0.689923;
    const static double POSE_ORIENTATION_W_WAYPOINT5 = 0.723883;

    // POSE TABLE GREEN
    const static double POSE_POSITION_X_TABLE_ID2 = 11.4580622631;
    const static double POSE_POSITION_Y_TABLE_ID2 = -1.01143845239;
    const static double POSE_ORIENTATION_Z_TABLE_ID2 = 0.728407926567;
    const static double POSE_ORIENTATION_W_TABLE_ID2 = 0.685143702091;

    // POSE TABLE RED
    const static double POSE_POSITION_X_TABLE_ID3 = 10.524329443;
    const static double POSE_POSITION_Y_TABLE_ID3 = -1.01143845239;
    const static double POSE_ORIENTATION_Z_TABLE_ID3 = 0.709314931812;
    const static double POSE_ORIENTATION_W_TABLE_ID3 = 0.704891713321;

    // POSE TABLE BLUE
    const static double POSE_POSITION_X_TABLE_ID1 = 12.6226028683;
    const static double POSE_POSITION_Y_TABLE_ID1 = -1.01143845239;
    const static double POSE_ORIENTATION_Z_TABLE_ID1 = 0.709314931812;
    const static double POSE_ORIENTATION_W_TABLE_ID1 = 0.704891713321;

    // TORSO LIFT JOINT INITIAL VALUE
    const static double TORSO_LIFT_JOINT_INITIAL = 0.15;

    // TORSO LIFT JOINT TARGET VALUE
    const static double TORSO_LIFT_JOINT_TARGET = 0.340;

    // HEIGHT LASER
    const static double HEIGHT_LASER = 0.0945;

    // BASE_LASER_LINK REFERENCE FRAME
    const static std::string BASE_LASER_LINK_REFERENCE_FRAME = "base_laser_link";

    // MAP REFERENCE FRAME
    const static std::string MAP_REFERENCE_FRAME = "map";

    // BASE_LINK REFERENCE FRAME
    const static std::string BASE_LINK_REFERENCE_FRAME = "base_link";

    // CIRCLES DETECTION PARAMETERS
    const static double MAX_CIRCLE_RADIUS_TABLE_PICK = 0.1;
    const static double MIN_CIRCLE_RADIUS_TABLE_PICK = 0.07;
    const static double RADIUS_ENLARMENT_TABLE_PICK = 0.05;

    const static double MAX_CIRCLE_RADIUS_DETECTION = 0.35;
    const static double MIN_CIRCLE_RADIUS_DETECTION = 0.05;
    const static double RADIUS_ENLARMENT_DETECTION = 0.05;

    // THREADSHOLD CIRCLES FOR PLACE TABLES
    const static double THRESHOLD_RADIUS_CIRCLES_PLACE_TABLE = 0.1;

    // PICK PLACE MODE
    const static unsigned char PICK_MODE = 0;
    const static unsigned char PLACE_MODE = 1;

    // ORIENTATION PLACE TABLE
    const static double ORIENTATION_W_PLACE_TABLES = 0.685143702091;
    const static double ORIENTATION_Z_PLACE_TABLES = 0.728407926567;

    // OFFSET PLACE TABLE
    const static double OFFSET_PLACE_POSITION_TABLE = -0.65;
}

#endif // CONSTANTSSOLUTION_H