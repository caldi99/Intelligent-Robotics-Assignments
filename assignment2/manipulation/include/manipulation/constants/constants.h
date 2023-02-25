#ifndef CONSTANTSMANIPULATION_H
#define CONSTANTSMANIPULATION_H

#include <string>

/**
 * Constants of the manipulation package
 * @author Francesco Caldivezzi, Manuel Barusco, Riccardo Rampon
 */
namespace manipulation
{
    // NODES
    static const std::string NODE_TORSO_LIFTER = "torso_lifter";
    static const std::string NODE_PICK_PLACE = "pick_place";
    static const std::string NODE_HEAD_MOVEMENT = "head_movement";

    // TOPICS
    static const std::string TOPIC_TORSO_LIFTER = "/torso_lifter";
    static const std::string TOPIC_HEAD_MOVEMENT = "/head_movement";
    static const std::string TOPIC_CAMERA_INFO = "/xtion/rgb/camera_info";
    static const std::string TOPIC_PICK_PLACE = "/pick_place";
    static const std::string TOPIC_LINK_ATTACHER_NODE = "/link_attacher_node";
    static const std::string TOPIC_PLANNING_SCENE = "/planning_scene";
    static const std::string TOPIC_MY_POSE = "/my_pose";

    // REF. FRAMES
    static const std::string XTION_RGB_OPTICAL_FRAME_REFERENCE_FRAME = "xtion_rgb_optical_frame";
    static const std::string BASE_LINK_REFERENCE_FRAME = "base_link";

    // INTERFACES
    static const std::string INTERFACE_ARM_TORSO = "arm_torso";
    static const std::string INTERFACE_ARM = "arm";
    static const std::string INTERFACE_GRIPPER = "gripper";

    // JOINTS
    static const std::string TORSO_LIFT_JOINT = "torso_lift_joint";
    static const std::string HEAD_1_JOINT = "head_1_joint";
    static const std::string HEAD_2_JOINT = "head_2_joint";
    static const std::string ARM_1_JOINT = "arm_1_joint";
    static const std::string ARM_2_JOINT = "arm_2_joint";
    static const std::string ARM_3_JOINT = "arm_3_joint";
    static const std::string ARM_4_JOINT = "arm_4_joint";
    static const std::string ARM_5_JOINT = "arm_5_joint";
    static const std::string ARM_6_JOINT = "arm_6_joint";
    static const std::string ARM_7_JOINT = "arm_7_joint";
    static const std::string GRIPPER_LEFT_FINGER_JOINT = "gripper_left_finger_joint";
    static const std::string GRIPPER_RIGHT_FINGER_JOINT = "gripper_right_finger_joint";

    // ACTION FOR FOLLOW JOINT TRAJECTORY
    static const std::string FOLLOW_JOINT_TRAJECTORY_ACTION = "/head_controller/follow_joint_trajectory";

    // ACTION FOR HEAD POINTING
    static const std::string POINT_HEAD_ACTION = "/head_controller/point_head_action";

    // PLANNER_ID
    static const std::string PLANNER_ID = "SBLkConfigDefault";

    // MAX_VELOCITY_SCALING_FACTOR
    static const double MAX_VELOCITY_SCALING_FACTOR = 1.0;

    // PLANNING TIME
    static const double PLANNING_TIME = 50.0;

    // GAZEBO NAMES
    static const std::string HEXAGON = "Hexagon";
    static const std::string TRIANGLE = "Triangle";
    static const std::string CUBE = "cube";
    static const std::string TIAGO = "tiago";

    // LINKS
    static const std::string ARM_TOOL_LINK = "arm_tool_link";
    static const std::string ARM_7_LINK = "arm_7_link";
    static const std::string HEXAGON_LINK = "Hexagon_link";
    static const std::string TRIANGLE_LINK = "Triangle_link";
    static const std::string CUBE_LINK = "cube_link";

    // JOINT VALUES
    static const double ARM_1_JOINT_VALUE_DEFAULT = 0.07;
    static const double ARM_2_JOINT_VALUE_DEFAULT = 0.34;
    static const double ARM_3_JOINT_VALUE_DEFAULT = -3.13;
    static const double ARM_4_JOINT_VALUE_DEFAULT = 1.31;
    static const double ARM_5_JOINT_VALUE_DEFAULT = 1.58;
    static const double ARM_6_JOINT_VALUE_DEFAULT = 0.0;
    static const double ARM_7_JOINT_VALUE_DEFAULT = 0.0;

    // GRIPPER MODALITIES
    static const std::string OPEN_GRIPPER_MODALITY = "open";
    static const std::string CLOSE_GRIPPER_MODALITY = "close";

    // ATTACH DETEACH MODALITIES
    static const std::string ATTACH_MODALITY = "attach";
    static const std::string DETACH_MODALITY = "detach";

    // COLLISION OBJECTS
    static const std::string PICK_TABLE_COLLISION_OBJECT = "pick_table";
    static const std::string PLACE_TABLE_COLLISION_OBJECT = "place_table";
    static const std::string HEXAGON_COLLISION_OBJECT = "hexagon";
    static const std::string TRIANGLE_COLLISION_OBJECT = "triangle";
    static const std::string CUBE_COLLISION_OBJECT = "cube";
    static const std::string COLLISION_OBJECTS_COLLISION_OBJECT = "collision_objects"; 

    // COLLISION OBJECTS DIMENSIONS
    static const double PICK_TABLE_LENGTH = 0.913;
    static const double PICK_TABLE_WIDTH = 0.913;
    static const double PICK_TABLE_HEIGHT = 0.775;

    static const double HEXAGON_HEIGHT = 0.105364;
    static const double HEXAGON_DIAMETER = 0.0325;

    static const double TRIANGLE_LENGTH = 0.0325;
    static const double TRIANGLE_WIDTH = 0.0325;
    static const double TRIANGLE_HEIGHT = 0.021664;

    static const double CUBE_LENGTH = 0.05;
    static const double CUBE_WIDTH = 0.05;
    static const double CUBE_HEIGHT = 0.05;

    static const double CYLINDER_HEIGHT = 0.69;
    static const double CYLINDER_RADIUS = 0.21;

    static const double COLLISION_HEIGHT = 0.21073;
    static const double COLLISION_DIAMETER = 0.0650;

    static const double PLACE_TABLE_HEIGHT = 0.69;
    static const double PLACE_TABLE_RADIUS = 0.225;

    // DIMENSION OF GRIPPER FROM arm_tool_link
    static const double GRIPPER_LENGTH = 0.23;

    // OFFSETS
    static const double OFFSET_APPROACH_DEPART = 0.1;

    // OPEN GRIPPER MAX SIZE
    static const double OPEN_GRIPPER_MAX_SIZE = 0.045;
} // namespace manipulation

#endif // CONSTANTSMANIPULATION_H