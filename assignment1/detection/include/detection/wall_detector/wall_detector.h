#ifndef WALLDETECTOR_H
#define WALLDETECTOR_H

//OPENCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

//ROS
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

/**
 * This Class is used for detecting walls
 * @author : Francesco Caldivezzi, Riccardo Rampon
 * */
class WallDetector{

public:
    //CONSTRUCTORS
    /**
     * Constructor of the class.
     * @param angleIncrement : angle_increment of sensor_msgs::LaserScan message
     * @param angleMin : angle_min of sensor_msgs::LaserScan message
     * @param angleMax : angle_max of sensor_msgs::LaserScan message
     * @param rangeMax : range_max of sensor_msgs::LaserScan message
     * @param rangeMin : range_min of sensor_msgs::LaserScan message
    */
    WallDetector(float angleIncrement,float angleMin, float angleMax, float rangeMax, float rangeMin);
    
    //MEMBER FUNCTIONS
    /**
     * This function find the Walls of the corridor
     * @param ranges : ranges of sensor_msgs::LaserScan message
     * @param finalLines : lines that represents the walls in the real robot reference frame
     * @return True if the lines (walls) where detected
    */
    bool findLines(const std::vector<float>& ranges, std::vector<cv::Vec4f>& finalLines);

private:
    //FUNCTIONS 

    /**
     * This function creates an image of the scan scan of the laser
     * @param ranges : ranges of sensor_msgs::LaserScan message
    */
    void buildImage(const std::vector<float>& ranges);

    /**
     * This function computes the number of rows of the image that represent the scan of the laser
     * @return The number of rows
    */
    int getNumberRows();

    /**
     * This function computes the number of columns of the image that represent the scan of the laser
     * @return The number of columns
    */
    int getNumberCols();

    /**
     * This function computes width of a pixel of the image that represent the scan of the laser
     * @return The width of a pixel
    */
    float getWidthPixel();

    /**
     * This function computes height of a pixel of the image that represent the scan of the laser
     * @return The height of a pixel
    */
    float getHeightPixel();

    /**
     * This function compute the "float" x coordinate in the image reference frame of a point detected by the laser
     * @param rho : The distance from the robot reference frame **rotated** and the point detected by the laser
     * @param theta : The angle formed by the x-axis of the robot reference frame **rotated** segment formed by the point detected by the laser and the origin of the robot reference frame **rotated**
     * @return The "float" x coordinate in the image reference frame
    */
    float getXRawImage(float rho, float theta);

    /**
     * This function compute the "float" y coordinate in the image reference frame of a point detected by the laser
     * @param rho : The distance from the robot reference frame **rotated** and the point detected by the laser
     * @param theta : The angle formed by the x-axis of the robot reference frame **rotated** segment formed by the point detected by the laser and the origin of the robot reference frame **rotated**
     * @return The "float" y coordinate in the image reference frame
    */
    float getYRawImage(float rho, float theta);

    /**
     * This function compute the "pixel" x coordinate in the image reference frame of a point detected by the laser
     * @param rho : The distance from the robot reference frame **rotated** and the point detected by the laser
     * @param theta : The angle formed by the x-axis of the robot reference frame **rotated** segment formed by the point detected by the laser and the origin of the robot reference frame **rotated**
     * @return The "pixel" x coordinate in the image reference frame
    */
    int getXImage(float rho, float theta);
        
    /**
     * This function compute the "pixel" y coordinate in the image reference frame of a point detected by the laser
     * @param rho : The distance from the robot reference frame **rotated** and the point detected by the laser
     * @param theta : The angle formed by the x-axis of the robot reference frame **rotated** segment formed by the point detected by the laser and the origin of the robot reference frame **rotated**
     * @return The "pixel" y coordinate in the image reference frame
    */
    int getYImage(float rho, float theta);

    /**
     * This function convert the "pixel" coodinates in the image reference frame to "robot" coordinates int the robot reference frame **not rotated**
     * @param x : "pixel" x coordinate in the image reference frame
     * @param y : "pixel" y coordinate in the image reference frame
     * @return The "robot" coordinates int the robot reference frame **not rotated**
    */
    cv::Point2f convertToRobotCoordinates(int x, int y);

    /**
     * This function merges lines beloging to the same cluster, by creating a single line obtained by taking the bisstest yMax, smallest yMin and average x.
     * @param lines : The total set of lines detected
     * @param labels : The total set of labels
     * @param label : The cluster of lines to consider for the merging
     * @return : The final line obtained by taking the bisstest yMax, smallest yMin and average x.
     * 
    */
    cv::Vec4i mergeLines(const std::vector<cv::Vec4i>& lines, const std::vector<int>& labels, int label);

    /**
     * This function refines two distinct lines by returing the same two lines but with yMax = max(yMaxLine1, yMaxLine2) and yMin = min(yMinLine1, yMinLine2)
     * @param line1 : The first line
     * @param line2 : The second line
     * @return The two lines refined
    */
    std::vector<cv::Vec4i> refineLines(const cv::Vec4i& line1, const cv::Vec4i& line2);

    /**
     * This function is used to draw lines in a empty image of the same size of the image of the scan of the laser
     * @param lines : Lines to draw
     * @return : The image with the lines drawn
    */
    cv::Mat drawLines(const std::vector<cv::Vec4i>& lines);

    /**
     * This function is used to understand if two lines must be merged or not
     * @param line1: The first line
     * @param line1: The second line
     * @return : True if the two lines must be merged
    */
    static bool areLinesToMerge(const cv::Vec4i& line1, const cv::Vec4i& line2);
    
    //DATA MEMBERS
    float angleMin;
    float angleIncrement;
    float angleMax;
    float rangeMax;
    float rangeMin;
    cv::Mat image;

    //CONSTANTS HOUGH TRANSFORM
    const double THETA_HOUGH_TRANSFORM = CV_PI / 180; 
    const double RHO_HOUGH_TRANSFORM = 1;
    const int THRESHOLD_HOUGH_LINES = 1;
    const double MIN_LINE_LENGHT_HOUGH_LINES = 8.0;

    //CONSTANTS DILATION AND EROSION
    const cv::Size SIZE_MORPHOLOGICAL_OPERATOR_DILATION = cv::Size(2,25);
    const cv::Size SIZE_MORPHOLOGICAL_OPERATOR_EROSION = cv::Size(1,35);

    //CONSTANTS FOR UNDERSTANDING IF TWO PIXELS ARE THE SAME POINT
    static const int MAX_DISTANCE_EQUAL_POINTS = 9;
    static const int MAX_DISTANCE_EQUAL_POINTS_HEIGHT = 10;

};

#endif //WALLDETECTOR_H