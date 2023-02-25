#ifndef OBSTACLEEXTRACTOR_H
#define OBSTACLEEXTRACTOR_H

// ROS
#include <ros/ros.h>

// DETECTION PACKAGE
#include <detection/obstacle_extractor/figures/circle.h>
#include <detection/obstacle_extractor/figures/segment.h>
#include <detection/obstacle_extractor/figures/pointset.h>
#include <detection/obstacle_extractor/figures/point.h>

// CONSTANTS
#include <detection/constants/constants.h>

// STL
#include <list>

/**
 * This class is an helper class that is able to extract lines from a set of Points
 * @author : Francesco Caldivezzi
 */
class ObstacleExtractor
{
public:
    // CONSTRUCTORS

    /**
     * Constructor of the class
     * @param inputPoints : List of points that represents the points that were scan by the laser
     * @param angleIncrement : angle_increment of the laser
     */
    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement);

    /**
     * TODO: REMOVE This constructor is used for debugging purposes.
     */
    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement,
                      int minGroupPoints, double maxGroupDistance,
                      double maxSplitDistance, double maxMergeSeparation,
                      double maxMergeSpread, double radiusEnlargement, double maxCircleRadius);

    /**
     * Constructor of the class
     * @param inputPoints : List of points that represents the points that were scan by the laser
     * @param angleIncrement : angle_increment of the laser
     * @param maxCircleRadius : (r_max in the paper) if a circle would have greater radius than this value, skip it
     * @param minCircleRadius : Threashold to remove circles that are not actually circles
     * @param radiusEnlargment : (r_d in the paper) artificially enlarge the circles radius by this value
     */
    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement, double maxCircleRadius, double minCircleRadius, double radiusEnlargement);

    // MEMBER FUNCTIONS

    /**
     * This function processes the input points and returns the list of segments detected
     * @return The list of segments detected
     */
    void processPoints(std::list<Segment> &segments, std::list<Circle> &circles);

private:
    // FUNCTIONS

    /**
     * This function groups points into pointsets that are further processed and that hypotetically identify a single segment
     */
    void groupPoints();

    /**
     * This function given a pointSet, detects segments inside of it
     * @param pointSet : The set of points to use to detect segments
     */
    void detectSegments(const PointSet &pointSet);

    /**
     * This function merges the segments detected
     */
    void mergeSegments();

    /**
     * This function checks if two segments satisfy the following conditions (both):
     * - the two segments are near each other according to checkSegmentsProximity function
     * - the fitted segment, obtained by the points of two segments, is collinear with the ones provided
     * If so, then such fitted segment is returned ad is the one that represents the merging between them
     * @param segment1 : The first segment to compare
     * @param segment2 : The second segment to compare
     * @param mergedSegment : The merged segment obtained eventually if the two segments are merged
     * @return True if the two segments are merged
     */
    bool compareSegments(const Segment &segment1, const Segment &segment2, Segment &mergedSegment);

    /**
     * This function, given two segments checks if they are near each other.
     * That means either one of the following two conditions are satisfied :
     * - segment1 is near either to firstPoint of segment2 or lastPoint of segment2
     * - segment2 is near either to firstPoint of segment1 or lastPoint of segment1
     * Where "nearness" is defined with the threashold MAX_MERGE_SEPARATION (less than)
     * @param segment1 : The first segment to compare
     * @param segment2 : The second segment to compare
     * @return True if the two segment are near each other
     */
    bool checkSegmentsProximity(const Segment &segment1, const Segment &segment2);

    /**
     * This functions checks if a segment is collinear with other two segments.
     * That means that the following conditions are satified
     * - segment is near to firstPoint of segment1
     * - segment is near to lastPoint of segment1
     * - segment is near to firstPoint of segment2
     * - segment is near to lastPoint of segment2
     * Where "nearness" is defined with the threashold MAX_MERGE_SPREAD (less than)
     * @param segment : The segment to check if it is collinear with other 2
     * @param segment1 : The first segment to compare
     * @param segment2 : The second segment to compare
     * @return : True if segment is collinear with segment1 and segment2
     */
    bool checkSegmentsCollinearity(const Segment &segment, const Segment &segment1, const Segment &segment2);

    /**
     * This functions finds among the detected segments if it can be built a circle that represents other kind of obstacles and not walls
     */
    void detectCircles();

    /**
     * This function merges two circles if the two circles intersect.
     */
    void mergeCircles();

    /**
     * This function merges two circles if either :
     * - one of the two circles are inside of the other
     * - they intersect and they are "small"
     * @param circle1 : The first circle to merge
     * @param circle2 : The second circle to merge
     * @param mergedCircle : The merged circle
     * @return True if the two circles are merged
     */
    bool compareCircles(const Circle &circle1, const Circle &circle2, Circle &mergedCircle);

    // DATA MEMEBERS
    std::list<Point> inputPoints;
    std::list<Segment> segments;
    std::list<Circle> circles;
    double distanceProportion;
    double maxCircleRadius = 0.35;
    double minCircleRadius = 0.05;
    double radiusEnlargement = 0.05;
};

#endif // OBSTACLEEXTRACTOR_H